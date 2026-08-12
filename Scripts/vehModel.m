% vehModel.m - Vehicle Model 
%
% Define vehicle model symbolic variables and equations using Casadi's 
% symbolic variables.
 
import casadi.*

% Import polynomial coefficients for aerodynamic functions
% load('D:\MATLAB_Projects\MLTP_AA_ATD\Aerodynamics\DATA_AA.mat');
% (dead pre-computation block removed: it referenced vx before its SX
%  definition and its bare Cl_*/Cd_tot outputs were never consumed - the
%  live aero coefficients are built in the ActAero ladder below)

%% Vehicle model (A) - state variables
nx = 9;                                                                         % number of state variables

% longitudinal velocity [m/s]
vx_n = SX.sym('vx_n');
vx_s = 100;
vx   = vx_s * vx_n;

% lateral velocity [m/s]
vy_n = SX.sym('vy_n');
vy_s = 10;
vy   = vy_s * vy_n;

% yaw rate [rad/s]
r_n = SX.sym('yawrate_n');
r_s = 1;
r   = r_s * r_n;

% lateral distance to centreline [m]                                            % left of centreline => n > 0; right => n < 0
n_n = SX.sym('n_n');
n_s = 5;
n   = n_s * n_n;

% angle to centreline tangent direction [rad]
eps_n = SX.sym('eps_n');
eps_s = 1;
eps   = eps_s * eps_n;

% angular velocity front left tyre [rad/s]
Om_fl_n = SX.sym('Om_fl_n');
Om_fl_s = vx_s/vp.Rw;
Om_fl   = Om_fl_s * Om_fl_n;

% angular velocity front tyre [rad/s]
Om_fr_n = SX.sym('Om_fr_n');
Om_fr_s = vx_s/vp.Rw;
Om_fr   = Om_fr_s * Om_fr_n;

% angular velocity rear left tyre [rad/s]
Om_rl_n = SX.sym('Om_rl_n');
Om_rl_s = vx_s/vp.Rw;
Om_rl   = Om_rl_s * Om_rl_n;

% angular velocity rear right tyre [rad/s]
Om_rr_n = SX.sym('Om_rr_n');
Om_rr_s = vx_s/vp.Rw;
Om_rr   = Om_rr_s * Om_rr_n;

%%-State limits
vx_lim = 1/vx_s * [OPT_e pt.Vmax];
vy_lim = 1/vy_s * [-10 10];
r_lim  = 1/r_s * [-pi/2 pi/2];
n_lim = 1/n_s * [-4 4];                                                         % Constant track limits
eps_lim = 1/eps_s * [-pi/4 pi/4];
Om_fl_lim = 1/Om_fl_s * [OPT_e/vp.Rw pt.Vmax/vp.Rw];
Om_fr_lim = 1/Om_fr_s * [OPT_e/vp.Rw pt.Vmax/vp.Rw];
Om_rl_lim = 1/Om_rl_s * [OPT_e/vp.Rw pt.Vmax/vp.Rw];
Om_rr_lim = 1/Om_rr_s * [OPT_e/vp.Rw pt.Vmax/vp.Rw];

% scaling factors
x_s = [vx_s; vy_s; r_s; n_s; eps_s; Om_fl_s; Om_fr_s; Om_rl_s; Om_rr_s];

% states vector (scaled)
x = [vx_n; vy_n; r_n; n_n; eps_n; Om_fl_n; Om_fr_n; Om_rl_n; Om_rr_n]; 

% limits
x_lim = [vx_lim; vy_lim; r_lim; n_lim; eps_lim; Om_fl_lim; Om_fr_lim; Om_rl_lim; Om_rr_lim];
x_min = x_lim(:,1);
x_max = x_lim(:,2);

%Check number of states
if nx ~= length(x) 
    error('Number of states is not consistent');
end

%% Vehicle model (A) - control variables (inputs)

% number of control variables
if pt.ATD == 0 && vp.ActAero == 0
    nu = 3;  
elseif pt.ATD == 0 && vp.ActAero == 1
    nu = 4;
    if any(getfielddef(vp,'rwMandate',0) == [6 7]), nu = 5; end   % ARFWd/ARFWr: + activeAeroFW
elseif pt.ATD == 0 && vp.ActAero == 2
    nu = 5;     
elseif pt.ATD == 0 && vp.ActAero == 3
    nu = 7;
elseif pt.ATD == 1 && vp.ActAero == 0
    nu = 7;
elseif pt.ATD == 1 && vp.ActAero == 1
    nu = 8;
    if any(getfielddef(vp,'rwMandate',0) == [6 7]), nu = 9; end   % ARFWd/ARFWr: + activeAeroFW
elseif pt.ATD == 1 && vp.ActAero == 2
    nu = 9;     
elseif pt.ATD == 1 && vp.ActAero == 3
    nu = 11;
end


% driving torque motor  (Nm)
T_motor_n = SX.sym('T_motor_n');
T_motor_s = pt.Tmax;
T_motor = T_motor_s * T_motor_n;
T_motor_lim = 1/T_motor_s * [0 pt.Tmax];

% braking torque        (Nm)
T_brake_n = SX.sym('T_brake_n');
T_brake_s = vp.Tbrake_max;
T_brake = T_brake_s * T_brake_n;
T_brake_lim = 1/T_brake_s * [-vp.Tbrake_max 0];

% tyre steer angle      (rad)
delta_n = SX.sym('delta_n');
delta_s = pi/8;
delta = delta_s * delta_n;
delta_lim   = 1/delta_s   * [-pi/4 pi/4];

% torque distribution
if pt.ATD == 1
    ATD_FL_n = SX.sym('ATD_FL_n');                                  % fraction to front left wheel
    ATD_FL_s = 1;
    ATD_FL = ATD_FL_s * ATD_FL_n;
    ATD_FL_lim = 1/ATD_FL_s * [0 1]; 

    ATD_FR_n = SX.sym('ATD_FR_n');                                  % fraction to front right wheel
    ATD_FR_s = 1;
    ATD_FR = ATD_FR_s * ATD_FR_n;
    ATD_FR_lim = 1/ATD_FR_s * [0 1];

    ATD_RL_n = SX.sym('ATD_RL_n');                                  % fraction to rear left wheel
    ATD_RL_s = 1;
    ATD_RL = ATD_RL_s * ATD_RL_n;
    ATD_RL_lim = 1/ATD_RL_s * [0 1];

    ATD_RR_n = SX.sym('ATD_RR_n');                                  % fraction to rear right wheel
    ATD_RR_s = 1;
    ATD_RR = ATD_RR_s * ATD_RR_n;
    ATD_RR_lim = 1/ATD_RR_s * [0 1];
end


% aerodynamics          (deg)
if vp.ActAero == 1
    activeAeroRW_n = SX.sym('activeAeroRW_n');
    activeAeroRW_s = 30;
    activeAeroRW = activeAeroRW_s * activeAeroRW_n;
    % Upper bound is +15 deg as of 2026-08-05: the map's last node is +15, and the
    % evaluator must not be asked to extrapolate past it. ARW previously sat AT the
    % old +20 bound for 0.55 % (BCN) / 0.76 % (NUR) of the lap, so this genuinely
    % removes the free wing's airbrake and ARW will be slower than its 5-node lap.
    activeAeroRW_lim = 1/activeAeroRW_s * [-10 15];      % ActiveRW: the rear-wing angle of attack is a
                                                         % continuous control over the DIGITISED sweep
                                                         % range [-10, +15] deg (= rwAeroMap2D's node
                                                         % span; the map errors outside it). The scale
                                                         % stays 30, so the normalised range is
                                                         % [-1/3, +1/2] - deliberate, the control keeps
                                                         % its name/scale for the userOpts + MLTP ladders.
    % The Velocity variant (vp.rwMandate == 3) ELIMINATES the +20 deg setting, so the
    % control's own upper bound comes down to its airbrake angle. Dropping the +20
    % well from the snap penalty is not enough on its own: the penalty only reshapes
    % where the optimum sits, and the first ARWv solve duly ran the wing to +20.00 deg
    % - a setting its own schedule never requests. Making it a BOUND is what actually
    % removes the state. Lower bound and scale are untouched, so nu, the ladders and
    % the map's valid span are all unaffected.
    %
    % SINCE 2026-08-05 THIS CLAMP IS A NO-OP ON THE DEFAULT PATH. The mode-0 bound
    % above is now +15 and alphaMax defaults to aBrake = +15, so it re-assigns the
    % same number. It is kept because it is an ASSIGNMENT, not a min(): it is still
    % what makes the ARWv bound track alphaMax if either value is ever moved, and
    % Validation/validateRWVelocity.m's D3 leg grades aBrake <= +15 through it.
    %
    % alphaMax is the wing's REACHABLE upper bound and defaults to aBrake, but the
    % two are deliberately separate fields. "What the schedule asks for" and "what
    % the wing can physically reach" are different quantities, and collapsing them
    % is precisely what let the first ARWv solve run to +20 deg while its own
    % schedule never requested more than +15: the +20 well had been dropped from
    % the snap penalty, which discourages a state without removing it. Keeping the
    % bound as its own field also makes that defect reproducible on demand
    % (rwVelAlphaMax = 20 with aBrake = 15 recreates it exactly) instead of being
    % a claim in a commit message.
    %
    % WARNING - THAT REPRODUCTION RECIPE IS NOW UNSAFE, AND IT FAILS QUIETLY.
    % Because this is an assignment it can RAISE the bound as well as lower it, and
    % since 2026-08-05 the aero map's last node is +15. rwVelAlphaMax > 15 therefore
    % drives the wing past the map's node span, and NOTHING ON THE SOLVE PATH STOPS
    % IT: the SX evaluator below (the ActAero==1 block) carries no range guard, so
    % it silently extrapolates the top segment's cubic for the whole multi-hour
    % solve, and Scripts/MLTP.m's post-processing only WARNS and clamps
    % (search: alphaMin/alphaMax) after the fact. The numeric twin
    % Functions/rwAeroMapEvalNum.m does error, but the solve never calls it. Net
    % effect: a plausible-looking but wrong lap plus one warning, which is worse
    % than the fail-fast this recipe used to give. If you need the +20 defect back,
    % rebuild the map with the five-node list too - do not raise this bound alone.
    if getfielddef(vp,'rwMandate',0) == 3
        activeAeroRW_lim(2) = 1/activeAeroRW_s * ...
            getfielddef(vp.rwVelOpts, 'alphaMax', getfielddef(vp.rwVelOpts,'aBrake',15));
    elseif getfielddef(vp,'rwMandate',0) == 5
        % AFWd: the control keeps the name/scale activeAeroRW/activeAeroRW_s for
        % the u/u_lim ladders and MLTP.m's uRow discovery (see
        % docs/superpowers/plans/2026-08-07-afwd-front-wing.md), but it is the
        % combined FW+RW "unload axis" over Functions/fwAeroDelta.m's digitised
        % range [-25, 0] deg (== rwAeroMap2D's node span for this mode, built in
        % Parameters/vehParams.m), not the rear-wing sweep's [-10, 15]. Full
        % reassignment, not a one-sided cap like the mode-3 block above, because
        % BOTH ends move.
        activeAeroRW_lim = 1/activeAeroRW_s * [-25 0];
    end
    if any(getfielddef(vp,'rwMandate',0) == [6 7])
        % ARFWd/ARFWr: SECOND wing surface - front-wing flap, its own continuous
        % control over Functions/fwOnlyDelta.m's axis. NOT the legacy
        % ActAero==2 activeAeroFW (scale 10, [0,10], dead code) - mode 6/7
        % defines its own. RW symbol/bounds above are untouched ([-10, +15]).
        activeAeroFW_n = SX.sym('activeAeroFW_n');
        activeAeroFW_s = 30;
        activeAeroFW = activeAeroFW_s * activeAeroFW_n;
        activeAeroFW_lim = 1/activeAeroFW_s * [-25 0];
    end
elseif vp.ActAero == 2
    activeAeroFW_n = SX.sym('activeAeroFW_n');
    activeAeroFW_s = 10;
    activeAeroFW = activeAeroFW_s * activeAeroFW_n;
    activeAeroFW_lim = 1/activeAeroFW_s * [0 10];

    activeAeroRW_n = SX.sym('activeAeroRW_n');
    activeAeroRW_s = 30;
    activeAeroRW = activeAeroRW_s * activeAeroRW_n;   
    activeAeroRW_lim = 1/activeAeroRW_s * [0 30];
elseif vp.ActAero == 3
    activeAeroFL_n = SX.sym('activeAeroFL_n');
    activeAeroFL_s = 10;
    activeAeroFL = activeAeroFL_s * activeAeroFL_n;
    activeAeroFL_lim = 1/activeAeroFL_s * [0 10];

    activeAeroFR_n = SX.sym('activeAeroFR_n');
    activeAeroFR_s = 10;
    activeAeroFR = activeAeroFR_s * activeAeroFR_n;
    activeAeroFR_lim = 1/activeAeroFR_s * [0 10];

    activeAeroRW_n = SX.sym('activeAeroRW_n');
    activeAeroRW_s = 30;
    activeAeroRW = activeAeroRW_s * activeAeroRW_n;
    activeAeroRW_lim = 1/activeAeroRW_s * [0 30];

    activeAeroTW_n = SX.sym('activeAeroTW_n');
    activeAeroTW_s = 12;
    activeAeroTW = activeAeroTW_s * activeAeroTW_n;
    activeAeroTW_lim = 1/activeAeroTW_s * [-12 12];
end


% scaling factors for inputs
if pt.ATD == 0 && vp.ActAero == 0
    u_s   = [T_motor_s;   T_brake_s;   delta_s];
    u     = [T_motor_n;   T_brake_n;   delta_n];
    u_lim = [T_motor_lim; T_brake_lim; delta_lim];
elseif pt.ATD == 0 && vp.ActAero == 1
    if any(getfielddef(vp,'rwMandate',0) == [6 7])        % ARFWd/ARFWr: FW at nu-2, RW at nu-1
        u_s   = [T_motor_s;   T_brake_s;   activeAeroFW_s;   activeAeroRW_s;   delta_s];
        u     = [T_motor_n;   T_brake_n;   activeAeroFW_n;   activeAeroRW_n;   delta_n];
        u_lim = [T_motor_lim; T_brake_lim; activeAeroFW_lim; activeAeroRW_lim; delta_lim];
    else
        u_s   = [T_motor_s;   T_brake_s;   activeAeroRW_s;   delta_s];
        u     = [T_motor_n;   T_brake_n;   activeAeroRW_n;   delta_n];
        u_lim = [T_motor_lim; T_brake_lim; activeAeroRW_lim; delta_lim];
    end
elseif pt.ATD == 0 && vp.ActAero == 2
    u_s   = [T_motor_s;   T_brake_s;   activeAeroFW_s;   activeAeroRW_s;   delta_s];
    u     = [T_motor_n;   T_brake_n;   activeAeroFW_n;   activeAeroRW_n;   delta_n];
    u_lim = [T_motor_lim; T_brake_lim; activeAeroFW_lim; activeAeroRW_lim; delta_lim];
elseif pt.ATD == 0 && vp.ActAero == 3
    u_s   = [T_motor_s;   T_brake_s;   activeAeroFL_s;   activeAeroFR_s;   activeAeroRW_s;   activeAeroTW_s;   delta_s];
    u     = [T_motor_n;   T_brake_n;   activeAeroFL_n;   activeAeroFR_n;   activeAeroRW_n;   activeAeroTW_n;   delta_n];
    u_lim = [T_motor_lim; T_brake_lim; activeAeroFL_lim; activeAeroFR_lim; activeAeroRW_lim; activeAeroTW_lim; delta_lim];
elseif pt.ATD == 1 && vp.ActAero == 0
    u_s   = [T_motor_s;   T_brake_s;   ATD_FL_s;   ATD_FR_s;   ATD_RL_s;   ATD_RR_s;   delta_s];
    u     = [T_motor_n;   T_brake_n;   ATD_FL_n;   ATD_FR_n;   ATD_RL_n;   ATD_RR_n;   delta_n];
    u_lim = [T_motor_lim; T_brake_lim; ATD_FL_lim; ATD_FR_lim; ATD_RL_lim; ATD_RR_lim; delta_lim];
elseif pt.ATD == 1 && vp.ActAero == 1
    if any(getfielddef(vp,'rwMandate',0) == [6 7])        % ARFWd/ARFWr: FW at nu-2, RW at nu-1
        u_s   = [T_motor_s;   T_brake_s;   ATD_FL_s;   ATD_FR_s;   ATD_RL_s;   ATD_RR_s;   activeAeroFW_s;   activeAeroRW_s;   delta_s];
        u     = [T_motor_n;   T_brake_n;   ATD_FL_n;   ATD_FR_n;   ATD_RL_n;   ATD_RR_n;   activeAeroFW_n;   activeAeroRW_n;   delta_n];
        u_lim = [T_motor_lim; T_brake_lim; ATD_FL_lim; ATD_FR_lim; ATD_RL_lim; ATD_RR_lim; activeAeroFW_lim; activeAeroRW_lim; delta_lim];
    else
        u_s   = [T_motor_s;   T_brake_s;   ATD_FL_s;   ATD_FR_s;   ATD_RL_s;   ATD_RR_s;   activeAeroRW_s;   delta_s];
        u     = [T_motor_n;   T_brake_n;   ATD_FL_n;   ATD_FR_n;   ATD_RL_n;   ATD_RR_n;   activeAeroRW_n;   delta_n];
        u_lim = [T_motor_lim; T_brake_lim; ATD_FL_lim; ATD_FR_lim; ATD_RL_lim; ATD_RR_lim; activeAeroRW_lim; delta_lim];
    end
elseif pt.ATD == 1 && vp.ActAero == 2
    u_s   = [T_motor_s;   T_brake_s;   ATD_FL_s;   ATD_FR_s;   ATD_RL_s;   ATD_RR_s;   activeAeroFW_s;   activeAeroRW_s;   delta_s];
    u     = [T_motor_n;   T_brake_n;   ATD_FL_n;   ATD_FR_n;   ATD_RL_n;   ATD_RR_n;   activeAeroFW_n;   activeAeroRW_n;   delta_n];
    u_lim = [T_motor_lim; T_brake_lim; ATD_FL_lim; ATD_FR_lim; ATD_RL_lim; ATD_RR_lim; activeAeroFW_lim; activeAeroRW_lim; delta_lim];
elseif pt.ATD == 1 && vp.ActAero == 3
    u_s   = [T_motor_s;   T_brake_s;   ATD_FL_s;   ATD_FR_s;   ATD_RL_s;   ATD_RR_s;   activeAeroFL_s;     activeAeroFR_s;     activeAeroRW_s;     activeAeroTW_s;     delta_s];
    u     = [T_motor_n;   T_brake_n;   ATD_FL_n;   ATD_FR_n;   ATD_RL_n;   ATD_RR_n;   activeAeroFL_n;     activeAeroFR_n;     activeAeroRW_n;     activeAeroTW_n;     delta_n];
    u_lim = [T_motor_lim; T_brake_lim; ATD_FL_lim; ATD_FR_lim; ATD_RL_lim; ATD_RR_lim; activeAeroFL_lim;   activeAeroFR_lim;   activeAeroRW_lim;   activeAeroTW_lim;   delta_lim];
end

u_min = u_lim(:,1);
u_max = u_lim(:,2);

%Check number of inputs
if nu ~= length(u) 
    error('Number of inputs is not consistent');
end

% %Constraints on the rate of inputs
duk_ub = duk_ub./u_s;
duk_lb = duk_lb./u_s;

%% Vehicle model (A) - additional variables
%Additional decision variables of the NLP other than the states and inputs

ny = 2; % Number of aux variables

% longitudinal load transfer [N]
ltx_n = SX.sym('loadTransferX_n');
ltx_s = vp.m*vp.g*vp.hcg/vp.l;
ltx = ltx_s * ltx_n;

% lateral load transfer [N]
lty_n = SX.sym('loadTransferY_n');
lty_s = vp.m*vp.g*vp.hcg/vp.t;
lty = lty_s * lty_n;

% scaling factors
y_s = [ltx_s; lty_s];

% aux. variables vector (scaled)
y = [ltx_n; lty_n];

% aux. variables limits
ltx_lim = 1/ltx_s * [-vp.m*vp.g vp.m*vp.g];
lty_lim = 1/lty_s * [-vp.m*vp.g vp.m*vp.g];

y_lim = [ltx_lim; lty_lim];

y_min = y_lim(:,1);
y_max = y_lim(:,2);

% Check number of inputs
if ny ~= length(y) 
    error('Number of aux. variables is not consistent');
end

%% Vechicle model (B) - variables
%Symbolic variables that are not decision variables of the NLP - defined as such because their value changes (it is like a variable value parameter)

kappa = SX.sym('kappa'); % kappa > 0 for left turns

pv = [kappa]; % collect variable parameters

%% Vehicle model (B) - equations
%Calculate variables of the system other than the states and inputs

%%-aerodynamic force coefficients

if vp.ActAero == 0
%% Zenvo Aurora Tur — ride-height aero map + discrete rear-wing angle sweep
% CL_f(vx)/CL_r(vx) are the quasi-statically collapsed map curves for the
% selected rear-wing angle (Functions/aeroCollapse.m, evaluated SX-safe below);
% the RW-angle force-equivalent increments were folded into vp.aero.sel back in
% Parameters/vehParams.m, so a single collapsed curve covers every setting.
% Drag: the wing increment vp.dCd_rw (also RW-angle-derived) adds to vp.Cd0 and
% routes through f_dragRW at vp.hw (the f_drag0/f_dragRW split below is unchanged).

[clf_sel, clr_sel] = aeroEvalSX(vp.aero.sel, vx);
vp.Cl_front = clf_sel;
vp.Cl_rear  = clr_sel;

vp.Cl_left  = 0.5*(vp.Cl_front + vp.Cl_rear);          % symmetric car: HALF of total
vp.Cl_right = vp.Cl_left;
vp.Cd       = vp.Cd0 + vp.dCd_rw;           % Cd ~ RH-independent per the aero sheet;
                                            % wing increment routes through f_dragRW at vp.hw
vp.Cs_front = 0;                            % no side-force data
vp.Cs_rear  = 0;


elseif vp.ActAero == 1
%% Zenvo Aurora Tur — ACTIVE rear wing: 2-D (alpha, vx) aero map
% The rear-wing angle of attack alpha = activeAeroRW [deg] is a continuous NLP
% control on [-10, +15]. vp.aeroARW (built once in Parameters/vehParams.m by
% Functions/rwAeroMap2D.m) carries the four NODE collapse fits - one per wing
% angle [-10 0 +10 +15] deg (the +20 deg node was dropped 2026-08-05; the map
% must never be asked to extrapolate past its last node, which is why the
% control bound above came down with it) - plus a fixed cardinal basis L_i in
% alpha only, so the map is separable:
%       CL_f(alpha,vx) = sum_i L_i(alpha)*clf_i(vx)
%       CL_r(alpha,vx) = sum_i L_i(alpha)*clr_i(vx)
%       dCdA(alpha)    = sum_i L_i(alpha)*dCdA_i        [m^2 PRODUCT, alpha only]
% with [clf_i, clr_i] = aeroEvalSX(col{i}, vx) - the SAME local collapsed-curve
% evaluator the static path uses, unchanged, so the speed dependence (ride-height
% aeroelastic collapse, clamp knees, floored tail) is never re-fitted here. At
% alpha = alphaNodes(i) the basis collapses to the i-th unit vector and the model
% reduces to that static setting; alpha = 0 reproduces 'Mid'.
% Sign convention unchanged: NEGATIVE CL = DOWNFORCE. dCdA is the opposite
% polarity in spirit (POSITIVE = more drag) and is a Cd*A product, hence /vp.A.
% NUMERIC TWIN: Functions/rwAeroMapEvalNum.m - KEEP THE FORMULAS IDENTICAL
% (same rule as aeroEvalSX/aeroEvalNum and tyreMF/tyreMFnum).

assert(isfield(vp,'aeroARW') && isstruct(vp.aeroARW) && isfield(vp.aeroARW,'basis'), ...
    ['vehModel:ActiveRW - vp.aeroARW is missing. The ActiveRW configuration needs ' ...
     'the 2-D rear-wing aero map built in Parameters/vehParams.m ' ...
     '(vp.aeroARW = rwAeroMap2D([], vp)).']);
assert(strcmp(vp.aeroARW.basis.kind, 'hermite3tanh'), ...
    ['vehModel:ActiveRW - only the hermite3tanh basis is implemented symbolically ' ...
     '(vp.aeroARW.basis.kind = ''%s''). Rebuild the map with the default basis - ' ...
     'the SX twin must never guess the basis kind.'], vp.aeroARW.basis.kind);

% cardinal basis weights L_i(alpha): tanh-blended piecewise cubics in the local
% variable t = alpha - knots(j), coefficients highest order first. Raw SX
% arithmetic only (tanh + polynomials): no interpolant nodes, no if/else, no MX.
% Cardinal-basis blend weights. SAME FORMULA as Functions/rwBasisWeights.m, which
% is the numeric owner - keep the two in step exactly as aeroEvalSX/aeroEvalNum and
% tyreMF/tyreMFnum are kept. Written out here rather than called because this block
% is raw SX arithmetic and must stay free of cell-array plumbing.
% Switches sit on the INTERIOR knots only: n-2 switches, n-1 segments. The previous
% version hardcoded three switches and read kn_rw(4), which on a four-node map is
% the endpoint, not an interior knot.
kn_rw = vp.aeroARW.basis.knots;
n_rw  = numel(kn_rw);
s_rw  = cell(1, n_rw-2);
for m_rw = 1:(n_rw-2)
    s_rw{m_rw} = 0.5*(1 + tanh((activeAeroRW - kn_rw(m_rw+1))/vp.aeroARW.basis.wb));
end
w_rw      = cell(1, n_rw-1);
w_rw{1}   = 1 - s_rw{1};
acc_rw    = s_rw{1};
for j_rw = 2:(n_rw-2)
    w_rw{j_rw} = acc_rw*(1 - s_rw{j_rw});
    acc_rw     = acc_rw*s_rw{j_rw};
end
w_rw{n_rw-1} = acc_rw;

clf_sel = 0; clr_sel = 0; dCdA_sel = 0;
for i_rw = 1:numel(vp.aeroARW.col)
    L_rw = 0;
    for j_rw = 1:numel(w_rw)
        L_rw = L_rw + w_rw{j_rw}*hornSX(vp.aeroARW.basis.C(j_rw,:,i_rw), activeAeroRW - kn_rw(j_rw));
    end
    [clf_rw, clr_rw] = aeroEvalSX(vp.aeroARW.col{i_rw}, vx);
    % dClFadd/dClRadd are the liftMode='additive' constant lift shifts (see
    % Functions/rwAeroMap2D.m) - exactly zeros(1,n) under the default
    % liftMode='collapse', so this is bit-identical to the pre-existing formula
    % for every standard (non-override) map. No isfield guard here (unlike the
    % numeric twin, rwAeroMapEvalNum.m): vp.aeroARW is always freshly built by
    % Parameters/vehParams.m in this code path, never loaded from an archive.
    clf_sel  = clf_sel  + L_rw*(clf_rw + vp.aeroARW.dClFadd(i_rw));
    clr_sel  = clr_sel  + L_rw*(clr_rw + vp.aeroARW.dClRadd(i_rw));
    dCdA_sel = dCdA_sel + L_rw*vp.aeroARW.dCdA(i_rw);
end
clear kn_rw n_rw s_rw w_rw acc_rw m_rw i_rw j_rw L_rw clf_rw clr_rw

if any(getfielddef(vp,'rwMandate',0) == [6 7])
% ARFWd/ARFWr: FW additive delta layer on top of the RW-map eval. vp.aeroAFW
% (Parameters/vehParams.m mode 6/7, liftMode='additive') is a DELTA layer only -
% its per-node speed curves are never evaluated, the RW map owns the
% ride-height lift. Lift shifts are already /vp.A (map build); the drag
% product joins dCdA_sel and is divided ONCE at vp.Cd below.
% NUMERIC TWIN: Functions/rwAeroMapEvalNum.m FW-layer args - KEEP THE
% FORMULAS IDENTICAL.
assert(isfield(vp,'aeroAFW') && isstruct(vp.aeroAFW) && isfield(vp.aeroAFW,'basis'), ...
    'vehModel:ARFWd - vp.aeroAFW is missing (built in Parameters/vehParams.m, rwMandate==6)');
assert(strcmp(vp.aeroAFW.basis.kind, 'hermite3tanh'), ...
    ['vehModel:ARFWd - only the hermite3tanh basis is implemented symbolically ' ...
     '(vp.aeroAFW.basis.kind = ''%s'')'], vp.aeroAFW.basis.kind);
kn_fw = vp.aeroAFW.basis.knots;
n_fw  = numel(kn_fw);
s_fw  = cell(1, n_fw-2);
for m_fw = 1:(n_fw-2)
    s_fw{m_fw} = 0.5*(1 + tanh((activeAeroFW - kn_fw(m_fw+1))/vp.aeroAFW.basis.wb));
end
w_fw    = cell(1, n_fw-1);
w_fw{1} = 1 - s_fw{1};
acc_fw  = s_fw{1};
for j_fw = 2:(n_fw-2)
    w_fw{j_fw} = acc_fw*(1 - s_fw{j_fw});
    acc_fw     = acc_fw*s_fw{j_fw};
end
w_fw{n_fw-1} = acc_fw;
for i_fw = 1:numel(vp.aeroAFW.col)
    L_fw = 0;
    for j_fw = 1:numel(w_fw)
        L_fw = L_fw + w_fw{j_fw}*hornSX(vp.aeroAFW.basis.C(j_fw,:,i_fw), activeAeroFW - kn_fw(j_fw));
    end
    clf_sel  = clf_sel  + L_fw*vp.aeroAFW.dClFadd(i_fw);
    clr_sel  = clr_sel  + L_fw*vp.aeroAFW.dClRadd(i_fw);
    dCdA_sel = dCdA_sel + L_fw*vp.aeroAFW.dCdA(i_fw);
end
clear kn_fw n_fw s_fw w_fw acc_fw m_fw i_fw j_fw L_fw
end

vp.Cl_front = clf_sel;
vp.Cl_rear  = clr_sel;

vp.Cl_left  = 0.5*(vp.Cl_front + vp.Cl_rear);          % symmetric car: HALF of total
vp.Cl_right = vp.Cl_left;
vp.Cd       = vp.Cd0 + dCdA_sel/vp.A;       % SYMBOLIC (alpha-dependent) - the body/wing
                                            % split below is untouched: f_drag0 keeps
                                            % vp.Cd0 at vp.hcg and f_dragRW = f(vp.Cd -
                                            % vp.Cd0) carries the wing increment at vp.hw
vp.Cs_front = 0;                            % no side-force data
vp.Cs_rear  = 0;

elseif vp.ActAero == 2

% front wing - left
vp.FW_L.Cl_front = aero.FW_L.Cl_front * activeAeroFW;
vp.FW_L.Cl_rear  = aero.FW_L.Cl_rear  * activeAeroFW;
vp.FW_L.Cl_left  = vp.FW_L.Cl_front + vp.FW_L.Cl_rear;

% front wing - right
vp.FW_R.Cl_front = aero.FW_R.Cl_front * activeAeroFW;
vp.FW_R.Cl_rear  = aero.FW_R.Cl_rear  * activeAeroFW;
vp.FW_R.Cl_right = vp.FW_R.Cl_front + vp.FW_R.Cl_rear;

% rear wing - angle of attack
vp.RW.Cl_front = aero.RW.Cl_front(1)*(activeAeroRW.^3) + aero.RW.Cl_front(2)*(activeAeroRW.^2) + aero.RW.Cl_front(3)*activeAeroRW;
vp.RW.Cl_rear  = aero.RW.Cl_rear(1)*(activeAeroRW.^5) + aero.RW.Cl_rear(2)*(activeAeroRW.^4) + aero.RW.Cl_rear(3)*(activeAeroRW.^3) + aero.RW.Cl_rear(4)*(activeAeroRW.^2) + aero.RW.Cl_rear(5)*activeAeroRW;
vp.RW.Cl_left  = 0.5*(vp.RW.Cl_front + vp.RW.Cl_rear);
vp.RW.Cl_right = 0.5*(vp.RW.Cl_front + vp.RW.Cl_rear);
vp.RW.Cd       = aero.RW.Cd(1)*(activeAeroRW.^2) + aero.RW.Cd(2)*(activeAeroRW.^1);

% rear wing - tilt angle
vp.TW.Cl_left   = aero.TW.Cl_left(1)  * (vp.alpha_TW.^2) + aero.TW.Cl_left(2)  * (vp.alpha_TW.^1);
vp.TW.Cl_right  = aero.TW.Cl_right(1)  * (vp.alpha_TW.^2) + aero.TW.Cl_right(2)  * (vp.alpha_TW.^1);
vp.TW.Cl_rear   = vp.TW.Cl_left + vp.TW.Cl_right;
vp.TW.Cs_rear    = aero.TW.Cs_rear  * vp.alpha_TW;

% combined aerodynamic coefficients (P for higher order polynomial fitted functions)
vp.Cl_front = vp.Cl0_front  +  vp.FW_L.Cl_front  +  vp.FW_R.Cl_front +  vp.RW.Cl_front;
vp.Cl_rear  = vp.Cl0_rear   +  vp.FW_L.Cl_rear   +  vp.FW_R.Cl_rear  +  vp.RW.Cl_rear  +  vp.TW.Cl_rear;
vp.Cl_left  = vp.Cl0_left   +  vp.FW_L.Cl_left   +  vp.RW.Cl_left    +  vp.TW.Cl_left;
vp.Cl_right = vp.Cl0_right  +  vp.FW_R.Cl_right  +  vp.RW.Cl_right   +  vp.TW.Cl_right;
vp.Cd       = vp.Cd0        +  vp.RW.Cd;
vp.Cs_front = vp.Cs0_front;
vp.Cs_rear  = vp.Cs0_rear   +  vp.TW.Cs_rear;

elseif vp.ActAero == 3

% front wing - left
vp.FW_L.Cl_front = aero.FW_L.Cl_front * activeAeroFL;
vp.FW_L.Cl_rear  = aero.FW_L.Cl_rear  * activeAeroFL;
vp.FW_L.Cl_left  = vp.FW_L.Cl_front + vp.FW_L.Cl_rear;

% front wing - right
vp.FW_R.Cl_front = aero.FW_R.Cl_front * activeAeroFR;
vp.FW_R.Cl_rear  = aero.FW_R.Cl_rear  * activeAeroFR;
vp.FW_R.Cl_right = vp.FW_R.Cl_front + vp.FW_R.Cl_rear;

% rear wing - angle of attack
vp.RW.Cl_front = aero.RW.Cl_front(1)*(activeAeroRW.^3) + aero.RW.Cl_front(2)*(activeAeroRW.^2) + aero.RW.Cl_front(3)*activeAeroRW;
vp.RW.Cl_rear  = aero.RW.Cl_rear(1)*(activeAeroRW.^5) + aero.RW.Cl_rear(2)*(activeAeroRW.^4) + aero.RW.Cl_rear(3)*(activeAeroRW.^3) + aero.RW.Cl_rear(4)*(activeAeroRW.^2) + aero.RW.Cl_rear(5)*activeAeroRW;
vp.RW.Cl_left   = 0.5*(vp.RW.Cl_front + vp.RW.Cl_rear);
vp.RW.Cl_right  = 0.5*(vp.RW.Cl_front + vp.RW.Cl_rear);
vp.RW.Cd        = aero.RW.Cd(1)*(activeAeroRW.^2) + aero.RW.Cd(2)*(activeAeroRW.^1);

% rear wing - tilt angle
vp.TW.Cl_left   = aero.TW.Cl_left(1)  * (activeAeroTW.^2) + aero.TW.Cl_left(2)  * (activeAeroTW.^1);
vp.TW.Cl_right  = aero.TW.Cl_right(1)  * (activeAeroTW.^2) + aero.TW.Cl_right(2)  * (activeAeroTW.^1);
vp.TW.Cl_rear   = vp.TW.Cl_left + vp.TW.Cl_right;
vp.TW.Cs_rear    = aero.TW.Cs_rear  * activeAeroTW;

% combined aerodynamic coefficients (P for higher order polynomial fitted functions)
vp.Cl_front = vp.Cl0_front  +  vp.FW_L.Cl_front  +  vp.FW_R.Cl_front +  vp.RW.Cl_front;
vp.Cl_rear  = vp.Cl0_rear   +  vp.FW_L.Cl_rear   +  vp.FW_R.Cl_rear  +  vp.RW.Cl_rear  +  vp.TW.Cl_rear;
vp.Cl_left  = vp.Cl0_left   +  vp.FW_L.Cl_left   +  vp.RW.Cl_left    +  vp.TW.Cl_left;
vp.Cl_right = vp.Cl0_right  +  vp.FW_R.Cl_right  +  vp.RW.Cl_right   +  vp.TW.Cl_right;
vp.Cd       = vp.Cd0        +  vp.RW.Cd;
vp.Cs_front = vp.Cs0_front;
vp.Cs_rear  = vp.Cs0_rear   +  vp.TW.Cs_rear;

end

vp.Cl       = vp.Cl_front + vp.Cl_rear;       
vp.Cl_fl    = vp.Cl_front * (vp.Cl_left/vp.Cl);
vp.Cl_fr    = vp.Cl_front * (vp.Cl_right/vp.Cl);
vp.Cl_rl    = vp.Cl_rear * (vp.Cl_left/vp.Cl);
vp.Cl_rr    = vp.Cl_rear * (vp.Cl_right/vp.Cl);

%-aerodynamic forces [N]
% downforce: positive in downwards direction
f_lift_fl = -0.5*vp.rho*vp.Cl_fl*vp.A*vx^2;
f_lift_fr = -0.5*vp.rho*vp.Cl_fr*vp.A*vx^2;
f_lift_rl = -0.5*vp.rho*vp.Cl_rl*vp.A*vx^2;
f_lift_rr = -0.5*vp.rho*vp.Cl_rr*vp.A*vx^2;
f_lift = f_lift_fl + f_lift_fr + f_lift_rl + f_lift_rr; 

%drag: positive along longitudinal axis
f_drag  = 0.5*vp.rho*vp.A*vx^2*vp.Cd;
f_drag0 = 0.5*vp.rho*vp.A*vx^2*vp.Cd0;                      % drag of vehicle body excluding rear wing contribution
f_dragRW  = 0.5*vp.rho*vp.A*vx^2*(vp.Cd - vp.Cd0);

%side force: orginally positive towards the right hand side of vehicle, now positive to left hand side
f_side_fl = -0.5*(0.5*vp.rho*vp.Cs_front*vp.A*vx^2);
f_side_fr = -0.5*(0.5*vp.rho*vp.Cs_front*vp.A*vx^2);
f_side_rl = -0.5*(0.5*vp.rho*vp.Cs_rear*vp.A*vx^2);
f_side_rr = -0.5*(0.5*vp.rho*vp.Cs_rear*vp.A*vx^2);
f_side = f_side_fl + f_side_fr + f_side_rl + f_side_rr; 

%%-fz, vertical tyre forces [N]  |  ltx: [Acceleration +; Brake -]  |  lty: [Left turn +; Right turn -]
fz_fl = vp.Wfl0 + f_lift_fl - ltx/2 - (1-vp.lty_dis)*lty;             
fz_fr = vp.Wfr0 + f_lift_fr - ltx/2 + (1-vp.lty_dis)*lty;                
fz_rl = vp.Wrl0 + f_lift_rl + ltx/2 - vp.lty_dis*lty;
fz_rr = vp.Wrr0 + f_lift_rr + ltx/2 + vp.lty_dis*lty ;

%%- tyre slip angles [rad]
sa_fl = delta - atan((vp.l_f*r+vy)/(vx-r*vp.t/2));
sa_fr = delta - atan((vp.l_f*r+vy)/(vx+r*vp.t/2));
sa_rl = atan((vp.l_r*r-vy)/(vx-r*vp.t/2));
sa_rr = atan((vp.l_r*r-vy)/(vx+r*vp.t/2));

%%- tyre slip ratio
v_fl = sqrt((vy+vp.l_f*r)^2 + (vx-r*vp.t/2)^2);
v_fr = sqrt((vy+vp.l_f*r)^2 + (vx+r*vp.t/2)^2);
v_rl = sqrt((vy-vp.l_r*r)^2 + (vx-r*vp.t/2)^2);
v_rr = sqrt((vy-vp.l_r*r)^2 + (vx+r*vp.t/2)^2);
v_flx = v_fl*cos(sa_fl);
v_frx = v_fr*cos(sa_fr);
v_rlx = v_rl*cos(sa_rl);
v_rrx = v_rr*cos(sa_rr);
sx_fl = (vp.Rw*Om_fl - v_flx)/v_flx;
sx_fr = (vp.Rw*Om_fr - v_frx)/v_frx;
sx_rl = (vp.Rw*Om_rl - v_rlx)/v_rlx;
sx_rr = (vp.Rw*Om_rr - v_rrx)/v_rrx;

%%- tyre forces [N]: per-axle Zenvo Pacejka MF5.2 with combined slip
% (see Parameters/tyreParams_DoNotPublish.m and tyreMF() at the end of this
% file). The load-sensitivity nominal can be shifted per axle - Fz0_shift_f/r
% are 1 in normal runs and become design parameters in MLTP_TyreOptim.m.
Fz0eff_f = vp.tyre_f.Fz0 * vp.Fz0_shift_f;
Fz0eff_r = vp.tyre_r.Fz0 * vp.Fz0_shift_r;

[fx_fl, fy_fl, mux_fl, muy_fl] = tyreMF(vp.tyre_f, Fz0eff_f, fz_fl, sx_fl, sa_fl);
[fx_fr, fy_fr, mux_fr, muy_fr] = tyreMF(vp.tyre_f, Fz0eff_f, fz_fr, sx_fr, sa_fr);
[fx_rl, fy_rl, mux_rl, muy_rl] = tyreMF(vp.tyre_r, Fz0eff_r, fz_rl, sx_rl, sa_rl);
[fx_rr, fy_rr, mux_rr, muy_rr] = tyreMF(vp.tyre_r, Fz0eff_r, fz_rr, sx_rr, sa_rr);

%-rolling resistance [N]: body-force equivalent of My = qsy1*Fz*Rw (QSY2-4 = 0)
f_roll = vp.tyre_f.qsy1*(fz_fl + fz_fr) + vp.tyre_r.qsy1*(fz_rl + fz_rr);

% e-motor speed (rad/s)
Om_motor = (Om_fl + Om_fr)/4*vp.gear + (Om_rl + Om_rr)/4*vp.gear;

%- individual wheel torque

if pt.ATD == 0
    T_fl = 0.5*T_motor*vp.gear*(1-vp.Tdist) + T_brake*vp.brkB;
    T_fr = 0.5*T_motor*vp.gear*(1-vp.Tdist) + T_brake*vp.brkB;
    T_rl = 0.5*T_motor*vp.gear*vp.Tdist + T_brake*(1-vp.brkB);
    T_rr = 0.5*T_motor*vp.gear*vp.Tdist + T_brake*(1-vp.brkB);
elseif pt.ATD == 1
    if getfielddef(vp,'atdBrakeFixed',0)
        % ATD vectors DRIVE torque only; brake torque keeps the fixed vp.brkB
        % split of the AWD model (same per-wheel terms as the branch above, so
        % total brake torque is identical: 2*T_brake either way). This is the
        % audit variant that separates drive-torque vectoring from brake-bias
        % scheduling - see docs/atd-audit.md and Scripts/userOpts.m's ATDBrake.
        T_fl = ATD_FL*T_motor*vp.gear + T_brake*vp.brkB;
        T_fr = ATD_FR*T_motor*vp.gear + T_brake*vp.brkB;
        T_rl = ATD_RL*T_motor*vp.gear + T_brake*(1-vp.brkB);
        T_rr = ATD_RR*T_motor*vp.gear + T_brake*(1-vp.brkB);
    else
        % SHIPPED DEFAULT: the same fraction multiplies drive AND brake, i.e. the
        % optimiser also chooses brake distribution per wheel, per point.
        T_fl = ATD_FL*T_motor*vp.gear + ATD_FL*T_brake*2;
        T_fr = ATD_FR*T_motor*vp.gear + ATD_FR*T_brake*2;
        T_rl = ATD_RL*T_motor*vp.gear + ATD_RL*T_brake*2;
        T_rr = ATD_RR*T_motor*vp.gear + ATD_RR*T_brake*2;
    end
end

% power powertrain
P_motor = T_motor*Om_motor;

% Change of independent variable
sf = (1-n*kappa)/(vx*cos(eps)-vy*sin(eps));

%% Vehicle model (B) - state derivatives
%Define derivatives of the state-space model

% State derivatives equations
dvx    = (fx_rl + fx_rr + (fx_fl + fx_fr)*cos(delta) - (fy_fl + fy_fr)*sin(delta) + vp.m*vy*r - f_drag - f_roll)*sf/vp.m;
dvy    = (fy_rl + fy_rr + (fy_fl + fy_fr)*cos(delta) + (fx_fl + fx_fr)*sin(delta) - vp.m*vx*r + f_side)*sf/vp.m;
dr     = (-(fy_rl + fy_rr)*vp.l_r + (fx_rr - fx_rl)*vp.t/2 + ((fy_fl + fy_fr)*cos(delta) + (fx_fl + fx_fr)*sin(delta))*vp.l_f ...
       + ((fy_fl - fy_fr)*sin(delta))*vp.t/2 + ((fx_fr - fx_fl)*cos(delta))*vp.t/2 - (f_side_rl + f_side_rr)*vp.l_r + (f_side_fl + f_side_fr)*vp.l_f)*sf/vp.I_z;

dn     = (vx*sin(eps) + vy*cos(eps))*sf;
deps   = sf*r-kappa;

dOm_fl = sf * (T_fl - fx_fl*vp.Rw)/vp.Jw;
dOm_fr = sf * (T_fr - fx_fr*vp.Rw)/vp.Jw;
dOm_rl = sf * (T_rl - fx_rl*vp.Rw)/vp.Jw;
dOm_rr = sf * (T_rr - fx_rr*vp.Rw)/vp.Jw;

% State derivatives vector (scaled to match the scaled states vector)
dx = [dvx; dvy; dr; dn; deps; dOm_fl; dOm_fr; dOm_rl; dOm_rr]./x_s;

%% -------------------------------------------------------------------------
% Local functions (legal at the end of a script since R2016b)
function [clf, clr] = aeroEvalSX(c, vx)
% Collapsed aero map curves: tanh-blended piecewise polynomials in vx^2/qs,
% 3 segments per axle with boundaries at the clamp-onset speeds vc1/vc2
% (segment 3 = both axles floored = constant). SX-safe: scalar arithmetic +
% tanh only, no interpolant nodes.
% NUMERIC TWIN: Functions/aeroEvalNum.m - KEEP THE FORMULAS IDENTICAL.
qn = vx^2/c.qs;
s1 = 0.5*(1 + tanh((vx - c.vc1)/c.wb));
s2 = 0.5*(1 + tanh((vx - c.vc2)/c.wb));
clf = (1-s1)*hornSX(c.pf1, qn) + s1*(1-s2)*hornSX(c.pf2, qn) + s1*s2*c.pf3;
clr = (1-s1)*hornSX(c.pr1, qn) + s1*(1-s2)*hornSX(c.pr2, qn) + s1*s2*c.pr3;
end

function y = hornSX(p, x)
% Horner evaluation, highest-order coefficient first (unrolls into SX graph)
y = p(1);
for k = 2:numel(p)
    y = y*x + p(k);
end
end

function [fx, fy, mux, muy] = tyreMF(ty, Fz0eff, fz, sx, sa)
% Pacejka MF5.2 per-axle tyre model: pure-slip sine curves weighted by the
% combined-slip cosine functions Gxa/Gyk (Pacejka 3rd ed., ch. 4.3). All MF
% shift and vertical-shift parameters are zero in the supplier data, so the
% weighting denominators G(SH)=1 and the vertical shift SVyk=0 drop out.
% Camber terms (pDx3/pDy3) are dormant: the 7DoF model has no camber DOF.
% ty = vp.tyre_f or vp.tyre_r; Fz0eff = (possibly shifted) nominal load.
% SX-safe: sin/cos/atan compositions only, smooth everywhere incl. zero slip.
% NUMERIC TWIN: Functions/tyreMFnum.m - KEEP THE FORMULAS IDENTICAL.
dfz = (fz - Fz0eff)/Fz0eff;
mux = ty.pDx1 + ty.pDx2*dfz;
muy = ty.pDy1 + ty.pDy2*dfz;
% pure slip
fx0 = mux*fz*sin(ty.Cx*atan(ty.Bx*sx - ty.Ex*(ty.Bx*sx - atan(ty.Bx*sx))));
fy0 = muy*fz*sin(ty.Cy*atan(ty.By*sa - ty.Ey*(ty.By*sa - atan(ty.By*sa))));
% combined-slip weighting: Gxa knocks Fx down with slip angle, Gyk knocks Fy
% down with slip ratio
Bxa = ty.rBx1*cos(atan(ty.rBx2*sx));
Gxa = cos(ty.rCx1*atan(Bxa*sa - ty.rEx1*(Bxa*sa - atan(Bxa*sa))));
Byk = ty.rBy1*cos(atan(ty.rBy2*sa));
Gyk = cos(ty.rCy1*atan(Byk*sx - ty.rEy1*(Byk*sx - atan(Byk*sx))));
fx = Gxa*fx0;
fy = Gyk*fy0;
end
