% Minimum lap time problem solver
%
% Solve the basic version of the Minimum Lap Time Problem, i.e. find the optimal race line and control inputs to minimise the lap time.


%% Initialisation
% Clear Simulink SDI callback if it exists
global figures
try Simulink.sdi.unregisterCursorCallback(figures.callbackID); end 

% Clear workspace
clc
clear; clear global;

% Import casadi framework (needs to be on MATLAB path, otherwise the complete route of the folder must be specified)
import casadi.*
assert(exist('casadi.SX','class')==8, ...
    'CasADi is not on the MATLAB path. addpath your casadi-3.x-windows64-matlabXXXX folder first.');

%% Repo path bootstrap - locate the repo from this file, not from the cwd
% Every run()/load()/importfile() below is repo-root-relative and the helper
% functions live in Functions\, so both must be pinned before anything else runs.
repoRoot = fileparts(fileparts(mfilename('fullpath')));    % ...\Scripts -> repo root
addpath(repoRoot, fullfile(repoRoot,'Scripts'), fullfile(repoRoot,'Parameters'));
addpath(genpath(fullfile(repoRoot,'Functions')));          % genpath: PolyfitnTools is double-nested
cd(repoRoot);                                              % repo convention: cwd == repo root
clear repoRoot

% Options
run('userOpts.m');

%% Initialisation
% Warm start from the newest cached MLTP_initial result for THIS circuit and
% aero setting; if none exists, solve the simplified case now (MLTP_initial
% caches it on the way out) so a new track self-heals on first run.
% nu is not defined yet (vehModel.m runs below), so derive it from the config
% ladder: without it latestInit cannot tell an AWD cache from an ATD one for
% the same aeroSetting, and an ATD run would warm-start from an AWD lap.
initFile = latestInit(circuit, vp.aeroSetting, nuForConfig(pt.ATD, vp.ActAero, getfielddef(vp,'rwMandate',0)));
if isempty(initFile)
    fprintf('MLTP: no cached init for %s / %s - running MLTP_initial.m\n', circuit, vp.aeroSetting);
    run('MLTP_initial.m');                  % populates data.init.* AND caches a .mat
else
    fprintf('MLTP: warm start from %s\n', initFile);
    importfile(initFile);                   % the .mat holds `data`, which already has .init
end
clear initFile

%% Start time counter
elapsedTime = 0; tic 

%% Vehicle model
run('vehModel.m');

%% Optimal Control Problem - Dynamics and objetive function

% Objective function
L = sf;

% Continuous time dynamics and objective function
f_dyn = Function('f_dyn',{x,u,y,pv},{dx,L},{'x','u','y','pv'},{'dx','L'});

% Change of variable
f_sf = Function('sf',{x,kappa},{sf},{'x','kappa'},{'sf'});

%% Optimal Control Problem - Path Constraints

% Grip limits live in the combined-slip Pacejka forces themselves (tyreMF in
% vehModel.m) - the old isotropic friction-circle constraints are gone: they
% would wrongly clip legitimate combined states, since mux > muy for the
% Zenvo tyre data.
if pt.ATD == 0
    nh = 9; %number of constraints
elseif pt.ATD == 1
    nh = 10; %number of constraints
end

%-brake and throttle overlap
BrTh_1 = (T_motor_n*T_brake_n)/1e-3;


%-longitudinal load transfer
ltx_eq = ((fx_fl*cos(delta) + fx_fr*cos(delta) - fy_fl*sin(delta) - fy_fr*sin(delta) + fx_rl + fx_rr + f_drag0)*(vp.hcg/vp.l) + (f_dragRW)*(vp.hw/vp.l) - ltx)/1e-3;

%-lateral load transfer
lty_eq = (((fy_fl*cos(delta) + fy_fr*cos(delta) + fx_fl*sin(delta) + fx_fr*sin (delta) + fy_rl + fy_rr)*(vp.hcg/vp.t) + f_side*(vp.hw/vp.t))- lty)/1e-3;

%-Motor power
motor_power = (pt.Pmax - Om_motor*T_motor) / pt.Pmax;

%-Motor max RPM
motor_rpm = (pt.OMmax - Om_motor)/pt.OMmax;


%-per-tyre vertical load limit (smooth-ground working limit; kerb margin to 12.5 kN)
fz_lim_fl = (vp.Fz_lim - fz_fl)/vp.Fz_lim;   % ∈ [0,1] enforces 0 <= fz <= Fz_lim
fz_lim_fr = (vp.Fz_lim - fz_fr)/vp.Fz_lim;
fz_lim_rl = (vp.Fz_lim - fz_rl)/vp.Fz_lim;
fz_lim_rr = (vp.Fz_lim - fz_rr)/vp.Fz_lim;

if pt.ATD == 0
    hnames = {'motor_power','motor_rpm', ...
              'ltx_eq','lty_eq','BrTh_1','fz_lim_fl','fz_lim_fr','fz_lim_rl','fz_lim_rr'};
    h    = [motor_power; motor_rpm; ...
            ltx_eq; lty_eq; BrTh_1; fz_lim_fl; fz_lim_fr; fz_lim_rl; fz_lim_rr];
    h_lb = [0; 0; -1; -1; -1;  0; 0; 0; 0];
    h_ub = [1; 1;  1;  1;  1;  1; 1; 1; 1];
elseif pt.ATD == 1
    ATD_eq = (1 - (ATD_FL + ATD_FR + ATD_RL + ATD_RR));
    hnames = {'motor_power','motor_rpm', ...
              'ltx_eq','lty_eq','BrTh_1','ATD_eq','fz_lim_fl','fz_lim_fr','fz_lim_rl','fz_lim_rr'};
    h    = [motor_power; motor_rpm; ...
            ltx_eq; lty_eq; BrTh_1; ATD_eq; fz_lim_fl; fz_lim_fr; fz_lim_rl; fz_lim_rr];
    h_lb = [0; 0; -1; -1; -1; 0-1e-3;  0; 0; 0; 0];
    h_ub = [1; 1;  1;  1;  1; 0+1e-3;  1; 1; 1; 1];
end



%-Check number of constraints
if nh ~= length(h) 
    error('Number of path constraints of the OCP is not consistent');
end

% Path constraints
h_eq = Function('h_eq', {x, u, y, pv}, {h}, {'x','u','y','pv'}, {'h'}); 

%% NLP - Collocation, polynomial coefficients

% Collocation points in normalised interval [0,1]
tau = collocation_points(OPT_d, 'legendre');

% Collocation linear maps
[C,D,B] = collocation_coeff(tau); %see >>help collocation_coeff (Casadi documentation)

%% NLP - Collocation, discretisation
%Create a discrete set of points for collocation

%-number of grid intervals (the number of points, Xk, is N+1)
N = round(track.s(end)/OPT_ds); 

%-value of the independent variable at the discretisation points. 
s_knot = linspace(min(track.s),max(track.s),N+1); 

%-length of the discretisation interval
dsk = diff(s_knot); % Note that the size of s_knot is N+1 whereas for dsk it is N

%-value of the independent variable at the collocation points
s_col = kron(dsk,tau)+kron([0 cumsum(dsk(1:end-1))],ones(1,OPT_d)); %value of s at the collocation points (in between grid points)

%-full array of points including knot (grid) points and collocation points
s_full = kron(s_knot(1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(1,N); reshape(s_col,OPT_d,N)],1,[]); %collect all values of the independent variable at the grid points and collocation points
s_full(end+1) = s_knot(end);

% Value of varying parameters (in this case only the curvature)
k_knot = interp1(track.s, track.k, s_knot); 
k_col = interp1(track.s, track.k, s_col);
k_full = interp1(track.s, track.k, s_full);

%-collect values of variable parameters; dimensions: nv by N
pv_knot = [k_knot(:)']; 
pv_col  = [k_col(:)'];
pv_full = [k_full(:)'];

%% NLP - initial guesses

%-States (interpolated from initialisation data)
vx_0    = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(1,:))), data.init.x_opt(1,:), linspace(min(track.s),max(track.s),N+1));
vy_0    = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(2,:))), data.init.x_opt(2,:), linspace(min(track.s),max(track.s),N+1));
r_0     = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(3,:))), data.init.x_opt(3,:), linspace(min(track.s),max(track.s),N+1));
n_0     = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(4,:))), data.init.x_opt(4,:), linspace(min(track.s),max(track.s),N+1));
eps_0   = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(5,:))), data.init.x_opt(5,:), linspace(min(track.s),max(track.s),N+1));

Om_fl_0 = vx_0/vp.Rw;
Om_fr_0 = vx_0/vp.Rw; 
Om_rl_0 = vx_0/vp.Rw;
Om_rr_0 = vx_0/vp.Rw;

%-Inputs (interpolated from initialisation data)
% The init source is EITHER the simplified MLTP_initial model (always nu=3,
% u = [T_motor; T_brake; delta]) OR a cached full solution of THIS config (nu rows,
% same ladder). delta is the LAST row in every ladder, so u_opt(end,:) is the only
% index that is correct for both - u_opt(3,:) would pick up the wing angle from a
% warm-started ActiveRW solution (nu=4: row 3 = activeAeroRW) and seed it as steer.
assert(any(size(data.init.u_opt,1) == [3 nu]), ...
    ['MLTP: init u_opt has %d rows - expected 3 (MLTP_initial simplified model) or ' ...
     'nu = %d (a cached full solution of this config). Delete/regenerate the init cache.'], ...
    size(data.init.u_opt,1), nu);

T_motor_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(1,:))), data.init.u_opt(1,:), linspace(min(track.s),max(track.s),N+1));
T_brake_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(2,:))), data.init.u_opt(2,:), linspace(min(track.s),max(track.s),N+1));
delta_0   = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(end,:))), data.init.u_opt(end,:), linspace(min(track.s),max(track.s),N+1));

if vp.ActAero == 1
    if size(data.init.u_opt,1) == nu
        % warm start is a full ActiveRW solution -> reuse its wing trace. The wing is
        % the SECOND-TO-LAST input in both ActiveRW ladders (ATD off, nu=4:
        % [T_motor T_brake activeAeroRW delta]; ATD on, nu=8:
        % [T_motor T_brake ATD_FLx4 activeAeroRW delta]), delta always last.
        activeAeroRW_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(end-1,:))), data.init.u_opt(end-1,:), linspace(min(track.s),max(track.s),N+1));
    else
        activeAeroRW_0 = 0 *ones(1,N+1);        % simplified (nu=3) init: flat 0 deg wing
    end
    % Clamp the seeded wing trace into the CURRENT bound, and say so. A cached
    % solution carries the wing angle its own model was bounded by, which is not
    % necessarily this one: the five-node era allowed +20 and the bound is now +15
    % (vehModel.m, 2026-08-05). Handing IPOPT a guess outside its own variable
    % bounds is not an error - it quietly pushes it inside and solves on - so
    % without this the only trace of a mismatched warm start is a lap that started
    % somewhere it could never have gone. The init-cache model token
    % (Functions/latestInit.m) makes TODAY's stale caches inert; this covers the
    % general case, including Data/*/results warm starts and the next time a bound
    % or node list moves. Bound is in deg here: activeAeroRW_lim is normalised.
    rwLimDeg = activeAeroRW_lim * activeAeroRW_s;
    nOut = sum(activeAeroRW_0 < rwLimDeg(1) - 1e-9 | activeAeroRW_0 > rwLimDeg(2) + 1e-9);
    if nOut > 0
        warning('MLTP:rwInitOutOfBounds', ...
            ['warm-start wing angle is outside the current control bound at %d of %d knots ' ...
             '(seed range [%+.3f, %+.3f] deg vs bound [%+g, %+g] deg) - clamping the initial ' ...
             'guess. The cached solution was produced under a DIFFERENT rear-wing bound or ' ...
             'node list; the resulting lap is not comparable with it. Regenerate the init ' ...
             'cache for this model if you care about the comparison.'], ...
            nOut, numel(activeAeroRW_0), min(activeAeroRW_0), max(activeAeroRW_0), ...
            rwLimDeg(1), rwLimDeg(2));
    end
    activeAeroRW_0 = min(max(activeAeroRW_0, rwLimDeg(1)), rwLimDeg(2));
    clear rwLimDeg nOut

    if any(vp.rwMandate == [6 7])
        % ARFWd/ARFWr: FW flap guess. Row nu-2 in a cached mode-6/7 solution
        % ([...; activeAeroFW; activeAeroRW; delta]) - RW path above unchanged.
        if size(data.init.u_opt,1) == nu
            activeAeroFW_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(end-2,:))), data.init.u_opt(end-2,:), linspace(min(track.s),max(track.s),N+1));
        else
            activeAeroFW_0 = 0 *ones(1,N+1);        % simplified (nu=3) init: flat 0 deg flap
        end
        % Same clamp rationale as the RW block above (bound in deg; *_lim is normalised).
        fwLimDeg = activeAeroFW_lim * activeAeroFW_s;
        activeAeroFW_0 = min(max(activeAeroFW_0, fwLimDeg(1)), fwLimDeg(2));
        clear fwLimDeg
    end
elseif vp.ActAero == 2
    activeAeroFW_0 = 0 *ones(1,N+1);
    activeAeroRW_0 = 0 *ones(1,N+1);
elseif vp.ActAero == 3
    activeAeroFL_0 = 0 *ones(1,N+1);
    activeAeroFR_0 = 0 *ones(1,N+1);
    activeAeroRW_0 = 0 *ones(1,N+1);
    activeAeroTW_0 = 0 *ones(1,N+1);                                                      
end

if pt.ATD == 1
ATD_FL_0  = 0.25*ones(1,N+1);
ATD_FR_0  = 0.25*ones(1,N+1);
ATD_RL_0  = 0.25*ones(1,N+1);
ATD_RR_0  = 0.25*ones(1,N+1);
end

%-Aux variables. The init source decides how lty is obtained: a cached full
% MLTP solution carries ny=2 (ltx,lty), whereas MLTP_initial's simplified
% model carries ny=1 (ltx only) and lty must be rebuilt from its tyre forces.
sInit = linspace(min(track.s), max(track.s), size(data.init.y_opt,2));
sGrid = linspace(min(track.s), max(track.s), N+1);
ltx_0 = interp1(sInit, data.init.y_opt(1,:), sGrid);
if size(data.init.y_opt,1) >= 2
    lty_0 = interp1(sInit, data.init.y_opt(2,:), sGrid);
else
    % u_opt(3,:) is delta HERE and only here: this branch is reached only when the
    % init carries ny==1, which only the simplified MLTP_initial model does, and that
    % model is always nu=3 with delta in row 3 (a cached full solution always has ny=2).
    lty_init = (reshape(0.85*data.init.vehicle.fy_f,1,[]).*sin(reshape(data.init.u_opt(3,:),1,[])) ...
                + reshape(0.85*data.init.vehicle.fy_r,1,[]))*vp.hcg/vp.t;
    lty_0 = interp1(sInit, lty_init, sGrid);
end
clear sInit sGrid lty_init

% Collect initial guesses
x0 = [vx_0; vy_0; r_0; n_0; eps_0; Om_fl_0; Om_fr_0; Om_rl_0; Om_rr_0]./x_s;

if pt.ATD == 0 && vp.ActAero == 0
    u0 = [T_motor_0; T_brake_0; delta_0]./u_s;                                                
elseif pt.ATD == 0 && vp.ActAero == 1
    if any(vp.rwMandate == [6 7])
        u0 = [T_motor_0; T_brake_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s;   % ARFWd/ARFWr: FW at nu-2
    else
        u0 = [T_motor_0; T_brake_0; activeAeroRW_0; delta_0]./u_s;
    end
elseif pt.ATD == 0 && vp.ActAero == 2
    u0 = [T_motor_0; T_brake_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s; 
elseif pt.ATD == 0 && vp.ActAero == 3
    u0 = [T_motor_0; T_brake_0; activeAeroFL_0; activeAeroFR_0; activeAeroRW_0; activeAeroTW_0; delta_0]./u_s; 
elseif pt.ATD == 1 && vp.ActAero == 0
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; delta_0]./u_s;                                                
elseif pt.ATD == 1 && vp.ActAero == 1
    if any(vp.rwMandate == [6 7])
        u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s;   % ARFWd/ARFWr: FW at nu-2
    else
        u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroRW_0; delta_0]./u_s;
    end
elseif pt.ATD == 1 && vp.ActAero == 2
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s; 
elseif pt.ATD == 1 && vp.ActAero == 3
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroFL_0; activeAeroFR_0; activeAeroRW_0; activeAeroTW_0; delta_0]./u_s;     
end

xc0 = reshape(kron(x0(:,1:end-1),ones(1,OPT_d)),OPT_d*N*nx,1); %constant interpolation between x0
y0 = [ltx_0; lty_0]./y_s;


%% NLP - formulation

% Decision variables
Xk = SX.sym('Xk', nx,N+1);                              % States
Uk = SX.sym('Uk', nu,N+1);                              % Inputs
Yk = SX.sym('Yk', ny,N+1);                              % Aux variables
Xkj = SX.sym('Xkj', nx,N*OPT_d);                        % Helper states for the collocation constraints

% Linear interpolation
duk = diff(Uk')'./repmat(dsk,nu,1);                     %derivative of the input at each interval (du/ds)
dyk = diff(Yk')'./repmat(dsk,ny,1);                     %derivative of the aux. variables at each interval (dy/ds)
dxk = diff(Xk')'./repmat(dsk,nx,1);                     %derivative of the input at each interval (du/ds)

% Second derivatives (for regularisarion and minimising oscillations)
duk2 = diff(duk')'; duk2 = [duk2 duk2(:,end)];
dyk2 = diff(dyk')'; dyk2 = [dyk2 dyk2(:,end)]; 
dxk2 = diff(dxk')'; dxk2 = [dxk2 dxk2(:,end)];

% Scaling of the objective function
J_s = 1;                                                %Note that using 's_full(end)/vx_0' here instead of 1 would make the objective function be close to 1;

%% NLP - formulation, boundary conditions

x0_min = max(x_min, Xi./x_s-OPT_e);
x0_max = min(x_max, Xi./x_s+OPT_e);
xf_min = max(x_min, Xf./x_s-OPT_e);
xf_max = min(x_max, Xf./x_s+OPT_e);

%-Initial state
gb = {Xk(:,1)}; lbg = x0_min; ubg = x0_max; %Note that gb, lbg and ubg are being initialised here

%-Final state
gb = [gb(:); {Xk(:,end)}]; lbg = [lbg; xf_min]; ubg = [ubg; xf_max];

%% NLP - formulation, collocation constraints, path constraints and objective function

% Monitored variables
dt_opt = cell(1,N);

% Loop over discretisation points to create collocation constraints
gck = {};                                           % Collector for collocation constraints
J = 0;                                              % Initialise objective function

for k = 0:N-1

    % Collocation constraints
    Z = [Xk(:,k+1) Xkj(:,OPT_d*k+(1:OPT_d))];       % Concatenate states

    %-Dynamics
    %%-calculate derivatives of the approximating polynomial at the collocation points
    dPi = Z*C; 

    %%-calculate derivatives of the system at the collocation points
    switch OPT_uinter 
        case 'constant'
            [dXkj, Qk] = f_dyn(Xkj(:,OPT_d*k+(1:OPT_d)), Uk(:,k+1), Yk(:,k+1), pv_col(:,OPT_d*k+(1:OPT_d))); 
        case 'linear'
            [dXkj, Qk] = f_dyn(Xkj(:,OPT_d*k+(1:OPT_d)), Uk(:,k+1)+kron(duk(:,k+1),tau), Yk(:,k+1)+kron(dyk(:,k+1),tau), pv_col(:,OPT_d*k+(1:OPT_d)));
        otherwise
            error('Choose "linear" or "constant" for the interpolation method of the inputs');
    end

    %-State of approximating polynomial the end of the colocation interval
    Xk_end = Z*D;

    gck = [gck(:); {dsk(k+1)*dXkj(:) - dPi(:)}; {Xk_end-Xk(:,k+2)}]; % Add collocation constraints

    % Integrate quadrature function
    J = J   +   Qk*B*dsk(k+1)/J_s  +  sumsqr(ru.*Uk(:,k+1))  +  sumsqr(rdu.*duk(:,k+1))  +  sumsqr(rdu2.*duk2(:,k+1))  +  sumsqr(rdy.*dyk(:,k+1))  +  sumsqr(rdy2.*dyk2(:,k+1));

    % Collect contribution to time of each interval
    dt_opt{k+1} = Qk*B*dsk(k+1); 
end

% Path constraints
ghk = h_eq(Xk, Uk, Yk, pv); ghk = {ghk(:)};

% Rate of inputs constraints
Sfk = f_sf(Xk,k_knot); % calculate Sf to 'undo' the change of independent variable
duk_t = duk./repmat(Sfk(1:end-1),nu,1); % calculate time derivatives: du/dt = du/ds * 1/sf = du/ds * ds/dt
gduk = {duk_t(:)};

%% NLP - ActiveRW velocity-schedule study (docs/plans/2026-07-28-discrete-rw-mandate.md)
% ONE OPTIONAL add-on for vp.ActAero == 1, OFF by default (vp.rwMandate == 0, set in
% userOpts.m):
%
%   vp.rwMandate == 3 the VELOCITY SCHEDULE + brake trigger: alpha is held inside a
%                     two-sided band around a speed-driven target at every knot,
%                     wired as a SEPARATE CONSTRAINT BLOCK on the assembled control
%                     matrix Uk - exactly the pattern the input-rate block above
%                     already uses (duk differences adjacent columns of Uk) - and
%                     deliberately NOT as a row of `h`. Two independent reasons:
%                     (i) h_eq(Xk,Uk,Yk,pv) is evaluated per knot from THAT knot's
%                     own columns, so it structurally cannot see the neighbouring
%                     knots the band's window below depends on; (ii) an `h` row
%                     would move nh (9 ATD-off / 10 ATD-on) and force every branch
%                     of the nu/nh config ladder to change - this codebase's classic
%                     breakage. nh is UNCHANGED by this feature.
%
% RETIRED 2026-08-04, in this file and in userOpts.m: the brake-torque MANDATE
% (vp.rwMandate 1 = Braking, 2 = ForceOpt) with its slew-aware look-ahead, and the
% multi-well SNAP PENALTY on the objective. vp.rwMandate is now only ever compared
% against 3. vp.rwSnap survives as a userOpts field but is ALWAYS 0 - RWDiscrete =
% 'Snap' is rejected there rather than accepted and ignored, precisely so that no run
% can quietly ask for a penalty nothing applies. The `|| vp.rwSnap == 1` disjunct
% below is therefore dead, and is kept only to hold that field-existence invariant
% visible at its one remaining use.
%
% Inertness: when the schedule is off, no row is appended to g and no term is added
% to J, so numel(w) and numel(g) are identical to the pre-feature NLP (measured:
% ActiveRW ATD-off BCN nW = 19545, nG = 22812 before and after).
gmnd = {}; mnd_lb = []; mnd_ub = [];
if vp.ActAero == 1 && (vp.rwMandate == 3 || vp.rwMandate == 4 || vp.rwMandate == 5 || vp.rwMandate == 6 || vp.rwMandate == 7 || vp.rwSnap == 1)
    % Wing row LOCATED BY SYMBOL NAME, never hardcoded - it is row 3 in the ATD-off
    % ladder and row 7 with ATD on (same rule and same idiom as the ActiveRW
    % post-processing further down; see CLAUDE.md's config-matrix section).
    iRW = find(arrayfun(@(kk) strcmp(erase(name(u(kk)),'_n'),'activeAeroRW'), 1:nu), 1);
    assert(~isempty(iRW), 'MLTP: activeAeroRW not found in u');

    % Uk is in NORMALISED units; rwVelocityTarget below is written in DEGREES / Nm,
    % so undo the scaling on the way in and re-apply it on the way out.
    alpha_k     = Uk(iRW,:) * u_s(iRW);          % rear-wing angle at every knot   (deg)

    % vx is state row 1 (CLAUDE.md: [vx vy r n eps Om_fl..Om_rr]) and Xk is normalised,
    % so undo x_s on the way in exactly as alpha_k undoes u_s above. Shared by modes 3
    % (two-sided band) and 4 (one-sided floor) - both feed vx/Tbrake into
    % rwVelocityTarget, so this is built once here rather than once per mode.
    vx_k = Xk(1,:) * x_s(1);                                    % (m/s) speed at every knot
    % Row 2 is T_brake in EVERY ladder (CLAUDE.md: T_motor/T_brake are the first two
    % inputs everywhere) and it is SIGNED, in [-vp.Tbrake_max, 0]. Pass the SIGNED
    % value: rwVelocityTarget's brakeSwitch applies tbSign = -1 itself, deliberately,
    % so the magnitude stays LINEAR in the decision variable - abs() would hand IPOPT
    % a kinked constraint Jacobian at exactly the T_brake = 0 bound, where most of the
    % lap sits. Do NOT pre-abs() it. Full argument: rwVelocityTarget.m :59-65, :233-236.
    Tb_k = Uk(2,:) * u_s(2);                                    % (Nm, signed)

    if any(vp.rwMandate == [6 7])
        % ARFWd/ARFWr second surface: FW flap row, same by-name rule as iRW
        % above (row nu-2: [...; activeAeroFW; activeAeroRW; delta], delta last).
        iFW = find(arrayfun(@(kk) strcmp(erase(name(u(kk)),'_n'),'activeAeroFW'), 1:nu), 1);
        assert(~isempty(iFW), 'MLTP: activeAeroFW not found in u');
        alphaFW_k = Uk(iFW,:) * u_s(iFW);                       % FW flap angle at every knot (deg)
    end
end

% ---- velocity schedule + brake trigger: TWO-SIDED block (vp.rwMandate == 3) ---
% The production speed-lookup wing: alpha is PINNED to a schedule of vehicle speed
% (low drag on the straights, high downforce at corner speeds) with a brake-torque
% trigger raising it to the single +15 deg airbrake setting. The band is TWO-SIDED -
% it is a ceiling as well as a floor, so the schedule takes the control away in both
% directions. That is the point: the question this variant answers is "what does lap
% time cost if the wing is scheduled rather than free", and a one-sided floor would
% leave the optimiser free to out-perform the law wherever more angle happens to pay.
if vp.ActAero == 1 && vp.rwMandate == 3
    [tVel, tvOnly, sOnV] = rwVelocityTarget(vx_k, Tb_k, vp.Tbrake_max, vp.rwVelOpts);

    % --- why the band has to OPEN where the schedule moves ---------------------
    % A two-sided |alpha - target| <= band is INFEASIBLE at a stepping target, and a
    % slew-aware reachability look-ahead does NOT fix it. Such a look-ahead (discount
    % every future target by what the wing can still climb, then take the window's
    % smooth max) only works for a ONE-SIDED row: it produces a lower bound that
    % anticipates. Mirror it for an upper bound and the two envelopes cross - both
    % contain the j = k term, so the forward-relaxed floor lands ABOVE target_k while
    % the backward-relaxed ceiling lands BELOW it, and the admissible interval is
    % empty. Verified on paper before writing this, which is why the code below does
    % something else.
    %
    % The physics says what the right answer is: at a step the wing is SLEWING, and a
    % slewing wing is by definition not at its target. So the band must widen by
    % exactly the amount the target moves locally, and stay tight everywhere else:
    %
    %   band_k = band0 + kappa * ( softmax_win(target) - softmin_win(target) )
    %
    % Over a window where the target is flat this collapses to band0 (the schedule
    % binds hard - straights, corner plateaus, established brake zones). Across a
    % transition it opens to span the transition (the wing is allowed to be mid-slew).
    % FEASIBILITY: any slew-limited trajectory crossing the step stays inside
    % [min_win target, max_win target], so kappa >= 1 makes alpha_k = target_k and the
    % whole slewing arc admissible. kappa is set above 1 because the softmax/softmin
    % below are the WEIGHTED-AVERAGE form (a convex combination of the window's own
    % values, NOT log-sum-exp), which UNDERSHOOTS the true max - the margin buys that
    % undershoot back.
    band0 = vp.rwVelBand;                                       % (deg) tight-region half-width
    kappa = vp.rwVelKappa;                                      % (-)   transition opening factor

    % Symmetric window [k-Mb, k+Mb], clamped at both ends (the OCP is not periodic).
    % Mb is vp.rwVelBandM, the BAND's own window - it is deliberately much shorter than
    % the retired mandate's look-ahead window was. Reusing that longer window here is
    % what made the first ARWv solve's law non-binding (median band 12.72 deg).
    Mb   = vp.rwVelBandM;
    tLoC = cell(1, N+1); tHiC = cell(1, N+1);
    for kk = 1:N+1
        idx = max(kk - Mb, 1) : min(kk + Mb, N+1);
        tw  = tVel(idx);                                        % 1 x m
        % Weighted-average soft extrema: convex combinations of the window's own
        % values, so they can never exceed the true max nor fall below the true min.
        wHi = exp( vp.rwVelBeta * tw);
        wLo = exp(-vp.rwVelBeta * tw);
        m   = numel(idx);
        tHiC{kk} = (wHi*tw.') / (wHi*ones(m,1));
        tLoC{kk} = (wLo*tw.') / (wLo*ones(m,1));
    end
    tHiWin = [tHiC{:}];  tLoWin = [tLoC{:}];                    % 1 x (N+1)
    clear tLoC tHiC idx tw wHi wLo m kk

    % The band window must span the worst slew, or the wing is asked to be inside a
    % tight band while it is still physically travelling. Worst step is the full
    % schedule range; the wing needs that/c.ub.RW seconds, i.e. that/c.ub.RW*vx metres.
    schedRange = vp.rwVelOpts.aBrake - vp.rwVelOpts.aStraight;          % (deg)
    slewMetres = schedRange/c.ub.RW * max(vi, 1);                       % (m) at the entry speed
    assert(Mb*min(dsk) >= 0.5*slewMetres, 'MLTP:rwVelBandWindow', ...
        ['RW velocity band window is too short: Mb = %d x %.2f m = %.1f m either side, ' ...
         'but a %.0f deg slew at %g deg/s takes %.1f m at %.1f m/s. Raise ' ...
         'vp.rwVelBandM.'], Mb, min(dsk), Mb*min(dsk), schedRange, c.ub.RW, slewMetres, max(vi,1));

    bandK = band0 + kappa * (tHiWin - tLoWin);                  % 1 x (N+1), >= band0

    % --- the constraint rows --------------------------------------------------
    %   alpha_k - target_k + band_k >= 0        (lower edge)
    %   target_k + band_k - alpha_k >= 0        (upper edge)
    % TWO rows per knot -> 2(N+1). Scaled by the wing scale u_s(iRW) so each row is
    % O(1) like the h rows. nh is untouched: this is a separate block on the assembled
    % Uk/Xk, not a row of h.
    gmndVec = [ (alpha_k - tVel + bandK).' ; ...
                (tVel + bandK - alpha_k).' ] / u_s(iRW);        % 2(N+1) x 1
    gmnd    = {gmndVec};
    mnd_lb  = zeros(2*(N+1),1);
    mnd_ub  = inf(2*(N+1),1);

    % Same expressions the NLP enforces are what post-processing evaluates, so the
    % reported schedule cannot drift from the imposed one (standing repo rule).
    f_rwVel = Function('f_rwVel', {Xk, Uk}, {gmndVec, tVel, tvOnly, sOnV, bandK}, ...
        {'Xk','Uk'}, {'gv','target','tv','sOn','band'});

    % Feasibility of the SCHEDULE ITSELF against the actuator, asserted rather than
    % trusted: max|d(tv)/dt| = |aStraight-aCorner|*|ax|/(2w). ax is not known before
    % the solve, so this checks the bound the schedule can demand per unit of
    % acceleration and reports it - the solved trace is re-checked in the gate.
    % Worst case over BOTH switches of the three-level staircase.
    axRef     = 25.2;                                           % (m/s^2) solved |ax|max, BCN/AWD
    dtvdv1    = abs(vp.rwVelOpts.aMid    - vp.rwVelOpts.aCorner) / (2*vp.rwVelOpts.w1);
    dtvdv2    = abs(vp.rwVelOpts.aStraight - vp.rwVelOpts.aMid)  / (2*vp.rwVelOpts.w2);
    dtvdv_max = max(dtvdv1, dtvdv2);
    fprintf(['MLTP: RW VELOCITY schedule ON (vp.rwMandate=3, %s) - %d extra g rows ' ...
             '(two-sided), %+.0f corner / %+.0f mid / %+.0f straight / %+.0f brake deg, ' ...
             'v1=%.1f w1=%.1f | v2=%.1f w2=%.1f m/s, band0=%.2f deg kappa=%.2f, Mb=%d beta=%g\n'], ...
        vp.aeroSetting, 2*(N+1), ...
        vp.rwVelOpts.aCorner, vp.rwVelOpts.aMid, vp.rwVelOpts.aStraight, vp.rwVelOpts.aBrake, ...
        vp.rwVelOpts.v1, vp.rwVelOpts.w1, vp.rwVelOpts.v2, vp.rwVelOpts.w2, ...
        band0, kappa, vp.rwVelBandM, vp.rwVelBeta);
    fprintf(['MLTP:   schedule demands %.3f deg/(m/s) => %.1f deg/s at |ax| = %.1f m/s^2, ' ...
             'against c.ub.RW = %g deg/s\n'], dtvdv_max, dtvdv_max*axRef, axRef, c.ub.RW);
    assert(dtvdv_max*axRef <= c.ub.RW, 'MLTP:rwVelSlew', ...
        ['velocity schedule is too sharp for the actuator: it demands %.1f deg/s at ' ...
         '|ax| = %.1f m/s^2 against c.ub.RW = %g deg/s. Widen vp.rwVelOpts.w1/w2.'], ...
        dtvdv_max*axRef, axRef, c.ub.RW);

    % The MID plateau must actually exist: tv only reaches aMid if the two switches
    % are far enough apart. Without this the "three-level" staircase silently
    % degenerates into the two-level slide it was changed away from.
    plateauSep = vp.rwVelOpts.v2 - vp.rwVelOpts.v1;
    plateauReq = 4*max(vp.rwVelOpts.w1, vp.rwVelOpts.w2);
    tvMidCheck = rwVelocityTarget(0.5*(vp.rwVelOpts.v1+vp.rwVelOpts.v2), 0, vp.Tbrake_max, vp.rwVelOpts);
    fprintf('MLTP:   mid plateau: v2-v1 = %.1f m/s vs 4w = %.1f, tv at plateau centre = %+.4f deg\n', ...
        plateauSep, plateauReq, tvMidCheck);
    assert(abs(tvMidCheck - vp.rwVelOpts.aMid) <= 0.25, 'MLTP:rwVelPlateau', ...
        ['velocity staircase never reaches its MID setting: tv at the plateau centre is ' ...
         '%+.3f deg against aMid = %+.3f. v2-v1 = %.1f m/s needs to be >~ 4w = %.1f. ' ...
         'Either separate v1/v2 or narrow w1/w2.'], ...
        tvMidCheck, vp.rwVelOpts.aMid, plateauSep, plateauReq);

elseif vp.ActAero == 1 && vp.rwMandate == 7
    % ---- ARFWr: reactive dual-wing schedule, TWO-SIDED band on BOTH rows ----
    % Same band-opening machinery as mode 3 above (see the long rationale there);
    % target is reactiveWingTarget(vx, r, Tbrake), one call, two surfaces.
    rl   = vp.reactLaw;
    r_k  = Xk(3,:) * x_s(3);                                  % yaw rate (rad/s)
    [tRW_k, tFW_k, sOnR] = reactiveWingTarget(vx_k, r_k, Tb_k, vp.Tbrake_max, rl);

    surf7 = { {alpha_k,   u_s(iRW), tRW_k, rl.bandRW, 25, 'RW'}, ...   % range -10..+15
              {alphaFW_k, u_s(iFW), tFW_k, rl.bandFW, 25, 'FW'} };     % range -25..0
    gmndAll = {}; bandStore = cell(1,2);
    for si = 1:numel(surf7)
        [aTr, aUs, tT, band0s, rngS, lblS] = surf7{si}{:};
        Mb  = rl.bandM;
        tLoC = cell(1, N+1); tHiC = cell(1, N+1);
        for kk = 1:N+1
            idx = max(kk - Mb, 1) : min(kk + Mb, N+1);
            tw  = tT(idx);
            wHi = exp( rl.beta * tw);  wLo = exp(-rl.beta * tw);
            m   = numel(idx);
            tHiC{kk} = (wHi*tw.') / (wHi*ones(m,1));
            tLoC{kk} = (wLo*tw.') / (wLo*ones(m,1));
        end
        tHiWin = [tHiC{:}]; tLoWin = [tLoC{:}];
        slewMetres = rngS/c.ub.RW * max(vi, 1);
        assert(Mb*min(dsk) >= 0.5*slewMetres, 'MLTP:arfwrBandWindow', ...
            ['ARFWr %s band window too short: Mb=%d x %.2f m vs %.0f deg slew ' ...
             '= %.1f m at %.1f m/s. Raise law.bandM (re-fit).'], ...
            lblS, Mb, min(dsk), rngS, slewMetres, max(vi,1));
        bandK = band0s + rl.kappaOpen * (tHiWin - tLoWin);
        gmndAll{end+1} = [ (aTr - tT + bandK).' ; (tT + bandK - aTr).' ] / aUs; %#ok<AGROW>
        bandStore{si} = bandK;
        clear tLoC tHiC idx tw wHi wLo m kk tHiWin tLoWin bandK
    end
    gmndVec = vertcat(gmndAll{:});                             % 4(N+1) x 1
    gmnd    = {gmndVec};
    mnd_lb  = zeros(4*(N+1),1);
    mnd_ub  = inf(4*(N+1),1);
    f_arfwr = Function('f_arfwr', {Xk, Uk}, ...
        {gmndVec, tRW_k, tFW_k, sOnR, bandStore{1}, bandStore{2}}, ...
        {'Xk','Uk'}, {'gv','tRW','tFW','sOn','bandRW','bandFW'});
    fprintf(['MLTP: ARFWr REACTIVE schedule ON (mode 7) - %d rows, k1=%.5f k2=%.5f ' ...
             'w1=%.5f w2=%.5f bandRW=%.2f bandFW=%.2f Mb=%d\n'], 4*(N+1), ...
        rl.k1, rl.k2, rl.w1, rl.w2, rl.bandRW, rl.bandFW, rl.bandM);
    if rl.provisional == 1, fprintf('MLTP:   [PROVISIONAL LAW - re-fit before quoting]\n'); end
    clear surf7 gmndAll bandStore aTr aUs tT band0s rngS lblS si

elseif vp.ActAero == 1 && any(vp.rwMandate == [4 5 6])
    % ARWd (mode 4) / AFWd (mode 5) / ARFWd (mode 6): one-sided lagged
    % airbrake(-analogue) floor. Axis must reach ab_aBrake only after braking
    % persists one slew-time; brake stabs shorter than that are exempt. Mode 4
    % floors the rear wing UP to its +15 deg airbrake station; mode 5 floors the
    % combined FW+RW axis UP to its 0 deg max-downforce end; mode 6 runs BOTH
    % instances (RW to +15, rw* ids; independent FW flap to 0, fw* ids) - same
    % one-sided-lag mechanism and constraint direction (ab_aBrake > ab_alphaMin
    % throughout), one loop pass per floored surface.
    % Surface rows: {alpha trace, u_s scale, alphaMin, aBrake, slew id, label}.
    switch vp.rwMandate
        case 4
            abSurf = { {alpha_k,   u_s(iRW), -10, +15, 'MLTP:rwDiscSlew', 'RW'} };
        case 5
            abSurf = { {alpha_k,   u_s(iRW), -25,   0, 'MLTP:fwDiscSlew', 'FW'} };
        case 6
            abSurf = { {alpha_k,   u_s(iRW), -10, +15, 'MLTP:rwDiscSlew', 'RW'}, ...
                       {alphaFW_k, u_s(iFW), -25,   0, 'MLTP:fwDiscSlew', 'FW'} };
    end
    for si = 1:numel(abSurf)
        [abAlpha_k, ab_us, ab_alphaMin, ab_aBrake, ab_slewId, ab_label] = abSurf{si}{:};
        oF = vp.rwVelOpts; oF.aStraight = ab_alphaMin; oF.aMid = ab_alphaMin; oF.aCorner = ab_alphaMin; oF.aBrake = ab_aBrake;
        [~, ~, sOn_k] = rwVelocityTarget(vx_k, Tb_k, vp.Tbrake_max, oF);

        % The floor must span the worst slew, or the wing is asked to already be at
        % the floor while still physically travelling from ab_alphaMin to ab_aBrake.
        % Same reasoning as mode 3's slewMetres assert, one-sided instead of half.
        % Each surface sizes its OWN window from its own swing (both 25 deg here).
        slewM = (ab_aBrake - ab_alphaMin)/c.ub.RW * max(vi,1);      % (m) at the entry speed
        Mb_ab = ceil(slewM / min(dsk));
        % Guards against the trailing window swallowing too much of the lap (the ARWv
        % non-binding-law failure mode): >10% of the knot count means the floor would
        % lag so far behind sOn that it stops tracking individual brake zones.
        assert(Mb_ab <= ceil(0.10*(N+1)), ab_slewId, ...
            ['airbrake window %d intervals is >10%% of the %d-knot lap - floor would not ' ...
             'bind; slower schedule range or faster actuator needed'], Mb_ab, N+1);

        % Trailing soft-min of sOn over [kk-Mb_ab, kk] - the mode-3 tLoWin
        % weighted-average pattern mirrored TRAILING, not symmetric: a one-sided
        % floor has no upper partner to keep the admissible interval open across a
        % forward step (see mode 3's comments for why a symmetric window would be
        % infeasible here too), so it must only look BACKWARD in s.
        tLoC = cell(1, N+1);
        for kk = 1:N+1
            idx = max(kk - Mb_ab, 1) : kk;
            tw  = sOn_k(idx);                                       % 1 x m
            % Weighted-average soft minimum: convex combination of the window's own
            % values, so it can never fall below the true min.
            wLo = exp(-vp.rwVelBeta * tw);
            m   = numel(idx);
            tLoC{kk} = (wLo*tw.') / (wLo*ones(m,1));
        end
        sOnLag = [tLoC{:}];                                          % 1 x (N+1)
        clear tLoC idx tw wLo m kk

        floorWin_k = ab_alphaMin + sOnLag*(ab_aBrake - ab_alphaMin); % 1 x (N+1), deg

        % ONE row per knot per surface -> (N+1) each: a floor, not a band, so only
        % the lower edge is constrained: alpha_k - floorWin_k >= 0. Scaled by the
        % surface's own u_s row so each row is O(1) like the h rows. nh is untouched:
        % this is a separate block on the assembled Uk/Xk, not a row of h.
        gmndVec = (abAlpha_k - floorWin_k).' / ab_us;                % (N+1) x 1
        gmnd    = [gmnd(:); {gmndVec}];
        mnd_lb  = [mnd_lb; zeros(size(gmndVec))];
        mnd_ub  = [mnd_ub; inf(size(gmndVec))];

        % Same expression the NLP enforces is what post-processing evaluates (Task 4
        % audit), same standing rule as f_rwVel above.
        if vp.rwMandate == 6 && strcmp(ab_label, 'FW')
            f_fwDisc = Function('f_fwDisc', {Xk, Uk}, {gmndVec, floorWin_k, sOn_k});
        else
            f_rwDisc = Function('f_rwDisc', {Xk, Uk}, {gmndVec, floorWin_k, sOn_k});
        end

        if vp.rwMandate == 4
            fprintf(['MLTP: RW DISCRETE airbrake floor ON (vp.rwMandate=4, %s) - %d extra g rows ' ...
                     '(one-sided), floor in [%+.0f, %+.0f] deg, Mb_ab=%d (%.1f m slew span), beta=%g\n'], ...
                vp.aeroSetting, N+1, ab_alphaMin, ab_aBrake, Mb_ab, Mb_ab*min(dsk), vp.rwVelBeta);
        elseif vp.rwMandate == 5
            fprintf(['MLTP: FW DISCRETE airbrake-analogue floor ON (axis->0 under braking) ' ...
                     '(vp.rwMandate=5, %s) - %d extra g rows (one-sided), floor in [%+.0f, %+.0f] deg, ' ...
                     'Mb_ab=%d (%.1f m slew span), beta=%g\n'], ...
                vp.aeroSetting, N+1, ab_alphaMin, ab_aBrake, Mb_ab, Mb_ab*min(dsk), vp.rwVelBeta);
        else
            fprintf(['MLTP: %s COMBINED airbrake(-analogue) floor ON (vp.rwMandate=6, %s) - ' ...
                     '%d extra g rows (one-sided), floor in [%+.0f, %+.0f] deg, Mb_ab=%d ' ...
                     '(%.1f m slew span), beta=%g\n'], ...
                ab_label, vp.aeroSetting, N+1, ab_alphaMin, ab_aBrake, Mb_ab, Mb_ab*min(dsk), vp.rwVelBeta);
        end
    end
    clear abSurf abAlpha_k ab_us si
end

%% NLP - define NLP problem
%-decision variables
w = [Xk(:); Uk(:); Yk(:); Xkj(:)]; % Collect all the decision variables [states; inputs; aux. variables; helper states]
lbw = [repmat(x_min(:),N+1,1); repmat(u_min(:,1),N+1,1); repmat(y_min(:),N+1,1); repmat(x_min(:),N*OPT_d,1)]; % Lower bounds
ubw = [repmat(x_max(:),N+1,1); repmat(u_max(:,1),N+1,1); repmat(y_max(:),N+1,1); repmat(x_max(:),N*OPT_d,1)]; % Upper bounds
w0 = [x0(:); u0(:); y0(:); xc0(:)]; % Collect initial guesses for the decision variables

%-constraints
% gmnd/mnd_lb/mnd_ub are EMPTY unless the ActiveRW velocity schedule is on, so with it
% off these three lines produce byte-for-byte the pre-feature g/lbg/ubg.
g = [gb(:);gck(:);ghk(:);gduk(:);gmnd(:)]; g = vertcat(g{:}); %[boundary conditions; collocation constraints; path constraints; rate of inputs; RW velocity schedule]
lbg = [lbg; zeros((OPT_d+1)*N*nx,1);repmat(h_lb,N+1,1);repmat(duk_lb,N,1); mnd_lb]; % Lower bounds
ubg = [ubg; zeros((OPT_d+1)*N*nx,1);repmat(h_ub,N+1,1);repmat(duk_ub,N,1); mnd_ub]; % Upper bounds

%-nlp problem [objective function 'f', decision variables 'x', constraints 'g']
nlp = struct('f', J, 'x', w, 'g', g); 

% Create solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

elapsedTime(end+1) = toc - elapsedTime(end);

%% NLP - solve 

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

% Capture the IPOPT exit status. Without this the lap time printed at the end
% of this script is identical whether IPOPT returned Solve_Succeeded or
% Restoration_Failed / Maximum_Iterations_Exceeded / Infeasible_Problem_Detected,
% and a non-converged number is indistinguishable from a lap time.
% (attached to `data` further down - `global data` at ~line 366 would discard
%  any value written to `data` before that declaration)
solverStats = solver.stats();
solverStatus = solverStats.return_status;
if isfield(solverStats, 'iter_count'), solverIters = solverStats.iter_count; else, solverIters = NaN; end
if strcmp(solverStatus, 'Solve_Succeeded')
    fprintf('MLTP: IPOPT %s in %d iterations.\n', solverStatus, solverIters);
else
    warning('MLTP:notConverged', ...
        ['IPOPT returned %s after %d iterations - the lap time below is NOT a ' ...
         'converged optimum and must not be quoted.'], solverStatus, solverIters);
end

elapsedTime(end+1) = toc - elapsedTime(end);

%% Postprocessing - Collect NLP variables
%Retrieve decision variables from the solution
global data;

%-Independent variables
data.s_full = s_full;
% data.s_knot = s_knot;
data.k_full = k_full;
% data.k_knot = k_knot;

%-Solver exit status (captured right after the solve, attached here because
% `global data` above discards anything written to `data` before it)
data.solver_status = solverStatus;
data.iter_count    = solverIters;

%-Collect decision variables in numeric array
data.w_opt = full(sol.x);

%-Note scaling is undone here too
data.x_opt = reshape(data.w_opt(1:nx*(N+1)),nx,N+1).*x_s;
data.u_opt = reshape(data.w_opt(nx*(N+1)+(1:nu*(N+1))),nu,N+1).*u_s;
data.y_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+(1:ny*(N+1))),ny,N+1).*y_s;
data.xc_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+ny*(N+1)+1:end),nx,N*OPT_d).*x_s;

%% Postprocessing - Reconstruct solution
% Collect values of x at all interpolation points 
%States, x_full = [X1 X11...X1j...X1d,..., Xk Xk1...Xkj...Xkd,..., XN XN1..XNj...XNd, XN+1]
data.x_full = kron(data.x_opt(:,1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(nx,N); reshape(data.xc_opt,nx*OPT_d,N)],nx,[]);
data.x_full(:,end+1) = data.x_opt(:,end);

%Inputs and Aux. variables
switch OPT_uinter
    case 'constant'
        data.u_full = interp1(s_knot', data.u_opt', s_full, 'previous')'; % for constant inputs
        data.y_full = reshape(interp1(s_knot', data.y_opt', s_full, 'previous')',ny,[]); % for constant inputs
    case 'linear'
        data.u_full = interp1(s_knot, data.u_opt', s_full, 'linear')'; % for linear inputs
        data.y_full = reshape(interp1(s_knot, data.y_opt', s_full, 'linear')',ny,[]); % for linear inputs
    otherwise
        error('Choose "linear" or "constant" for the interpolation method of the inputs');
end

%% Postprocessing - Collect additional data

% Time
%-time at grid points
dt_opt_val = cell(N,1);
for i=0:N-1
    f_t_opt = Function('f_t_opt',{Xkj(:,(i*OPT_d)+(1:OPT_d)), Uk(:,i+1), Yk(:,i+1)},{dt_opt{i+1}});                         % Create a casadi function to evaluate dt_opt
    dt_opt_val{i+1} = f_t_opt(data.xc_opt(:,i*OPT_d+(1:OPT_d))./x_s,data.u_opt(:,i+1)./u_s, data.y_opt(:,i+1)./y_s);        % Evaluate dt_opt at the solution
end
data.t_opt = [0 cumsum(full([dt_opt_val{:}]))];                                                                             % Time at each grid point

% Track
data.track0 = track;                                                                                                        % Track before discretisation
data.track.s = s_full;
data.track.k = k_full;

%%-calculate track coordinates at grid points
if ~isfield(data.track0,'x') || ~isfield(data.track0,'y')
    [data.track.x, data.track.y] = curv2cart(data.track.s, data.track.k);
else
    data.track.x = interp1(data.track0.s, data.track0.x, data.track.s);
    data.track.y = interp1(data.track0.s, data.track0.y, data.track.s);
end
[data.track.xopt,data.track.yopt] = cartPath(data.track.x,data.track.y, data.x_full(4,:));                                  % x_full(4) is the distance to the centerline
[data.track.Xl,data.track.Xr] = trackLimits(data.track.x,data.track.y, x_s(4)*x_max(4)*2+2);

% Vehicle
%-aerodynamic forces
f_aeroF = Function('f_aeroF',{x,u,y,pv}, {[f_drag; f_lift; f_lift_fl; f_lift_fr; f_lift_rl; f_lift_rr; f_side; f_side_fl; f_side_fr; f_side_rl; f_side_rr]},{'x','u','y','pv'},{'Ftyres'});
aeroF = reshape(full(f_aeroF(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),11,1,N+1);
data.vehicle.f_drag = aeroF(1,1,:);
data.vehicle.f_lift = aeroF(2,1,:);
data.vehicle.f_lift_fl = aeroF(3,1,:);
data.vehicle.f_lift_fr = aeroF(4,1,:);
data.vehicle.f_lift_rl = aeroF(5,1,:);
data.vehicle.f_lift_rr = aeroF(6,1,:);
data.vehicle.f_side = aeroF(7,1,:);
data.vehicle.f_side_fl = aeroF(8,1,:);
data.vehicle.f_side_fr = aeroF(9,1,:);
data.vehicle.f_side_rl = aeroF(10,1,:);
data.vehicle.f_side_rr = aeroF(11,1,:);
% data.vehicle.Mx_rc = aeroF(12,1,:);
% data.vehicle.My_fl = aeroF(13,1,:);

%-active rear wing (ActAero == 1): 2-D (alpha, vx) aero map along the solved
% trajectory, on the FULL grid s_full (knot + collocation points), evaluated with
% the NUMERIC TWIN of the SX block in vehModel.m. The wing row is LOCATED BY SYMBOL
% NAME, never hardcoded: it is row 3 in the ATD-off ladder
% (u = [T_motor; T_brake; activeAeroRW; delta]) but row 7 with ATD on
% (u = [T_motor; T_brake; ATD_FLx4; activeAeroRW; delta]). u_opt/u_full already have
% the scaling undone, so that row is the wing angle in DEGREES. Sign conventions match
% vehModel.m: f_lift positive DOWNWARDS, dCdA a Cd*A product [m^2] vs the 0 deg wing.
if vp.ActAero == 1
    % name(u(k)), NOT u(k).name: chained index-then-property on an SX slice throws
    % ("Can only convert 1-by-1 matrices to scalars") in this CasADi MATLAB binding -
    % the functional form dispatches correctly. Verified for both ladders (row 3 / 7).
    iRW = find(arrayfun(@(k) strcmp(erase(name(u(k)),'_n'),'activeAeroRW'), 1:nu), 1);
    assert(~isempty(iRW), 'MLTP: activeAeroRW not found in u');
    alpha_arw = data.u_full(iRW,:);                                     % rear-wing angle   (deg)
    vx_arw    = data.x_full(1,:);                                       % longitudinal speed (m/s)
    %-rwAeroMapEvalNum THROWS more than 1e-6 deg outside [-10,20]; IPOPT only honours
    % its own variable bounds to bound_relax_factor, so a solution can legitimately
    % sit a hair outside. Clamp (and warn loudly) rather than let a throw here discard
    % every derived quantity of a multi-hour solve.
    if any(alpha_arw < vp.aeroARW.alphaMin - 1e-3 | alpha_arw > vp.aeroARW.alphaMax + 1e-3)
        warning('MLTP:arwAlphaOutOfRange', ...
            ['ActiveRW: solved wing angle leaves [%g, %g] deg by more than 1e-3 (min %.6g, max %.6g) - ' ...
             'clamping for post-processing. Check the NLP bounds (activeAeroRW_lim) before trusting the aero trace.'], ...
            vp.aeroARW.alphaMin, vp.aeroARW.alphaMax, min(alpha_arw), max(alpha_arw));
    end
    alpha_arw = min(max(alpha_arw, vp.aeroARW.alphaMin), vp.aeroARW.alphaMax);
    [ClF_arw, ClR_arw, dCdA_arw] = rwAeroMapEvalNum(vp.aeroARW, alpha_arw, vx_arw);
    data.aeroARW.alpha     = alpha_arw;                                 % (deg) AUTHORITATIVE wing trace
    data.aeroARW.uRow      = iRW;                                       % row of u/u_full holding it
    data.aeroARW.vx        = vx_arw;                                    % (m/s)
    data.aeroARW.Cl_front  = ClF_arw;                                   % (-) NEGATIVE = downforce
    data.aeroARW.Cl_rear   = ClR_arw;                                   % (-)
    data.aeroARW.dCdA      = dCdA_arw;                                  % (m^2) vs the 0 deg wing
    data.aeroARW.Cd        = vp.Cd0 + dCdA_arw/vp.A;                    % (-)
    %-forces: same formulas as f_drag / f_lift in vehModel.m
    data.aeroARW.f_drag    =  0.5*vp.rho*vp.A*vx_arw.^2.*data.aeroARW.Cd;        % (N)
    data.aeroARW.f_lift    = -0.5*vp.rho*vp.A*vx_arw.^2.*(ClF_arw + ClR_arw);    % (N) positive downwards
    data.aeroARW.f_lift_f  = -0.5*vp.rho*vp.A*vx_arw.^2.*ClF_arw;                % (N) front axle
    data.aeroARW.f_lift_r  = -0.5*vp.rho*vp.A*vx_arw.^2.*ClR_arw;                % (N) rear axle
    clear alpha_arw vx_arw ClF_arw ClR_arw dCdA_arw iRW
end

%-active front-wing flap (ARFWd rwMandate==6 / ARFWr rwMandate==7): the FW additive delta layer of
% vp.aeroAFW along the solved trajectory, on the FULL grid s_full. Flap row
% LOCATED BY SYMBOL NAME (row nu-2: FW before RW, delta last - do NOT copy
% packRun's uRow==nu-1 assert here). All three vp.aeroAFW nodes share ONE
% baseline collapse fit, so differencing rwAeroMapEvalNum against alphaFW = 0
% isolates the pure FW increments (partition of unity, asserted by the T5 gate)
% without restating the blend arithmetic - one owner, the numeric twin.
if vp.ActAero == 1 && any(vp.rwMandate == [6 7])
    iFW = find(arrayfun(@(k) strcmp(erase(name(u(k)),'_n'),'activeAeroFW'), 1:nu), 1);
    assert(~isempty(iFW), 'MLTP: activeAeroFW not found in u');
    alpha_afw = data.u_full(iFW,:);                                     % FW flap angle (deg)
    vx_afw    = data.x_full(1,:);                                       % (m/s)
    %-same clamp-and-warn rationale as the ActiveRW block above (IPOPT bound slack)
    if any(alpha_afw < vp.aeroAFW.alphaMin - 1e-3 | alpha_afw > vp.aeroAFW.alphaMax + 1e-3)
        warning('MLTP:afwAlphaOutOfRange', ...
            ['ARFWd: solved FW flap angle leaves [%g, %g] deg by more than 1e-3 (min %.6g, max %.6g) - ' ...
             'clamping for post-processing. Check the NLP bounds (activeAeroFW_lim) before trusting the aero trace.'], ...
            vp.aeroAFW.alphaMin, vp.aeroAFW.alphaMax, min(alpha_afw), max(alpha_afw));
    end
    alpha_afw = min(max(alpha_afw, vp.aeroAFW.alphaMin), vp.aeroAFW.alphaMax);
    [ClF_a, ClR_a, dCdA_fw] = rwAeroMapEvalNum(vp.aeroAFW, alpha_afw, vx_afw);
    [ClF_0, ClR_0]          = rwAeroMapEvalNum(vp.aeroAFW, 0,         vx_afw);
    dClF_fw = ClF_a - ClF_0;                                            % (-) FW shift, 0 at 0 deg
    dClR_fw = ClR_a - ClR_0;
    data.aeroAFW.alpha    = alpha_afw;                                  % (deg) AUTHORITATIVE flap trace
    data.aeroAFW.uRow     = iFW;                                        % row of u/u_full holding it (nu-2)
    data.aeroAFW.vx       = vx_afw;                                     % (m/s)
    data.aeroAFW.dClF     = dClF_fw;                                    % (-) additive front shift (+ = unload)
    data.aeroAFW.dClR     = dClR_fw;                                    % (-) additive rear shift
    data.aeroAFW.dCdA     = dCdA_fw;                                    % (m^2) vs the 0 deg flap
    %-combined car totals: RW map value (data.aeroARW above) + FW delta layer,
    % same composition as the SX evaluator in vehModel.m
    data.aeroAFW.Cl_front = data.aeroARW.Cl_front + dClF_fw;            % (-) NEGATIVE = downforce
    data.aeroAFW.Cl_rear  = data.aeroARW.Cl_rear  + dClR_fw;            % (-)
    data.aeroAFW.Cd       = data.aeroARW.Cd + dCdA_fw/vp.A;             % (-)
    data.aeroAFW.f_drag   =  0.5*vp.rho*vp.A*vx_afw.^2.*data.aeroAFW.Cd;        % (N)
    data.aeroAFW.f_lift_f = -0.5*vp.rho*vp.A*vx_afw.^2.*data.aeroAFW.Cl_front;  % (N) front axle
    data.aeroAFW.f_lift_r = -0.5*vp.rho*vp.A*vx_afw.^2.*data.aeroAFW.Cl_rear;   % (N) rear axle
    data.aeroAFW.f_lift   = data.aeroAFW.f_lift_f + data.aeroAFW.f_lift_r;      % (N) positive downwards
    clear iFW alpha_afw vx_afw ClF_a ClR_a ClF_0 ClR_0 dClF_fw dClR_fw dCdA_fw
end

%-active rear wing, velocity-schedule study (docs/plans/2026-07-28-discrete-rw-mandate.md).
% Everything here is evaluated by CALLING the very CasADi expression that was put into
% the NLP (f_rwVel, built in the constraint section above) at the solution - never by
% restating its formulas - so the reported target, slack and band cannot drift from the
% ones that were actually enforced.
if vp.ActAero == 1 && vp.rwMandate == 3
    iRWv = data.aeroARW.uRow;
    [gv_v, tgt_v, tv_v, sOnv_v, band_v] = f_rwVel(data.x_opt./x_s, data.u_opt./u_s);

    data.rwVel.mode        = vp.rwMandate;                      % 3 = Velocity
    data.rwVel.aeroSetting = vp.aeroSetting;                    % 'ARWv'
    data.rwVel.opts        = vp.rwVelOpts;                      % the EXACT schedule used
    data.rwVel.band0       = vp.rwVelBand;
    data.rwVel.kappa       = vp.rwVelKappa;
    data.rwVel.M           = vp.rwVelBandM;   % the band's OWN window; this line recorded
                                              % rwMandateM (the retired look-ahead's)
                                              % until 2026-08-04
    data.rwVel.beta        = vp.rwVelBeta;
    data.rwVel.target      = full(tgt_v);                       % (deg) schedule + brake trigger
    data.rwVel.tv          = full(tv_v);                        % (deg) velocity schedule alone
    data.rwVel.sOn         = full(sOnv_v);                      % (-)   brake-trigger blend
    data.rwVel.band        = full(band_v);                      % (deg) per-knot band actually applied
    data.rwVel.alpha       = data.u_opt(iRWv,:);                % (deg) solved wing angle
    data.rwVel.err         = data.rwVel.alpha - data.rwVel.target;   % (deg) tracking error

    %-slack from the constraint rows themselves (both edges), un-scaled back to degrees
    gvDeg = full(gv_v).' * u_s(iRWv);
    data.rwVel.slackLo     = gvDeg(1:numel(data.rwVel.alpha));
    data.rwVel.slackHi     = gvDeg(numel(data.rwVel.alpha)+1:end);
    data.rwVel.slackMin    = min([data.rwVel.slackLo, data.rwVel.slackHi]);
    data.rwVel.tolDeg      = opts.ipopt.constr_viol_tol * u_s(iRWv);
    data.rwVel.satisfied   = data.rwVel.slackMin >= -data.rwVel.tolDeg;

    %-how tightly the law actually held. In the flat regions the band is band0, so
    % these are the numbers that say whether the wing was really ON the schedule.
    data.rwVel.errRMS      = sqrt(mean(data.rwVel.err.^2));
    data.rwVel.errMaxAbs   = max(abs(data.rwVel.err));
    tight                  = data.rwVel.band <= vp.rwVelBand*1.05;   % transition-free knots
    data.rwVel.fracTight   = mean(tight);
    %-BAND HEALTH. The law only means anything where the band is narrow, so record how
    % narrow it actually got. The first ARWv solve had a median band of 12.72 deg
    % against a 30 deg control range - the rows were all satisfied and the law was
    % nonetheless doing nothing. A satisfied constraint is not evidence of a binding one.
    data.rwVel.bandMedian  = median(data.rwVel.band);
    data.rwVel.bandP90     = prctile(data.rwVel.band, 90);
    data.rwVel.fracBand1   = mean(data.rwVel.band <= 1.0);
    data.rwVel.fracErr1    = mean(abs(data.rwVel.err) <= 1.0);
    if any(tight)
        data.rwVel.errRMStight = sqrt(mean(data.rwVel.err(tight).^2));
    else
        data.rwVel.errRMStight = NaN;
    end

    %-ACTUATOR CHECK. The schedule was sized against |ax| = 25.2 m/s^2 before the
    % solve; this is the solved trace re-checked against the real actuator bound.
    dtv                    = diff(data.rwVel.alpha);
    dtsol                  = diff(data.t_opt(:).');
    dtsol                  = dtsol(1:numel(dtv));
    data.rwVel.alphaRate   = dtv ./ max(dtsol, 1e-9);            % (deg/s)
    data.rwVel.rateMax     = max(abs(data.rwVel.alphaRate));
    data.rwVel.rateOK      = data.rwVel.rateMax <= c.ub.RW*1.02;

    fprintf(['MLTP: RW velocity law - tracking RMS %.3f deg, max |err| %.3f deg, ' ...
             '|err| <= 1 deg on %.0f%% of knots, max slew %.1f deg/s (limit %g)\n'], ...
        data.rwVel.errRMS, data.rwVel.errMaxAbs, 100*data.rwVel.fracErr1, ...
        data.rwVel.rateMax, c.ub.RW);
    fprintf(['MLTP:   band health - median %.2f deg, p90 %.2f deg, <= 1 deg on %.0f%% of ' ...
             'knots. A WIDE band means the law is not binding, whatever the rows say.\n'], ...
        data.rwVel.bandMedian, data.rwVel.bandP90, 100*data.rwVel.fracBand1);

    if ~data.rwVel.satisfied
        warning('MLTP:rwVelViolated', ...
            'RW velocity law violated by %.4f deg (tol %.4f deg).', ...
            -data.rwVel.slackMin, data.rwVel.tolDeg);
    end
    if ~data.rwVel.rateOK
        warning('MLTP:rwVelSlewExceeded', ...
            ['solved wing slew reaches %.1f deg/s against c.ub.RW = %g deg/s - the ' ...
             'reported lap time uses an actuator the car does not have.'], ...
            data.rwVel.rateMax, c.ub.RW);
    end
    clear gv_v tgt_v tv_v sOnv_v band_v gvDeg tight dtv dtsol

%-active rear-wing / front-wing, discrete-station airbrake(-analogue) floor
% study (rwMandate==4 ARWd; ==5 AFWd; ==6 ARFWd, BOTH surfaces -
% docs/superpowers/specs/2026-08-09-arfwd-combined-wings-design.md).
% Same standing rule as the velocity-schedule block above: each floor row is
% evaluated by CALLING the very CasADi expression built into the NLP (f_rwDisc /
% f_fwDisc, constraint section above) at the solution - never by restating its
% formula. rwDiscretize then post-processes each CONTINUOUS solved trace (the
% quoted lap) into the nearest-station staircase for that surface's detents:
% the four Static stations [-10 0 10 15] deg for the RW (modes 4 and 6), the
% three additive-map nodes [-25 -20 0] deg for the FW axis (modes 5 and 6).
% Mode 6 runs the loop TWICE: RW -> data.rwDisc (mode recorded 6), FW -> data.fwDisc.
% Mode 7 (ARFWr) reuses the SAME rwDiscretize/dwell/coverage audit machinery as a
% COSMETIC overlay on the reactive law's continuous solve, even though mode 7 has
% no discrete-station floor of its own - there is no f_rwDisc/f_fwDisc twin to call
% (f_arfwr carries the combined TWO-SIDED band instead), so the fD dispatch below is
% bypassed for mode 7 and its per-surface slice of f_arfwr's gv/target/band outputs
% substituted in its place (row-slice map: design brief / band block above).
elseif vp.ActAero == 1 && any(vp.rwMandate == [4 5 6 7])
    % Audit rows mirror the constraint-side abSurf:
    % {data field, u row, stations, brake-target station, viol id, slew warn id, label}.
    switch vp.rwMandate
        case 4
            audT = { {'rwDisc', data.aeroARW.uRow, [-10 0 10 15], 15, 'MLTP:rwDiscViolated', 'MLTP:rwDiscRoundedSlew', 'RW'} };
        case 5
            audT = { {'rwDisc', data.aeroARW.uRow, [-25 -20 0],    0, 'MLTP:fwDiscViolated', 'MLTP:fwDiscRoundedSlew', 'FW'} };
        case {6, 7}
            audT = { {'rwDisc', data.aeroARW.uRow, [-10 0 10 15], 15, 'MLTP:rwDiscViolated', 'MLTP:rwDiscRoundedSlew', 'RW'}, ...
                     {'fwDisc', data.aeroAFW.uRow, [-25 -20 0],    0, 'MLTP:fwDiscViolated', 'MLTP:fwDiscRoundedSlew', 'FW'} };
    end
    if vp.rwMandate == 7
        % f_arfwr has 6 outputs (gv, tRW, tFW, sOn, bandRW, bandFW) and ONE combined
        % 4(N+1) gv row - evaluated ONCE here (mode-3 idiom, :898/:1000), sliced per
        % surface inside the loop below. Kept alive past this block: data.lawTrack
        % right after the audits reuses tRv/tFv/sOnv/bRv/bFv verbatim.
        [gvA, tRv, tFv, sOnv, bRv, bFv] = f_arfwr(data.x_opt./x_s, data.u_opt./u_s);
        gvA = full(gvA); tRv = full(tRv); tFv = full(tFv);
        sOnv = full(sOnv); bRv = full(bRv); bFv = full(bFv);
    end
    for si = 1:numel(audT)
        [ab_fld, iRWd, ab_st, ab_covSt, ab_violId, ab_slewWarnId, ab_label] = audT{si}{:};
        if vp.rwMandate == 7
            % Surface si owns rows (si-1)*2*(N+1) + (1:2*(N+1)) of gvA; within a
            % surface, first N+1 rows = lower edge, next N+1 = upper edge.
            r0    = (si-1)*2*(N+1);
            loDeg = gvA(r0 + (1:N+1)).'       * u_s(iRWd);      % 1 x (N+1), deg
            hiDeg = gvA(r0 + (N+2:2*(N+1))).' * u_s(iRWd);      % 1 x (N+1), deg
            slack = min(loDeg, hiDeg);                          % per-knot min over both edges
            if strcmp(ab_fld, 'fwDisc'), floor_v = tFv - bFv; else, floor_v = tRv - bRv; end
            sOn_v = sOnv;
            % mode-3 pattern (:921): tolerance scaled from IPOPT's own constraint
            % tolerance, NOT a fixed literal - IPOPT only guarantees ~3e-3 deg here.
            tolDeg    = opts.ipopt.constr_viol_tol * u_s(iRWd);
            satisfied = all(slack >= -tolDeg);
        else
            if strcmp(ab_fld, 'fwDisc'), fD = f_fwDisc; else, fD = f_rwDisc; end
            [gd_v, floor_v, sOn_v] = fD(data.x_opt./x_s, data.u_opt./u_s);
            slack     = full(gd_v).' * u_s(iRWd);
            satisfied = all(slack > -1e-3);
        end
        alpha_v = data.u_opt(iRWd,:);                              % (deg) solved continuous trace
        [alphaDisc, info] = rwDiscretize(s_knot, alpha_v, ab_st, c.ub.RW, data.x_opt(1,:));

        %-airbrake coverage: fraction of sustained-braking knots (the floor's own
        % trigger blend) landing on this surface's brake-target station. Guarded
        % NaN when no knot crosses the trigger - a real, reportable case.
        onMask = full(sOn_v) > 0.9;
        if any(onMask)
            brakeCoverage = mean(alphaDisc(onMask) == ab_covSt);
        else
            brakeCoverage = NaN;
        end

        data.(ab_fld) = struct('mode',vp.rwMandate,'aeroSetting',vp.aeroSetting,'alpha',alpha_v, ...
            'alphaDisc',alphaDisc,'stations',ab_st,'floorWin',full(floor_v), ...
            'sOn',full(sOn_v),'slack',slack,'satisfied',satisfied, ...
            'nTransitions',info.nTransitions,'merged',info.merged, ...
            'roundedSlewOK',info.roundedSlewOK,'devRMS',info.devRMS,'devMax',info.devMax, ...
            'brakeCoverage',brakeCoverage,'dwell',info.dwell);

        fprintf(['MLTP: %s DISCRETE audit - %d transitions (%d dwell-merges), dev RMS %.3f deg, ' ...
                 'dev max %.3f deg, rounded-slew OK: %d, brake coverage %s\n'], ...
            ab_label, data.(ab_fld).nTransitions, data.(ab_fld).merged, data.(ab_fld).devRMS, ...
            data.(ab_fld).devMax, data.(ab_fld).roundedSlewOK, mat2str(round(100*data.(ab_fld).brakeCoverage)) );

        if ~data.(ab_fld).satisfied
            warning(ab_violId, 'airbrake floor violated: min slack %.3f deg', min(slack));
        end
        if ~data.(ab_fld).roundedSlewOK
            warning(ab_slewWarnId, 'rounded staircase infeasible at %.0f deg/s', c.ub.RW);
        end
    end
    clear audT si fD gd_v floor_v sOn_v alpha_v alphaDisc info slack onMask brakeCoverage iRWd ab_fld ab_st ab_covSt ab_violId ab_slewWarnId ab_label satisfied r0 loDeg hiDeg tolDeg
end

%-reactive dual-wing law tracking (ARFWr, rwMandate==7): how well the solved trace
% followed the closed-form target it was banded around. gvA/tRv/tFv/sOnv/bRv/bFv
% already evaluated once above (audit block); reused verbatim, not re-evaluated.
if vp.ActAero == 1 && vp.rwMandate == 7
    aR = data.u_opt(data.aeroARW.uRow,:);  aF = data.u_opt(data.aeroAFW.uRow,:);  % knot traces, deg
                                                       % (mode-3 pattern :913 - data.aeroARW.uRow,
                                                       % NOT data.aeroARW.alpha which is full-grid)
    dsv = diff(s_knot); vxv = data.x_opt(1,:);
    % BAND HEALTH via the single owner Functions/lawTrackStats.m (shared with the
    % post-hoc recompute, so the two can never drift). Tolerance is IPOPT's own,
    % as at :968/:1072 - NOT a literal: occupancy at 1e-6 counts knots the solver
    % never promised to place, and mode 7's band opens wide at every classifier
    % transition, so occ saturates and says nothing on its own. bandMedian/P90 +
    % errRMStight are what actually say whether the law binds.
    tolR = opts.ipopt.constr_viol_tol * u_s(data.aeroARW.uRow);
    tolF = opts.ipopt.constr_viol_tol * u_s(data.aeroAFW.uRow);
    stR  = lawTrackStats(aR, tRv, bRv, vp.reactLaw.bandRW, tolR);
    stF  = lawTrackStats(aF, tFv, bFv, vp.reactLaw.bandFW, tolF);
    data.lawTrack = struct('s',s_knot,'tRW',tRv,'tFW',tFv,'sOn',sOnv,'bandRW',bRv,'bandFW',bFv, ...
        'band0RW',vp.reactLaw.bandRW,'band0FW',vp.reactLaw.bandFW,'tolDegRW',tolR,'tolDegFW',tolF, ...
        'trkRMS_RW',stR.errRMS,'trkMax_RW',stR.errMaxAbs, ...
        'trkRMS_FW',stF.errRMS,'trkMax_FW',stF.errMaxAbs, ...
        'occRW',stR.occ,'occFW',stF.occ, ...
        'bandMedianRW',stR.bandMedian,'bandP90RW',stR.bandP90,'fracBand1RW',stR.fracBand1, ...
        'fracTightRW',stR.fracTight,'errRMStightRW',stR.errRMStight,'fracErr1RW',stR.fracErr1, ...
        'bandMedianFW',stF.bandMedian,'bandP90FW',stF.bandP90,'fracBand1FW',stF.fracBand1, ...
        'fracTightFW',stF.fracTight,'errRMStightFW',stF.errRMStight,'fracErr1FW',stF.fracErr1, ...
        'slewMaxTgtRW',max(abs(diff(tRv))./dsv .* vxv(1:end-1)), ...
        'slewMaxTgtFW',max(abs(diff(tFv))./dsv .* vxv(1:end-1)));
    data.sdi.solData.inputs.alphaTgt   = timeseries(tRv, s_knot, 'name', 'alphaTgt');
    data.sdi.solData.inputs.alphaFWTgt = timeseries(tFv, s_knot, 'name', 'alphaFWTgt');
    fprintf(['MLTP: ARFWr law tracking - RW RMS %.3f deg (max %.3f), FW RMS %.3f deg (max %.3f), ' ...
             'in-band RW %.0f%% / FW %.0f%% (tol %.4f deg)\n'], data.lawTrack.trkRMS_RW, ...
        data.lawTrack.trkMax_RW, data.lawTrack.trkRMS_FW, data.lawTrack.trkMax_FW, ...
        100*data.lawTrack.occRW, 100*data.lawTrack.occFW, tolR);
    fprintf(['MLTP:   band health RW - median %.2f deg, p90 %.2f, tight (<=%.2f) on %.0f%% of knots, ' ...
             'RMS |err| there %.3f deg | FW - median %.2f, p90 %.2f, tight %.0f%%, RMS %.3f deg. ' ...
             'A WIDE band means the law is not binding, whatever the rows say.\n'], ...
        stR.bandMedian, stR.bandP90, 1.05*vp.reactLaw.bandRW, 100*stR.fracTight, stR.errRMStight, ...
        stF.bandMedian, stF.bandP90, 100*stF.fracTight, stF.errRMStight);
    clear aR aF dsv vxv gvA tRv tFv sOnv bRv bFv tolR tolF stR stF
end

%-tyre forces
f_Ftyres = Function('f_Ftyres',{x,u,y,pv},{[fx_fl, fy_fl, fz_fl; fx_fr, fy_fr, fz_fr; fx_rl, fy_rl, fz_rl; fx_rr, fy_rr, fz_rr]},{'x','u','y','pv'},{'Ftyres'});
Ftyres = reshape(full(f_Ftyres(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),4,3,N+1);
data.vehicle.fx_fl = Ftyres(1,1,:);
data.vehicle.fy_fl = Ftyres(1,2,:);
data.vehicle.fz_fl = Ftyres(1,3,:);
data.vehicle.fx_fr = Ftyres(2,1,:);
data.vehicle.fy_fr = Ftyres(2,2,:);
data.vehicle.fz_fr = Ftyres(2,3,:);
data.vehicle.fx_rl = Ftyres(3,1,:);
data.vehicle.fy_rl = Ftyres(3,2,:);
data.vehicle.fz_rl = Ftyres(3,3,:);
data.vehicle.fx_rr = Ftyres(4,1,:);
data.vehicle.fy_rr = Ftyres(4,2,:);
data.vehicle.fz_rr = Ftyres(4,3,:);
% grip utilisation (dimensionless 0..~1): elliptical usage of the per-axle
% Pacejka friction budget - report-safe signal (no absolute forces/parameters)
gripUtil = @(F,row,ty,Fz0eff) sqrt( ...
    (reshape(F(row,1,:),1,N+1)./((ty.pDx1 + ty.pDx2*(reshape(F(row,3,:),1,N+1)-Fz0eff)/Fz0eff).*reshape(F(row,3,:),1,N+1))).^2 + ...
    (reshape(F(row,2,:),1,N+1)./((ty.pDy1 + ty.pDy2*(reshape(F(row,3,:),1,N+1)-Fz0eff)/Fz0eff).*reshape(F(row,3,:),1,N+1))).^2 );
data.vehicle.gu_fl = gripUtil(Ftyres,1,vp.tyre_f,vp.tyre_f.Fz0*vp.Fz0_shift_f);
data.vehicle.gu_fr = gripUtil(Ftyres,2,vp.tyre_f,vp.tyre_f.Fz0*vp.Fz0_shift_f);
data.vehicle.gu_rl = gripUtil(Ftyres,3,vp.tyre_r,vp.tyre_r.Fz0*vp.Fz0_shift_r);
data.vehicle.gu_rr = gripUtil(Ftyres,4,vp.tyre_r,vp.tyre_r.Fz0*vp.Fz0_shift_r);
clear gripUtil

%-slip angles
f_slipAng = Function('f_slipAng',{x,u,y,pv},{[sa_fl; sa_fr; sa_rl; sa_rr]},{'x','u','y','pv'},{'slipAng'});
slipAng = reshape(full(f_slipAng(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),4,1,N+1);
data.vehicle.sa_fl = slipAng(1,1,:);
data.vehicle.sa_fr = slipAng(2,1,:);
data.vehicle.sa_rl = slipAng(3,1,:);
data.vehicle.sa_rr = slipAng(4,1,:);

%-slip ratios
f_slipX = Function('f_slipX',{x,u,y,pv},{[sx_fl; sx_fr; sx_rl; sx_rr]},{'x','u','y','pv'},{'slipX'});
slipX = reshape(full(f_slipX(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),4,1,N+1);
data.vehicle.sx_fl = slipX(1,1,:);
data.vehicle.sx_fr = slipX(2,1,:);
data.vehicle.sx_rl = slipX(3,1,:);
data.vehicle.sx_rr = slipX(4,1,:);

%- e-motor speed
f_OmMotor = Function('f_OmMotor',{x,u,y,pv},{[Om_motor]},{'x','u','y','pv'},{'Om_motor'});
OmMotor = reshape(full(f_OmMotor(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),1,1,N+1);
data.vehicle.Om_motor = OmMotor(1,1,:);

% -wheel torque
f_Torque = Function('f_Torque',{x,u,y,pv},{[T_fl; T_fr; T_rl; T_rr]},{'x','u','y','pv'},{'Torque'});
Torque = reshape(full(f_Torque(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),4,1,N+1);
data.vehicle.T_fl = Torque(1,1,:);
data.vehicle.T_fr = Torque(2,1,:);
data.vehicle.T_rl = Torque(3,1,:);
data.vehicle.T_rr = Torque(4,1,:);


% Energy
f_P_motor = Function('f_P_motor',{x,u,y,pv},{[P_motor]},{'x','u','y','pv'},{'P_motor'});
Pmotor = reshape(full(f_P_motor(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),1,1,N+1);
data.vehicle.P_motor = Pmotor(1,1,:).*1e-3;
E_motor(1) = 0;

for i = 1:N
        E_motor(i+1) = E_motor(i) + 0.5*(data.vehicle.P_motor(i) + data.vehicle.P_motor(i+1))*(data.t_opt(i+1) - data.t_opt(i))*2.7778e-4/pt.eff;                     %kWh
end

E_motor = reshape(E_motor,1,1,N+1);
data.vehicle.E_motor = E_motor(1,1,:);

% Constraints
hval = full(h_eq(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot));
for i=1:length(hnames)
    data.constraints.(hnames{i}) = hval(i,:);
end

%% Postprocessing - Log data with Simulation Data Inspector

% Initialise figures and SDI
% Simulink.sdi.clear; %clears all data in SDI **USE CAREFULLY**
%-define container for external figures
global figures;
figures = struct();

%-sync data cursors between SDI and external figures
figures.callbackID = Simulink.sdi.registerCursorCallback(@(t1,t2)onCursorMove(t1,t2));

%-initialise external figures
%%-track
figures.track.fig = figure('Name','Circuit Map'); clf
figures.track.ax = axes;
updateTrackPlot(nan,nan);

% Create time series objects for sdi
%-states, inputs and aux. variables
for i = 1:length(x)
    aux = x(i); name = erase(aux.name,'_n'); %get name of variable
    data.sdi.solData.states.(name) = timeseries(data.x_full(i,:), s_full,'name',name); %create timeseries object
end
for i = 1:length(u)
    aux = u(i); name = erase(aux.name,'_n');
    data.sdi.solData.inputs.(name) = timeseries(data.u_full(i,:), s_full,'name',name);
end
%-discrete-station rear-wing / front-wing (ARWd rwMandate==4, AFWd ==5,
% ARFWd ==6, ARFWr ==7 - the latter a cosmetic overlay on the reactive law's
% continuous solve, not a real floor): rounded staircase(s) pushed as siblings of
% the continuous input trace(s) just created above (HRP Gear/n_gear dual-trace
% pattern, docs/hrp-gear-shift-and-framework-diffs.md), so SDI can overlay the pairs.
% Knot-only (s_knot), not s_full, because rwDiscretize (and the floors/bands it
% audits) only ever operate at the knots. Modes 6/7 push TWO named series -
% they cannot share a name.
if vp.ActAero == 1 && any(vp.rwMandate == [4 5 6 7])
    data.sdi.solData.inputs.alphaDisc = timeseries(data.rwDisc.alphaDisc, s_knot, 'name', 'alphaDisc');
    if any(vp.rwMandate == [6 7])
        data.sdi.solData.inputs.alphaFWDisc = timeseries(data.fwDisc.alphaDisc, s_knot, 'name', 'alphaFWDisc');
    end
end
for i = 1:length(y)
    aux = y(i); name = erase(aux.name,'_n'); 
    data.sdi.solData.aux.(name) = timeseries(data.y_full(i,:), s_full,'name',name); 
end
data.sdi.solData.time = timeseries(data.t_opt, s_knot, 'name', 'time');
clear aux name

%-track data
data.sdi.track.s = timeseries(s_full,s_full, 'name', 's');
data.sdi.track.k = timeseries(k_full,s_full, 'name', 'k');
data.sdi.track.x = timeseries(data.track.x,s_full, 'name', 'x');
data.sdi.track.y = timeseries(data.track.y,s_full, 'name', 'y');

%-vehicle data
%%-parameters (just as a way to store the values that were used for each simulation)
fn = fieldnames(vp); %get names of vehicle parameters
for i = 1:length(fn)
    if isa(vp.(fn{i}),'double')
        data.sdi.vehicle.params.(fn{i}) =  timeseries(repmat(vp.(fn{i}),1,length(s_knot)),s_knot);
    end
end
clear fn

%%-aerodynamic forces
data.sdi.vehicle.f_drag = timeseries(data.vehicle.f_drag, s_knot, 'name', 'f_drag');
data.sdi.vehicle.f_lift = timeseries(data.vehicle.f_lift, s_knot, 'name', 'f_lift');
data.sdi.vehicle.f_lift_fl = timeseries(data.vehicle.f_lift_fl, s_knot, 'name', 'f_lift_fl');
data.sdi.vehicle.f_lift_fr = timeseries(data.vehicle.f_lift_fr, s_knot, 'name', 'f_lift_fr');
data.sdi.vehicle.f_lift_rl = timeseries(data.vehicle.f_lift_rl, s_knot, 'name', 'f_lift_rl');
data.sdi.vehicle.f_lift_rr = timeseries(data.vehicle.f_lift_rr, s_knot, 'name', 'f_lift_rr');
data.sdi.vehicle.f_side = timeseries(data.vehicle.f_side, s_knot, 'name', 'f_side');
data.sdi.vehicle.f_side_fl = timeseries(data.vehicle.f_side_fl, s_knot, 'name', 'f_side_fl');
data.sdi.vehicle.f_side_fr = timeseries(data.vehicle.f_side_fr, s_knot, 'name', 'f_side_fr');
data.sdi.vehicle.f_side_rl = timeseries(data.vehicle.f_side_rl, s_knot, 'name', 'f_side_rl');
data.sdi.vehicle.f_side_rr = timeseries(data.vehicle.f_side_rr, s_knot, 'name', 'f_side_rr');

%%-tyre forces
data.sdi.vehicle.fx_fl = timeseries(data.vehicle.fx_fl, s_knot, 'name', 'fx_fl');
data.sdi.vehicle.fy_fl = timeseries(data.vehicle.fy_fl, s_knot, 'name', 'fy_fl');
data.sdi.vehicle.fz_fl = timeseries(data.vehicle.fz_fl, s_knot, 'name', 'fz_fl');
data.sdi.vehicle.fx_fr = timeseries(data.vehicle.fx_fr, s_knot, 'name', 'fx_fr');
data.sdi.vehicle.fy_fr = timeseries(data.vehicle.fy_fr, s_knot, 'name', 'fy_fr');
data.sdi.vehicle.fz_fr = timeseries(data.vehicle.fz_fr, s_knot, 'name', 'fz_fr');
data.sdi.vehicle.fx_rl = timeseries(data.vehicle.fx_rl, s_knot, 'name', 'fx_rl');
data.sdi.vehicle.fy_rl = timeseries(data.vehicle.fy_rl, s_knot, 'name', 'fy_rl');
data.sdi.vehicle.fz_rl = timeseries(data.vehicle.fz_rl, s_knot, 'name', 'fz_rl');
data.sdi.vehicle.fx_rr = timeseries(data.vehicle.fx_rr, s_knot, 'name', 'fx_rr');
data.sdi.vehicle.fy_rr = timeseries(data.vehicle.fy_rr, s_knot, 'name', 'fy_rr');
data.sdi.vehicle.fz_rr = timeseries(data.vehicle.fz_rr, s_knot, 'name', 'fz_rr');
data.sdi.vehicle.gu_fl = timeseries(data.vehicle.gu_fl, s_knot, 'name', 'gu_fl');
data.sdi.vehicle.gu_fr = timeseries(data.vehicle.gu_fr, s_knot, 'name', 'gu_fr');
data.sdi.vehicle.gu_rl = timeseries(data.vehicle.gu_rl, s_knot, 'name', 'gu_rl');
data.sdi.vehicle.gu_rr = timeseries(data.vehicle.gu_rr, s_knot, 'name', 'gu_rr');

%%-slip angles
data.sdi.vehicle.sa_fl = timeseries(data.vehicle.sa_fl, s_knot, 'name', 'sa_fl');
data.sdi.vehicle.sa_fr = timeseries(data.vehicle.sa_fr, s_knot, 'name', 'sa_fr');
data.sdi.vehicle.sa_rl = timeseries(data.vehicle.sa_rl, s_knot, 'name', 'sa_rl');
data.sdi.vehicle.sa_rr = timeseries(data.vehicle.sa_rr, s_knot, 'name', 'sa_rr');

%%-slip ratios
data.sdi.vehicle.sx_fl = timeseries(data.vehicle.sx_fl, s_knot, 'name', 'sx_fl');
data.sdi.vehicle.sx_fr = timeseries(data.vehicle.sx_fr, s_knot, 'name', 'sx_fr');
data.sdi.vehicle.sx_rl = timeseries(data.vehicle.sx_rl, s_knot, 'name', 'sx_rl');
data.sdi.vehicle.sx_rr = timeseries(data.vehicle.sx_rr, s_knot, 'name', 'sx_rr');

% e-motor speed
data.sdi.vehicle.Om_motor = timeseries(data.vehicle.Om_motor, s_knot, 'name', 'Om_motor');

%%-torque wheels
data.sdi.vehicle.T_fl = timeseries(data.vehicle.T_fl, s_knot, 'name', 'T_fl');
data.sdi.vehicle.T_fr = timeseries(data.vehicle.T_fr, s_knot, 'name', 'T_fr');
data.sdi.vehicle.T_rl = timeseries(data.vehicle.T_rl, s_knot, 'name', 'T_rl');
data.sdi.vehicle.T_rr = timeseries(data.vehicle.T_rr, s_knot, 'name', 'T_rr');

% data.sdi.vehicle.ATD_fl = timeseries(data.vehicle.ATD_fl, s_knot, 'name', 'ATD_fl');
% data.sdi.vehicle.ATD_fr = timeseries(data.vehicle.ATD_fr, s_knot, 'name', 'ATD_fr');
% data.sdi.vehicle.ATD_rl = timeseries(data.vehicle.ATD_rl, s_knot, 'name', 'ATD_rl');
% data.sdi.vehicle.ATD_rr = timeseries(data.vehicle.ATD_rr, s_knot, 'name', 'ATD_rr');

%Energy and power
data.sdi.vehicle.P_motor = timeseries(data.vehicle.P_motor, s_knot, 'name', 'P_motor (kW)');
data.sdi.vehicle.E_motor = timeseries(data.vehicle.E_motor, s_knot, 'name', 'E_motor (kWh)');

%-path constraints
for i = 1:length(hnames)
    data.sdi.constraints.(hnames{i}) = timeseries(data.constraints.(hnames{i}), s_knot, 'name', hnames{i});
end

% Import data to SDI
newRun = Simulink.sdi.Run.create;
newRun.Name = ['(version)_' 'ds' num2str(OPT_ds) '_' OPT_uinter '_sol:' num2str(data.t_opt(end)) 's_X0=' num2str(Xi(:)') '_Xf=' num2str(Xf(:)') '_ET:' num2str(elapsedTime(3)) 's_' datestr(datetime)];
solutionData = data.sdi.solData;
trackData = data.sdi.track;
vehicleData = data.sdi.vehicle;
pathConstraints = data.sdi.constraints;
add(newRun, 'vars', solutionData, trackData, vehicleData, pathConstraints);
data.sdi.run = newRun;
clear newRun solutionData trackData

%% -plot data in SDI
plotSDI;


%%-time 
elapsedTime(end+1) = toc - elapsedTime(end);
disp(' ')
disp('Elapsed time:')
disp(table(elapsedTime(2),elapsedTime(3),elapsedTime(4),'VariableNames',{'Transcription','Solution','Post-processing'}))
data.duration = elapsedTime;


disp('Objective function value:')
disp(full(sol.f))

disp('Lap time:')
disp(data.t_opt(end))

apexSpeeds;
run('Functions\validateAeroCollapse.m');