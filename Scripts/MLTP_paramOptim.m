% Minimum lap time problem solver - parameter optimisation

% Solve the Minimum Lap Time Problem including the optimal value of parameters of the vehicle model

%% Initialisation
% Clear Simulink SDI callback if it exists
global figures
try Simulink.sdi.unregisterCursorCallback(figures.callbackID); end

% Clear workspace
clc
clear; clear global;

% Import functions
% addpath('C:\Devtools\Install\Matlab\2021b\Matlab/casadi-3.6.5-windows64-matlab2018b')

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

% ActiveRW is a MLTP.m-only configuration: vehModel.m's ActAero==1 branch is shared,
% so this script BUILDS fine, but its post-processing has no ARW block (no
% data.aeroARW: wing trace, per-axle Cl, f_drag/f_lift from the 2-D map) and
% figAeroActuator/plotSDI's ActiveRW panels would have nothing to read. Fail here
% rather than after a multi-hour solve.
assert(vp.ActAero ~= 1, ...
    ['MLTP_paramOptim: ActiveRW (AeroConfig=''ActiveRW'') is only supported by MLTP.m ' ...
     '- its post-processing lacks the data.aeroARW block']);

%% Initialisation
% Warm start from the newest cached MLTP_initial result for THIS circuit and
% aero setting; if none exists, solve the simplified case now (MLTP_initial
% caches it on the way out) so a new track self-heals on first run.
% nu is not defined yet (vehModel.m runs below), so derive it from the config
% ladder: without it latestInit cannot tell an AWD cache from an ATD one for
% the same aeroSetting, and an ATD run would warm-start from an AWD lap.
initFile = latestInit(circuit, vp.aeroSetting, nuForConfig(pt.ATD, vp.ActAero, getfielddef(vp,'rwMandate',0)));
if isempty(initFile)
    fprintf('MLTP_paramOptim: no cached init for %s / %s - running MLTP_initial.m\n', circuit, vp.aeroSetting);
    run('MLTP_initial.m');                  % populates data.init.* AND caches a .mat
else
    fprintf('MLTP_paramOptim: warm start from %s\n', initFile);
    importfile(initFile);                   % the .mat holds `data`, which already has .init
end
clear initFile

elapsedTime = 0; tic %Start time counter


%% Parametric Optimisation

% Design parameters to optimise

% number of parameters
if pt.ATD == 0 && vp.ActAero == 0
    np = 5;  
elseif pt.ATD == 0 && vp.ActAero == 1
    np = 4;
elseif pt.ATD == 0 && vp.ActAero == 2
    np = 3;     
elseif pt.ATD == 0 && vp.ActAero == 3
    np = 3;
elseif pt.ATD == 1 && vp.ActAero == 0
    np = 3;
elseif pt.ATD == 1 && vp.ActAero == 1
    np = 2;
elseif pt.ATD == 1 && vp.ActAero == 2
    np = 1;     
elseif pt.ATD == 1 && vp.ActAero == 3
    np = 1;
end

if pt.ATD == 0
% brake distribution [%front]
brkB_n = SX.sym('brkB_n');
brkB_s = 1;
brkB = brkB_s * brkB_n;
brkB_lim = 1/brkB_s * [0.5 1];

% torque distribution [%rear]
Tdist_n = SX.sym('Tdist_n');
Tdist_s = 1;
Tdist = Tdist_s * Tdist_n;
Tdist_lim = 1/Tdist_s * [0.5 1];

% roll stiffness distribution [%rear]
ksD_n = SX.sym('ksD_n');
ksD_s = 1;
ksD = ksD_s * ksD_n;
ksD_lim = 1/ksD_s * [0.3 0.7];

elseif pt.ATD == 1

% roll stiffness distribution [%rear]
ksD_n = SX.sym('ksD_n');
ksD_s = 1;
ksD = ksD_s * ksD_n;
ksD_lim = 1/ksD_s * [0.3 0.7];

end

if vp.ActAero == 0   
% AoA FW [deg]
alpha_FL_n = SX.sym('alpha_FL_n');
alpha_FL_s = 10;
alpha_FL = alpha_FL_s * alpha_FL_n;
alpha_FL_lim = 1/alpha_FL_s * [0 10];

% AoA RW [deg]
alpha_RW_n = SX.sym('alpha_RW_n');
alpha_RW_s = 30;
alpha_RW = alpha_RW_s * alpha_RW_n;
alpha_RW_lim = 1/alpha_RW_s * [0 30];
elseif vp.ActAero == 1   
% AoA FW [deg]
alpha_FL_n = SX.sym('alpha_FL_n');
alpha_FL_s = 10;
alpha_FL = alpha_FL_s * alpha_FL_n;
alpha_FL_lim = 1/alpha_FL_s * [0 10];
end

if pt.ATD == 0 && vp.ActAero == 0
    p_s = [brkB_s; Tdist_s; ksD_s; alpha_FL_s; alpha_RW_s];
    p = [brkB_n; Tdist_n; ksD_n; alpha_FL_n; alpha_RW_n];
    p_lim = [brkB_lim; Tdist_lim; ksD_lim; alpha_FL_lim; alpha_RW_lim];
elseif pt.ATD == 0 && vp.ActAero == 1
    p_s = [brkB_s; Tdist_s; ksD_s; alpha_FL_s];
    p = [brkB_n; Tdist_n; ksD_n; alpha_FL_n];
    p_lim = [brkB_lim; Tdist_lim; ksD_lim; alpha_FL_lim];
elseif pt.ATD == 0 && vp.ActAero == 2
    p_s = [brkB_s; Tdist_s; ksD_s];
    p = [brkB_n; Tdist_n; ksD_n];
    p_lim = [brkB_lim; Tdist_lim; ksD_lim];
elseif pt.ATD == 0 && vp.ActAero == 3
    p_s = [brkB_s; Tdist_s; ksD_s];
    p = [brkB_n; Tdist_n; ksD_n];
    p_lim = [brkB_lim; Tdist_lim; ksD_lim];
elseif pt.ATD == 1 && vp.ActAero == 0
    p_s = [ksD_s; alpha_FL_s; alpha_RW_s];
    p = [ksD_n; alpha_FL_n; alpha_RW_n];
    p_lim = [ksD_lim; alpha_FL_lim; alpha_RW_lim];
elseif pt.ATD == 1 && vp.ActAero == 1
    p_s = [ksD_s; alpha_FL_s];
    p = [ksD_n; alpha_FL_n];
    p_lim = [ksD_lim; alpha_FL_lim];
elseif pt.ATD == 1 && vp.ActAero == 2
    p_s = [ksD_s];
    p = [ksD_n];
    p_lim = [ksD_lim];
elseif pt.ATD == 1 && vp.ActAero == 3
    p_s = [ksD_s];
    p = [ksD_n];
    p_lim = [ksD_lim];
end 

p_min = p_lim(:,1);
p_max = p_lim(:,2);

%Check number of parameters
if np ~= length(p) 
    error('Number of parameters is not consistent');
end

%Substitute fixed value parameters with symbolic optimisation variables
fn = fieldnames(vp); %get names of vehicle parameters
for i = 1:length(p)
    aux = p(i);
    aux = erase(aux.name, '_n');
    if sum(strcmp(fn, aux)) %Compare the name of the vehicle parameters with the symbolic optimisation variables in 'p'
        vp.(aux) = p(i)*p_s(i);
    else
        error(['Parameter "' aux '" not recognised']);
    end
end

% Recalculate vehicle parameters
vp.ksf = (1-vp.ksD)*2000;                                                                   % roll stiffness front axle                     (Nm/deg)
vp.ksr = vp.ksD*2000;                                                                       % roll stiffness rear axle                      (Nm/deg)
vp.ks = vp.ksf + vp.ksr;                                                                    % total roll stiffness                          (Nm/deg)
vp.ksf_rad = vp.ksf*(180/pi);                                                               % roll stiffness front axle                     (Nm/rad)
vp.ksr_rad = vp.ksr*(180/pi);                                                               % roll stiffness rear axle                      (Nm/rad)
vp.ks_rad = vp.ksf_rad + vp.ksr_rad;                                                        % total roll stiffness                          (Nm/rad)
vp.lty_dis_f = (vp.huf/vp.t)*(vp.muf/vp.m) + ((vp.l_r*vp.hRCf)/(vp.l*vp.t))*(vp.ms/vp.m) + ((vp.ksf_rad*(vp.d/(vp.ks_rad - vp.ms*vp.g*vp.d)))/vp.t)*(vp.ms/vp.m);   % fraction of load transfer reacted at front axle
vp.lty_dis_r = (vp.hur/vp.t)*(vp.mur/vp.m) + ((vp.l_f*vp.hRCr)/(vp.l*vp.t))*(vp.ms/vp.m) + ((vp.ksr_rad*(vp.d/(vp.ks_rad - vp.ms*vp.g*vp.d)))/vp.t)*(vp.ms/vp.m);   % fraction of load transfer reacted at rear axle
vp.lty_dis = vp.lty_dis_r / (vp.lty_dis_f + vp.lty_dis_r);

if vp.ActAero == 0 || vp.ActAero == 1
    vp.alpha_FR = vp.alpha_FL;
end


clear aux fn

%% load vehicle model

run('vehModel.m');

%% Optimal Control Problem - Dynamics and objetive function

% Objective function
L = sf;

% Continuous time dynamics and objective function
f_dyn = Function('f_dyn',{x,u,y,pv,p},{dx,L},{'x','u','y','pv','p'},{'dx','L'});

% Change of variable
f_sf = Function('sf',{x,kappa},{sf},{'x','kappa'},{'sf'});


%% Optimal Control Problem - Path Constraints

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


% Grip limits live in the combined-slip Pacejka forces themselves (tyreMF in
% vehModel.m); the old isotropic friction-circle constraints are gone. The
% per-tyre vertical load limit (Zenvo requirement, as in MLTP.m) is enforced
% here instead - it was missing from this entry point.
fz_lim_fl = (vp.Fz_lim - fz_fl)/vp.Fz_lim;   % ∈ [0,1] enforces 0 <= fz <= Fz_lim
fz_lim_fr = (vp.Fz_lim - fz_fr)/vp.Fz_lim;
fz_lim_rl = (vp.Fz_lim - fz_rl)/vp.Fz_lim;
fz_lim_rr = (vp.Fz_lim - fz_rr)/vp.Fz_lim;

if pt.ATD == 0
    hnames = {'motor_power', 'motor_rpm', 'ltx_eq', 'lty_eq', 'BrTh_1', 'fz_lim_fl', 'fz_lim_fr', 'fz_lim_rl', 'fz_lim_rr'};
    h = [motor_power; motor_rpm; ltx_eq; lty_eq; BrTh_1; fz_lim_fl; fz_lim_fr; fz_lim_rl; fz_lim_rr];
    h_lb = [0;     0;    -1;  -1;     -1;  0; 0; 0; 0];
    h_ub = [1;     1;     1;   1;      1;  1; 1; 1; 1];
elseif pt.ATD == 1
    ATD_eq = (1 - (ATD_FL + ATD_FR + ATD_RL + ATD_RR));
    hnames = {'motor_power', 'motor_rpm', 'ltx_eq', 'lty_eq', 'BrTh_1', 'ATD_eq', 'fz_lim_fl', 'fz_lim_fr', 'fz_lim_rl', 'fz_lim_rr'};
    h = [motor_power; motor_rpm; ltx_eq; lty_eq; BrTh_1; ATD_eq; fz_lim_fl; fz_lim_fr; fz_lim_rl; fz_lim_rr];
    h_lb = [0;     0;    -1;  -1;     -1;  0-1e-3;  0; 0; 0; 0];
    h_ub = [1;     1;     1;   1;      1;  0+1e-3;  1; 1; 1; 1];
end

%-Check number of constraints
if nh ~= length(h) 
    error('Number of path constraints of the OCP is not consistent');
end

% Path constraints
h_eq = Function('h_eq', {x, u, y, pv, p}, {h}, {'x','u','y','pv','p'}, {'h'}); 


%% NLP - Collocation, polynomial coefficients

% Degree of interpolating polynomial
d = OPT_d;

% Collocation points in normalised interval [0,1]
tau = collocation_points(d, 'legendre');

% Collocation linear maps
[C,D,B] = collocation_coeff(tau); %see >>help collocation_coeff (Casadi documentation)

%% NLP - Collocation, discretisation
%Create a discrete set of points for collocation

%-step of discretisation (independent variable)
ds = OPT_ds; %(m)

%-number of grid intervals (the number of points, Xk, is N+1)
N = round(track.s(end)/ds); 

%-value of the independent variable at the discretisation points. 
s_knot = linspace(min(track.s),max(track.s),N+1); 

%-length of the discretisation interval
dsk = diff(s_knot); % Note that the size of s_knot is N+1 whereas for dsk it is N

%-value of the independent variable at the collocation points
s_col = kron(dsk,tau)+kron([0 cumsum(dsk(1:end-1))],ones(1,d)); %value of s at the collocation points (in between grid points)

%-full array of points including knot (grid) points and collocation points
s_full = kron(s_knot(1:end-1), [1 zeros(1,d)]) + reshape([zeros(1,N); reshape(s_col,d,N)],1,[]); %collect all values of the independent variable at the grid points and collocation points
s_full(end+1) = s_knot(end);

% Value of variable parameters (in this case only the curvature)
k_knot = interp1(track.s, track.k, s_knot); 
k_col = interp1(track.s, track.k, s_col);
k_full = interp1(track.s, track.k, s_full);

%-collect values of variable parameters; dimensions: nv by N
pv_knot = [k_knot(:)']; 
pv_col = [k_col(:)'];
pv_full = [k_full(:)'];

%% NLP - initial guesses

%-States
vx_0    = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(1,:))), data.init.x_opt(1,:), linspace(min(track.s),max(track.s),N+1));
vy_0    = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(2,:))), data.init.x_opt(2,:), linspace(min(track.s),max(track.s),N+1));
r_0     = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(3,:))), data.init.x_opt(3,:), linspace(min(track.s),max(track.s),N+1));
n_0     = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(4,:))), data.init.x_opt(4,:), linspace(min(track.s),max(track.s),N+1));
eps_0   = interp1(linspace(min(track.s),max(track.s),length(data.init.x_opt(5,:))), data.init.x_opt(5,:), linspace(min(track.s),max(track.s),N+1));

Om_fl_0 = vx_0/vp.Rw;
Om_fr_0 = vx_0/vp.Rw; 
Om_rl_0 = vx_0/vp.Rw;
Om_rr_0 = vx_0/vp.Rw;

%-Inputs
T_motor_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(1,:))), data.init.u_opt(1,:), linspace(min(track.s),max(track.s),N+1));
T_brake_0 = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(2,:))), data.init.u_opt(2,:), linspace(min(track.s),max(track.s),N+1));
delta_0   = interp1(linspace(min(track.s),max(track.s),length(data.init.u_opt(3,:))), data.init.u_opt(3,:), linspace(min(track.s),max(track.s),N+1));

if vp.ActAero == 1
    activeAeroRW_0 = 0 *ones(1,N+1);                                                 
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
    lty_init = (reshape(0.85*data.init.vehicle.fy_f,1,[]).*sin(reshape(data.init.u_opt(3,:),1,[])) ...
                + reshape(0.85*data.init.vehicle.fy_r,1,[]))*vp.hcg/vp.t;
    lty_0 = interp1(sInit, lty_init, sGrid);
end
clear sInit sGrid lty_init

%- initial guesses
if pt.ATD == 0 && vp.ActAero == 0
    u0 = [T_motor_0; T_brake_0; delta_0]./u_s; 
%   p0 = [0.67; 0.74; 0.46; 1; 1];
    p0 = [0.68; 0.73; 0.46; 1; 0.27];
elseif pt.ATD == 0 && vp.ActAero == 1
    u0 = [T_motor_0; T_brake_0; activeAeroRW_0; delta_0]./u_s;  
%   p0 = [0.677; 0.74; 0.469; 1];
%     p0 = [0.67; 0.74; 0.46; 1];
    p0 = [0.69; 0.75; 0.47; 1];
elseif pt.ATD == 0 && vp.ActAero == 2
    u0 = [T_motor_0; T_brake_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s;
    p0 = [0.66; 0.74; 0.47];
elseif pt.ATD == 0 && vp.ActAero == 3
    u0 = [T_motor_0; T_brake_0; activeAeroFL_0; activeAeroFR_0; activeAeroRW_0; activeAeroTW_0; delta_0]./u_s; 
    p0 = [0.66; 0.74; 0.48];
elseif pt.ATD == 1 && vp.ActAero == 0
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; delta_0]./u_s;       
    p0 = [0.45; 1; 1];
elseif pt.ATD == 1 && vp.ActAero == 1
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroRW_0; delta_0]./u_s;  
    p0 = [0.44; 1];
elseif pt.ATD == 1 && vp.ActAero == 2
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroFW_0; activeAeroRW_0; delta_0]./u_s;
    p0 = [0.46];
elseif pt.ATD == 1 && vp.ActAero == 3
    u0 = [T_motor_0; T_brake_0; ATD_FL_0; ATD_FR_0; ATD_RL_0; ATD_RR_0; activeAeroFL_0; activeAeroFR_0; activeAeroRW_0; activeAeroTW_0; delta_0]./u_s; 
    p0 = [0.45];
end

% Collect initial guesses
x0 = [vx_0; vy_0; r_0; n_0; eps_0; Om_fl_0; Om_fr_0; Om_rl_0; Om_rr_0]./x_s;


xc0 = reshape(kron(x0(:,1:end-1),ones(1,OPT_d)),OPT_d*N*nx,1); %constant interpolation between x0
y0 = [ltx_0; lty_0]./y_s;

%% NLP - formulation

% Decision variables
Xk = SX.sym('Xk', nx,N+1);   % States
Uk = SX.sym('Uk', nu,N+1);   % Inputs
Yk = SX.sym('Yk', ny,N+1);   % Aux variables
Xkj = SX.sym('Xkj', nx,N*OPT_d); % Helper states for the collocation constraints
P = SX.sym('P',np); %Design parameters

% Linear interpolation
duk = diff(Uk')'./repmat(dsk,nu,1); %derivative of the input at each interval (du/ds)
dyk = diff(Yk')'./repmat(dsk,ny,1); %derivative of the aux. variables at each interval (dy/ds)
dxk = diff(Xk')'./repmat(dsk,nx,1);                     %derivative of the input at each interval (du/ds)

% Second derivatives (for regularisarion and minimising oscillations)
duk2 = diff(duk')'; duk2 = [duk2 duk2(:,end)];
dyk2 = diff(dyk')'; dyk2 = [dyk2 dyk2(:,end)]; 
dxk2 = diff(dxk')'; dxk2 = [dxk2 dxk2(:,end)];


% Scaling of the objective function
J_s = 1; %Note that using 's_full(end)/vx_0' here instead of 1 would make the objective function be close to 1;

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
gck = {}; % Collector for collocation constraints
J = 0; % Initialise objective function

for k = 0:N-1
    % Collocation constraints
    Z = [Xk(:,k+1) Xkj(:,d*k+(1:d))]; % Concatenate states
    %-Dynamics
    %%-calculate derivatives of the approximating polynomial at the collocation points
    dPi = Z*C; 
    %%-calculate derivatives of the system at the collocation points
    switch OPT_uinter 
        case 'constant'
            [dXkj, Qk] = f_dyn(Xkj(:,d*k+(1:d)), Uk(:,k+1), Yk(:,k+1), pv_col(:,d*k+(1:d)), P); 
        case 'linear'
            [dXkj, Qk] = f_dyn(Xkj(:,d*k+(1:d)), Uk(:,k+1)+kron(duk(:,k+1),tau), Yk(:,k+1)+kron(dyk(:,k+1),tau), pv_col(:,d*k+(1:d)), P);
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
ghk = h_eq(Xk, Uk, Yk, pv, P); ghk = {ghk(:)};

% Rate of inputs constraints
Sfk = f_sf(Xk,k_knot); % calculate Sf to 'undo' the change of independent variable
duk_t = duk./repmat(Sfk(1:end-1),nu,1); % calculate time derivatives: du/dt = du/ds * 1/sf = du/ds * ds/dt
gduk = {duk_t(:)};
% duk2_t = duk2./repmat(Sfk(1:end-1),nu,1);
% gduk2 = {duk2_t(:)};

%% NLP - define NLP problem
%-decision variables
w = [Xk(:); Uk(:); Yk(:); Xkj(:); P(:)]; % Collect all the decision variables [states; inputs; aux. variables; helper states; parameters]
lbw = [repmat(x_min(:),N+1,1); repmat(u_min(:,1),N+1,1); repmat(y_min(:),N+1,1); repmat(x_min(:),N*d,1); p_min(:)-OPT_e]; % Lower bounds
ubw = [repmat(x_max(:),N+1,1); repmat(u_max(:,1),N+1,1); repmat(y_max(:),N+1,1); repmat(x_max(:),N*d,1); p_max(:)+OPT_e]; % Upper bounds
w0 = [x0(:); u0(:); y0(:); xc0(:); p0(:)]; % Collect initial guesses for the decision variables

%-constraints
g = [gb(:);gck(:);ghk(:);gduk(:); {P(:)}]; g = vertcat(g{:});
lbg = [lbg; zeros((d+1)*N*nx,1);repmat(h_lb,N+1,1);repmat(duk_lb,N,1); p_min(:)]; % Lower bounds [boundary conditions; collocation constraints; path constraints]
ubg = [ubg; zeros((d+1)*N*nx,1);repmat(h_ub,N+1,1);repmat(duk_ub,N,1); p_max(:)]; % Upper bounds 

%-nlp problem
nlp = struct('f', J, 'x', w, 'g', g);

% Create solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

elapsedTime(end+1) = toc - elapsedTime(end);
%% NLP - solve 

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

elapsedTime(end+1) = toc - elapsedTime(end);
%% Postprocessing - Collect NLP variables
%Retrieve decision variables from the solution
global data;

%-Independent variables
data.s_full = s_full;
% data.s_knot = s_knot;
data.k_full = k_full;
% data.k_knot = k_knot;

%-Collect decision variables in numeric array
data.w_opt = full(sol.x);
% The shape of w_opt is w_opt = [Xk(:); Uk(:); Yk(:); Xkj(:)];
% length(Xk) = nx*(N+1);
% length(Uk) = nu*(N+1);
% length(Yk) = ny*(N+1);
% legth(Xkj) = nx*d*N;
%-Note scaling is undone here too
data.x_opt = reshape(data.w_opt(1:nx*(N+1)),nx,N+1).*x_s;
data.u_opt = reshape(data.w_opt(nx*(N+1)+(1:nu*(N+1))),nu,N+1).*u_s;
data.y_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+(1:ny*(N+1))),ny,N+1).*y_s;
data.xc_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+ny*(N+1)+(1:nx*d*N)),nx,N*d).*x_s;
data.p_opt = data.w_opt(nx*(N+1)+nu*(N+1)+ny*(N+1)+nx*d*N+(1:np));

%% Postprocessing - Reconstruct solution
% Collect values of x at all interpolation points 
%States, x_full = [X1 X11...X1j...X1d,..., Xk Xk1...Xkj...Xkd,..., XN XN1..XNj...XNd, XN+1]
data.x_full = kron(data.x_opt(:,1:end-1), [1 zeros(1,d)]) + reshape([zeros(nx,N); reshape(data.xc_opt,nx*d,N)],nx,[]);
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
    f_t_opt = Function('f_t_opt',{Xkj(:,(i*OPT_d)+(1:OPT_d)), Uk(:,i+1), Yk(:,i+1)},{dt_opt{i+1}}); %Create a casadi function to evaluate dt_opt
    dt_opt_val{i+1} = f_t_opt(data.xc_opt(:,i*OPT_d+(1:OPT_d))./x_s,data.u_opt(:,i+1)./u_s, data.y_opt(:,i+1)./y_s); %Evaluate dt_opt at the solution
end
data.t_opt = [0 cumsum(full([dt_opt_val{:}]))]; %Time at each grid point

% Track
data.track0 = track; %Track before discretisation
data.track.s = s_full;
data.track.k = k_full;

%%-calculate track coordinates at grid points
if ~isfield(data.track0,'x') || ~isfield(data.track0,'y')
    [data.track.x, data.track.y] = curv2cart(data.track.s, data.track.k);
else
    data.track.x = interp1(data.track0.s, data.track0.x, data.track.s);
    data.track.y = interp1(data.track0.s, data.track0.y, data.track.s);
end
[data.track.xopt,data.track.yopt] = cartPath(data.track.x,data.track.y, data.x_full(4,:)); %x_full(4) is the distance to the centerline
[data.track.Xl,data.track.Xr] = trackLimits(data.track.x,data.track.y, x_s(4)*x_max(4)*2+2);


% Vehicle
%-aerodynamic forces
f_aeroF = Function('f_aeroF',{x,u,y,pv,p}, {[f_drag; f_lift; f_lift_fl; f_lift_fr; f_lift_rl; f_lift_rr; f_side; f_side_fl; f_side_fr; f_side_rl; f_side_rr]},{'x','u','y','pv','p'},{'Ftyres'});
aeroF = reshape(full(f_aeroF(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),11,1,N+1);
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

%-tyre forces
f_Ftyres = Function('f_Ftyres',{x,u,y,pv,p},{[fx_fl, fy_fl, fz_fl; fx_fr, fy_fr, fz_fr; fx_rl, fy_rl, fz_rl; fx_rr, fy_rr, fz_rr]},{'x','u','y','pv', 'p'},{'Ftyres'});
Ftyres = reshape(full(f_Ftyres(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),4,3,N+1);
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
f_slipAng = Function('f_slipAng',{x,u,y,pv,p},{[sa_fl; sa_fr; sa_rl; sa_rr]},{'x','u','y','pv', 'p'},{'slipAng'});
slipAng = reshape(full(f_slipAng(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),4,1,N+1);
data.vehicle.sa_fl = slipAng(1,1,:);
data.vehicle.sa_fr = slipAng(2,1,:);
data.vehicle.sa_rl = slipAng(3,1,:);
data.vehicle.sa_rr = slipAng(4,1,:);

%-slip ratios
f_slipX = Function('f_slipX',{x,u,y,pv,p},{[sx_fl; sx_fr; sx_rl; sx_rr]},{'x','u','y','pv', 'p'},{'slipX'});
slipX = reshape(full(f_slipX(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),4,1,N+1);
data.vehicle.sx_fl = slipX(1,1,:);
data.vehicle.sx_fr = slipX(2,1,:);
data.vehicle.sx_rl = slipX(3,1,:);
data.vehicle.sx_rr = slipX(4,1,:);

%- e-motor speed
f_OmMotor = Function('f_OmMotor',{x,u,y,pv,p},{[Om_motor]},{'x','u','y','pv','p'},{'Om_motor'});
OmMotor = reshape(full(f_OmMotor(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),1,1,N+1);
data.vehicle.Om_motor = OmMotor(1,1,:);

% -wheel torque
f_Torque = Function('f_Torque',{x,u,y,pv,p},{[T_fl; T_fr; T_rl; T_rr]},{'x','u','y','pv','p'},{'Torque'});
Torque = reshape(full(f_Torque(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),4,1,N+1);
data.vehicle.T_fl = Torque(1,1,:);
data.vehicle.T_fr = Torque(2,1,:);
data.vehicle.T_rl = Torque(3,1,:);
data.vehicle.T_rr = Torque(4,1,:);


% Energy
f_P_motor = Function('f_P_motor',{x,u,y,pv,p},{[P_motor]},{'x','u','y','pv','p'},{'P_motor'});
Pmotor = reshape(full(f_P_motor(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt)),1,1,N+1);
data.vehicle.P_motor = Pmotor(1,1,:).*1e-3;
E_motor(1) = 0;

for i = 1:N
        E_motor(i+1) = E_motor(i) + 0.5*(data.vehicle.P_motor(i) + data.vehicle.P_motor(i+1))*(data.t_opt(i+1) - data.t_opt(i))*2.7778e-4/pt.eff;                     %kWh
end

E_motor = reshape(E_motor,1,1,N+1);
data.vehicle.E_motor = E_motor(1,1,:);

% Constraints
hval = full(h_eq(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot, data.p_opt));
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
for i = 1:length(y)
    aux = y(i); name = erase(aux.name,'_n'); 
    data.sdi.solData.aux.(name) = timeseries(data.y_full(i,:), s_full,'name',name); 
end
for i = 1:length(p)
    aux = p(i); name = erase(aux.name,'_n'); 
    data.sdi.solData.parameters.(name) = timeseries(repmat(data.p_opt(i),length(s_full),1), s_full,'name',name); 
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

%Energy and power
data.sdi.vehicle.P_motor = timeseries(data.vehicle.P_motor, s_knot, 'name', 'P_motor (kW)');
data.sdi.vehicle.E_motor = timeseries(data.vehicle.E_motor, s_knot, 'name', 'E_motor (kWh)');

%-path constraints
for i = 1:length(hnames)
    data.sdi.constraints.(hnames{i}) = timeseries(data.constraints.(hnames{i}), s_knot, 'name', hnames{i});
end

% Import data to SDI
newRun = Simulink.sdi.Run.create;
pstr = [];
for i = 1:length(p)
    aux = p(i);
    pstr = [pstr erase(aux.name,'_n') '=' num2str(data.p_opt(i)) '_'];
end
clear aux
newRun.Name = ['(version)_' 'ds' num2str(OPT_ds) '_' OPT_uinter '_sol:' num2str(data.t_opt(end)) 's_X0=' num2str(Xi(:)') '_Xf=' num2str(Xf(:)') '_ET:' num2str(elapsedTime(3)) 's_' datestr(datetime)];
solutionData = data.sdi.solData;
trackData = data.sdi.track;
vehicleData = data.sdi.vehicle;
pathConstraints = data.sdi.constraints;
add(newRun, 'vars', solutionData, trackData, vehicleData, pathConstraints);
data.sdi.run = newRun;
clear newRun solutionData trackData

% -plot data in SDI
plotSDI;

%%
elapsedTime(end+1) = toc - elapsedTime(end);
disp(' ')
disp('Elapsed time:')
disp(table(elapsedTime(2),elapsedTime(3),elapsedTime(4),'VariableNames',{'Transcription','Solution','Post-processing'}))

disp('Objective function value:')
disp(full(sol.f))

disp('Lap time:')
disp(data.t_opt(end))