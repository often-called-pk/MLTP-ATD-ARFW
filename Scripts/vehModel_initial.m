% vehModel.m - Vehicle Model 
%
% Define vehicle model symbolic variables and equations using Casadi's 
% symbolic variables.

import casadi.*

%% Vehicle model (A) - state variables
nx = 7; % number of state variables

% longitudinal velocity [m/s]
vx_n = SX.sym('vx_n');
vx_s = 100;
vx = vx_s * vx_n;

% lateral velocity [m/s]
vy_n = SX.sym('vy_n');
vy_s = 10;
vy = vy_s * vy_n;

% yaw rate [rad/s]
r_n = SX.sym('yawrate_n');
r_s = 1;
r = r_s * r_n;

% lateral distance to centreline [m] - left of centreline => n > 0; right => n < 0
n_n = SX.sym('n_n');
n_s = 5;
n = n_s * n_n;

% angle to centreline tangent direction [rad]
eps_n = SX.sym('eps_n');
eps_s = 1;
eps = eps_s * eps_n;

% angular velocity front tyre [rad/s]
Om_f_n = SX.sym('Om_f_n');
Om_f_s = vx_s/vp.Rw;
Om_f = Om_f_s * Om_f_n;

% angular velocity rear tyre [rad/s]
Om_r_n = SX.sym('Om_r_n');
Om_r_s = vx_s/vp.Rw;
Om_r = Om_r_s * Om_r_n;

%%-State limits
vx_lim = 1/vx_s * [OPT_e pt.Vmax];
vy_lim = 1/vy_s * [-10 10];
r_lim = 1/r_s * [-pi/2 pi/2];
n_lim = 1/n_s * [-4 4]; 
eps_lim = 1/eps_s * [-pi/4 pi/4];
Om_f_lim = 1/Om_f_s * [0 pt.Vmax/vp.Rw];
Om_r_lim = 1/Om_r_s * [0 pt.Vmax/vp.Rw];

% scaling factors
x_s = [vx_s; vy_s; r_s; n_s; eps_s; Om_f_s; Om_r_s];  

% states vector (scaled)
x = [vx_n; vy_n; r_n; n_n; eps_n; Om_f_n; Om_r_n]; 

% limits
x_lim = [vx_lim; vy_lim; r_lim; n_lim; eps_lim; Om_f_lim; Om_r_lim];
x_min = x_lim(:,1);
x_max = x_lim(:,2);

%Check number of states
if nx ~= length(x) 
    error('Number of states is not consistent');
end

%% Vehicle model (A) - control variables (inputs)
% number of control variables
nu = 3;

% driving torque (Nm)
Tdrive_n = SX.sym('T_drive_n');
Tdrive_s = pt.Tmax;
Tdrive = Tdrive_s * Tdrive_n;

% braking torque [Nm]
Tbrake_n = SX.sym('T_brake_n');
Tbrake_s = vp.Tbrake_max;
Tbrake = Tbrake_s * Tbrake_n;

% steer angle [rad]
delta_n = SX.sym('delta_n');
delta_s = pi/8;
delta = delta_s * delta_n;

%%-Input limits
Tdrive_lim = 1/Tdrive_s * [0 pt.Tmax];
Tbrake_lim = 1/Tbrake_s * [-vp.Tbrake_max 0];
delta_lim = 1/delta_s * [-pi/4 pi/4];

% scaling factors for inputs
u_s = [Tdrive_s; Tbrake_s; delta_s];

% inputs vector (scaled)
u = [Tdrive_n; Tbrake_n; delta_n];

% input limits
u_lim = [Tdrive_lim; Tbrake_lim; delta_lim];
u_min = u_lim(:,1);
u_max = u_lim(:,2);

%Check number of inputs
if nu ~= length(u) 
    error('Number of inputs is not consistent');
end

% %Constraints on the rate of inputs
duk_ub_init = duk_ub_init./u_s;
duk_lb_init = duk_lb_init./u_s;

%% Vehicle model (A) - additional variables
%Additional decision variables of the NLP other than the states and inputs

ny = 1; % Number of aux variables

% longitudinal load transfer [N]
ltx_n = SX.sym('loadTransferX_n');
ltx_s = vp.m*vp.g*vp.hcg/vp.l;
ltx = ltx_s * ltx_n;

% scaling factors
y_s = [ltx_s]; 
% aux. variables vector (scaled)
y = [ltx_n]; 
% aux. variables limits
ltx_lim = 1/ltx_s * [-vp.m*vp.g vp.m*vp.g];

y_lim = [ltx_lim];
y_min = y_lim(:,1);
y_max = y_lim(:,2);

%Check number of inputs
if ny ~= length(y) 
    error('Number of aux. variables is not consistent');
end

%% Vechicle model (B) - variables
%Symbolic variables that are not decision variables of the NLP - defined as such because their value changes (it is like a variable value parameter)

kappa = SX.sym('kappa'); % kappa > 0 for left turns

pv = [kappa]; % collect variable parameters

%% Vehicle model (B) - equations
%Calculate variables of the system other than the states and inputs

%-aerodynamic forces [N] - initialise with max aero settings
f_drag = 0.5*vp.rho*vp.A*vx^2*max(vp.Cd);
f_lift = 0.5*vp.rho*vp.A*vx^2*max(vp.Cl);

% Tyres
%%-slip angles [rad]
sa_f = delta - atan((vp.l_f*r+vy)/vx);
sa_r = atan((vp.l_r*r-vy)/vx);

%%-slip ratio
v_f = sqrt((vy+vp.l_f*r)^2 + vx^2); 
v_r = sqrt((vy-vp.l_r*r)^2 + vx^2);
v_fx = v_f*cos(sa_f); 
v_rx = v_r*cos(sa_r);
sx_f = (vp.Rw*Om_f - v_fx)/v_fx;
sx_r = (vp.Rw*Om_r - v_rx)/v_rx;

%-forces
%%-fz, vertical tyre forces [N] (Longitudinal load transfer: [Acceleration +; Brake -])
fz_f = (vp.Wfl0 + vp.Wfr0) - ltx  + f_lift*0.45;   
fz_r = (vp.Wrl0 + vp.Wrr0) + ltx  + f_lift*0.55;

f_roll_f = vp.tyre_f.qsy1*fz_f;
f_roll_r = vp.tyre_r.qsy1*fz_r;

% per-axle Zenvo Pacejka sets, axle-lumped (2x nominal vertical force because
% the two tyres of an axle are combined). Pure slip only - the warm-start
% model keeps a friction-ellipse path constraint instead of combined-slip
% weighting; the load-sensitivity law matches vehModel.m exactly.
dfz_f = (fz_f - 2*vp.tyre_f.Fz0*vp.Fz0_shift_f)/(2*vp.tyre_f.Fz0*vp.Fz0_shift_f);
dfz_r = (fz_r - 2*vp.tyre_r.Fz0*vp.Fz0_shift_r)/(2*vp.tyre_r.Fz0*vp.Fz0_shift_r);
mux_f = vp.tyre_f.pDx1 + vp.tyre_f.pDx2*dfz_f;
muy_f = vp.tyre_f.pDy1 + vp.tyre_f.pDy2*dfz_f;
mux_r = vp.tyre_r.pDx1 + vp.tyre_r.pDx2*dfz_r;
muy_r = vp.tyre_r.pDy1 + vp.tyre_r.pDy2*dfz_r;

%%-fx, longitudinal tire forces  [N]
%·magic formula parameters
Dx_f = mux_f*fz_f;
Dx_r = mux_r*fz_r;

%%·Fx(sx)
fx_f = Dx_f*sin(vp.tyre_f.Cx*atan(vp.tyre_f.Bx*sx_f-vp.tyre_f.Ex*(vp.tyre_f.Bx*sx_f-atan(vp.tyre_f.Bx*sx_f))));
fx_r = Dx_r*sin(vp.tyre_r.Cx*atan(vp.tyre_r.Bx*sx_r-vp.tyre_r.Ex*(vp.tyre_r.Bx*sx_r-atan(vp.tyre_r.Bx*sx_r))));

%%-fy, lateral tyre forces [N]
%·magic formula parameters
Dy_f = muy_f*fz_f;
Dy_r = muy_r*fz_r;

%%-Fy(sa)
fy_f = Dy_f*sin(vp.tyre_f.Cy*atan(vp.tyre_f.By*sa_f-vp.tyre_f.Ey*(vp.tyre_f.By*sa_f-atan(vp.tyre_f.By*sa_f))));
fy_r = Dy_r*sin(vp.tyre_r.Cy*atan(vp.tyre_r.By*sa_r-vp.tyre_r.Ey*(vp.tyre_r.By*sa_r-atan(vp.tyre_r.By*sa_r))));

%Wheel torque 
T_f = vp.gear*Tdrive*(1-vp.Tdist) + 2*Tbrake*vp.brkB; 
T_r = vp.gear*Tdrive*vp.Tdist + 2*Tbrake*(1-vp.brkB);

Om_motor = (Om_f)/2*vp.gear + (Om_r)/2*vp.gear;

% power powertrain
P_motor = Tdrive*Om_motor;

% Change of independent variable
sf = (1-n*kappa)/(vx*cos(eps)-vy*sin(eps));

%% Vehicle model (B) - state derivatives
%Define derivatives of the state-space model

% State derivatives equations
% dvx   = (fx_r + fx_f*cos(delta) - fy_f*sin(delta) + vp.m*vy*r - f_drag  - f_roll_f - f_roll_r  )*sf/vp.m;
% dvy   = (fy_r + fy_f*cos(delta) + fx_f*sin(delta) - vp.m*vx*r             )*sf/vp.m;
% dr    = (fy_f*cos(delta)*vp.l_f + fx_f*sin(delta)*vp.l_f - fy_r*vp.l_r)*sf/vp.I_z;
% dn    = (vx*sin(eps) + vy*cos(eps))*sf;
% deps  = sf*r-kappa;
% dOm_f = sf * (T_f - fx_f*vp.Rw)/vp.Jw;
% dOm_r = sf * (T_r - fx_r*vp.Rw)/vp.Jw;

dvx   = (fx_r + fx_f*cos(delta) - 0.85*fy_f*sin(delta) + vp.m*vy*r - f_drag  - f_roll_f - f_roll_r  )*sf/vp.m;
dvy   = (0.85*fy_r + 0.85*fy_f*cos(delta) + fx_f*sin(delta) - vp.m*vx*r             )*sf/vp.m;
dr    = (0.85*fy_f*cos(delta)*vp.l_f + fx_f*sin(delta)*vp.l_f - 0.85*fy_r*vp.l_r)*sf/vp.I_z;
dn    = (vx*sin(eps) + vy*cos(eps))*sf;
deps  = sf*r-kappa;
dOm_f = sf * (T_f - fx_f*vp.Rw)/vp.Jw;
dOm_r = sf * (T_r - fx_r*vp.Rw)/vp.Jw;


% State derivatives vector (scaled to match the scaled states vector)
dx = [dvx; dvy; dr; dn; deps; dOm_f; dOm_r]./x_s;
