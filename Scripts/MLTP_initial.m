% Minimum lap time problem solver
%
% Solve the basic version of the Minimum Lap Time Problem, i.e. find the 
% optimal race line and control inputs to minimise the lap time.


%% Initialisation
% Clear Simulink SDI callback if it exists
global figures
try Simulink.sdi.unregisterCursorCallback(figures.callbackID); end 

% Import functions
import casadi.* %Import casadi framework (needs to be on MATLAB path, otherwise the complete route of the folder must be specified)
assert(exist('casadi.SX','class')==8, ...
    'CasADi is not on the MATLAB path. addpath your casadi-3.x-windows64-matlabXXXX folder first.');

%% Repo path bootstrap - locate the repo from this file, not from the cwd
% Every run()/load()/importfile() below is repo-root-relative and the helper
% functions live in Functions\, so both must be pinned before anything else runs.
% Idempotent: this script is also run() from MLTP.m, which bootstraps too.
repoRoot = fileparts(fileparts(mfilename('fullpath')));    % ...\Scripts -> repo root
addpath(repoRoot, fullfile(repoRoot,'Scripts'), fullfile(repoRoot,'Parameters'));
addpath(genpath(fullfile(repoRoot,'Functions')));          % genpath: PolyfitnTools is double-nested
cd(repoRoot);                                              % repo convention: cwd == repo root
clear repoRoot

% Standalone use: userOpts defines track/vp/pt/OPT_* that this script needs.
% MLTP.m already runs it, so only do it when those are absent.
if ~exist('vp','var') || ~exist('track','var')
    run('userOpts.m');
end

% Start time counter
elapsedTime = 0; tic

run('vehModel_initial.m');

%% Optimal Control Problem - Dynamics and objetive function

% Objective function
L = sf;

% Continuous time dynamics and objective function
f_dyn = Function('f_dyn',{x,u,y,pv},{dx,L},{'x','u','y','pv'},{'dx','L'});

% Change of variable
f_sf = Function('sf',{x,kappa},{sf},{'x','kappa'},{'sf'});

%% Optimal Control Problem - Path Constraints

nh = 6; %number of constraints

%-brake and throttle overlap
BrTh = Tdrive_n*Tbrake_n; 

%-longitudinal load transfer
ltx_eq = ((fx_f*cos(delta) - fy_f*sin(delta) + fx_r + f_drag)*vp.hcg/vp.l - ltx)/ltx_s;

%-Motor power
motor_power = (Om_motor*Tdrive - pt.Pmax) / (Tdrive_s * ((Om_f_s + Om_r_s)/2)*vp.gear);

%-Motor max RPM
motor_rpm = Om_motor - pt.OMmax;

%-friction ellipse, per axle (anisotropic: mux > muy for the Zenvo tyre data;
% the warm-start model has no combined-slip weighting so the ellipse bounds
% the pure-slip forces instead; >= 0 means inside the ellipse)
mu_lim_f = 1 - (fx_f/(mux_f*fz_f))^2 - (fy_f/(muy_f*fz_f))^2;
mu_lim_r = 1 - (fx_r/(mux_r*fz_r))^2 - (fy_r/(muy_r*fz_r))^2;

% Collect constraints
hnames = {'mu_lim_f', 'mu_lim_r', 'motor_power', 'motor_rpm', 'ltx_eq', 'BrTh'};
h = [mu_lim_f; mu_lim_r; motor_power; motor_rpm; ltx_eq; BrTh];
h_lb = [0; 0; -inf;  -inf; 0-OPT_e; 0];
h_ub = [1; 1; 0;  0; 0+OPT_e; inf];

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
pv_col = [k_col(:)'];
pv_full = [k_full(:)'];

%% NLP - initial guesses (constant values)

%-States
vx_0 = vi*ones(1,N+1);
vy_0 = OPT_e*ones(1,N+1); 
r_0 = OPT_e*ones(1,N+1); 
n_0 = OPT_e*ones(1,N+1);  
eps_0 = zeros(1,N+1); 
Om_f_0 = vx_0/vp.Rw; 
Om_r_0 = vx_0/vp.Rw;

%-Inputs
Tdrive_0 = 0.85*pt.Tmax*ones(1,N+1);
Tbrake_0 = 0*ones(1,N+1);
delta_0 = 0*ones(1,N+1);

%-Aux variables
ltx_0 = zeros(ny,N+1);

% Collect initial guesses
x0 = [vx_0; vy_0; r_0; n_0; eps_0; Om_f_0; Om_r_0]./x_s;
u0 = [Tdrive_0; Tbrake_0; delta_0]./u_s;
xc0 = reshape(kron(x0(:,1:end-1),ones(1,OPT_d)),OPT_d*N*nx,1);          %constant interpolation between x0
y0 = [ltx_0]./y_s;

%% NLP - formulation

% Decision variables
Xk = SX.sym('Xk', nx,N+1);                                              % States
Uk = SX.sym('Uk', nu,N+1);                                              % Inputs
Yk = SX.sym('Yk', ny,N+1);                                              % Aux variables
Xkj = SX.sym('Xkj', nx,N*OPT_d);                                        % Helper states for the collocation constraints

% Linear interpolation
duk = diff(Uk')'./repmat(dsk,nu,1);                                     % derivative of the input at each interval (du/ds)
dyk = diff(Yk')'./repmat(dsk,ny,1);                                     % derivative of the aux. variables at each interval (dy/ds)
dxk = diff(Xk')'./repmat(dsk,nx,1);                     %derivative of the input at each interval (du/ds)

% Second derivatives (for regularisarion and minimising oscillations)
duk2 = diff(duk')'; duk2 = [duk2 duk2(:,end)];
dyk2 = diff(dyk')'; dyk2 = [dyk2 dyk2(:,end)]; 
dxk2 = diff(dxk')'; dxk2 = [dxk2 dxk2(:,end)];

% Scaling of the objective function
J_s = 1;                                                                % Note that using 's_full(end)/vx_0' here instead of 1 would make the objective function be close to 1;

%% NLP - formulation, boundary conditions

x0_min = max(x_min, Xi_init./x_s-OPT_e); x0_max = min(x_max, Xi_init./x_s+OPT_e);
xf_min = max(x_min, Xf_init./x_s-OPT_e); xf_max = min(x_max, Xf_init./x_s+OPT_e);

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
    Z = [Xk(:,k+1) Xkj(:,OPT_d*k+(1:OPT_d))];                           % Concatenate states

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
    J = J + Qk*B*dsk(k+1)/J_s + sumsqr(rdu_init.*duk(:,k+1)) + sumsqr(rdu2_init.*duk2(:,k+1)) + sumsqr(rdy.*dyk(:,k+1))  + sumsqr(rdy2.*dyk2(:,k+1));

    % Collect contribution to time of each interval
    dt_opt{k+1} = Qk*B*dsk(k+1); 
end

% Path constraints
ghk = h_eq(Xk, Uk, Yk, pv); ghk = {ghk(:)};

% Rate of inputs constraints
Sfk = f_sf(Xk,k_knot);                                              % calculate Sf to 'undo' the change of independent variable
duk_t = duk./repmat(Sfk(1:end-1),nu,1);                             % calculate time derivatives: du/dt = du/ds * 1/sf = du/ds * ds/dt
gduk = {duk_t(:)};


%% NLP - define NLP problem
%-decision variables
w = [Xk(:); Uk(:); Yk(:); Xkj(:)]; % Collect all the decision variables [states; inputs; aux. variables; helper states]
lbw = [repmat(x_min(:),N+1,1); repmat(u_min(:,1),N+1,1); repmat(y_min(:),N+1,1); repmat(x_min(:),N*OPT_d,1)]; % Lower bounds
ubw = [repmat(x_max(:),N+1,1); repmat(u_max(:,1),N+1,1); repmat(y_max(:),N+1,1); repmat(x_max(:),N*OPT_d,1)]; % Upper bounds
w0 = [x0(:); u0(:); y0(:); xc0(:)]; % Collect initial guesses for the decision variables

%-constraints
g = [gb(:);gck(:);ghk(:);gduk(:)]; g = vertcat(g{:}); %[boundary conditions; collocation constraints; path constraints; rate of inputs]
lbg = [lbg; zeros((OPT_d+1)*N*nx,1);repmat(h_lb,N+1,1);repmat(duk_lb_init,N,1)]; % Lower bounds 
ubg = [ubg; zeros((OPT_d+1)*N*nx,1);repmat(h_ub,N+1,1);repmat(duk_ub_init,N,1)]; % Upper bounds 

%-nlp problem [objective function 'f', decision variables 'x', constraints 'g']
nlp = struct('f', J, 'x', w, 'g', g); 

% Create solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

elapsedTime(end+1) = toc - elapsedTime(end);
%% NLP - solve

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                       'lbg', lbg, 'ubg', ubg);

elapsedTime(end+1) = toc - elapsedTime(end);
%% Postprocessing - Collect NLP variables
%Retrieve decision variables from the solution
global data;

%-Independent variables
data.init.s_full = s_full;
% data.s_knot = s_knot;
data.init.k_full = k_full;
% data.k_knot = k_knot;

%-Collect decision variables in numeric array
data.init.w_opt = full(sol.x);
% The shape of w_opt is w_opt = [Xk(:); Uk(:); Xkj(:)];
% length(Xk) = nx*(N+1);
% length(Uk) = nu*(N+1);
% legth(Xkj) = nx*d*N;
%-Note scaling is undone here too
data.init.x_opt = reshape(data.init.w_opt(1:nx*(N+1)),nx,N+1).*x_s;
data.init.u_opt = reshape(data.init.w_opt(nx*(N+1)+(1:nu*(N+1))),nu,N+1).*u_s;
data.init.y_opt = reshape(data.init.w_opt(nx*(N+1)+nu*(N+1)+(1:ny*(N+1))),ny,N+1).*y_s;
data.init.xc_opt = reshape(data.init.w_opt(nx*(N+1)+nu*(N+1)+ny*(N+1)+1:end),nx,N*OPT_d).*x_s;

%% Postprocessing - Reconstruct solution
% Collect values of x at all interpolation points 
%States, x_full = [X1 X11...X1j...X1d,..., Xk Xk1...Xkj...Xkd,..., XN XN1..XNj...XNd, XN+1]
data.init.x_full = kron(data.init.x_opt(:,1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(nx,N); reshape(data.init.xc_opt,nx*OPT_d,N)],nx,[]);
data.init.x_full(:,end+1) = data.init.x_opt(:,end);
%Inputs and Aux. variables
switch OPT_uinter
    case 'constant'
        data.init.u_full = interp1(s_knot', data.init.u_opt', s_full, 'previous')'; % for constant inputs
        data.init.y_full = reshape(interp1(s_knot', data.init.y_opt', s_full, 'previous')',ny,[]); % for constant inputs
    case 'linear'
        data.init.u_full = interp1(s_knot, data.init.u_opt', s_full, 'linear')'; % for linear inputs
        data.init.y_full = reshape(interp1(s_knot', data.init.y_opt', s_full, 'previous')',ny,[]); % for constant inputs
    otherwise
        error('Choose "linear" or "constant" for the interpolation method of the inputs');
end

%% Postprocessing - Collect additional data

% Time
%-time at grid points
dt_opt_val = cell(N,1);
for i=0:N-1
    f_t_opt = Function('f_t_opt',{Xkj(:,(i*OPT_d)+(1:OPT_d)), Uk(:,i+1)},{dt_opt{i+1}}); %Create a casadi function to evaluate dt_opt
    dt_opt_val{i+1} = f_t_opt(data.init.xc_opt(:,i*OPT_d+(1:OPT_d))./x_s,data.init.u_opt(:,i+1)./u_s); %Evaluate dt_opt at the solution
end
data.init.t_opt = [0 cumsum(full([dt_opt_val{:}]))]; %Time at each grid point

% Track
data.init.track0 = track; %Track before discretisation
data.init.track.s = s_full;
data.init.track.k = k_full;
%%-calculate track coordinates at grid points
if ~isfield(data.init.track0,'x') || ~isfield(data.init.track0,'y')
    [data.init.track.x, data.init.track.y] = curv2cart(data.init.track.s, data.init.track.k);
else
    data.init.track.x = interp1(data.init.track0.s, data.init.track0.x, data.init.track.s);
    data.init.track.y = interp1(data.init.track0.s, data.init.track0.y, data.init.track.s);
end
[data.init.track.xopt,data.init.track.yopt] = cartPath(data.init.track.x,data.init.track.y, data.init.x_full(4,:)); %x_full(4) is the distance to the centerline
[data.init.track.Xl,data.init.track.Xr] = trackLimits(data.init.track.x,data.init.track.y, x_s(4)*x_max(4)*2+2);

% Vehicle
%-aerodynamic forces
f_aeroF = Function('f_aeroF',{x,u,y,pv}, {[f_drag; f_lift]},{'x','u','y','pv'},{'aeroF'});
aeroF = reshape(full(f_aeroF(data.init.x_opt./x_s, data.init.u_opt./u_s, data.init.y_opt./y_s, pv_knot)),2,1,N+1);
data.init.vehicle.f_drag = aeroF(1,1,:);
data.init.vehicle.f_lift = aeroF(2,1,:);
%-tyre forces
f_Ftyres = Function('f_Ftyres',{x,u,y,pv},{[fx_f, fy_f, fz_f; fx_r, fy_r, fz_r]},{'x','u','y','pv'},{'Ftyres'});
Ftyres = reshape(full(f_Ftyres(data.init.x_opt./x_s, data.init.u_opt./u_s, data.init.y_opt./y_s, pv_knot)),2,3,N+1);
data.init.vehicle.fx_f = Ftyres(1,1,:);
data.init.vehicle.fy_f = Ftyres(1,2,:);
data.init.vehicle.fz_f = Ftyres(1,3,:);
data.init.vehicle.fx_r = Ftyres(2,1,:);
data.init.vehicle.fy_r = Ftyres(2,2,:);
data.init.vehicle.fz_r = Ftyres(2,3,:);
%-slip angles
% f_slipAng = Function('f_slipAng',{x,u,pv},{[sa_f; sa_r]},{'x','u','pv'},{'slipAng'});
% slipAng = reshape(full(f_slipAng(data.x_opt./x_s, data.u_opt./u_s, pv_knot)),2,1,N+1);
% data.vehicle.sa_f = slipAng(1,1,:);
% data.vehicle.sa_r = slipAng(2,1,:);
% %-slip ratios
% f_slipX = Function('f_slipX',{x,u,pv},{[sx_f; sx_r]},{'x','u','pv'},{'slipX'});
% slipX = reshape(full(f_slipX(data.x_opt./x_s, data.u_opt./u_s, pv_knot)),2,1,N+1);
% data.vehicle.sx_f = slipX(1,1,:);
% data.vehicle.sx_r = slipX(2,1,:);

% Constraints
hval = full(h_eq(data.init.x_opt./x_s, data.init.u_opt./u_s, data.init.y_opt./y_s, pv_knot));
for i=1:length(hnames)
    data.init.constraints.(hnames{i}) = hval(i,:);
end

%% Cache this warm start so MLTP.m can reuse it (see Functions\latestInit.m)
initDir = fullfile('Data', circuit, 'initialisation');
if ~exist(initDir, 'dir'), mkdir(initDir); end
% AFWd's model-version token is its own (zenvoMF52fw3n), not a bump of the RW
% one every other ActiveRW variant shares - token MUST match Functions\latestInit.m's
% selection or a cache written here becomes permanently unreachable by name.
initToken = 'zenvoMF52rw4n';
if strcmp(vp.aeroSetting, 'AFWd'), initToken = 'zenvoMF52fw3n'; end
if strcmp(vp.aeroSetting, 'ARFWd'), initToken = 'zenvoMF52arfw'; end   % must match Functions\latestInit.m
if strcmp(vp.aeroSetting, 'ARFWr'), initToken = 'zenvoMF52arfwr'; end   % must match Functions\latestInit.m
initFile = fullfile(initDir, sprintf('init_%s_%s_%s_%s.mat', circuit, vp.aeroSetting, initToken, ...
                    char(datetime('now','Format','yyyyMMdd_HHmmss'))));   % token must match Functions\latestInit.m
save(initFile, 'data');
fprintf('MLTP_initial: warm start cached -> %s\n', initFile);
clear initDir initFile

%% Postprocessing - Log data with Simulation Data Inspector
% 
% % Initialise figures and SDI
% Simulink.sdi.clear; %clears all data in SDI **USE CAREFULLY**
%-define container for external figures
% global figures;
% figures = struct();
% %-sync data cursors between SDI and external figures
% figures.callbackID = Simulink.sdi.registerCursorCallback(@(t1,t2)onCursorMove(t1,t2));
% %-initialise external figures
% %%-track
% figures.track.fig = figure('Name','Circuit Map'); clf
% figures.track.ax = axes;
% updateTrackPlot(nan,nan);
% 
% % Create time series objects for sdi
% %-states, inputs and aux. variables
% for i = 1:length(x)
%     aux = x(i); name = erase(aux.name,'_n'); %get name of variable
%     data.sdi.solData.states.(name) = timeseries(data.x_full(i,:), s_full,'name',name); %create timeseries object
% end
% for i = 1:length(u)
%     aux = u(i); name = erase(aux.name,'_n');
%     data.sdi.solData.inputs.(name) = timeseries(data.u_full(i,:), s_full,'name',name);
% end
% data.sdi.solData.time = timeseries(data.t_opt, s_knot, 'name', 'time');
% clear aux name
% %-track data
% data.sdi.track.s = timeseries(s_full,s_full, 'name', 's');
% data.sdi.track.k = timeseries(k_full,s_full, 'name', 'k');
% data.sdi.track.x = timeseries(data.track.x,s_full, 'name', 'x');
% data.sdi.track.y = timeseries(data.track.y,s_full, 'name', 'y');
% %-vehicle data
% %%-parameters (just as a way to store the values that were used for each simulation)
% fn = fieldnames(vp); %get names of vehicle parameters
% for i = 1:length(fn)
%     if isa(vp.(fn{i}),'double')
%         data.sdi.vehicle.params.(fn{i}) =  timeseries(repmat(vp.(fn{i}),1,length(s_knot)),s_knot);
%     end
% end
% clear fn
% %%-aerodynamic forces
% data.sdi.vehicle.f_drag = timeseries(data.vehicle.f_drag, s_knot, 'name', 'f_drag');
% data.sdi.vehicle.f_lift = timeseries(data.vehicle.f_lift, s_knot, 'name', 'f_lift');
% %%-tyre forces
% data.sdi.vehicle.fx_f = timeseries(data.vehicle.fx_f, s_knot, 'name', 'fx_f');
% data.sdi.vehicle.fy_f = timeseries(data.vehicle.fy_f, s_knot, 'name', 'fy_f');
% data.sdi.vehicle.fz_f = timeseries(data.vehicle.fz_f, s_knot, 'name', 'fz_f');
% data.sdi.vehicle.fx_r = timeseries(data.vehicle.fx_r, s_knot, 'name', 'fx_r');
% data.sdi.vehicle.fy_r = timeseries(data.vehicle.fy_r, s_knot, 'name', 'fy_r');
% data.sdi.vehicle.fz_r = timeseries(data.vehicle.fz_r, s_knot, 'name', 'fz_r');
% %%-slip angles
% data.sdi.vehicle.sa_f = timeseries(data.vehicle.sa_f, s_knot, 'name', 'sa_f');
% data.sdi.vehicle.sa_r = timeseries(data.vehicle.sa_r, s_knot, 'name', 'sa_r');
% %%-slip ratios
% data.sdi.vehicle.sx_f = timeseries(data.vehicle.sx_f, s_knot, 'name', 'sx_f');
% data.sdi.vehicle.sx_r = timeseries(data.vehicle.sx_r, s_knot, 'name', 'sx_r');
% %-path constraints
% for i = 1:length(hnames)
%     data.sdi.constraints.(hnames{i}) = timeseries(data.constraints.(hnames{i}), s_knot, 'name', hnames{i});
% end

%%
elapsedTime(end+1) = toc - elapsedTime(end);
disp(' ')
disp('Elapsed time:')
disp(table(elapsedTime(2),elapsedTime(3),elapsedTime(4),'VariableNames',{'Transcription initialisation','Solution','Post-processing'}))
data.init.duration = elapsedTime;

disp('Lap Time Initialisation:')
disp(data.init.t_opt(end))
