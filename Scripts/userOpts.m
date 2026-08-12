% userOpts.m - User Options 

% Define user options for the optimisation problem and collocation method

% - Select aerodynamic configuration and torque distribution configuration

% - Select track data: it can be loaded from a matlab file containing at least two arrays of data for the distance 's' and the curvature 'k'. 
%   Optionally, it can also contain cartesian coordinates 'x' and 'y' 
%   (these are used for the plots of the track in the postprocessing, but the information for the optimisation problem is still taken from 's' and 'k'). 
%   Alternatively, the 's' and 'k' arrays can be defined here directly (three examples are provided).
%   Note that when using track data, raw data should be checked and preprocessed as required to avoid noisy signals, which can be detrimental for the collocation method.

% - Optimisation problem and collocation:
%   - Set initial and final conditions: set as nan to let the solver find the optimal value. (Use carefully since not every combination of boundary conditions is feasable).
%   - Collocation options
%   - Solver options
%   - Limits for the rate of inputs
%   - Regularisation factors

%% Selection of aerodynamic and torque distribution configuration:
% Options:
% - 'Static':   fixed front and rear wing angle
% - 'ActiveRW': fixed front wing angle and active rear wing control
% - 'Active':   active front and rear wing control
% - 'AALB':     active asymmetrical front and rear wing control (split front wing & tilt rear wing)

% --- run-config override (see runComparisonBatch.m). No-op when absent. ---
% PINNED TO THIS CHECKOUT'S ROOT, deliberately. A bare
% exist('runOverride.mat','file')/load('runOverride.mat') resolves through the WHOLE
% MATLAB search path, so a stray runOverride.mat left in another checkout (e.g. the
% main repo, which is on the machine's saved path) silently hijacks every run started
% from a worktree. Anchoring on this file's own location - NOT on pwd - is what makes
% that impossible while staying correct wherever the caller happens to be: MATLAB's
% run() cd's into the target's folder whenever the name it is given has a directory
% component, so pwd is not reliably the repo root inside a run()-invoked script.
optsOvrFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'runOverride.mat');   % ...\Scripts -> repo root
ovr = struct; if isfile(optsOvrFile), ovr = load(optsOvrFile); end
clear optsOvrFile

AeroConfig = getfielddef(ovr,'AeroConfig','Static');                                                % select aerodynamic configuration 'Static', 'ActiveRW', 'Active', or 'AALB'
ATD        = getfielddef(ovr,'ATD','Off');                                                          % select active torque distribution 'On' or 'Off'

switch AeroConfig
    case 'Static'                                                                                   % static aero configuration -> front wing & rear wing AoA's determined in vehParams
        vp.ActAero = 0;
    case 'ActiveRW'                                                                                % active aero configuration -> active rear wing AoA | static front wing AoA determined in vehParams
        vp.ActAero = 1;
    case 'Active'                                                                                   % active aero configuration -> active symmetric front and rear wing AoA
        vp.ActAero = 2;
    case 'AALB'                                                                                     % active aerodynamic load balancing configuration -> front wing AoA's & rear wing AoA/Tilt actively controlled
        vp.ActAero = 3;
end

switch ATD
    case 'Off'                                                                                      % active torque distribution OFF (4WD)
        pt.ATD = 0;
    case 'On'                                                                                       % active torque distribution ON
        pt.ATD = 1;
end

% ATD BRAKE SPLIT - audit knob, default 'Free' = the shipped behaviour.
%   'Free'  (default) the ATD fractions multiply BOTH drive and brake torque, so
%           the optimiser schedules brake bias per corner (measured on BCN/Mid:
%           front brake share swings 0.323-0.799 against AWD's fixed 0.65).
%   'Fixed' the ATD fractions multiply DRIVE torque only; brake torque is split
%           by the fixed vp.brkB exactly as in the AWD model.
% Why it exists: ATD is 4.5 s/lap faster than AWD on BCN, and free brake-bias
% scheduling is a capability a real torque-vectoring driveline does NOT have
% (it needs independent brake-by-wire calipers). This flag isolates that effect
% from genuine drive-torque vectoring. See docs/atd-audit.md.
ATDBrake = getfielddef(ovr,'ATDBrake','Free');
switch ATDBrake
    case 'Free',  vp.atdBrakeFixed = 0;
    case 'Fixed', vp.atdBrakeFixed = 1;
    otherwise
        error('userOpts:ATDBrake', 'unknown ATDBrake ''%s'' (expected Free | Fixed)', ATDBrake);
end

%% Rear-wing schedule / discrete study (ActiveRW only)  -- added 2026-07-28
% docs/plans/2026-07-28-discrete-rw-mandate.md. BOTH FLAGS ARE OFF BY DEFAULT and
% every consumer (Scripts\MLTP.m, Parameters\vehParams.m) is guarded on them, so a
% default run is byte-for-byte the continuous ActiveRW problem that solved BCN in
% 107.228 s - no extra decision variables, no extra constraint rows, no extra
% objective terms. Verified by counting numel(w)/numel(g) before and after.
%
% This block MUST stay AHEAD of the Parameters\vehParams.m call below: vp.aeroSetting
% - and with it the init-cache token, the apex CSV name and the solutions\ folder -
% is keyed off vp.rwMandate there ('ARW' / 'ARWv').
%
% - RWMandate  'Off'      -> vp.rwMandate = 0  no mandate (plain continuous ActiveRW)
%              'Velocity' -> vp.rwMandate = 3  SPEED-SCHEDULED wing + brake trigger:
%                            low drag (-10 deg) on the straights, high downforce
%                            (+10 deg) at corner speeds, +15 deg under braking. This
%                            is the production speed-lookup law, and it is TWO-SIDED -
%                            the wing is pinned to the schedule rather than merely
%                            floored by it, so the reported lap time is the cost of
%                            giving up free control of the wing. Single airbrake
%                            setting by design: +20 deg is absent from this variant.
%                            Applied as a SEPARATE constraint block on the assembled
%                            control matrix in MLTP.m, deliberately NOT a row of `h`
%                            (nh stays 9/10).
% - RWDiscrete 'Off'      -> vp.rwSnap = 0    alpha stays smoothly continuous. This is
%                            the ONLY accepted value.
%              'Snap'     -> RETIRED 2026-08-04, now an ERROR. It added a multi-well
%                            objective penalty pulling alpha towards the named
%                            settings; that term went with the mandate study and the
%                            velocity schedule above replaced the whole idea.
RWMandate  = getfielddef(ovr,'RWMandate','Off');
RWDiscrete = getfielddef(ovr,'RWDiscrete','Off');

% Braking (1) and ForceOpt (2) were the discrete/mandated rear-wing study, retired
% 2026-08-04. Their value 3 is KEPT for Velocity rather than renumbered to 1: it is
% written into every archived ARWv run's data.rwVel and into runOverride.mat, and
% renumbering would make an archive read back as a different variant.
switch RWMandate
    case 'Off',       vp.rwMandate = 0;
    case 'Velocity',  vp.rwMandate = 3;
    case 'Discrete',  vp.rwMandate = 4;
    case 'FrontWing', vp.rwMandate = 5;
    case 'Combined',  vp.rwMandate = 6;   % ARFWd: RW control + FW-only flap (nu +1)
    case 'Reactive',  vp.rwMandate = 7;   % ARFWr: reactive dual-wing schedule (nu +1)
    otherwise
        error('userOpts:RWMandate', ...
            'unknown RWMandate ''%s'' (expected Off | Velocity | Discrete | FrontWing | Combined | Reactive)', RWMandate);
end

% 'Snap' is REJECTED, not accepted-and-ignored. The snap penalty's only consumer was
% the objective term in MLTP.m, deleted 2026-08-04 with the mandate study, so a run
% asking for it would silently solve an UNPENALISED lap - and vehParams.m keeps
% rwSnap out of vp.aeroSetting by design (see its note at ~:174), so that lap would
% be written to the SAME init cache, apex CSV and solutions\ folder as a plain ARW
% run and be indistinguishable from one. A silent wrong answer is worse than a stop.
switch RWDiscrete
    case 'Off',  vp.rwSnap = 0;
    case 'Snap'
        error('userOpts:RWDiscreteRetired', ...
            ['RWDiscrete = ''Snap'' was retired 2026-08-04 along with the discrete/' ...
             'mandated rear-wing study: nothing in MLTP.m consumes vp.rwSnap any more, ' ...
             'so this run would solve an UNPENALISED lap and write it over a plain ARW ' ...
             'run''s init cache / apex CSV / solutions folder. What replaced it is ' ...
             'RWMandate = ''Velocity'' (vp.rwMandate = 3), which PINS alpha to a speed ' ...
             'schedule outright instead of nudging it towards the named settings. ' ...
             'Set RWDiscrete = ''Off''.']);
    otherwise
        error('userOpts:RWDiscrete', ...
            ['unknown RWDiscrete ''%s'' (expected Off; ''Snap'' was retired ' ...
             '2026-08-04)'], RWDiscrete);
end

% Both act on the continuous rear-wing control, so they are only defined for
% AeroConfig = 'ActiveRW'. Fail loudly rather than silently ignore them.
if vp.ActAero ~= 1 && (vp.rwMandate > 0 || vp.rwSnap == 1)
    error('userOpts:rwStudyNeedsActiveRW', ...
        ['RWMandate/RWDiscrete act on the continuous rear-wing angle and are only ' ...
         'defined for AeroConfig = ''ActiveRW'' (got ''%s''). Set both to ''Off''.'], AeroConfig);
end

% Snap-penalty settings, RETAINED THOUGH THE PENALTY ITSELF IS GONE (retired
% 2026-08-04; RWDiscrete = 'Snap' now errors above, and MLTP.m no longer builds the
% objective term). Three things still read these fields and would break if they
% vanished: Validation\validateRWVelocity.m asserts on rwSnapOpts.settings (it is the
% list of named wing settings this variant recognises, +20 deg deliberately absent),
% runComparisonBatch.m's config check compares vp.rwSnapRho, and archived runs carry
% both. rwSnapRho is therefore a recorded 0, no longer a homotopy knob.
% The wells/width live in a STRUCT on purpose: the SDI bookkeeping loop in MLTP.m
% turns every scalar double field of vp into a timeseries on the knot grid, which a
% 1x4 double field would break.
% vp.rwSnap is now always 0, but the FIELD must keep existing - MLTP.m's
% `vp.ActAero == 1 && vp.rwSnap == 1` evaluates its right-hand side on every
% ActiveRW run.
rwSnapSettingsDef = [-10 0 10 15];
vp.rwSnapRho  = getfielddef(ovr,'rwSnapRho', 0);                                                % penalty weight (s-equivalent)
vp.rwSnapOpts = struct( ...
    'settings', getfielddef(ovr,'rwSnapSettings', rwSnapSettingsDef), ...                       % Low/Mid/High/RWp15 (deg)
    'sigma',    getfielddef(ovr,'rwSnapSigma',    1.8));                                        % well half-width (deg)

% Sharpness of the soft extrema MLTP.m uses to open the velocity law's band across a
% transition (weighted-average form; see the rwVelKappa note below).
vp.rwVelBeta = getfielddef(ovr,'rwVelBeta', 4);                                                 % smooth-max sharpness (1/deg) - see MLTP.m

% Brake-trigger options for the velocity schedule. zeroTrim is a deliberate departure
% from the retired rwMandateTarget's own default (false): without it every non-braking
% knot is pulled ~+1.9 deg toward the airbrake. aLight/aHard are gone with the function
% that consumed them.
vp.rwVelBrakeOpts = struct('zeroTrim', true);

% Velocity schedule (Functions\rwVelocityTarget.m), mode 3 only. A THREE-LEVEL
% staircase in speed - all three static downforce settings, not just the extremes:
%   +10 deg (corner) -> 0 deg (mid) -> -10 deg (straight), plus +15 deg on the brake
%   trigger. The switch speeds are MEASURED, not guessed - on the solved free-ARW
%   BCN/AWD lap, restricted to OFF-BRAKE knots (where this schedule is what governs),
%   the optimum's own median speed is 27.3 m/s where it sits above +5 deg, 47.9 m/s
%   where it sits within +-2.5 deg of zero, and 78.3 m/s where it sits below -5 deg.
%   v1/v2 are placed midway between adjacent medians.
% w1/w2 = 3.5 m/s is squeezed from both sides and both bounds are checked, not assumed:
%   LOWER by the actuator - each switch moves 10 deg, so the demand is
%   10*|ax|/(2w) = 36.0 deg/s at the solved |ax|max = 25.19, against c.ub.RW = 25;
%   the hard floor is w >= 5.04. Note 36.0 is still BELOW the free continuous
%   optimum's own peak actuator usage on the same lap (38.3 deg/s), so the schedule
%   never asks more of the wing than the car already does. MLTP.m asserts it.
%   UPPER by the mid plateau - tv only REACHES 0 deg if v2-v1 >~ 4w. Here
%   v2-v1 = 25.5 m/s against 4w = 14 m/s, so the plateau is well formed (residual
%   6e-4 deg at its centre) and holds the MID setting over 10.7 % of the solved lap.
%   Raising w erodes that; MLTP.m asserts the plateau survives.
% rwVelBand is the two-sided tolerance |alpha - target| <= band. It is NOT a slack to
% be widened for convergence: widening it hands the wing back the freedom this variant
% exists to take away, so any change must be quoted with the result.
% rwVelKappa opens the band across a transition by kappa x the schedule's own local
% variation, which is what makes a two-sided constraint on a STEPPING target feasible
% at all: at a step the wing is slewing, and a slewing wing is by definition not at
% its target. kappa >= 1 admits the whole slew arc; it is set slightly above 1 because
% MLTP.m's soft extrema are the weighted-average form, which undershoots a true max.
% Where the schedule is flat (straights, corner plateaus) the term vanishes and only
% rwVelBand binds.
% rwVelBandM is the HALF-WIDTH of the window the band's opening term is measured over,
% in intervals. Measured the hard way: the first ARWv solve reused the retired braking
% mandate's look-ahead window of 8 here and the law did not bind - median band
% 12.72 deg against a 30 deg control range, |err| up to 20.87 deg, and the wing ran to
% +20 deg. The two windows answered different questions. That one sized a one-sided
% look-ahead's PRE-POSITIONING horizon, which wants to be long. This one asks "how
% long is the wing legitimately mid-slew", which is short. This bandM=2 tuning was
% done at the PRE-DIRECTIVE c.ub.RW = 60 deg/s: the worst step (25 deg) took 0.417 s,
% ~2.5 intervals of 10 m at 60 m/s, so +-2 (40 m) covered it with margin (+-1/20 m did
% not). Measured then: +-2 gave a median band of 1.36 deg, 47 % of knots inside 1 deg,
% vs +-8's 12.72 deg / 13.5 %.
%
% Does NOT hold at the Zenvo-directed 25 deg/s: the same 25 deg step now needs ~6
% intervals (60 m), and the actual required half-width is 5 (BCN) / 6 (NUR) - over the
% BAND_M_MAX = 4 ceiling validateRWVelocity.m enforces. That overshoot is WHY ARWv is
% retired at 25 deg/s (docs/rw-slew-rate-change.md SS2.2); bandM=2 stays only as a
% record of the pre-directive tuning, not a value ARWv can still solve with.
vp.rwVelBand  = getfielddef(ovr,'rwVelBand',  0.50);                                            % two-sided band (deg)
vp.rwVelKappa = getfielddef(ovr,'rwVelKappa', 1.25);                                            % transition band opening (-)
vp.rwVelBandM = getfielddef(ovr,'rwVelBandM',    2);                                            % band window half-width (intervals)
vp.rwVelOpts = struct( ...
    'aStraight', getfielddef(ovr,'rwVelAStraight', -10), ...                                    % straight-line / low-drag angle (deg)
    'aMid',      getfielddef(ovr,'rwVelAMid',        0), ...                                    % MID downforce setting (deg)
    'aCorner',   getfielddef(ovr,'rwVelACorner',   +10), ...                                    % cornering / high-downforce angle (deg)
    'aBrake',    getfielddef(ovr,'rwVelABrake',    +15), ...                                    % airbrake angle (deg) - single setting, no +20
    'alphaMax',  getfielddef(ovr,'rwVelAlphaMax',  getfielddef(ovr,'rwVelABrake',+15)), ...     % REACHABLE upper bound (deg); see below
    'v1',        getfielddef(ovr,'rwVelV1',       37.5), ...                                    % corner -> mid switch speed (m/s)
    'v2',        getfielddef(ovr,'rwVelV2',       63.0), ...                                    % mid -> straight switch speed (m/s)
    'w1',        getfielddef(ovr,'rwVelW1',        3.5), ...                                    % corner -> mid half-width (m/s)
    'w2',        getfielddef(ovr,'rwVelW2',        3.5));                                       % mid -> straight half-width (m/s)
vp.rwVelOpts.brakeOpts = vp.rwVelBrakeOpts;                                                     % same sOn switch, one source

%% Load powertrain and vehicle parameters

run('Parameters\vehParams.m'); 
run('Parameters\Powertrain.m');

%% Selection of track:

circuit = getfielddef(ovr,'circuit','BCN');

switch circuit
%-A) Self defined virtual tracks
    case 'Hairpin'
        track.k = [zeros(1,75) 0.0975*ones(1,16) zeros(1,73)];                                      % Hairpin
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
    case 'Straight'
        track.k = zeros(1,300);                                                                     % Straight line
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
    case 'Sturn'
        track.k = [zeros(1,100) pi/45*ones(1,20) zeros(1,30) -pi/45*ones(1,20) zeros(1,100)];       % S-turn
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
    case 'VirtualTrack'
        track.k = [zeros(1,150) pi/25*ones(1,20) zeros(1,50) -pi/125*ones(1,90) -pi/50*ones(1,35) pi/50*ones(1,35) -pi/500*(1:0.05:6) zeros(1,200) -pi/20*ones(1,20) zeros(1,80)]; %Virtual track
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
%-B) Real circuits data 
    case 'BCN'
        track = load('Circuits/Barcelona_circuit.mat');
    case 'BCN_S1'
        track = load('Circuits/Barcelona_circuit_s1.mat');
    case 'BCN_S2'
        track = load('Circuits/Barcelona_circuit_s2.mat');
    case 'BCN_S3'
        track = load('Circuits/Barcelona_circuit_s3.mat');
    case 'Jarama'
        track = load('Circuits/Jarama_circuit.mat');
    case 'Spa'
        track = load('Circuits/Spa_circuit.mat');
    case 'NUR'
        track = load('Circuits/Nurburgring_circuit.mat');
end

%% User options

% Optimal Control Problem - Boundary constraints
%-initial   ( x = [vx vy r n eps Om_fl Om_fr Om_rl Om_rr] )
vi = getfielddef(ovr,'vi',79.65);                                                               % inital velocity [m/s]
% 2026-07-20 BCN/Mid/Static/ATD=Off, Zenvo Pacejka tyre, vp.Fz_lim = 12500.
%
% RESEED WITH vi = vend - 1, NOT vi = vend. The initial state is a WINDOW, not a
% pin: MLTP.m:263-264 build it as Xi./x_s +- OPT_e in NORMALISED units, and
% vx_s = 100 (vehModel.m:19), so OPT_e = 1e-2 is +-1 m/s on vx. The solver always
% takes the fastest admissible entry, so vx(1) lands on vi+1 every time. Seeding
% vi = vend therefore overshoots by exactly the slack and the iteration stalls at
% miss ~= 1.0 forever. Accept when |vx(1) - vx(end)| <= 0.5 m/s.
%   iter 1: vi 75.00 -> vx(1) 76.00, vend 80.6549, lap 108.518 s, 885 iters
%   iter 2: vi 80.65 -> vx(1) 81.65, vend 80.6535, lap 107.879 s, 1033 iters
%   vend is the stable quantity (80.6549 vs 80.6535 = 1.4 mm/s apart), so the
%   fixed point is vx(1) = vx(end) = 80.65 at vi = 79.65.
% fixD (mu=1.30 generic tyre, same relaxed cap) closed its fixed point at 78.33.
%
% 2026-07-27 ActiveRW (BCN/ARW/AWD, docs/plans/2026-07-27-active-rw-control.md
% D1 solve): vi = 79.65 above is reused unchanged as the ARW starting guess -
% it is the static Mid fixed point, copied over for lack of an ARW-specific
% one, NOT re-derived for ARW. UNTESTED: the continuous [-10,+20] deg wing
% gives the optimiser more downforce/drag authority than static Mid, so the
% true vx(1)=vx(end) fixed point may sit elsewhere. If IPOPT fails to close
% on this seed, re-run the vi = vend - 1 fixed-point iteration above for the
% ARW config specifically before assuming a model defect.

% STALE (old generic tyre, pre-Zenvo-Pacejka 2026-07): the recorded per-config
% velocities below were tuned against mu=1.2/pD2=-0.6 and are ~15-20 m/s too
% slow for the Zenvo tyre data (fixD closed the BCN/Mid flying lap at vi=78.33
% with mu=1.30). Re-derive per config via the vi=vend fixed-point iteration.
% initial velocities AWD [static, activeRW, active, AALB]: [60.4664  60.7631  60.7726  60.8071]
% initial velocities ATD [static, activeRW, active, AALB]: [60.5942  61.5     61.62    61.7428]
% initial velocities TyreOpt AWD [static, active, AALB]: [60.936  61.1497  61.227]
% initial velocities TyreOpt ATD [static, active, AALB]: [61.695  61.695   61.94]

Xi = [vi; 0; nan; nan; nan; nan; nan; nan; nan];
Xi_init = [vi; 0; nan; nan; nan; nan; nan]; 

%-final
vf = nan;                                                                                       % final velocity [m/s]
Xf = [vf; nan; nan; nan; nan; nan; nan; nan; nan];
Xf_init = [vf; nan; nan; nan; nan; nan; nan];

% Collocation
OPT_ds = 10;                                                                                    % collocation step (m)
OPT_d = 3;                                                                                      % degree of interpolating polynomials
OPT_uinter = 'linear';                                                                          % assume 'linear' or 'constant' inputs
OPT_e = 1e-2;                                                                                   % Small quantity. Used as the slack for the path constraints and wherever helpful (like setting the initial guesses of the solver)

% Solver options
opts = struct;

%%-ipopt 
opts.ipopt.max_iter = 5000;                                                                     %max number of iterations.
opts.ipopt.fixed_variable_treatment = 'make_constraint';                                        %with default options, multipliers for the decision variables are wrong for equality constraints. Change the Tfixed_variable_treatmentT to hmake_constrainte or  relax_boundsf to obtain correct results (comment from Casadi's documentation. No difference has been observed in the solution or the solver when disabling this option)
opts.ipopt.tol = 1e-6;                                                                          % accuracy to which the NLP is solver                                       (1e-8 standard)                              
opts.ipopt.acceptable_tol = 1e-5;                                                               % acceptable tolerance when tolerance can not be reached                    (1e-6 standard)
opts.ipopt.mu_init = 1e-1;                                                                      % initial value of the barrier parameter                                    (1e-1 standard)
opts.ipopt.bound_push = 1e-2;                                                                   % how much initial point is pushed into the feasible region                 (1e-2 standard)
opts.ipopt.bound_frac = 1e-2;                                                                   % fraction of the variable bounds within which the initial point should lie (1e-2 standard)
opts.ipopt.constr_viol_tol = 1e-4;                                                              % tolerance for violation of constraints [inf_pr]                           (1e-4 standard)
opts.ipopt.dual_inf_tol = 1e-4;                                                                 % tolerance for violation of optimality condition [inf_du]                  (1e-4 standard)
opts.ipopt.compl_inf_tol = 1e-4;                                                                % tolerance for complementary conditions                                    (1e-4 standard)

%%-Constraints on the rate of inputs (units of each input per second) 
[c.ub.T_motor, c.lb.T_motor]  = deal(2e4, -2e4);                                                % upper & lower limits on rate of torque e-motor input  (Nm/s)
[c.ub.T_brake, c.lb.T_brake]  = deal(1.45e5, -1.45e5);                                          % upper & lower limits on rate of torque brake input    (Nm/s)
[c.ub.ATD, c.lb.ATD]          = deal(2e4, -2e4);                                                % upper & lower limits on rate of ATD input             (1/s)
[c.ub.delta, c.lb.delta]      = deal(0.1, -0.1);                                                % upper & lower limits on rate of steering angle input  (rad/s)
[c.ub.FW, c.lb.FW]            = deal(20, -20);                                                  % upper & lower limits on rate of FW angle input        (deg/s)
[c.ub.RW, c.lb.RW]            = deal(25, -25);                                                  % upper & lower limits on rate of RW angle input        (deg/s)
[c.ub.TW, c.lb.TW]            = deal(24, -24);                                                  % upper & lower limits on rate of TW angle input        (deg/s)

% Regularisation factors for the inputs (ru) and their first (rdu) and second (rdu2) derivatives
[c.ru.T_motor, c.rdu.T_motor, c.rdu2.T_motor]  = deal(0,  0,  0);                               % torque e-motor input  (-)
[c.ru.T_brake, c.rdu.T_brake, c.rdu2.T_brake]  = deal(0,  0,  0);                               % torque brake input    (-)
[c.ru.delta,   c.rdu.delta,   c.rdu2.delta]    = deal(0,  10, 15);                              % steering angle input  (-)
[c.ru.ATD,     c.rdu.ATD,     c.rdu2.ATD]      = deal(0,  0, 1.5);                              % ATD input             (-)
[c.ru.FW,      c.rdu.FW,      c.rdu2.FW]       = deal(0,  0,  0.3);                             % FW angle input        (-)
[c.ru.RW,      c.rdu.RW,      c.rdu2.RW]       = deal(0,  0,  0.9);                             % RW angle input        (-)
[c.ru.TW,      c.rdu.TW,      c.rdu2.TW]       = deal(0.005,  0,  0.4);                         % TW angle input        (-)

% runOverride hook for the RW second-derivative weight, needed by stages C1/C2 of
% Validation\runARWMandate.m (locked decision 4: sweep rdu2.RW over {0.9, 0.1, 0} to
% test whether the shipped continuous ActiveRW result is regularisation-limited -
% the mandated wing shape costs +0.029 s-equivalent of objective at 0.9, ~25 % of the
% airbrake's whole lap-time prize). Applied AFTER the block above so the override
% wins, and BEFORE the per-config rdu2 vectors are assembled below, which is the only
% window in which changing it has any effect. Deliberately not extended to the other
% inputs: nothing needs them swept, and every extra hook is another way for a stray
% override file to silently change a run.
if isfield(ovr,'rdu2RW')
    validateattributes(ovr.rdu2RW, {'numeric'}, {'scalar','real','finite','nonnegative'}, ...
        'userOpts', 'ovr.rdu2RW');
    c.rdu2.RW = ovr.rdu2RW;
    fprintf('userOpts: c.rdu2.RW overridden to %g (runOverride.mat)\n', c.rdu2.RW);
end

%Regularisation factors for the states and aux variables                               
rdy  = [0; 0];                                                                                  % first derivative aux variables [ltx; lty_f; lty_r]
rdy2 = [1; 0]; 

if pt.ATD == 0 && vp.ActAero == 0
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.delta];                                                  
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.delta];    
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.delta]; 
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.delta]; 
elseif pt.ATD == 0 && vp.ActAero == 1
    if any(getfielddef(vp,'rwMandate',0) == [6 7])   % ARFWd/ARFWr: FW row (nu-2) BEFORE RW row (nu-1), both on RW actuator limits
        duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.RW;   c.ub.RW;   c.ub.delta];
        duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.RW;   c.lb.RW;   c.lb.delta];
        ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.RW;   c.ru.RW;   c.ru.delta];
        rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.RW;  c.rdu.RW;  c.rdu.delta];
        rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.RW; c.rdu2.RW; c.rdu2.delta];
    else
        duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.RW;   c.ub.delta];
        duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.RW;   c.lb.delta];
        ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.RW;   c.ru.delta];
        rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.RW;  c.rdu.delta];
        rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.RW; c.rdu2.delta];
    end
elseif pt.ATD == 0 && vp.ActAero == 2
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.FW;   c.ub.RW;   c.ub.delta];                                         
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.FW;   c.lb.RW;   c.lb.delta];     
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.FW;   c.ru.RW;   c.ru.delta]; 
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.FW;  c.rdu.RW;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.FW; c.rdu2.RW; c.rdu2.delta]; 
elseif pt.ATD == 0 && vp.ActAero == 3
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.FW;   c.ub.FW;   c.ub.RW;   c.ub.TW;   c.ub.delta];              
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.FW;   c.lb.FW;   c.lb.RW;   c.lb.TW;   c.lb.delta];    
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.FW;   c.ru.FW;   c.ru.RW;   c.ru.TW;   c.ru.delta];     
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.FW;  c.rdu.FW;  c.rdu.RW;  c.rdu.TW;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.FW; c.rdu2.FW; c.rdu2.RW; c.rdu2.TW; c.rdu2.delta];
elseif pt.ATD == 1 && vp.ActAero == 0
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.delta];                                                  
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.delta];    
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.delta]; 
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.delta]; 
elseif pt.ATD == 1 && vp.ActAero == 1
    if any(getfielddef(vp,'rwMandate',0) == [6 7])   % ARFWd/ARFWr ATD-on (nu=9): FW row 7, RW row 8
        duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.RW;   c.ub.RW;   c.ub.delta];
        duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.RW;   c.lb.RW;   c.lb.delta];
        ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.RW;   c.ru.RW;   c.ru.delta];
        rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.RW;  c.rdu.RW;  c.rdu.delta];
        rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.RW; c.rdu2.RW; c.rdu2.delta];
    else
        duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.RW;   c.ub.delta];
        duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.RW;   c.lb.delta];
        ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.RW;   c.ru.delta];
        rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.RW;  c.rdu.delta];
        rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.RW; c.rdu2.delta];
    end
elseif pt.ATD == 1 && vp.ActAero == 2
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.FW;   c.ub.RW;   c.ub.delta];                                                  
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.FW;   c.lb.RW;   c.lb.delta];    
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.FW;   c.ru.RW;   c.ru.delta]; 
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.FW;  c.rdu.RW;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.FW; c.rdu2.RW; c.rdu2.delta];
elseif pt.ATD == 1 && vp.ActAero == 3
    duk_ub  =   [c.ub.T_motor;   c.ub.T_brake;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.ATD;   c.ub.FW;   c.ub.FW;   c.ub.RW;   c.ub.TW;   c.ub.delta];                                                  
    duk_lb  =   [c.lb.T_motor;   c.lb.T_brake;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.ATD;   c.lb.FW;   c.lb.FW;   c.lb.RW;   c.lb.TW;   c.lb.delta];    
    ru      =   [c.ru.T_motor;   c.ru.T_brake;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.ATD;   c.ru.FW;   c.ru.FW;   c.ru.RW;   c.ru.TW;   c.ru.delta]; 
    rdu     =   [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.ATD;  c.rdu.FW;  c.rdu.FW;  c.rdu.RW;  c.rdu.TW;  c.rdu.delta]; 
    rdu2    =   [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.ATD; c.rdu2.FW; c.rdu2.FW; c.rdu2.RW; c.rdu2.TW; c.rdu2.delta];   
end

% Constraints on the rate of inputs for the initalisation program (units of each input per second)  u = [Tdrive_n; T_brake_n; delta_n];
duk_ub_init = [c.ub.T_motor; c.ub.T_brake; c.ub.delta];                                                                             
duk_lb_init = [c.lb.T_motor; c.lb.T_brake; c.lb.delta];    

%Regularisation for the initalisation program
ru_init   = [c.ru.T_motor;   c.ru.T_brake;   c.ru.delta];
rdu_init  = [c.rdu.T_motor;  c.rdu.T_brake;  c.rdu.delta];
rdu2_init = [c.rdu2.T_motor; c.rdu2.T_brake; c.rdu2.delta];
rdy_init  = rdy(1);
rdy2_init = rdy2(1);
