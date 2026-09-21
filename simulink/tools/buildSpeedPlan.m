function [vPlan, info] = buildSpeedPlan(kap, vRaw, vp, pt, opts)
%BUILDSPEEDPLAN Grip- and feasibility-limited speed plan for the closed-loop
% driver, built from the LIVE tyre law and the LIVE ride-height aero.
%
%   [vPlan, info] = buildSpeedPlan(kap, vRaw, vp, pt, opts)
%
%   kap   N x 1  reference-line curvature [1/m] (chord estimate, ISO sign; only
%                |kap| is used)
%   vRaw  N x 1  the solved MLTP speed profile on the same 1 m grid [m/s]
%   vp,pt        vehicle / powertrain parameter structs
%   opts  struct, all fields optional:
%     .rhTable   K x 3 [v RHf RHr], MEASURED plant ride heights (mm) binned by
%                speed. Interpolated (linear, nearest-edge HELD, never
%                extrapolated) to give the aero evaluation its ride height at
%                each planned speed. REQUIRED.
%     .wingMode  'law'   (DEFAULT) wing stations taken from the ECU's OWN
%                        reactive law at each plan point -- see item 3 below
%                'fixed' constant stations opts.alphaRW / opts.alphaFW
%     .alphaRW   rear-wing angle [deg] for wingMode 'fixed' (default +10)
%     .alphaFW   front-wing angle [deg] for wingMode 'fixed' (default 0)
%     .fzFrontOffsetN  constant RESIDUAL front-axle load transfer [N], additive
%                on the STATIC load only and conserved onto the rear, on top of
%                the plan's own computed m*ax*hcg/l term. Default 0; the shipped
%                DriverPath value is set by simulink/tools/rebuildDriverPlan.m.
%                See item 4 for what it represents and why it is additive.
%     .axLook    forward window [m] over which the corner-EXIT acceleration is
%                read for the load-transfer term (default 25) -- see item 4(i).
%     .ayCap     hard ceiling on PLANNED lateral acceleration [m/s^2], default
%                inf. This is a DRIVER-CAPABILITY ceiling, NOT an aerodynamic
%                or tyre correction -- see item 5.
%     .brkMargin longitudinal-decel budget fraction (default 0.7)
%     .ds        grid spacing [m] (default 1)
%     .nIter     grip fixed-point iterations (default 40)
%
% =========================================================================
% WHY THIS FILE EXISTS, AND WHAT IT REPLACES
% =========================================================================
% simulink/work/build_driver.m originally computed this plan inline from
%   (a) the PLACEHOLDER VDB tyre block's Magic Formula coefficients, and
%   (b) two CONSTANT placeholder aero coefficients read out of Plant.slx's
%       model workspace, phAeroClF / phAeroClR.
% The real-tyre swap replaced (a) with the real MF5.2 set and the live-aero
% rebuild replaced (b) with a ride-height aero subsystem, DELETING
% phAeroClF/phAeroClR. Neither change rebuilt the plan, so until the
% torque-vectoring integration the shipped drvRefVplan was still a
% placeholder-grip, placeholder-aero profile -- and build_driver.m could no
% longer even be re-run, because those two variables no longer exist.
% This function is the tracked replacement for that inline block, and
% simulink/tools/rebuildDriverPlan.m is its tracked entry point.
%
% =========================================================================
% MODEL (four changes from the placeholder version, all physical)
% =========================================================================
% 1. THE CAP IS THE WEAKER AXLE, AT ITS ACTUAL LOAD -- NOT muY*m*g, AND NOT
%    THE AXLE SUM EITHER.
%    The old form solved  m v^2 |k| = muY (m g + 0.5 rho A ClT v^2)  with a
%    single constant muY taken at the STATIC corner load. Two things are wrong
%    with that, and both were measured to matter:
%
%    (a) LOAD SENSITIVITY. The MF5.2 set has pDy2 < 0, i.e. muY FALLS with
%        load, so evaluating muY at the static load and then adding downforce
%        on top double-counts the benefit: the extra load arrives with LESS
%        grip per newton than the static load had. muY_f/muY_r are therefore
%        read from Functions/tyreMFnum.m at the ACTUAL downforce-loaded
%        per-corner load (the tyre set's own known load-sensitivity caveat,
%        applied exactly where it bites -- a corner-speed ceiling built on
%        downforce).
%
%    (b) THE AXLES DO NOT SATURATE TOGETHER. Summing the two axles' capacity
%        assumes the car can borrow rear grip to turn the front, which it
%        cannot. With the static lateral split (front takes l_r/l of m*ay,
%        rear l_f/l) the cap is the MINIMUM over axles:
%            ay_max = min( 2*muY_f*Fz_f / (m*l_r/l),
%                          2*muY_r*Fz_r / (m*l_f/l) )
%        MEASURED on BCN at drvVScale = 0.95 with the axle-sum form: through
%        the s ~ 1300-1405 m stretch the FRONT axle ran at ellipse utilisation
%        0.91 median / 1.37 peak while the REAR sat at 0.67 / 0.84, the car
%        understeered off the line and |n| reached 11.8 m. The asymmetry is
%        real and has two causes -- the front axle carries the brake bias and
%        the whole of the TV shift, and the reactive law's MIDDLE station
%        (RW 0 deg with FW -20 deg, which is where the law sits for BCN's
%        150-350 m radius corners) is a deliberately FRONT-UNLOADED aero state.
%        The per-axle form reproduces exactly the friction ellipse the traction
%        pedal map already applies at run time, so plan and driver now agree.
%
%    The corner-speed condition m v^2 |k| = m*ay_max(v) is solved by damped
%    fixed-point iteration on v, clamped to [0, Vmax].
%
% 2. DOWNFORCE COMES FROM THE LIVE AERO, AT A MEASURED RIDE HEIGHT.
%    ClF/ClR/Cd are evaluated with simulink/aero/aeroLiveEval.m -- the same
%    reference the shipped Plant aero subsystem mirrors -- at (RHf,RHr)
%    interpolated from opts.rhTable, which is MEASURED off a plant lap rather
%    than assumed. It is speed-dependent and per-axle, where the old model had
%    one constant total. The aero split matters as well as the total: the
%    front/rear downforce split moves the per-axle loads apart, and with a
%    load-sensitive muY the axle carrying more load contributes proportionally
%    less grip.
%
%    THE TABLE MUST COVER THE SPEEDS THE CAR REACHES, and a table that stops
%    short is not merely imprecise, it is optimistic in the worst place. The
%    first such table was binned from a slow baseline lap and stopped at
%    75 m/s; held at its top row it predicted ClF = +0.101 at 85-90 m/s, while
%    the plant measured RHf ~ 147 mm (nose-up, at the map's edge) and
%    ClF = +0.42 -- i.e. four times the FRONT LIFT the plan assumed, in the
%    fastest and least forgiving part of the lap. Re-bin the table from a lap
%    that actually reaches top speed; rebuildDriverPlan.m documents the
%    two-pass procedure.
%
% 3. THE WING STATE COMES FROM THE ECU'S OWN LAW, NOT FROM AN ASSUMPTION.
%    A plan needs a wing state, because downforce is what sets the corner-speed
%    ceiling. The first attempt at this assumed the MAX-DOWNFORCE cornering pair
%    (RW +10 deg, FW 0 deg) everywhere a corner was planned. That was MEASURED
%    to be wrong and it lost the car: the reactive law is a kappa classifier,
%    and RW only reaches its +10 station above k2 = 0.022565 1/m (R < 44 m).
%    Through BCN's long turn-3 right-hander (s ~ 1200-1320 m, R ~ 120 m, i.e.
%    kappa ~ 0.0085 between k1 and k2) the ECU actually sits at the MIDDLE
%    station, RW 0 deg -- so the plan was sizing corner speed on downforce the
%    car does not have there. At drvVScale = 1.0 the car ran wide out of that
%    corner, the traction pedal map's ellipse ceiling collapsed to 0, the
%    steering saturated at 45 deg and |n| reached 456 m.
%
%    wingMode 'law' therefore evaluates simulink/ecu/ecuLawStep.m -- the SAME
%    ported law the AeroECU runs -- at each plan point, using the point's own
%    planned speed and the kinematic yaw rate r = kappa*v the reference line
%    implies, and snaps the result to the nearest station of each wing (the
%    hysteresis and minimum-dwell logic of snapStationRef is deliberately NOT
%    applied: it only delays transitions, and a planner wants the steady-state
%    station). The plan and the controller then agree about the car's
%    aerodynamic state by construction.
%
%    Tbrake is passed as 0, so the BRAKE overlay (RW +15) is never assumed.
%    That is conservative in both places it matters: it withholds downforce
%    from the corner-entry cap, and it withholds it from aBrk.
%
%    wingMode 'fixed' keeps the old constant-station behaviour for comparison.
%
% 4. THE FRONT AXLE CARRIES LESS LOAD THAN A STEADY-STATE CAP ASSUMES, AND
%    BOTH CORRECTIONS FOR IT ARE ADDITIVE NEWTONS, NOT A MULTIPLICATIVE DERATE.
%    A steady-state corner-speed cap implicitly assumes ax = 0, so it uses the
%    static + aero front load. A car EXITING a corner does not have that load.
%    MEASURED on the BCN closed loop (461,017 cornering samples, |ay| > 4
%    m/s^2): front-axle load, measured over static+aero predicted, has median
%    0.949 but p10 0.645 -- not a steady bias, a corner-exit transient, and the
%    transient is where the car leaves the line. Worked example, BCN s = 1859 m
%    (R = 143 m, a right-hander opening onto a straight): an uncorrected plan
%    allowed 42.7 m/s = 12.75 m/s^2 of ay, the front axle was carrying 4,620 N
%    against a predicted 6,668 N (a 31 % deficit), the car delivered only
%    7.2 m/s^2, ran wide, and the Stanley controller wound the steering to
%    29.6 deg -- far past the tyre's ~7 deg peak-force slip angle, so the extra
%    lock made it worse -- before |n| reached 36.8 m.
%
%    Two terms cover this, in newtons, both subtracted from the STATIC front
%    load and both conserved onto the rear:
%      (i)  dFz = m*ax*hcg/l, from the PLAN'S OWN acceleration, iterated three
%           times. ax is the more front-unloading of a centred difference and a
%           FORWARD difference over opts.axLook metres. The forward term is
%           what makes this work at all: a centred difference is ~0 at every
%           apex by construction, so a transfer term built from it alone
%           vanishes exactly where the corner-speed cap is applied, while the
%           real car is still turning hard and already feeding in throttle for
%           the exit. At the worked example the exit acceleration (~5 m/s^2)
%           accounts for ~1,250 N of the 2,200 N deficit -- but only when read
%           ahead of the apex.
%           This term was DEAD CODE until the 2026-09-02 review (an off-by-one
%           nargin guard, see lateralCapacity); any claim about it written
%           before that fix described behaviour the shipped plan did not have.
%      (ii) fzFrontOffsetN, a constant residual, because (i) accounts for only
%           about half the measured deficit. The remainder is attitude-
%           dependent aero the planner's static RH(v) table cannot see: nose-up
%           rake on corner exit moves the front toward the ride-height map's
%           lift-reversal region, which removes front downforce outright rather
%           than merely transferring it. A pitch-aware RH(v, ax) table would
%           replace this term with physics.
%
%    WHY ADDITIVE AND NOT A FACTOR. The shipped code until 2026-09-02 used a
%    multiplicative fzFrontFactor = 0.60 applied to (static + aero). That
%    scaled the AERODYNAMIC front load down by 40 % as well, which is exactly
%    backwards for this project: it made the planner value front downforce less
%    the more of it the wings produced, understating the benefit of the one
%    quantity the whole study is about. An additive newton offset leaves every
%    newton of front downforce fully credited.
%
% 5. ayCap IS A DRIVER CEILING, AND IT IS KEPT SEPARATE FROM THE PHYSICS ON
%    PURPOSE. Items 1-4 model what the CAR can hold in a steady corner. They do
%    not model what this forward-time Stanley + pedal-map DRIVER can hold, and
%    measurement says the two differ: on the clean shipped lap the closed loop
%    sustained a lateral acceleration of about 8.5 m/s^2 through BCN's 40-150 m
%    radius corners with large margins everywhere (max |n| 2.28 m, peak steer
%    11.3 deg of 45 available), whereas the same plan built from items 1-4
%    alone allows 11-13 m/s^2 at those same corners and loses the car (8
%    off-track excursions, steering saturated a quarter of the lap). The gap is
%    not steady-state grip -- it is the driver: once the line is lost, the
%    Stanley controller adds lock past the tyre's ~7 deg peak-slip angle, which
%    REDUCES front force, so the excursion is self-reinforcing.
%
%    ayCap therefore states that limit explicitly, as one measured number with
%    a name, instead of smuggling it into the load model where it would corrupt
%    the aero and tyre terms. That separation is the whole point:
%      - it does NOT scale downforce (the I3 defect the multiplicative
%        fzFrontFactor had): every newton of front downforce is still fully
%        credited in items 1-2, and the axle physics still binds wherever it is
%        TIGHTER than the cap;
%      - it is falsifiable and it is expected to be RAISED: any improvement to
%        the driver (a better lateral controller, an anti-windup on steer, or
%        the front/rear torque authority the plant currently lacks) should be
%        accompanied by re-measuring it upward. If a future run holds more than
%        ayCap cleanly, the number is wrong and should be changed.
%      - it must NEVER be quoted as a vehicle or aerodynamic limit.
%
% FEASIBILITY. The plan is made feasible in BOTH directions (see feasiblePass):
% a backward decel sweep, a forward acceleration sweep, then a backward sweep
% again, two wraps each for loop closure. The forward sweep is not decoration --
% without it the plan jumps discontinuously from a corner speed to the
% reference speed within one 1 m step on every corner exit, and a difference
% taken on that plan is not an acceleration at all: measured on the first run
% after the nargin fix it produced |ax| up to 2,064 m/s^2 and drove the planned
% lap from 106.55 s to 257.19 s of nonsense.
%
% NOT MODELLED, and conservative in every case: longitudinal load transfer
% under braking (see the aBrk asymmetry note in lateralCapacity), aerodynamic
% drag as a decelerating force, the brake overlay's extra downforce, and the
% reactive law's hysteresis/dwell. The plan is a TARGET for a closed-loop
% driver that carries its own safety factors (drvSFa/drvSFb inside the traction
% pedal map, drvVScale on the target itself), not a lap-time prediction.

if nargin < 5 || isempty(opts), opts = struct(); end
opts = defaults(opts);

kap  = kap(:);  vRaw = vRaw(:);
N    = numel(kap);
assert(numel(vRaw) == N, 'buildSpeedPlan:len', 'kap and vRaw must be the same length');

Vmax = pt.Vmax;
akap = abs(kap);

assert(isfield(opts,'rhTable') && ~isempty(opts.rhTable), 'buildSpeedPlan:rh', ...
    'opts.rhTable is required -- see this file''s header');
rhV = opts.rhTable(:,1);  rhF = opts.rhTable(:,2);  rhR = opts.rhTable(:,3);

FzfSt = vp.m*vp.g*vp.l_r/(2*vp.l);      % static per-CORNER loads
FzrSt = vp.m*vp.g*vp.l_f/(2*vp.l);
law   = ecuLawParams();

% ---- 1. outer loop: wing stations <-> grip-limited speed -----------------
% The wing station depends on speed only through the law's classifier
% kap2 = r^2/(vx^2+1) with r = kappa*v, i.e. kap2 = kappa^2 v^2/(v^2+1), which
% is within 4 % of kappa^2 for any v above 5 m/s. Two outer passes are
% therefore ample; the second is a confirmation, not a correction.
v       = min(vRaw, Vmax);
alphaRW = repmat(opts.alphaRW, N, 1);
alphaFW = repmat(opts.alphaFW, N, 1);
useLaw  = strcmpi(opts.wingMode, 'law');
for outer = 1:(1 + useLaw)
    if useLaw
        for i = 1:N
            [tRW, tFW] = ecuLawStep(v(i), kap(i)*v(i), 0, law);
            alphaRW(i) = snapNearest(tRW, law.stRW);
            alphaFW(i) = snapNearest(tFW, law.stFW);
        end
    end
    for it = 1:opts.nIter
        FyMax = lateralCapacity(v, alphaRW, alphaFW, rhV, rhF, rhR, vp, pt, ...
                                FzfSt, FzrSt, opts.fzFrontOffsetN);
        vn = min(sqrt(FyMax ./ (vp.m*max(akap, 1e-12))), Vmax);
        v  = 0.5*v + 0.5*vn;             % damped, for monotone convergence
    end
end
FyMax = lateralCapacity(v, alphaRW, alphaFW, rhV, rhF, rhR, vp, pt, ...
                        FzfSt, FzrSt, opts.fzFrontOffsetN);
resid = max(abs(v - min(sqrt(FyMax ./ (vp.m*max(akap,1e-12))), Vmax)));
vGrip = min(min(v, Vmax), sqrt(opts.ayCap ./ max(akap, 1e-12)));
vGrip(akap <= 1e-9) = Vmax;

% ---- 2. forward/backward feasibility ------------------------------------
% DECEL BUDGET: evaluated at each point's OWN cornering wing stations, with the
% front-load corrections applied -- i.e. the SAME aerodynamic and load state as
% the grip cap. That is NOT the aerodynamically exact choice, and the exact one
% was built, measured and REJECTED. Recorded here so it is not re-attempted:
%   The ECU's brake overlay actually puts the rear wing on its +15 airbrake
%   station for ~65 % of the braking DISTANCE of a lap (gate 6 section [F]), and
%   that station carries much more downforce than the RW -10 the classifier
%   holds on the straight before it. Sizing aBrk at the airbrake station instead
%   (and without the front correction, since braking LOADS the front) raised
%   aBrk from a flat 8.6 m/s^2 to 10.6-15.5 and the planned top speed from 101.6
%   to 109.4 m/s against ref.vx's 112.1 -- much closer to the ~20.7 m/s^2 the
%   offline solve uses out of BCN's main straight.
%   MEASURED RESULT: the plant could not realise it. The same driver that runs a
%   clean lap on the conservative plan lost the car on the aggressive one -- 20
%   off-track excursions, max |n| 99.7 m, steering saturated 61 % of the lap,
%   lap not completed. The car arrives at each corner at the higher planned
%   speed and cannot shed it: the ECU's airbrake station arrives late
%   (hysteresis plus a 0.2 s minimum dwell) and the traction pedal map's own
%   brake ceiling is built on static-load mu, so the realised decel is well
%   under the budget. The conservative form is kept DELIBERATELY, as an
%   empirical match to what the closed loop can actually do.
[~, ~, aBrk, ~, aAcc] = lateralCapacity(vGrip, alphaRW, alphaFW, rhV, rhF, rhR, vp, pt, ...
                                        FzfSt, FzrSt, opts.fzFrontOffsetN);
aBrk  = opts.brkMargin * aBrk;
vPlan = feasiblePass(min(vRaw, vGrip), aBrk, aAcc, opts.ds);

% ---- 3. longitudinal load transfer, from the plan's OWN ax ---------------
% See item 4(i). Three passes: the plan only moves where the cap actually
% binds, and each pass changes ax less than the last.
W = max(round(opts.axLook/opts.ds), 1);
for lt = 1:3
    ip = [2:N 1];  im = [N 1:N-1];
    iw = mod((0:N-1)' + W, N) + 1;
    axC = vPlan.*(vPlan(ip) - vPlan(im))./(2*opts.ds);     % local, centred
    axF = vPlan.*(vPlan(iw) - vPlan)./(W*opts.ds);         % corner EXIT
    ax  = max(axC, axF);                % the more front-unloading of the two
    dFz = vp.m*ax*vp.hcg/vp.l;          % + = load leaves the front
    FyMax = lateralCapacity(vPlan, alphaRW, alphaFW, rhV, rhF, rhR, vp, pt, ...
                            FzfSt, FzrSt, opts.fzFrontOffsetN, dFz);
    vGrip = min(sqrt(min(FyMax./vp.m, opts.ayCap) ./ max(akap, 1e-12)), Vmax);
    vGrip(akap <= 1e-9) = Vmax;
    [~, ~, aBrk, ~, aAcc] = lateralCapacity(vPlan, alphaRW, alphaFW, rhV, rhF, rhR, vp, pt, ...
                                            FzfSt, FzrSt, opts.fzFrontOffsetN);
    aBrk  = opts.brkMargin * aBrk;
    vPlan = feasiblePass(min(vRaw, vGrip), aBrk, aAcc, opts.ds);
end

info = struct('vGrip', vGrip, 'alphaRW', alphaRW, 'alphaFW', alphaFW, ...
    'FyMax', FyMax, 'aBrk', aBrk, 'aAcc', aAcc, 'ax', ax, ...
    'rhSource', sprintf('aeroLiveEval at measured RH(v) %g..%g m/s, wingMode=%s', ...
                        min(rhV), max(rhV), opts.wingMode), ...
    'fixedPointResid', resid, 'opts', opts, ...
    'nCut', sum(vPlan < vRaw - 1e-6), 'maxCut', max(vRaw - vPlan), ...
    'meanCut', mean(vRaw - vPlan));
end

% =========================================================================
function [FyMax, Fz, aBrk, util, aAcc] = lateralCapacity(v, aRW, aFW, rhV, rhF, rhR, vp, pt, FzfSt, FzrSt, fzOff, dFz)
%LATERALCAPACITY Effective lateral capacity m*ay_max [N] at speed v with the
% given wing stations, plus the matching longitudinal decel and accel budgets
% [m/s^2]. ay_max is the MINIMUM over the two axles (see item 1(b)), each
% evaluated at its actual downforce-loaded per-corner load, with muY/muX read
% from the SHIPPED tyre law (Functions/tyreMFnum.m) so the planner cannot drift
% from the plant's tyres. util is the front/rear capacity ratio (>1 = the rear
% is the weaker axle), returned for diagnostics only.
%
%   fzOff  [N] constant residual front-axle load transfer (item 4(ii))
%   dFz    [N] the plan's own m*ax*hcg/l longitudinal transfer, OPTIONAL
%
% ARGUMENT-COUNT NOTE (review fix C1, 2026-09-02): dFz is the LAST declared
% input, so the default guard below must test nargin against ITS POSITION. It
% previously read `nargin < 12` while dFz sat at position 11, which is always
% true -- dFz was silently forced to 0 on every call and the entire item-4
% load-transfer pass was dead code that nothing detected for a full task cycle.
% The guard is therefore written against nargin(...) of this very function
% rather than a literal, so adding an argument can never silently disable it
% again.
if nargin < nargin('buildSpeedPlan>lateralCapacity') || isempty(dFz), dFz = 0; end

% BOTH front-load corrections are ADDITIVE LOAD TRANSFERS in newtons, applied
% to the STATIC term only, and both are conserved onto the rear axle. They must
% NOT be applied as a multiplicative derate of (static + aero): that would scale
% the AERODYNAMIC front load down as well, which structurally under-rewards
% exactly the quantity this project exists to measure -- front downforce -- and
% would make the planner value a front wing less the more of it there is.
% (Review fix I3, 2026-09-02; the shipped code did precisely that at
% fzFrontFactor = 0.60, i.e. it discarded 40 % of every newton of front
% downforce the wings generated.)
vC   = max(v, 0.5);
RHfq = interpHold(rhV, rhF, vC);
RHrq = interpHold(rhV, rhR, vC);
[ClF, ClR, Cd] = aeroLiveEval(RHfq, RHrq, aRW, aFW, vC, vp);
q    = 0.5*vp.rho*vp.A*vC.^2;
Fzf  = max(FzfSt - (fzOff + dFz)/2 - ClF.*q/2, 1);   % per corner
Fzr  = max(FzrSt + (fzOff + dFz)/2 - ClR.*q/2, 1);
[~,~,muXf,muYf] = tyreMFnum(vp.tyre_f, vp.tyre_f.Fz0, Fzf, 0, 0);
[~,~,muXr,muYr] = tyreMFnum(vp.tyre_r, vp.tyre_r.Fz0, Fzr, 0, 0);

% Static lateral split: the front axle takes l_r/l of m*ay, the rear l_f/l.
% m*ay_max is therefore the smaller of the two axles' own m*ay allowances.
mAyF  = (2*muYf.*Fzf) * (vp.l/vp.l_r);
mAyR  = (2*muYr.*Fzr) * (vp.l/vp.l_f);
FyMax = min(mAyF, mAyR);
util  = mAyF ./ max(mAyR, 1);
Fz    = 2*Fzf + 2*Fzr;

% ASYMMETRY, stated because it is deliberate and it is NOT physical (review
% note M8): aBrk is returned from the same front-unloaded axle loads as the
% lateral cap, yet braking TRANSFERS LOAD ONTO the front. The decel budget is
% therefore under-stated by roughly the front correction's share. It is kept
% that way because the aerodynamically exact alternative was built and measured
% and the plant could not realise it (see the DECEL BUDGET note at the call
% site); brkMargin and this asymmetry together are an empirical match to the
% closed loop's achievable deceleration, not a model of it.
aBrk  = (2*muXf.*Fzf + 2*muXr.*Fzr) ./ vp.m;

% ---- forward (acceleration) budget --------------------------------------
% The smaller of the powertrain envelope and what the DRIVEN axles can put
% down, minus aerodynamic drag. This is the ONE place vp.Tdist enters the plan:
% with a rear fraction Tds the rear axle must carry Tds of the tractive force
% and the front (1 - Tds), so a split that does not match the axles' load
% distribution limits traction at whichever axle runs out first.
%   powertrain: the same envelope Plant/Torque Path/torqueStep applies
%   traction  : min over axles of (axle mu*Fz) / (that axle's force share)
%   drag      : 0.5 rho A Cd v^2, from the SAME aeroLiveEval call above, so the
%               wing station that buys downforce also pays its drag here
Tds   = vp.Tdist;
TwCap = min(pt.Tmax*vp.gear, pt.Pmax ./ max(vC/vp.Rw, 1));
Fpt   = TwCap / vp.Rw;
FtrF  = (2*muXf.*Fzf) ./ max(1 - Tds, 1e-3);
FtrR  = (2*muXr.*Fzr) ./ max(Tds,     1e-3);
Fdrag = 0.5*vp.rho*vp.A*Cd.*vC.^2;
aAcc  = max((min(min(Fpt, FtrF), FtrR) - Fdrag) ./ vp.m, 0.1);
end

% =========================================================================
function y = snapNearest(x, stations)
[~, i] = min(abs(x - stations(:).'));
y = stations(i);
end

% =========================================================================
function vP = feasiblePass(vP, aBrk, aAcc, ds)
%FEASIBLEPASS Quasi-steady-state forward/backward feasibility on a CLOSED lap.
% Backward sweep first (a corner is entered from ahead), then forward, then
% backward again to restore decel feasibility against any speed the forward
% sweep lowered. The result is a plan the car can both reach and shed, which is
% what makes its own dv/ds a usable acceleration.
%
% NOT CONVERGENCE-CHECKED, stated because it is a real (if small) gap: the final
% backward sweep can in principle lower a speed that the forward sweep had
% already used as the basis for a later point, so a strict fixed point would
% iterate decel/accel until nothing moves rather than stopping after three
% sweeps. It is not iterated because each sweep only ever LOWERS speeds, so the
% sequence is monotone, bounded below and cannot oscillate; and because the
% caller runs the whole thing three more times inside its own load-transfer
% loop. If a future plan shows accel-infeasible segments AFTER the last decel
% sweep, replace the three fixed sweeps with a while-loop on a max-change
% tolerance rather than adding a fourth.
vP = decelPass(vP, aBrk, ds);
vP = accelPass(vP, aAcc, ds);
vP = decelPass(vP, aBrk, ds);
end

% =========================================================================
function vP = decelPass(vP, aBrk, ds)
%DECELPASS Backward decel-feasibility sweep, two wraps for loop closure.
N = numel(vP);
for pass = 1:2
    for i = N:-1:1
        j = i+1; if j > N, j = 1; end
        vP(i) = min(vP(i), sqrt(vP(j)^2 + 2*aBrk(i)*ds));
    end
end
end

% =========================================================================
function vP = accelPass(vP, aAcc, ds)
%ACCELPASS Forward acceleration-feasibility sweep, two wraps for loop closure.
N = numel(vP);
for pass = 1:2
    for i = 1:N
        j = i+1; if j > N, j = 1; end
        vP(j) = min(vP(j), sqrt(vP(i)^2 + 2*aAcc(i)*ds));
    end
end
end

% =========================================================================
function o = defaults(o)
if ~isfield(o,'wingMode'),       o.wingMode       = 'law'; end
if ~isfield(o,'alphaRW'),        o.alphaRW        = 10;    end
if ~isfield(o,'alphaFW'),        o.alphaFW        = 0;     end
if ~isfield(o,'fzFrontOffsetN'), o.fzFrontOffsetN = 0;     end
if ~isfield(o,'axLook'),         o.axLook         = 25;    end
if ~isfield(o,'ayCap'),          o.ayCap          = inf;   end
if ~isfield(o,'brkMargin'),      o.brkMargin      = 0.7;   end
if ~isfield(o,'ds'),             o.ds             = 1;     end
if ~isfield(o,'nIter'),          o.nIter          = 40;    end
end

% =========================================================================
function yq = interpHold(x, y, xq)
%INTERPHOLD Linear interpolation with the END VALUES HELD outside the data
% range, never extrapolated. Ride height outside the measured speed band is
% unknown; holding the nearest measured value keeps the result inside
% aeroLiveEval's map envelope, whereas a linear extrapolation of a squat curve
% runs straight off it. Holding is NOT a substitute for measuring the band the
% car actually uses -- see item 2's front-lift example.
[x, i] = sort(x(:));  y = y(:); y = y(i);
yq = interp1(x, y, xq(:), 'linear');
yq(xq(:) < x(1))   = y(1);
yq(xq(:) > x(end)) = y(end);
end
