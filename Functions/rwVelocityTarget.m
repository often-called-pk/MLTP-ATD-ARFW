function [target, tv, sOn] = rwVelocityTarget(vx, Tbrake, TbMax, opts)
%RWVELOCITYTARGET Rear-wing angle demanded by the VELOCITY schedule + brake trigger.
%
%   target              = rwVelocityTarget(vx, Tbrake, TbMax)
%   target              = rwVelocityTarget(vx, Tbrake, TbMax, opts)
%   [target, tv, sOn]   = rwVelocityTarget(...)
%
%   FORMULA (the whole function - there is nothing else)
%
%       s1     = 0.5*(1 + tanh((vx - v1)/w1))          % corner -> mid
%       s2     = 0.5*(1 + tanh((vx - v2)/w2))          % mid    -> straight
%       tv     = aCorner + (aMid - aCorner).*s1 + (aStraight - aMid).*s2
%       Tb_mag = tbSign * Tbrake                       % tbSign = -1, see SIGN below
%       sOn    = 0.5*(1 + tanh((Tb_mag - onFrac*TbMax)/(w1frac*TbMax)))
%                  then affinely re-zeroed at Tbrake = 0 (zeroTrim, below)
%       target = tv + sOn .* (aBrake - tv)
%
%   The brake switch is the local function brakeSwitch() at the bottom of this
%   file. It was absorbed verbatim from Functions\rwMandateTarget.m when the
%   braking-mandate study was retired (2026-08-04); see its own header for the
%   sign and zeroTrim rationale in full.
%
%   Two independent schedules blended by ONE switch:
%
%     * `tv` is the VELOCITY schedule: a THREE-LEVEL staircase, monotone DECREASING
%       in vx, built from two stacked logistic switches - the same construction
%       brakeSwitch() below applies to brake torque, here applied to speed and
%       used twice. High downforce (aCorner = +10 deg) at corner speeds, the
%       MID setting (aMid = 0 deg) at intermediate speeds, low drag
%       (aStraight = -10 deg) on the straights. This is the production speed-lookup
%       wing the study is about, and it uses all three static downforce settings
%       rather than sliding between the extremes.
%
%       Level check: vx -> 0 gives s1=s2=0 -> aCorner. Between the switches s1=1,
%       s2=0 -> aCorner + (aMid-aCorner) = aMid EXACTLY. vx -> inf gives s1=s2=1 ->
%       aMid + (aStraight-aMid) = aStraight. At the plateau centre
%       (v1+v2)/2 = 50.25 m/s with the shipped constants the residual is 6e-4 deg.
%     * `sOn` is the brake trigger. Off brake it is 0 EXACTLY (zeroTrim, below), so
%       `target == tv` and the velocity schedule alone governs. Under braking it
%       rises to 1 and pulls the target to `aBrake` (+15 deg) regardless of speed.
%
%   WHY THE BRAKE TRIGGER IS NOT OPTIONAL. A function of vx alone cannot detect
%   deceleration: the car passes 60 m/s both accelerating out of a corner (wing must
%   be coming DOWN) and braking into the next one (wing must be going UP). No
%   single-valued f(vx) can serve both, which is the measured R^2 <= 0.33-0.68
%   ceiling on any velocity-only fit. The brake torque is what breaks the tie, so the
%   hybrid is the only velocity-scheduled law that can carry an airbrake at all.
%
%   ONE AIRBRAKE SETTING, NOT TWO. `aBrake` defaults to +15 deg and there is no
%   second, harder step: the +20 deg setting is deliberately absent from this variant
%   (user decision 2026-08-02). The retired braking mandate's two-stage
%   0 -> +15 -> +20 schedule is therefore NOT reproduced here: only its FIRST stage,
%   the on-switch, was carried across into brakeSwitch(), which is why that function
%   has no sHard/aHard - a single switch between `tv` and one airbrake angle is the
%   whole brake behaviour of this variant. The +20 deg well is therefore absent from
%   THIS variant's snap settings only; the mandate variants that used it carried
%   their own schedules and their own wells, and were never altered by this law.
%
%   SIGN AND SMOOTHNESS. `Tbrake` is passed SIGNED and brakeSwitch() forms the
%   magnitude as tbSign*Tbrake with tbSign = -1, NOT as abs(Tbrake): abs() would put
%   a kinked constraint Jacobian exactly at the T_brake = 0 bound, where most of the
%   lap sits, which is the classic way to stall a barrier method. Keeping the
%   magnitude LINEAR in the decision variable is what avoids that; brakeSwitch's own
%   header carries the argument in full. Everything here is tanh and arithmetic, so
%   the composed target is C-infinity in both vx and Tbrake.
%
%   ACTUATOR FEASIBILITY - WHY w IS NOT FREE TO BE SMALL
%   ----------------------------------------------------
%   The velocity schedule is chased through the trajectory, so the rate it DEMANDS of
%   the actuator is
%       |d(tv)/dt| = |d(tv)/d(vx)| * |ax| ,   max |d(tv)/d(vx)| = |aStraight-aCorner|/(2w)
%   Two different numbers follow from that and they must not be confused:
%
%     BOUND  = max|d(tv)/d(vx)| * max|ax|  - the worst case, assuming peak schedule
%              slope and peak braking coincide. This is what MLTP.m asserts on,
%              because a solve may put them anywhere.
%     TRACE  = max over the lap of the ACTUAL product - lower, because on the solved
%              lap the schedule is steepest at v1/v2 while the hardest braking
%              happens elsewhere.
%
%   Measured on the solved free-ActiveRW BCN/AWD lap (|ax| peaks at 25.19 m/s^2)
%   against c.ub.RW = 60 deg/s. Because the staircase is THREE-level, each switch
%   moves only |aMid - aCorner| = 10 deg rather than the full 20, which halves the
%   demanded rate for a given width:
%
%   w is squeezed from BOTH sides and the shipped value is where they balance. The
%   last column is the share of the solved lap the schedule would hold within 0.5 deg
%   of the MID setting - the thing a wider w destroys:
%
%       w [m/s]   BOUND [deg/s]   4w [m/s]   mid dwell   verdict
%         2.0         63.0          8.0        ~13 %     INFEASIBLE (actuator)
%         3.0         42.0         12.0        12.0 %    ok
%         3.5         36.0         14.0        10.7 %    shipped default
%         4.0         31.5         16.0         9.2 %    ok
%         5.0         25.2         20.0         7.5 %    mid plateau thinning
%         6.0         21.0         24.0         5.6 %    4w ~ v2-v1: plateau eroded
%
%   Hard floor: w >= |aMid - aCorner| * max|ax| / (2 * c.ub.RW) = 2.10 m/s. The
%   shipped w = 3.5 m/s clears it 1.7x over, and its demand (36.0 deg/s) is still
%   BELOW what the free continuous optimum already uses on the same lap (38.3 deg/s)
%   - so the schedule cannot be the binding actuator constraint. MLTP.m asserts this
%   rather than trusting the comment.
%
%   PLATEAU WIDTH is the opposing constraint: the two switches must be far enough
%   apart for tv to actually REACH aMid between them, which needs v2 - v1 >~ 4w.
%   Shipped: v2 - v1 = 25.5 m/s against 4w = 14 m/s, so the plateau is well formed
%   (residual 6e-4 deg at its centre). Raising w erodes it; MLTP.m asserts that too,
%   because a silently-eroded plateau turns the three-level staircase back into the
%   two-level slide it was deliberately changed away from.
%
%   v1 = 37.5 and v2 = 63.0 m/s are measured, not guessed. On the same free lap,
%   restricted to OFF-BRAKE knots (where the velocity schedule is what governs), the
%   optimum's own median speed is 27.3 m/s where it sits above +5 deg, 47.9 m/s where
%   it sits within +-2.5 deg of zero, and 78.3 m/s where it sits below -5 deg. The
%   switches are placed midway between adjacent medians.
%
%   INPUTS
%     vx      longitudinal speed [m/s]. Numeric array of any shape, or casadi
%             SX/MX/DM. Elementwise.
%     Tbrake  brake torque, SIGNED, model convention (<= 0) [Nm]. Same shape as vx.
%     TbMax   brake-torque bound magnitude [Nm], NUMERIC positive scalar
%             (vp.Tbrake_max). Every brake-switch quantity is expressed as a
%             FRACTION of it, so the switch point moves automatically if the bound
%             is re-sized. Asserted positive/finite/scalar in brakeSwitch().
%     opts    optional struct, all fields optional:
%               aStraight  -10    angle demanded at high speed   [deg]
%               aMid         0    angle demanded at mid speed    [deg]
%               aCorner    +10    angle demanded at low speed    [deg]
%               aBrake     +15    angle demanded under braking   [deg]
%               v1        37.5    corner -> mid switch speed     [m/s]
%               v2        63.0    mid -> straight switch speed   [m/s]
%               w1         3.5    corner -> mid half-width       [m/s]
%               w2         3.5    mid -> straight half-width     [m/s]
%               brakeOpts   struct configuring the sOn switch, passed to
%                           brakeSwitch() below. Fields it reads, with defaults:
%                           onFrac 0.05, w1frac 0.04, tbSign -1, zeroTrim false.
%                           Any other field is ignored. Defaults here to
%                           struct('zeroTrim',true) - see below.
%
%   OUTPUTS
%     target  demanded wing angle [deg], same size/type as vx.
%     tv      the velocity schedule alone [deg] (target with the brake trigger off).
%     sOn     brake-on blend in [0,1].
%
%   ZERO-BRAKE EXACTNESS. brakeOpts defaults to zeroTrim = true so sOn(Tbrake = 0)
%   == 0 EXACTLY and `target == tv` off brake. Without it sOn(0) = 0.0759 and every
%   non-braking knot would be pulled 0.0759*(15 - tv) toward the airbrake - about
%   +1.9 deg on the straights, which is most of the low-drag benefit this variant
%   exists to prescribe. The retired braking mandate this switch came from documented
%   the same trap; it bites harder here because tv is far from aBrake off brake.
%
%   SINGLE IMPLEMENTATION - NO SYMBOLIC/NUMERIC TWIN, BY CONSTRUCTION. Same argument
%   as RWSNAPPENALTY: the body is pure elementwise arithmetic and tanh, every
%   operator of which MATLAB overloads identically for casadi.SX, so this
%   one file is both the symbolic implementation that builds the constraint rows and
%   the numeric one used by post-processing and the gate. No second copy, so the two
%   cannot drift. Keep it that way: no if/else, switch, min/max, abs, interp1 or
%   data-dependent indexing on a VALUE of vx or Tbrake. Branching on nargin or on the
%   numeric `opts` fields is configuration, not data, and is fine.
%
%   See also RWSNAPPENALTY, RWAERODELTA.

% ---- defaults (branching on nargin/opts is configuration, not data) --------
if nargin < 4 || isempty(opts),  opts = struct();  end
assert(isstruct(opts) && isscalar(opts), 'rwVelocityTarget:opts', ...
    'opts must be a scalar struct');

aStraight = getfielddef(opts, 'aStraight', -10);
aMid      = getfielddef(opts, 'aMid',        0);
aCorner   = getfielddef(opts, 'aCorner',   +10);
aBrake    = getfielddef(opts, 'aBrake',    +15);
v1        = getfielddef(opts, 'v1',       37.5);
v2        = getfielddef(opts, 'v2',       63.0);
w1        = getfielddef(opts, 'w1',        3.5);
w2        = getfielddef(opts, 'w2',        3.5);
brakeOpts = getfielddef(opts, 'brakeOpts',  struct('zeroTrim', true));

% ---- validation ------------------------------------------------------------
chkNum(aStraight, 'aStraight');
chkNum(aMid,      'aMid');
chkNum(aCorner,   'aCorner');
chkNum(aBrake,    'aBrake');
chkNum(v1,        'v1');
chkNum(v2,        'v2');
chkNum(w1,        'w1');
chkNum(w2,        'w2');
assert(w1 > 0 && w2 > 0, 'rwVelocityTarget:width', ...
    'opts.w1 and opts.w2 must be strictly positive (transition half-widths in m/s)');
assert(v2 > v1, 'rwVelocityTarget:switchOrder', ...
    ['opts.v2 (%.2f) must exceed opts.v1 (%.2f): the staircase is monotone ' ...
     'DECREASING in speed, so the corner->mid switch has to come first.'], v2, v1);
assert(isstruct(brakeOpts) && isscalar(brakeOpts), 'rwVelocityTarget:brakeOpts', ...
    'opts.brakeOpts must be a scalar struct');

assert(~isempty(vx), 'rwVelocityTarget:vxEmpty', 'vx must be non-empty');
if ~iscasadiobj(vx)
    assert(isnumeric(vx) && isreal(vx) && all(isfinite(vx(:))), ...
        'rwVelocityTarget:vxType', 'vx must be a finite real array or a casadi SX/MX/DM');
end

% ---- the formula (SX and double take THESE lines) --------------------------
% Three-level velocity staircase, decreasing in vx. Both switches are 0 at low speed
% and 1 at high speed, so the levels accumulate: aCorner -> aMid -> aStraight.
s1 = 0.5 .* (1 + tanh((vx - v1)./w1));      % corner -> mid
s2 = 0.5 .* (1 + tanh((vx - v2)./w2));      % mid    -> straight
tv = aCorner + (aMid - aCorner).*s1 + (aStraight - aMid).*s2;

% Brake trigger. The logistic lives in brakeSwitch() at the bottom of this file -
% absorbed verbatim from the retired rwMandateTarget when the mandate study was
% removed (2026-08-04). aLight/aHard were only ever consumed by that function's
% discarded `target` output and are gone with it.
sOn = brakeSwitch(Tbrake, TbMax, brakeOpts);

% Blend. sOn == 0 off brake (zeroTrim) => target == tv exactly.
target = tv + sOn .* (aBrake - tv);

end

% =========================================================================
function chkNum(v, name)
%CHKNUM Numeric scalar option guard (always safe - options are never symbolic).
assert(isnumeric(v) && isreal(v) && isscalar(v) && isfinite(v), ...
    'rwVelocityTarget:optType', 'opts.%s must be a finite real scalar', name);
end

% =========================================================================
function sOn = brakeSwitch(Tbrake, TbMax, opts)
%BRAKESWITCH The brake-on logistic, absorbed verbatim from the retired
%  Functions\rwMandateTarget.m (its second output). Only the sOn half of that
%  function was ever on the ARWv path - aLight/aHard/sHard/target were computed
%  and discarded by the `[~, sOn]` call this replaces.
%
%  SIGN. Tbrake is passed SIGNED and tbSign = -1 is applied HERE rather than
%  abs() being used: abs() puts a kink in the constraint Jacobian at exactly
%  T_brake = 0, which is where most of the lap sits, and IPOPT does not
%  converge through it. The magnitude must stay LINEAR in the decision variable.
%
%  THE ZERO-BRAKE FLOOR (zeroTrim). Untrimmed, sOn(0) = 0.0759, which drags
%  every off-brake target about +1.9 deg toward the airbrake - most of the
%  low-drag benefit this variant exists to prescribe. sOn0 is a NUMERIC
%  constant built from opts alone (no dependence on Tbrake), so the re-zero
%  stays an elementwise affine map and the SX graph keeps its smoothness.
onFrac   = getfielddef(opts, 'onFrac',   0.05);
w1frac   = getfielddef(opts, 'w1frac',   0.04);
tbSign   = getfielddef(opts, 'tbSign',  -1);
zeroTrim = getfielddef(opts, 'zeroTrim', false);

% TbMax is ALWAYS numeric (it is vp.Tbrake_max), so this assert is SX-safe. It
% guards SILENT failures, not loud ones. TbMax = [] gives w1 = [] -> sOn = [] ->
% target = [], so the constraint rows would simply vanish from the NLP with
% nothing erroring anywhere. TbMax < 0 INVERTS the switch in Tbrake: measured
% with the shipped brakeOpts, sOn stays 0 off brake (the zeroTrim constant does
% not depend on TbMax) but reaches -0.0821 at full brake instead of +1, so the
% wing is demanded 0.0821*(aBrake - tv) BELOW the velocity schedule exactly where
% the airbrake was supposed to deploy - wrong way, no NaN, no complaint. TbMax = 0
% divides by zero. Inherited from rwMandateTarget, which asserted this before the
% switch was absorbed - keep it.
assert(isnumeric(TbMax) && isreal(TbMax) && isscalar(TbMax) && ...
    isfinite(TbMax) && TbMax > 0, 'rwVelocityTarget:TbMax', ...
    'TbMax must be a positive finite real scalar [Nm] (vp.Tbrake_max)');
chkNum(onFrac,   'onFrac');
chkNum(w1frac,   'w1frac');
chkNum(tbSign,   'tbSign');
assert(w1frac > 0, 'rwVelocityTarget:width', ...
    'brakeOpts.w1frac must be strictly positive (it is a transition half-width)');
assert(isscalar(zeroTrim) && (islogical(zeroTrim) || isnumeric(zeroTrim)), ...
    'rwVelocityTarget:zeroTrim', 'brakeOpts.zeroTrim must be a logical scalar');

assert(~isempty(Tbrake), 'rwVelocityTarget:TbrakeEmpty', 'Tbrake must be non-empty');
if ~iscasadiobj(Tbrake)
    assert(isnumeric(Tbrake) && isreal(Tbrake), 'rwVelocityTarget:TbrakeType', ...
        'Tbrake must be a real numeric array or a casadi.SX/MX/DM');
    assert(all(isfinite(Tbrake(:))), 'rwVelocityTarget:TbrakeFinite', ...
        'Tbrake must be finite');
end

% ---- the formula (SX and double take THESE lines) --------------------------
w1 = w1frac * TbMax;

Tb_mag = tbSign * Tbrake;                                   % linear, not abs()

sOn = 0.5*(1 + tanh((Tb_mag - onFrac*TbMax)./w1));

if zeroTrim
    sOn0 = 0.5*(1 + tanh(-onFrac/w1frac));
    sOn  = (sOn - sOn0)./(1 - sOn0);
end
end

% =========================================================================
function tf = iscasadiobj(x)
%ISCASADIOBJ True for CasADi symbolic/matrix types. Safe when CasADi is absent:
% isa() on a class that does not exist simply returns false.
tf = isa(x, 'casadi.SX') || isa(x, 'casadi.MX') || isa(x, 'casadi.DM');
end
