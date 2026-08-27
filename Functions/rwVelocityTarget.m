function [target, tv, sOn] = rwVelocityTarget(vx, Tbrake, TbMax, opts)
%RWVELOCITYTARGET Rear-wing angle demanded by a velocity schedule + brake trigger.
%
%   target            = rwVelocityTarget(vx, Tbrake, TbMax)
%   target            = rwVelocityTarget(vx, Tbrake, TbMax, opts)
%   [target, tv, sOn] = rwVelocityTarget(...)
%
%   The whole function is this:
%
%       s1     = 0.5*(1 + tanh((vx - v1)/w1))          % corner -> mid
%       s2     = 0.5*(1 + tanh((vx - v2)/w2))          % mid    -> straight
%       tv     = aCorner + (aMid - aCorner).*s1 + (aStraight - aMid).*s2
%       Tb_mag = tbSign * Tbrake                       % tbSign = -1, see SIGN
%       sOn    = 0.5*(1 + tanh((Tb_mag - onFrac*TbMax)/(w1frac*TbMax)))
%                  then affinely re-zeroed at Tbrake = 0 (zeroTrim, below)
%       target = tv + sOn .* (aBrake - tv)
%
%   Two schedules blended by one switch. `tv` is a three-level staircase in speed,
%   monotone decreasing: high downforce at corner speeds, mid at intermediate, low
%   drag on the straights. Levels are exact - between the switches s1=1, s2=0 gives
%   aMid identically, and the limits give aCorner and aStraight. `sOn` is the brake
%   trigger: exactly 0 off brake (see zeroTrim), so target == tv there; under
%   braking it rises to 1 and pulls the target to aBrake regardless of speed.
%
%   WHY THE BRAKE TRIGGER IS NOT OPTIONAL. A function of vx alone cannot detect
%   deceleration - the car passes a given speed both accelerating out of a corner
%   (wing coming down) and braking into the next (wing going up), and no
%   single-valued f(vx) serves both. Brake torque is what breaks the tie, which is
%   why a velocity-only law cannot carry an airbrake at all.
%
%   WHY THE TRANSITION WIDTH w CANNOT BE SMALL. The schedule is chased along the
%   trajectory, so the rate it demands of the actuator is
%       |d(tv)/dt| = |d(tv)/d(vx)| * |ax|,   max |d(tv)/d(vx)| = |aMid-aCorner|/(2w)
%   giving a hard floor w >= |aMid - aCorner|*max|ax| / (2*c.ub.RW). Because the
%   staircase is three-level, each switch moves only |aMid - aCorner| rather than
%   the full span, halving the demand for a given width. The opposing constraint is
%   plateau width: the switches must be far enough apart for tv to actually reach
%   aMid between them, needing v2 - v1 >~ 4w. Too small and the actuator cannot
%   follow; too large and the mid plateau erodes, collapsing the three-level
%   staircase into a two-level slide. MLTP.m asserts both rather than trusting this
%   comment. Switch speeds and widths are fitted per track from a solved free-wing
%   lap, not hand-picked.
%
%   SIGN AND SMOOTHNESS. Tbrake is passed SIGNED and the magnitude is formed as
%   tbSign*Tbrake with tbSign = -1, NOT as abs(Tbrake): abs() puts a kink in the
%   constraint Jacobian at exactly T_brake = 0, where most of the lap sits, which
%   is a reliable way to stall a barrier method. Keeping the magnitude LINEAR in
%   the decision variable avoids it. Everything here is tanh and arithmetic, so the
%   composed target is C-infinity in both vx and Tbrake.
%
%   INPUTS
%     vx      longitudinal speed [m/s]. Numeric array of any shape, or casadi
%             SX/MX/DM. Elementwise.
%     Tbrake  brake torque, SIGNED, model convention (<= 0) [Nm]. Same shape as vx.
%     TbMax   brake-torque bound magnitude [Nm], NUMERIC positive scalar
%             (vp.Tbrake_max). Every brake-switch quantity is a FRACTION of it, so
%             the switch point tracks the bound if it is re-sized.
%     opts    optional struct, all fields optional:
%               aStraight  -10    angle demanded at high speed   [deg]
%               aMid         0    angle demanded at mid speed    [deg]
%               aCorner    +10    angle demanded at low speed    [deg]
%               aBrake     +15    angle demanded under braking   [deg]
%               v1        37.5    corner -> mid switch speed     [m/s]
%               v2        63.0    mid -> straight switch speed   [m/s]
%               w1         3.5    corner -> mid half-width       [m/s]
%               w2         3.5    mid -> straight half-width     [m/s]
%               brakeOpts   struct passed to brakeSwitch() below. Fields read,
%                           with defaults: onFrac 0.05, w1frac 0.04, tbSign -1,
%                           zeroTrim false. Defaults here to zeroTrim = true.
%
%   OUTPUTS
%     target  demanded wing angle [deg], same size/type as vx.
%     tv      the velocity schedule alone [deg] (target with the brake trigger off).
%     sOn     brake-on blend in [0,1].
%
%   ZERO-BRAKE EXACTNESS. brakeOpts defaults to zeroTrim = true so that
%   sOn(Tbrake = 0) == 0 exactly and target == tv off brake. Without it sOn(0) =
%   0.0759, pulling every non-braking knot 0.0759*(aBrake - tv) toward the
%   airbrake - roughly +1.9 deg on the straights, i.e. most of the low-drag benefit
%   the schedule exists to produce.
%
%   SINGLE IMPLEMENTATION - NO SYMBOLIC/NUMERIC TWIN, BY CONSTRUCTION. The body is
%   pure elementwise arithmetic and tanh, every operator of which MATLAB overloads
%   identically for casadi.SX, so this one file is both the symbolic implementation
%   that builds the constraint rows and the numeric one used by post-processing and
%   the gates. There is no second copy, so the two cannot drift. Keep it that way:
%   no if/else, switch, min/max, abs, interp1 or data-dependent indexing on a VALUE
%   of vx or Tbrake. Branching on nargin or on numeric `opts` fields is
%   configuration, not data, and is fine.
%
%   See also RWAERODELTA, REACTIVEWINGTARGET.

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

% Brake trigger. The logistic is brakeSwitch() at the bottom of this file.
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
%BRAKESWITCH Brake-on logistic in [0,1], shared by every wing-schedule mode.
%
%  SIGN. Tbrake is passed SIGNED and tbSign = -1 is applied HERE rather than using
%  abs(): abs() puts a kink in the constraint Jacobian at exactly T_brake = 0,
%  where most of the lap sits, and IPOPT does not converge through it. The
%  magnitude must stay LINEAR in the decision variable.
%
%  THE ZERO-BRAKE FLOOR (zeroTrim). Untrimmed, sOn(0) = 0.0759, dragging every
%  off-brake target about +1.9 deg toward the airbrake. sOn0 is built from opts
%  alone with no dependence on Tbrake, so the re-zero stays an elementwise affine
%  map and the SX graph keeps its smoothness.
onFrac   = getfielddef(opts, 'onFrac',   0.05);
w1frac   = getfielddef(opts, 'w1frac',   0.04);
tbSign   = getfielddef(opts, 'tbSign',  -1);
zeroTrim = getfielddef(opts, 'zeroTrim', false);

% TbMax is always numeric (vp.Tbrake_max), so this assert is SX-safe. It guards
% SILENT failures: TbMax = [] gives sOn = [] and the constraint rows simply vanish
% from the NLP with nothing erroring; TbMax < 0 inverts the switch, so the wing is
% demanded BELOW the schedule exactly where the airbrake should deploy - wrong
% way, no NaN, no complaint. TbMax = 0 divides by zero.
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
