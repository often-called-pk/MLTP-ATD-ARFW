function drv = buildDriverRef(matPath, varargin)
%BUILDDRIVERREF Driver reference path in the PLANT frame, from a solved lap.
%   drv = BUILDDRIVERREF(matPath) turns the racing line of a solved MLTP run
%   into the six arrays DriverPath.slx carries in its model workspace --
%   drvRefX, drvRefY, drvRefPsi, drvRefKap, drvRefVraw and drvN -- expressed
%   in the frame Plant.slx integrates its global position in (Body 6DOF
%   starts at Xe_o = [0 0 0], eul_o = [0 0 0], so the lap must start at the
%   origin heading along +x).
%
%   matPath (optional) - path to the raw sidecar .mat holding the full `data`
%       struct, relative to the repo root or absolute. Passed straight to
%       buildRefPath, which owns the default and the repo-root anchoring.
%
%   Name-value options (defaults are the shipped recipe -- change them only
%   with a re-bake, they do not describe the committed arrays otherwise)
%     'SmoothWindow'      odd integer, circular moving-average window used to
%                         de-kink the racing line. Default 5.
%     'ChordHalfLength'   integer [m], half-length of the chord the curvature
%                         is differenced over. Default 10 (a 20 m chord).
%     'StartAt'           where the lap starts, on the 1 m grid:
%                           'auto'   (default) keep the solved lap's own
%                                    s = 0 when it is already benign, else
%                                    shift the start into the longest
%                                    straight -- see the section below.
%                           'solved' (or 0) never shift.
%                           <scalar> shift the start forward by exactly that
%                                    many metres along the reference.
%
%   drv fields (column vectors of length drv.N unless stated)
%     X       [Nx1] reference-line x in the plant frame [m], X(1) = 0
%     Y       [Nx1] reference-line y in the plant frame [m], Y(1) = 0
%     Psi     [Nx1] path heading [rad], unwrapped, Psi(1) = 0
%     Kap     [Nx1] path curvature [1/m], ISO sign (+ = left turn)
%     Vraw    [Nx1] the solved MLTP speed profile vx(s) [m/s]
%     N       scalar, numel(X) -- what DriverPath stores as drvN
%     origin  [1x2] the translation SUBTRACTED before the rotation [m]
%     psi1    scalar, the heading ROTATED OUT [rad]. UNWRAPPED once the start
%             has been shifted -- it is a heading read off the lap's unwrapped
%             Psi, so it can leave +-pi by up to the lap's net turn. Only its
%             sine and cosine are ever used (here and in buildTrackRibbon), so
%             the value is left as measured rather than folded back into a
%             range by arithmetic that would not be exact.
%     sQ      [Nx1] arc length along the smoothed line, 0:1:N-1 [m]
%     startShift  scalar [m], how far the start was moved along the solved
%                 lap. 0 means the arrays start where the solver's s = 0 did.
%     startWhy    char, one sentence saying why that shift was chosen.
%     matPath resolved absolute path of the sidecar that was read
%
%   WHY THIS FILE EXISTS
%   --------------------
%   The six arrays above were baked into DriverPath's model workspace by
%   simulink/work/build_driver.m, which lived under the git-ignored
%   simulink/work/ tree and has since been deleted: nothing tracked could
%   reproduce them, and a new track therefore could not be driven at all.
%   (The same gap was closed for the speed plan by rebuildDriverPlan.m; this
%   is the step upstream of it.) The recipe survived only in the prose of
%   buildTrackRibbon.m's PLANT FRAME NOTE, which reproduced the origin and
%   heading -- and only those -- in a local subfunction. That subfunction now
%   delegates here, so the rigid transform has ONE owner.
%
%   THE RECIPE, and how each step was re-verified (2026-09-21)
%   ---------------------------------------------------------
%   Against the committed DriverPath.slx bake (BCN / ARFWr / ATD, N = 4586):
%     1. ref = buildRefPath(matPath) -- racing line on a uniform 1 m grid.
%     2. Drop the closing point when it is within 0.6 m of the first, so the
%        array is a cyclic sequence with no repeated station.
%     3. Circular 5-point moving average ('SmoothWindow'). This is not
%        cosmetic: cartPath's forward-difference normals put ~0.5 m spurs
%        into the line at the tightest corner, and the smoothing is what
%        moves psi1 from -0.066 deg (raw first two points) to +0.380 deg,
%        which is the value that registers against the driven pose.
%     4. Re-resample the smoothed loop to exactly 1 m: sQ = 0:1:floor(L)-1,
%        linear interp1 against its own cumulative segment length.
%     5. origin = first point, psi1 = heading of the first two points;
%        subtract the origin, then rotate by -psi1.
%        -> X, Y reproduce drvRefX / drvRefY EXACTLY (max abs err 0.0e+00 m).
%     6. Psi = unwrap(atan2(gradient(Y,sQ), gradient(X,sQ))).
%        -> reproduces drvRefPsi EXACTLY (max abs err 0.0e+00 rad).
%     7. Vraw: the raw line's vx re-parameterised by LAP FRACTION, i.e.
%        interp1(linspace(0,1,M+1), [vx; vx(1)], sQ/L). The smoothed loop is
%        ~1.9 m shorter than the raw one at BCN, and this global re-stretch
%        -- not a local index or arc-length map -- is what the bake used.
%        -> reproduces drvRefVraw EXACTLY (max abs err 0.0e+00 m/s).
%     8. Kap: CHORD estimate, the term buildSpeedPlan.m's header already used
%        for its `kap` input. Kap(i) = [Psi(i+h) - Psi(i-h)] / (2h) with
%        h = 10 m and a PERIODIC extension that adds exactly one net turn per
%        lap, turn = 2*pi*round((Psi(end)-Psi(1))/(2*pi)) (-2*pi at BCN).
%        -> reproduces drvRefKap to max abs err 5.9e-17 1/m, i.e. 9.5e-16
%        relative to max|Kap| = 0.0618 1/m -- machine precision, exact.
%        The wrap constant is load-bearing, not cosmetic: differencing the
%        unwrapped Psi endpoints instead of the exact 2*pi leaves a constant
%        2.6e-05 1/m bias on the 20 knots either side of the start line.
%        Rejected candidates and their max abs residual against the bake:
%        a gradient-of-gradient curvature estimate (sign flipped) 1.9e-01;
%        gradient(unwrap(Psi),sQ) 1.1e-01; circular moving average of that
%        gradient, best window 21 pts, 3.2e-03; the same chord at h = 9 or
%        11 m, 7.2e-03 / 6.1e-03. The estimator is sharply identified --
%        h = 10 m is not a fitted parameter, it is the one that lands on
%        machine epsilon.
%     drv.N = 4586 = the committed drvN, exactly.
%
%   STARTING THE LAP ON A STRAIGHT ('StartAt', added 2026-09-21)
%   ------------------------------------------------------------
%   The closed-loop driver launches at the plan speed with zero steer and no
%   preview history, so the first couple of seconds of every lap is a
%   transient. That is harmless when the solved lap's s = 0 sits on a straight
%   and it is not when s = 0 sits mid-corner. Measured at Spa, whose start
%   line is on the exit of La Source (reference radius down to 29 m by
%   s = 9 m): the lap completed, and past 50 m it tracked to max |n| 2.059 m
%   with zero off-track samples -- but ALL 4492 off-track samples of that lap,
%   max |n| 4.745 m, fell inside the first 26 m / 1.85 s. A start-line
%   artefact, not a controller defect, and retuning the driver for it would be
%   fixing the wrong thing.
%
%   'auto' therefore does nothing at all unless the start really is in a
%   corner, and the keep test is deliberately the more lenient of the two so
%   that a track which already works is never disturbed:
%
%     KEEP the solved start when the tightest reference radius over the first
%     HEAD_M = 100 m is above KEEP_R = 200 m.  Measured: Barcelona 2913 m and
%     Nurburgring 351 m keep their own s = 0, so every array this function
%     returns for them is bit-identical to the pre-2026-09-21 bake.  Spa's
%     29 m fails, and only then does the search below run.
%
%     SHIFT to the longest circular run of stations whose radius exceeds
%     STRAIGHT_R = 300 m, entered MARGIN_M = 30 m (or a third of the run,
%     whichever is shorter) so the new start is clear of the corner exit that
%     opened the straight and still has far more than the driver's 10 m
%     preview and 15 m speed look-ahead in front of it.
%
%   The shift is applied to the finished arrays, not to the line they are
%   built from: X/Y are re-framed on the new first point (origin = that point,
%   heading rotated out = that point's own path heading), Psi is the same
%   circular rotation re-zeroed there with the lap's one net turn added to the
%   knots that wrap past the old start line, and Kap and Vraw are plain
%   circshifts. Rotating rather than recomputing is what makes it EXACT:
%   measured at every K tried on all three tracks, Kap and Vraw match
%   circshift to 0.0e+00 and the re-framed line matches the rotated one to
%   6e-13 m. It also keeps gradient()'s one-sided end difference -- the only
%   start-dependent step in the recipe -- attached to the station it was
%   computed at, instead of letting it move to wherever the new start is. The
%   two descriptions differ by 1.8e-4 rad of Psi at 2 knots and 9.2e-6 1/m of
%   Kap at 8 of Spa's 6941 (0.013 % of max |Kap|), all of them at the OLD
%   start line. Nothing downstream needs to
%   know a shift happened: drv.origin/drv.psi1 carry the new plant frame,
%   setupTrack feeds the rotated Kap/Vraw to buildSpeedPlan and records the
%   shift in the track pack, DriverPath's arrays are all rotated together, and
%   buildTrackRibbon('PlantFrame', true) reads the pose from the same place it
%   always did.
%
%   Note that drvN is NOT referenced by any block dialog in DriverPath: the
%   lap length is a LITERAL 4586 inside 'DriverPath/Lap Manager/Par'. The
%   reference arrays themselves feed Constant blocks (Lap Manager/RefX,
%   /RefY), so their length is a compile-time signal dimension. A track with
%   a different N therefore needs that literal updated in lockstep with the
%   arrays, and a recompile.
%
%   Path resolution is repo-root anchored by buildRefPath via
%   mfilename('fullpath'), never a bare-name load -- a folder holding old
%   runs can sit on the MATLAB path and shadow a bare filename with a stale
%   one.
%
%   See also buildRefPath, buildTrackRibbon, buildSpeedPlan, rebuildDriverPlan.

if nargin < 1, matPath = ''; end

p = inputParser;
p.FunctionName = 'buildDriverRef';
addParameter(p, 'SmoothWindow',    5,  @(v) isscalar(v) && isnumeric(v) && v >= 1 && mod(v,2) == 1);
addParameter(p, 'ChordHalfLength', 10, @(v) isscalar(v) && isnumeric(v) && v >= 1 && v == round(v));
addParameter(p, 'StartAt', 'auto', @(v) (ischar(v) || isstring(v)) || ...
    (isscalar(v) && isnumeric(v) && isfinite(v)));
parse(p, varargin{:});
o = p.Results;

% ---- 1. racing line on a uniform 1 m arc-length grid --------------------
ref = buildRefPath(matPath);
xy  = ref.xy;
vx  = ref.vx;

% ---- 2. drop the duplicated closing point -------------------------------
if norm(xy(end,:) - xy(1,:)) < 0.6
    xy(end,:) = [];
    vx(end)   = [];
end
M = size(xy, 1);

% ---- 3. circular moving average (de-kink) -------------------------------
W   = o.SmoothWindow;
sh  = (-floor(W/2)):floor(W/2);
xyS = zeros(size(xy));
for i = 1:numel(sh)
    xyS = xyS + circshift(xy, -sh(i), 1);
end
xyS = xyS / numel(sh);

% ---- 4. re-resample the closed, smoothed loop to exactly 1 m ------------
seg = vecnorm(diff([xyS; xyS(1,:)]), 2, 2);
sS  = [0; cumsum(seg)];
sQ  = (0:floor(sS(end))-1).';
xq  = interp1(sS, [xyS(:,1); xyS(1,1)], sQ, 'linear');
yq  = interp1(sS, [xyS(:,2); xyS(1,2)], sQ, 'linear');
N   = numel(sQ);

h = o.ChordHalfLength;
assert(N > 2*h, 'buildDriverRef:tooShort', ...
    ['buildDriverRef: the smoothed lap is only %d m long, which cannot carry the ' ...
     '%d m curvature chord. Pass a smaller ''ChordHalfLength''.'], N, 2*h);

% ---- 5. plant start frame: translate by -origin, rotate by -psi1 --------
origin = [xq(1) yq(1)];
psi1   = atan2(yq(2) - yq(1), xq(2) - xq(1));
Rz     = [cos(-psi1) sin(-psi1); -sin(-psi1) cos(-psi1)];
P      = ([xq yq] - origin) * Rz;
X      = P(:,1);
Y      = P(:,2);

% ---- 6. heading ---------------------------------------------------------
Psi = unwrap(atan2(gradient(Y, sQ), gradient(X, sQ)));

% ---- 7. speed: the raw profile re-parameterised by lap fraction ---------
Vraw = interp1(linspace(0, 1, M+1).', [vx; vx(1)], sQ/sS(end), 'linear');

% ---- 8. curvature: chord estimate over +-h, periodic by one net turn ----
% The lap is closed, so the heading gains exactly an integer number of turns
% around it; taking that integer rather than the unwrapped endpoint
% difference is what makes the wrap-around knots exact (see the header).
turn = 2*pi*round((Psi(end) - Psi(1)) / (2*pi));
Pe   = [Psi(end-h+1:end) - turn; Psi; Psi(1:h) + turn];
Kap  = (Pe(2*h+1:end) - Pe(1:end-2*h)) / (2*h);

% ---- 9. start the lap on a straight, if it is not on one already --------
% Everything above is the committed recipe, untouched. The shift below ROTATES
% those arrays rather than recomputing them from a rotated line, for two
% reasons. It is exact: Kap and Vraw come out as an exact circshift and Psi as
% an exact circshift plus one constant, where a recomputation would disagree
% by up to 2e-4 1/m over the 2h knots around either array end, because
% gradient() degrades to a one-sided difference there and that artefact would
% otherwise move with the start. And it is safe: K = 0 cannot touch a single
% bit of the block above, which is what keeps Barcelona and Nurburgring
% bit-identical to their committed bakes.
[K, startWhy] = startShiftIdx(o.StartAt, Kap, N);
if K ~= 0
    % The frame: origin is the new first point, and the heading rotated out is
    % that point's own path heading. psi1 + Psi(K+1) is exactly that, because
    % Psi is measured in the frame psi1 was already rotated out of.
    origin = [xq(K+1) yq(K+1)];
    psi1   = psi1 + Psi(K+1);
    Rz     = [cos(-psi1) sin(-psi1); -sin(-psi1) cos(-psi1)];
    P      = (circshift([xq yq], -K, 1) - origin) * Rz;
    X      = P(:,1);
    Y      = P(:,2);
    % Heading: the same circular rotation, re-zeroed at the new start. The
    % knots that wrap past the old start line gain the lap's one net turn, so
    % the array stays unwrapped and continuous.
    Psi  = [Psi(K+1:end); Psi(1:K) + turn] - Psi(K+1);
    Kap  = circshift(Kap,  -K);
    Vraw = circshift(Vraw, -K);
end

drv = struct( ...
    'X',          X, ...
    'Y',          Y, ...
    'Psi',        Psi, ...
    'Kap',        Kap, ...
    'Vraw',       Vraw, ...
    'N',          N, ...
    'origin',     origin, ...
    'psi1',       psi1, ...
    'sQ',         sQ, ...
    'startShift', K, ...
    'startWhy',   startWhy, ...
    'matPath',    ref.matPath);

end

% =========================================================================
function [K, why] = startShiftIdx(startAt, Kap, N)
%STARTSHIFTIDX How many metres to rotate the lap so it starts on a straight.
% Kap is the curvature of the UNSHIFTED lap, on the same 1 m grid, so an
% index and a distance are the same number here. See the header for the
% reasoning and the measured numbers behind the three constants.
HEAD_M     = 100;      % how far past the solved start the keep test looks [m]
KEEP_R     = 200;      % keep the solved start above this radius over HEAD_M [m]
STRAIGHT_R = 300;      % a station counts as straight above this radius [m]
MARGIN_M   = 30;       % how far into the chosen straight the new start goes [m]

if isnumeric(startAt)
    K = mod(round(double(startAt)), N);
    why = sprintf('start shifted %d m by request (''StartAt'', %g)', K, startAt);
    if K == 0, why = 'solved start kept by request (''StartAt'', 0)'; end
    return
end

mode = lower(char(startAt));
switch mode
    case 'solved'
        K = 0;
        why = 'solved start kept by request (''StartAt'', ''solved'')';
        return
    case 'auto'
        % fall through
    otherwise
        error('buildDriverRef:badStartAt', ...
            ['buildDriverRef: ''StartAt'' must be ''auto'', ''solved'' or a distance in ' ...
             'metres; got ''%s''.'], mode);
end

nHead  = min(HEAD_M, N);
kHead  = max(abs(Kap(1:nHead)));
rHead  = 1/max(kHead, eps);
if kHead < 1/KEEP_R
    K = 0;
    why = sprintf(['solved start kept: tightest radius over the first %d m is %.0f m ' ...
                   '(keep above %d m)'], nHead, rHead, KEEP_R);
    return
end

low = abs(Kap) < 1/STRAIGHT_R;
if ~any(low)
    K = 0;
    why = sprintf(['solved start kept: the start is in a corner (R = %.0f m over the first ' ...
                   '%d m) but no station on the lap is straighter than R = %d m, so there ' ...
                   'is nowhere better to start'], rHead, nHead, STRAIGHT_R);
    warning('buildDriverRef:noStraight', 'buildDriverRef: %s.', why);
    return
end

[runLen, runStart] = longestCircularRun(low);
into = min(MARGIN_M, floor(runLen/3));
K    = mod(runStart - 1 + into, N);
why  = sprintf(['start moved %d m: the solved start is in a corner (R = %.0f m over the ' ...
                'first %d m); the longest straight (R > %d m) is %d m from s = %d m, ' ...
                'entered %d m'], K, rHead, nHead, STRAIGHT_R, runLen, runStart-1, into);
end

% =========================================================================
function [L, a] = longestCircularRun(mask)
%LONGESTCIRCULARRUN Length and 1-based start of the longest true run, wrapping.
mask = logical(mask(:));
N    = numel(mask);
if all(mask), L = N; a = 1; return, end

% Rotating so that index 1 is FALSE turns the circular problem into a linear
% one: no run can then straddle the array ends.
z  = find(~mask, 1);
m2 = mask([z:N, 1:z-1]);
d  = diff([false; m2; false]);
st = find(d == 1);
en = find(d == -1) - 1;
[L, j] = max(en - st + 1);
a = mod(z - 1 + st(j) - 1, N) + 1;
end
