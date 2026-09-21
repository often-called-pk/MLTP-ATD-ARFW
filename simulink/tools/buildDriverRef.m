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
%
%   drv fields (column vectors of length drv.N unless stated)
%     X       [Nx1] reference-line x in the plant frame [m], X(1) = 0
%     Y       [Nx1] reference-line y in the plant frame [m], Y(1) = 0
%     Psi     [Nx1] path heading [rad], unwrapped, Psi(1) = 0
%     Kap     [Nx1] path curvature [1/m], ISO sign (+ = left turn)
%     Vraw    [Nx1] the solved MLTP speed profile vx(s) [m/s]
%     N       scalar, numel(X) -- what DriverPath stores as drvN
%     origin  [1x2] the translation SUBTRACTED before the rotation [m]
%     psi1    scalar, the heading ROTATED OUT [rad]
%     sQ      [Nx1] arc length along the smoothed line, 0:1:N-1 [m]
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

drv = struct( ...
    'X',       X, ...
    'Y',       Y, ...
    'Psi',     Psi, ...
    'Kap',     Kap, ...
    'Vraw',    Vraw, ...
    'N',       N, ...
    'origin',  origin, ...
    'psi1',    psi1, ...
    'sQ',      sQ, ...
    'matPath', ref.matPath);

end
