function ref = buildRefPath(matPath)
%BUILDREFPATH Build a uniform-arc-length reference path from a solved MLTP run.
%   ref = BUILDREFPATH(matPath) loads the FULL `data` struct written by an
%   MLTP entry point's raw sidecar (solutions/report/<track>/raw/
%   run_<track>_<config>_<dv>_data.mat, variable `data` -- NOT the packed
%   report R struct, which does not carry x_full/track.x/y) and reconstructs
%   the optimised racing line, resampled to a uniform 1 m arc-length grid.
%   Feeds the Simulink ARFWr RT sim's Predictive Driver block (design
%   decision: "rebuild via curv2cart + n(s)").
%
%   matPath (optional) - path to the raw sidecar .mat, relative to the repo
%       root or absolute. Default: whatever simulink/tools/activeTrack.m
%       resolves, i.e. the track the sim is currently set up for. With no
%       simulink/data/activeTrack.mat on disk that is
%       'solutions/report/BCN/raw/run_BCN_ARFWr_ATD_data.mat', exactly as it
%       was before the resolver existed.
%
%   Method (mirrors Scripts/MLTP.m's own post-processing, e.g. MLTP.m:832-838):
%     1. Centreline (x0,y0) from data.track.x/y if present, else rebuilt via
%        curv2cart(data.track.s, data.track.k) -- same call MLTP.m makes
%        (default o=1 left-hand-positive-k, theta=0).
%     2. Racing line = centreline offset by n = data.x_full(4,:) via
%        Functions/cartPath.m, the SAME helper MLTP.m uses for
%        data.track.xopt/yopt. Both live on data's FULL grid (knots +
%        interior collocation points), same length as data.x_full/s_full.
%     3. Arc length ref.s is the CUMULATIVE SEGMENT LENGTH of that racing
%        line (not the centreline s / not data.s_full) -- the line is an
%        offset path and is longer through corners, shorter is not assumed.
%     4. t(s): data.t_opt lives on the coarser KNOT grid (N+1 points), not
%        the FULL grid data.s_full/x_full sit on (see
%        Functions/report/packRun.m's header comment on this exact split).
%        The knot positions are recovered as an even stride through the full
%        grid (mirrors packRun.m's strideToKnot, but returns indices so they
%        can be used to look up the racing line's own arc length at each
%        knot) and t is then interpolated against that knot-arc-length pair.
%     5. Every channel (xy, vx, t) is linearly resampled onto a uniform 1 m
%        ref.s grid; ref.psi is atan2 of the gradient of the RESAMPLED xy,
%        unwrapped -- so it is already on the uniform grid, no re-pad needed.
%
%   ref fields (column vectors, all length numel(ref.s)):
%     ref.xy         [Nx2]  racing-line Cartesian coordinates [m]
%     ref.s          [Nx1]  arc length along the racing line [m], 0:1:end
%     ref.vx         [Nx1]  longitudinal speed vx(s) [m/s]
%     ref.t          [Nx1]  lap time t(s) [s]
%     ref.psi        [Nx1]  path heading psi(s) [rad], unwrapped
%     ref.closed     logical, true if the RAW (pre-resample) endpoint gap <= 5 m
%     ref.closeGap   [m]    RAW endpoint gap norm(xy(end,:)-xy(1,:))
%     ref.matPath    resolved absolute path of the sidecar actually loaded
%
%   Errors with a fieldnames() dump on any missing required field. Warns
%   (does not error) if the endpoint gap exceeds 5 m.
%
%   Path resolution is repo-root anchored via mfilename('fullpath') (same
%   idiom as Functions/apexCsvPath.m) -- NEVER a bare-name load: a folder
%   holding old runs can sit on the MATLAB path (genpath) and shadow a bare
%   filename with a stale one.

if nargin < 1 || isempty(matPath)
    matPath = activeTrack();       % single owner of "which track is active"
end

toolsDir = fileparts(mfilename('fullpath'));         % ...\simulink\tools
repoRoot = fileparts(fileparts(toolsDir));            % ...\simulink\tools -> simulink -> repo root

if ~java.io.File(matPath).isAbsolute()
    matPath = fullfile(repoRoot, matPath);
end

if ~isfile(matPath)
    error('buildRefPath:matNotFound', 'buildRefPath: raw sidecar not found: %s', matPath);
end

S = load(matPath);
if ~isfield(S, 'data')
    error('buildRefPath:noData', ...
        'buildRefPath: %s has no ''data'' variable. Variables found: {%s}', ...
        matPath, strjoin(fieldnames(S)', ', '));
end
data = S.data;

req = {'track', 'x_full', 's_full', 't_opt'};
missing = req(~isfield(data, req));
if ~isempty(missing)
    error('buildRefPath:missingField', ...
        'buildRefPath: data is missing field(s) {%s}. data fields: {%s}', ...
        strjoin(missing, ', '), strjoin(fieldnames(data)', ', '));
end

% -- centreline ---------------------------------------------------------
track = data.track;
if isfield(track, 'x') && isfield(track, 'y') && ~isempty(track.x) && ~isempty(track.y)
    x0 = track.x(:);
    y0 = track.y(:);
elseif isfield(track, 's') && isfield(track, 'k')
    [x0, y0] = curv2cart(track.s, track.k);   % same call as MLTP.m:832
    x0 = x0(:);
    y0 = y0(:);
else
    error('buildRefPath:missingTrackXY', ...
        'buildRefPath: data.track has neither x/y nor s/k. data.track fields: {%s}', ...
        strjoin(fieldnames(track)', ', '));
end

% -- racing line = centreline offset by n, on data's FULL grid ----------
n  = data.x_full(4, :);   % distance to centreline [m]
vx = data.x_full(1, :);   % longitudinal speed [m/s]

if numel(x0) ~= numel(n)
    error('buildRefPath:gridMismatch', ...
        ['buildRefPath: data.track x/y has %d points but n = data.x_full(4,:) has %d -- ' ...
         'centreline and control/state grids do not match.'], numel(x0), numel(n));
end

[xLine, yLine] = cartPath(x0, y0, n);      % Functions/cartPath.m, same as data.track.xopt/yopt
xy = [xLine(:), yLine(:)];

% -- closed-loop check (on the RAW, pre-resample line) -------------------
closeGap = norm(xy(end, :) - xy(1, :));
closed   = closeGap <= 5;
if ~closed
    warning('buildRefPath:notClosed', ...
        'buildRefPath: racing-line endpoint gap is %.3f m (> 5 m threshold).', closeGap);
end

% -- arc length along the RACING LINE (not data.track.s / data.s_full) ---
segLen = vecnorm(diff(xy), 2, 2);
sLine  = [0; cumsum(segLen)];

% de-duplicate any zero-length segments so interp1 sees a strictly
% increasing sample grid (cumsum of non-negative lengths is only
% non-decreasing in general).
[sLineU, iu] = unique(sLine);
xyU = xy(iu, :);
vxU = vx(iu(:));

% -- t(s): map data.t_opt (KNOT grid) onto the racing line's arc length --
M  = numel(sLine);
Nk = numel(data.t_opt);
idx = knotIndices(M, Nk);
tOpt = data.t_opt(:);
sKnot = sLine(idx(:));
[sKnotU, iku] = unique(sKnot);
tKnotU = tOpt(iku);

% -- resample everything onto a uniform 1 m arc-length grid --------------
sMax = floor(sLineU(end));
sQuery = (0:1:sMax)';
if sQuery(end) ~= sLineU(end)
    sQuery(end+1, 1) = sLineU(end);   % keep the true endpoint on the grid
end

xQ  = interp1(sLineU, xyU(:,1), sQuery, 'linear');
yQ  = interp1(sLineU, xyU(:,2), sQuery, 'linear');
vxQ = interp1(sLineU, vxU,      sQuery, 'linear');
tQ  = interp1(sKnotU, tKnotU,   sQuery, 'linear', 'extrap');

psiQ = unwrap(atan2(gradient(yQ, sQuery), gradient(xQ, sQuery)));

ref = struct();
ref.xy       = [xQ, yQ];
ref.s        = sQuery;
ref.vx       = vxQ;
ref.t        = tQ;
ref.psi      = psiQ;
ref.closed   = closed;
ref.closeGap = closeGap;
ref.matPath  = matPath;

end

% =========================================================================
function idx = knotIndices(M, Nk)
%KNOTINDICES Positions of the knot-grid samples within a FULL grid of length
%  M that holds Nk knots plus evenly spaced interior collocation points
%  between each pair (mirrors Functions/report/packRun.m's strideToKnot,
%  which does the same stride check on VALUES; here the INDICES are needed
%  so the knot-grid data.t_opt values can be paired to the racing line's own
%  arc length at those exact points).
if Nk >= 1 && M == Nk
    idx = 1:M;
    return
end
if Nk > 1
    stride = (M - 1) / (Nk - 1);
    if stride >= 1 && stride == round(stride)
        idx = 1:stride:M;
        return
    end
end
warning('buildRefPath:knotStrideMismatch', ...
    ['buildRefPath: full grid (%d pts) does not stride cleanly onto the %d-point knot grid; ' ...
     'falling back to a linear index resample for the t(s) map.'], M, Nk);
idx = round(linspace(1, M, Nk));
end
