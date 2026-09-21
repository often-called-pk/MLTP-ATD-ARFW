function rib = buildTrackRibbon(matPath, varargin)
%BUILDTRACKRIBBON Triangulated flat track surface for the sim3d/Unreal demo.
%   rib = BUILDTRACKRIBBON() builds a closed, triangulated ribbon (asphalt
%   lane + two kerb strips) from the SAME solved-run sidecar that
%   buildRefPath.m reads, expressed in the SIMULATION frame -- i.e. already
%   translated by the re-origin DriverPath applies to its copy of the racing
%   line, so the mesh can be drawn straight against VehStateBus x/y with no
%   further alignment (measured during the dashboard build -- the reference line and the driven
%   path differ by a ~136 m offset until that translation is applied).
%
%   rib = BUILDTRACKRIBBON(matPath, Name, Value, ...)
%
%   matPath (optional) - path to the raw sidecar .mat holding the full `data`
%       struct, relative to the repo root or absolute. Default is whatever
%       simulink/tools/activeTrack.m resolves -- the track the sim is
%       currently set up for. With no simulink/data/activeTrack.mat on disk
%       that resolver walks the SAME chain this function used to carry
%       inline: 'solutions/report/BCN/raw/run_BCN_ARFWr_ATD_data.mat' when
%       that solved-run sidecar exists (private checkout), else the shipped
%       slim lap, else 'simulink/data/trackRibbon_BCN.mat' -- a geometry-only
%       sidecar (the SAME track.s/k/x/y/Xl/Xr fields, no x_full/states/
%       controls) resolved relative to this function's own file, never pwd or
%       a bare-name load.
%       It ALSO carries two precomputed pose fields so the default origin and
%       'PlantFrame' still reproduce the private sidecar's ribbon exactly with
%       no racing line on hand: data.ribbonOrigin = [x y] (the racing line's
%       own first point, i.e. exactly what the default 'Origin' branch would
%       have computed from x_full) and data.plantStart = [x y psi1] (what
%       localPlantStartFrame would have computed from buildRefPath). Neither
%       is a solved state -- both are a single fixed pose, pinned to the SAME
%       track this sidecar already ships. Only when BOTH are absent does the
%       default fall back to the centreline first point and fire
%       'buildTrackRibbon:noRacingLine'.
%
%   Name-value options
%     'Origin'      [1x2] translation SUBTRACTED from every coordinate.
%                   Default [] = the racing line's own first point, which is
%                   exactly the transform DriverPath applies (it re-origins so
%                   the lap starts at (0,0)). Pass [0 0] to stay in the
%                   offline solve frame.
%     'HalfWidth'   scalar [m], fallback half-width used ONLY when the sidecar
%                   carries no track.Xl/track.Xr boundary polylines. Default 5.
%     'KerbWidth'   scalar [m], width of each kerb strip, measured inward from
%                   the track edge. Default 0.75. Set 0 for no kerbs.
%     'KerbPeriod'  scalar [m], arc length of one red or one white kerb block.
%                   Default 6.
%     'ZLift'       scalar [m], height of the surface above z = 0. Default
%                   0.02 -- a small lift so a car drawn at z = 0 does not
%                   z-fight with the road it is standing on.
%     'ColorAsphalt', 'ColorKerbA', 'ColorKerbB'  [1x3] RGB in 0..1.
%     'PlantFrame'  logical, default FALSE. When true the ribbon is put in the
%                   frame the PLANT integrates its global position in, which
%                   is what VehStateBus x/y/psi -- and therefore every posed
%                   actor in the two 3D routes -- actually live in. See the
%                   PLANT FRAME note below; this is not the same as the
%                   re-origin the default performs, and the difference is
%                   metres. Mutually exclusive with 'Origin'.
%     'Unreal'      logical, default TRUE. When true the returned V is in the
%                   sim3d/Unreal LOCAL mesh frame (x forward, y RIGHT, z up),
%                   i.e. the ISO y axis is negated -- see the frame note
%                   below. Pass false to get plain ISO/MATLAB x-y-z, which is
%                   what the unit test and any MATLAB `patch` plot want.
%     'STL'         char, path to write a binary STL of the surface to (via
%                   stlwrite). Default '' = do not write. Colours are not
%                   representable in STL; the mesh alone is written.
%
%   rib fields
%     V        [Nv x 3] vertices [m]
%     F        [Nf x 3] triangle faces, 1-based indices into V
%     C        [Nv x 3] per-vertex RGB colour, 0..1
%     N        [Nv x 3] per-vertex normals (all +z; +z in whichever frame V is)
%     origin   [1x2] the translation that was subtracted
%     lanes    struct of the four cross-track polylines actually used
%     stats    struct: length, meanWidth, areaTri, areaLW, areaErr, nVert,
%              nFace, closed, halfWidthSource
%     matPath  resolved absolute path of the sidecar that was read
%
%   PLANT FRAME NOTE (measured 2026-09-03, and the reason 'PlantFrame' exists).
%   Subtracting the racing line's first point does NOT put the mesh in the
%   frame the vehicle states are logged in, and the docstring above used to
%   claim it did. simulink/work/build_driver.m step 1c is the authority on
%   what DriverPath actually does, and it applies a RIGID transform, not a
%   translation: it de-kinks the line with a circular 5-point moving average,
%   re-resamples it to exactly 1 m, and then puts point 1 at the origin AND
%   rotates by -psi1 so the lap starts heading along +x -- because Plant.slx's
%   Body 6DOF starts at Xe_o = [0 0 0], eul_o = [0 0 0].
%
%   Dropping the rotation costs metres, and they grow with distance from the
%   start line. Measured against DriverPath's own drvRefX/drvRefY (the array
%   the car demonstrably tracks, to maxAbsN = 2.30 m): translation only is out
%   by up to 6.37 m, median 2.24 m; the full rigid transform below is out by
%   1.77 m / 0.45 m, the residual being the de-kink itself. On a corridor
%   whose half-width is 5 m that is the difference between a car drawn on the
%   tarmac and a car drawn on the grass for a quarter of the lap -- which is
%   exactly how it was found.
%
%   psi1 must be taken from the SMOOTHED, re-resampled line, not from the raw
%   one. The raw line's first two collocation points give psi1 = -0.066 deg
%   where the truth is +0.380 deg, and applying that WORSENS the registration
%   (7.47 m) over doing nothing. Hence the recipe here mirrors build_driver.m
%   step by step rather than reaching for the nearest available heading.
%
%   The default is FALSE only so that the documented solve-frame contract (and
%   simulink/validation/test_trackRibbon.m's frame check, which pins the
%   translation-only frame) is unchanged. Every consumer that draws the ribbon
%   against a driven pose -- simulink/viz/unrealPlayback.m and the in-model
%   Unreal3D Track/Ground init scripts -- passes 'PlantFrame', true.
%
%   FRAME NOTE. The solver, DriverPath and VehStateBus all use an ISO-style
%   right-handed frame: x forward, y LEFT, z up, psi positive counter-
%   clockwise. sim3d actor MESH data (Actor.Vertices) lives in the actor's own
%   local frame, which is Unreal's left-handed x-forward / y-RIGHT / z-up --
%   only Actor.Translation and Actor.Rotation are re-interpreted by
%   Actor.CoordinateSystem, never the vertex array. So a mesh authored in ISO
%   coordinates must have its y negated before it is handed to sim3d, and the
%   winding order must be reversed with it or every triangle faces down. Both
%   are done here when 'Unreal' is true (the default); the resulting mesh sits
%   correctly under actors placed with CoordinateSystem = 'ISO8855'.
%
%   Path resolution is repo-root anchored via mfilename('fullpath'), never a
%   bare-name load -- a folder holding old runs can sit on the MATLAB path
%   and shadow a bare filename with a stale one.
%
%   See also buildRefPath, cartPath, trackLimits.

% ---- options ----------------------------------------------------------
if nargin < 1, matPath = ''; end

p = inputParser;
p.FunctionName = 'buildTrackRibbon';
addParameter(p, 'Origin',       [],  @(v) isempty(v) || (isnumeric(v) && numel(v) == 2));
addParameter(p, 'HalfWidth',    5,   @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'KerbWidth',    0.75,@(v) isscalar(v) && isnumeric(v) && v >= 0);
addParameter(p, 'KerbPeriod',   6,   @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'ZLift',        0.02,@(v) isscalar(v) && isnumeric(v));
addParameter(p, 'ColorAsphalt', [0.17 0.17 0.19], @(v) isnumeric(v) && numel(v) == 3);
addParameter(p, 'ColorKerbA',   [0.80 0.10 0.10], @(v) isnumeric(v) && numel(v) == 3);
addParameter(p, 'ColorKerbB',   [0.93 0.93 0.93], @(v) isnumeric(v) && numel(v) == 3);
addParameter(p, 'PlantFrame',   false,@(v) islogical(v) || isnumeric(v));
addParameter(p, 'Unreal',       true, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'STL',          '',  @(v) ischar(v) || isstring(v));
parse(p, varargin{:});
o = p.Results;
o.PlantFrame = logical(o.PlantFrame);
if o.PlantFrame && ~isempty(o.Origin)
    error('buildTrackRibbon:originAndPlantFrame', ...
        ['buildTrackRibbon: ''PlantFrame'' defines the origin AND the heading itself, so it ' ...
         'cannot be combined with an explicit ''Origin''. Pass one or the other.']);
end

% ---- locate and load the sidecar --------------------------------------
toolsDir = fileparts(mfilename('fullpath'));          % ...\simulink\tools
repoRoot = fileparts(fileparts(toolsDir));            % -> repo root

if isempty(matPath)
    % activeTrack.m owns the whole default chain (active pack -> private
    % solved sidecar -> shipped slim lap -> geometry-only trackRibbon_BCN),
    % and returns an already-absolute path. It used to be spelled out here.
    matPath = activeTrack();
end
matPath = char(matPath);
if ~java.io.File(matPath).isAbsolute()
    matPath = fullfile(repoRoot, matPath);
end
if ~isfile(matPath)
    error('buildTrackRibbon:matNotFound', ...
        'buildTrackRibbon: raw sidecar not found: %s', matPath);
end

S = load(matPath);
if ~isfield(S, 'data')
    error('buildTrackRibbon:noData', ...
        'buildTrackRibbon: %s has no ''data'' variable. Variables found: {%s}', ...
        matPath, strjoin(fieldnames(S)', ', '));
end
data = S.data;
if ~isfield(data, 'track')
    error('buildTrackRibbon:noTrack', ...
        'buildTrackRibbon: data has no ''track'' field. data fields: {%s}', ...
        strjoin(fieldnames(data)', ', '));
end
track = data.track;

% ---- centreline -------------------------------------------------------
if isfield(track,'x') && isfield(track,'y') && ~isempty(track.x) && ~isempty(track.y)
    xc = track.x(:);  yc = track.y(:);
elseif isfield(track,'s') && isfield(track,'k')
    [xc, yc] = curv2cart(track.s, track.k);           % same call as MLTP.m:832
    xc = xc(:);  yc = yc(:);
else
    error('buildTrackRibbon:missingTrackXY', ...
        'buildTrackRibbon: data.track has neither x/y nor s/k. Fields: {%s}', ...
        strjoin(fieldnames(track)', ', '));
end
M = numel(xc);

% ---- track edges: prefer the solver's OWN boundaries -------------------
% track.Xl / track.Xr are the left/right limits the NLP was actually solved
% against (Functions/trackLimits.m). Using them rather than a constant
% half-width keeps the drawn ribbon identical to the drivable corridor,
% including the corner stretch a constant normal offset does not reproduce.
if isfield(track,'Xl') && isfield(track,'Xr') && ...
        isequal(size(track.Xl), [M 2]) && isequal(size(track.Xr), [M 2])
    XL = track.Xl;  XR = track.Xr;
    halfWidthSource = 'track.Xl/track.Xr';
else
    [xl, yl] = cartPath(xc, yc,  o.HalfWidth);
    [xr, yr] = cartPath(xc, yc, -o.HalfWidth);
    XL = [xl(:) yl(:)];  XR = [xr(:) yr(:)];
    halfWidthSource = sprintf('constant HalfWidth = %g m', o.HalfWidth);
end

% ---- origin (+ heading): mirror DriverPath's start-frame transform -----
psi1 = 0;
if o.PlantFrame
    % The full rigid transform of build_driver.m step 1c -- see the PLANT
    % FRAME NOTE. The transform itself now has a single owner,
    % simulink/tools/buildDriverRef.m (the tracked replacement for the
    % deleted build_driver.m); this subfunction only picks between computing
    % it and reading the precomputed pose out of a geometry-only sidecar.
    [origin, psi1] = localPlantStartFrame(matPath, data);
elseif isempty(o.Origin)
    if isfield(data,'x_full') && size(data.x_full,1) >= 4 && size(data.x_full,2) == M
        [xL1, yL1] = cartPath(xc, yc, data.x_full(4,:));
        origin = [xL1(1) yL1(1)];
    elseif isfield(data,'ribbonOrigin') && numel(data.ribbonOrigin) == 2
        % Geometry-only sidecar: the racing-line first point cannot be
        % recomputed without x_full, but it was pre-computed from the SAME
        % private default call and shipped verbatim, so the origin is
        % bit-identical without needing the racing line at load time.
        origin = data.ribbonOrigin(:).';
    else
        warning('buildTrackRibbon:noRacingLine', ...
            ['buildTrackRibbon: data.x_full is absent or does not match the %d-point ' ...
             'track grid, and data.ribbonOrigin is absent; falling back to the CENTRELINE ' ...
             'first point as the origin. This is not exactly DriverPath''s re-origin -- ' ...
             'pass ''Origin'' explicitly if the mesh must register against VehStateBus x/y.'], M);
        origin = [xc(1) yc(1)];
    end
else
    origin = o.Origin(:).';
end
XL = XL - origin;
XR = XR - origin;
xc = xc - origin(1);
yc = yc - origin(2);

% Rotation is a rigid motion, so every length, width, area and closure test
% downstream is invariant under it; only the registration against the driven
% pose changes. psi1 is 0 unless 'PlantFrame' was asked for.
if psi1 ~= 0
    Rz = [cos(-psi1) sin(-psi1); -sin(-psi1) cos(-psi1)];   % rotate by -psi1
    XL = XL * Rz;
    XR = XR * Rz;
    rc = [xc yc] * Rz;
    xc = rc(:,1);  yc = rc(:,2);
end

% ---- close the loop ---------------------------------------------------
% The lap wraps by INDEX, not by a duplicated station: the last quad joins
% station M straight back to station 1, so the ribbon is a genuinely closed
% (edge-manifold) band rather than a strip whose two ends merely coincide.
gapVec = [XL(end,:) - XL(1,:); XR(end,:) - XR(1,:)];
gap    = max(vecnorm(gapVec, 2, 2));
closed = gap <= 5;
if closed && gap < 1e-9
    % last station is an exact repeat of the first -- drop it, the wrap adds it back
    XL(end,:) = [];  XR(end,:) = [];  xc(end) = [];  yc(end) = [];
    M = M - 1;
end

% ---- cross-track lanes: outer kerb | inner kerb | asphalt --------------
% u is the unit cross-track vector at each station, pointing from right edge
% to left edge; the kerb seam sits KerbWidth inboard of each edge.
d  = XL - XR;
w  = vecnorm(d, 2, 2);                  % full track width at each station
u  = d ./ max(w, eps);
kw = min(o.KerbWidth, 0.45*min(w));     % never let the kerbs meet in the middle

lane.kerbLouter = XL;
lane.kerbLinner = XL - kw.*u;
lane.kerbRinner = XR + kw.*u;
lane.kerbRouter = XR;

% ---- arc length along the centreline (drives the kerb chequer) ---------
sc = [0; cumsum(vecnorm(diff([xc yc]), 2, 2))];
blk = mod(floor(sc./o.KerbPeriod), 2) == 0;             % alternating blocks
cKerb = zeros(M,3);
cKerb( blk,:) = repmat(o.ColorKerbA(:).', sum( blk), 1);
cKerb(~blk,:) = repmat(o.ColorKerbB(:).', sum(~blk), 1);
cAsph = repmat(o.ColorAsphalt(:).', M, 1);

% ---- vertex columns ---------------------------------------------------
% Six columns, of which two pairs are coincident: the seam is DUPLICATED so
% the kerb/asphalt colour boundary is crisp instead of interpolated across
% half the track (sim3d blends vertex colours over each triangle).
cols = {lane.kerbLouter, lane.kerbLinner, ...   % 1,2  left kerb  (kerb colour)
        lane.kerbLinner, lane.kerbRinner, ...   % 3,4  asphalt    (asphalt colour)
        lane.kerbRinner, lane.kerbRouter};      % 5,6  right kerb (kerb colour)
colC = {cKerb, cKerb, cAsph, cAsph, cKerb, cKerb};
if kw == 0
    cols = cols(3:4);  colC = colC(3:4);
end
nCol = numel(cols);

V = zeros(M*nCol, 3);
C = zeros(M*nCol, 3);
for c = 1:nCol
    idx = (c-1)*M + (1:M);
    V(idx,1:2) = cols{c};
    V(idx,3)   = o.ZLift;
    C(idx,:)   = colC{c};
end

% ---- faces: one quad grid per adjacent, non-coincident column pair -----
% i -> iNext wraps M back to 1 on a closed lap, so no vertex is duplicated
% to close the ring and every interior edge is shared by exactly two faces.
i0 = (1:M).';
if closed
    i1 = [ (2:M).'; 1 ];
else
    i0 = (1:M-1).';  i1 = (2:M).';
end
F = zeros(0,3);
for c = 1:nCol-1
    if max(vecnorm(cols{c} - cols{c+1}, 2, 2)) < 1e-12
        continue                                   % coincident seam pair
    end
    a  = (c-1)*M + i0;    an = (c-1)*M + i1;    % this column: station i, i+1
    b  = c*M     + i0;    bn = c*M     + i1;    % next column: station i, i+1
    F = [F; ...
         a,  b,  an; ...
         b,  bn, an];                                                       %#ok<AGROW>
end
F = double(F);

% ---- normals ----------------------------------------------------------
N = repmat([0 0 1], size(V,1), 1);

% ---- statistics (computed in the ISO frame, before any y flip) ---------
trackLen = sc(end);
if closed
    trackLen = trackLen + hypot(xc(1)-xc(end), yc(1)-yc(end));   % the wrap segment
end
meanWidth = mean(w);
areaTri   = triAreaSum(V, F);
areaLW    = trackLen * meanWidth;
stats = struct( ...
    'length',          trackLen, ...
    'meanWidth',       meanWidth, ...
    'minWidth',        min(w), ...
    'maxWidth',        max(w), ...
    'areaTri',         areaTri, ...
    'areaLW',          areaLW, ...
    'areaErr',         abs(areaTri - areaLW)/areaLW, ...
    'nVert',           size(V,1), ...
    'nFace',           size(F,1), ...
    'nStation',        M, ...
    'closed',          closed, ...
    'closeGap',        gap, ...
    'kerbWidth',       max(kw), ...
    'halfWidthSource', halfWidthSource);

% ---- ISO -> sim3d local mesh frame (y right, winding reversed) ---------
if o.Unreal
    V(:,2) = -V(:,2);
    F = F(:, [1 3 2]);
    N = repmat([0 0 1], size(V,1), 1);
end

rib = struct('V', V, 'F', F, 'C', C, 'N', N, ...
             'origin', origin, 'psi1', psi1, 'plantFrame', o.PlantFrame, ...
             'lanes', lane, 'stats', stats, ...
             'matPath', matPath, 'unrealFrame', logical(o.Unreal));

% ---- optional STL ------------------------------------------------------
if ~isempty(o.STL)
    stlPath = char(o.STL);
    if ~java.io.File(stlPath).isAbsolute()
        stlPath = fullfile(repoRoot, stlPath);
    end
    d0 = fileparts(stlPath);
    if ~isempty(d0) && ~isfolder(d0), mkdir(d0); end
    stlwrite(triangulation(F, V), stlPath);
    rib.stlPath = stlPath;
end

end

% =========================================================================
function [org, psi1] = localPlantStartFrame(matPath, data)
%LOCALPLANTSTARTFRAME The origin and heading build_driver.m step 1c produces.
%
% The transform is NOT reimplemented here. simulink/tools/buildDriverRef.m is
% the single owner: it is the tracked replacement for the deleted
% build_driver.m and rebuilds the whole DriverPath reference bake (X, Y, Psi,
% Kap, Vraw, N) from the same sidecar, with the same steps in the same order:
%   ref  = buildRefPath(matPath)      raw racing line, resampled to 1 m
%   drop the duplicated closing point (< 0.6 m) so the array is cyclic
%   circular 5-point moving average  (de-kink: cartPath's forward-difference
%       normals put ~0.5 m spurs into the line at BCN's tightest corner)
%   re-resample the smoothed line to exactly 1 m
%   org  = first point of THAT line, psi1 = heading of its first two points
%
% The smoothing is not cosmetic here: it moves psi1 from -0.066 deg to
% +0.380 deg, and only the latter registers the ribbon against the driven
% pose (see the PLANT FRAME NOTE in the main help).
%
% When the sidecar carries the solved racing line (x_full/s_full/t_opt --
% buildRefPath's own required fields, checked below), the pose is computed
% fresh exactly as above, bit-identical to every prior call. When it does
% not (the public, geometry-only sidecar), the SAME [x y psi1] pose --
% precomputed once from the private sidecar by this exact recipe -- is read
% back from data.plantStart instead: a fixed start pose is not confidential,
% only the solved states that would otherwise be needed to recompute it are.
refReq = {'x_full', 's_full', 't_opt'};
if all(isfield(data, refReq))
    drv  = buildDriverRef(matPath);
    org  = drv.origin;
    psi1 = drv.psi1;
elseif isfield(data, 'plantStart') && numel(data.plantStart) >= 3
    ps   = data.plantStart(:).';
    org  = ps(1:2);
    psi1 = ps(3);
else
    error('buildTrackRibbon:noPlantStart', ...
        ['buildTrackRibbon: ''PlantFrame'' needs either the solved racing line ' ...
         '(data.x_full/s_full/t_opt) or a precomputed data.plantStart = [x y psi1]; ' ...
         'neither is present in %s.'], matPath);
end
end

% =========================================================================
function a = triAreaSum(V, F)
%TRIAREASUM Total area of a triangle soup (robust to winding order).
e1 = V(F(:,2),:) - V(F(:,1),:);
e2 = V(F(:,3),:) - V(F(:,1),:);
a  = sum(0.5*vecnorm(cross(e1, e2, 2), 2, 2));
end
