%% apexSpeeds.m - geometry-driven corner detection + apex-speed table + map labels
% Called as the last line of MLTP.m (and siblings), after post-processing.
%
% Corners are detected from TRACK GEOMETRY (centreline curvature k(s)) only, NOT
% from the speed trace. The detection is therefore identical for every solution on
% a given track (aero config, ATD, tyre setting all leave it unchanged) and reports
% one apex speed per official corner. It works on any track that provides s and k;
% x/y are only needed for the optional map annotation.
%
% Uses the global `data` struct (populated by MLTP.m post-processing):
%   data.track.s   [1xN]  distance abscissa [m]        (== s_full)
%   data.track.k   [1xN]  centreline curvature [1/m]   (k>0 = LEFT turn)
%   data.x_full    [9xN]  states; row 1 = vx [m/s]
%   data.track.x/y [1xN]  centreline coords (optional, for the Circuit Map labels)
% and the workspace vars `circuit`, `vp` for the CSV name (fallbacks if absent).
%
% Sections:  (A) detection + table/CSV     (B) Circuit Map annotation

global data
global figures

%% ================= tuning knobs =================
% --- (A) geometry detection ---
Rgate      = 300;    % [m]   corner gate: a point is "in a corner" when |k| >= 1/Rgate
minGap     = 30;     % [m]   merge same-sign corner segments separated by a shorter gap
minAng     = 12;     % [deg] reject a segment whose total heading change |int k ds| is below this
valleyFrac = 0.30;   % [-]   split a same-sign segment at a valley where |k| < valleyFrac*min(neighbouring peaks)
                     %       (0.30 => valley radius must exceed ~3.3x the corner radius to count as two corners;
                     %        keeps a continuous same-sign sweep with a shallow double-apex as ONE corner)
peakProm   = 0.20;   % [-]   local-max prominence (fraction of segment peak |k|) to qualify as a distinct apex
% --- (B) map annotation ---
labelOff   = 80;     % [m]   text offset from apex, towards the OUTSIDE of the corner. Bumped
                     %       from 32 m on 2026-08-06: adding the label background box (BackgroundColor/
                     %       Margin/EdgeColor, below) means the box's own half-height can exceed the old
                     %       32 m offset, so an isolated label (one the de-collision pass never has to
                     %       move) can sit centred exactly on its OWN apex marker and hide it completely.
                     %       80 m clears that for every corner on BCN and all but a handful on NUR whose
                     %       local track tangent is close to due north-south (making the outward normal
                     %       near-horizontal, i.e. near-parallel to the box's long axis - no finite offset
                     %       clears those without exceeding the box HALF-WIDTH, which would push the label
                     %       implausibly far from its corner). Text legibility itself does not depend on
                     %       this at all - the opaque background covers the marker either way - this only
                     %       controls whether the small red dot stays visible next to its own label.
labelFont  = 8;      % [pt]  annotation font size

%% ================= (A) corner detection =================
sA  = data.track.s(:);          % abscissa [m]  (column)
kA  = data.track.k(:);          % curvature [1/m], k>0 = LEFT
vxA = data.x_full(1,:).';       % speed [m/s]   (column)
assert(numel(sA)==numel(kA) && numel(kA)==numel(vxA), ...
    'apexSpeeds: data.track.s / data.track.k / data.x_full(1,:) are not on the same grid');

knobs = struct('Rgate',Rgate,'minGap',minGap,'minAng',minAng, ...
               'valleyFrac',valleyFrac,'peakProm',peakProm);
segs  = local_detect(sA, kA, knobs);          % struct array: fields i1,i2,wrap

nC = numel(segs);
if nC == 0
    warning('apexSpeeds: no corners detected - check Rgate/minAng.');
    return
end

% -- per-corner quantities --
nPts    = numel(sA);
Corner  = (1:nC).';
s_m     = zeros(nC,1);   ApexV = zeros(nC,1);   Radius = zeros(nC,1);
Dir     = strings(nC,1); Sst   = zeros(nC,1);   Sen    = zeros(nC,1);
s_geo   = zeros(nC,1);   k_geo = zeros(nC,1);
for m = 1:nC
    if segs(m).wrap, r = [segs(m).i1:nPts, 1:segs(m).i2]; else, r = segs(m).i1:segs(m).i2; end
    ak = abs(kA(r));
    [kmax, loc] = max(ak);                 % geometric apex = tightest point
    Radius(m)  = 1/kmax;
    s_geo(m)   = sA(r(loc));
    k_geo(m)   = kA(r(loc));
    if k_geo(m) > 0, Dir(m) = "Left"; else, Dir(m) = "Right"; end
    [vmin, vloc] = min(vxA(r));            % apex SPEED = slowest point in the corner
    ApexV(m)   = vmin*3.6;                 % [km/h]
    s_m(m)     = sA(r(vloc));
    Sst(m)     = sA(r(1));   Sen(m) = sA(r(end));
end

T = table(Corner, round(s_m,1), round(ApexV,1), round(Radius,1), Dir, round(Sst,1), round(Sen,1), ...
    'VariableNames', {'Corner','s_m','ApexSpeed_kph','CentrelineRadius_m','Direction','S_start_m','S_end_m'});
disp(T)

% -- CSV export (graceful fallbacks so an unusual harness cannot error here) --
if exist('circuit','var') && (ischar(circuit) || isstring(circuit)), cname = char(circuit); else, cname = 'track'; end
if exist('vp','var') && isstruct(vp) && isfield(vp,'aeroSetting'), aname = char(vp.aeroSetting); else, aname = 'run'; end
% Written into solutions\apex\, not the repo root: the root used to accumulate one
% CSV per (circuit, aeroSetting) forever. Path is anchored to the repo root via
% mfilename('fullpath') (this file lives in Scripts\, so one fileparts() up is
% root) rather than pwd-relative, so the write no longer depends on the caller's
% working directory - the same idiom the reader side uses. Keep the two in step
% if you move the output location.
% them in step.
repoRoot = fileparts(fileparts(mfilename('fullpath')));   % ...\Scripts -> repo root
apexDir  = fullfile(repoRoot,'solutions','apex');
csv      = fullfile(apexDir, sprintf('apex_%s_%s.csv', cname, aname));

if ~exist(apexDir,'dir')
    [mkOK, mkMsg, mkID] = mkdir(apexDir);
    if ~mkOK
        warning('apexSpeeds:mkdirFailed', ...
            'apexSpeeds: mkdir(%s) failed (%s: %s) - apex CSV not written.', apexDir, mkID, mkMsg);
        apexDir = '';   % skip the write below, but still let (B) run
    end
end

if ~isempty(apexDir)
    % mkdir can report success before the directory is actually visible to a
    % subsequent low-level file-open call: the first writetable() right after a
    % fresh mkdir() has been observed to throw MATLAB:table:write:FileOpenError
    % even though exist(apexDir,'dir') already reports true (a second run, with
    % the directory now present, always succeeds). Confirm the directory is
    % really usable by round-tripping a throwaway probe file through fopen/fclose
    % before handing off to writetable - cheap, and avoids a spurious failure on
    % first use of a brand-new folder.
    dirReady = false;
    for attempt = 1:20
        if exist(apexDir,'dir')
            probe = tempname(apexDir);
            fid = fopen(probe,'w');
            if fid ~= -1
                fclose(fid);
                delete(probe);
                dirReady = true;
                break
            end
        end
        pause(0.05);
    end

    if ~dirReady
        warning('apexSpeeds:dirNotReady', ...
            'apexSpeeds: %s did not become writable after mkdir - apex CSV not written.', apexDir);
    else
        try
            writetable(T, csv);
            fprintf('apexSpeeds: %d corners written to %s\n', nC, csv);
        catch ME
            % Loud on purpose: a batch driver's solve wrapper catches exceptions
            % thrown after a solve completes so a post-processing failure can't
            % discard a finished IPOPT run - which means a plain error() here
            % vanishes with no trace in the batch log. A warning survives that
            % catch and still shows up.
            warning('apexSpeeds:writeFailed', ...
                'apexSpeeds: writetable to %s failed (%s: %s) - apex CSV not written.', ...
                csv, ME.identifier, ME.message);
        end
    end
end

%% ================= (B) Circuit Map annotation =================
% Never allow annotation trouble to break an MLTP run.
try
    hasAx  = isstruct(figures) && isfield(figures,'track') && isfield(figures.track,'ax') ...
             && ~isempty(figures.track.ax) && isgraphics(figures.track.ax);
    hasXY  = isfield(data.track,'x') && isfield(data.track,'y') ...
             && numel(data.track.x)==nPts && numel(data.track.y)==nPts;
    if hasAx && hasXY
        ax = figures.track.ax;
        hold(ax,'on');
        xc = data.track.x(:);  yc = data.track.y(:);
        % local tangent (unit) along the centreline, for outward label placement
        dx = gradient(xc, sA);  dy = gradient(yc, sA);
        mk = gobjects(nC,1);  lb = gobjects(nC,1);
        for m = 1:nC
            xa = interp1(sA, xc, s_geo(m));      % apex position (geometry)
            ya = interp1(sA, yc, s_geo(m));
            tx = interp1(sA, dx, s_geo(m));  ty = interp1(sA, dy, s_geo(m));
            nrm = hypot(tx,ty);  if nrm==0, nrm = 1; end
            tx = tx/nrm;  ty = ty/nrm;
            % outward normal = away from turn centre: k>0 (left) -> right normal (ty,-tx)
            ox = -sign(k_geo(m)) * (-ty);
            oy = -sign(k_geo(m)) * ( tx);
            px = xa + labelOff*ox;   py = ya + labelOff*oy;
            mk(m) = plot(ax, xa, ya, 'o', 'MarkerSize',4, 'MarkerFaceColor',[0.85 0.1 0.1], ...
                         'MarkerEdgeColor',[0.85 0.1 0.1]);
            % Two lines: corner + apex speed, then distance from the start line.
            % Narrower than one long line, which matters on corner sequences - but
            % taller, and the de-collision pass below nudges VERTICALLY, so watch
            % hairpin pairs. s_m(m) is the apex distance already computed for the
            % table above.
            lb(m) = text(ax, px, py, ...
                {sprintf('%d%c%.0f km/h', m, char(9474), ApexV(m)), ...
                 sprintf('%.0f m', s_m(m))}, ...
                         'FontSize',labelFont, 'FontWeight','bold', 'Color',[0.6 0 0], ...
                         'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
                         'Clipping','off', ...
                         'BackgroundColor',[1 1 1], 'EdgeColor',[0.6 0 0], 'Margin',1);
        end
        % -- de-collide labels of close-together corners (e.g. chicanes, or two
        %    corners far apart in s but geometrically close where the track
        %    loops back on itself - the Nurburgring GP infield does this): if
        %    two label boxes overlap, separate them vertically and move BOTH
        %    boxes apart, not just the later-indexed one.
        %
        %    The old version pushed only the later corner, by a fixed fraction
        %    of its own height. That deadlocks when a corner is sandwiched
        %    between two others on opposite sides (BCN is never dense enough
        %    to hit this; NUR's infield loop is): pushing the middle label up
        %    clears the neighbour below but drives it straight into the
        %    neighbour above, so the sandwiched label's net motion cancels pass
        %    after pass and no amount of extra passes converges it. Moving
        %    BOTH ends of every colliding pair means the two flanking labels
        %    also recede from the sandwiched one every pass, so the gap always
        %    grows even when the middle label's own net displacement is ~0.
        %    (A per-pair choice of x vs y axis was tried and discarded: NUR is
        %    elongated north-south, so letting labels drift horizontally to
        %    "escape" one overlap walks them into the next parallel leg of the
        %    track and creates more collisions than it resolves. Vertical-only
        %    is the axis this track's geometry actually has slack on.)
        %
        %    Second-order effect: the axis-limit widening below (needed so an
        %    outer-lying label isn't clipped at the figure edge) changes the
        %    data-units-per-pixel scale, and font size is fixed in POINTS, so
        %    every label's Extent (measured in DATA units) resizes when the
        %    limits change. A pair separated to a hairline gap before widening
        %    can end up overlapping again after it - so de-collide and widen
        %    are run as a fixed-point loop, not a single decollide-then-widen
        %    pass, until a round changes neither the label layout nor the
        %    limits. Generic geometry only (bounding-box overlap depths and
        %    axis limits) - no circuit-specific tuning.
        margin  = 0.15;    % [-] extra clearance beyond exact separation, as a fraction of each push
        pad     = 5;       % [m] margin around labels when widening axis limits
        for outerPass = 1:8
            for it = 1:40                          % generous cap; decollidePass's own return exits sooner
                if ~decollidePass(lb, nC, margin), break; end
            end
            ext = zeros(nC,4);
            for m = 1:nC, ext(m,:) = get(lb(m),'Extent'); end
            xl = xlim(ax);  yl = ylim(ax);
            newXl = [min(xl(1), min(ext(:,1))-pad),  max(xl(2), max(ext(:,1)+ext(:,3))+pad)];
            newYl = [min(yl(1), min(ext(:,2))-pad),  max(yl(2), max(ext(:,2)+ext(:,4))+pad)];
            xTol  = 1e-9*max(1, diff(xl));  yTol = 1e-9*max(1, diff(yl));
            limChanged = any(abs(newXl-xl) > xTol) || any(abs(newYl-yl) > yTol);
            xlim(ax, newXl);  ylim(ax, newYl);
            if ~limChanged, break; end             % limits (and therefore label scale) are stable
        end
        if labelOverlapAny(lb, nC)
            warning('apexSpeeds:labelOverlap', ...
                'apexSpeeds: %d de-collision rounds were not enough to separate every label pair on this map.', outerPass);
        end
        figures.track.apexMarkers = mk;
        figures.track.apexLabels  = lb;
    end
catch ME
    warning('apexSpeeds: map annotation skipped (%s)', ME.message);
end

%% ================= local functions =================
function tf = boxOverlap(A, B)
% AABB overlap test. A, B are text/rectangle Extent-style [x y w h] vectors,
% in the axes' data units (metres here). A strict ">0" test flags two boxes
% that the de-collision loop has already pushed to a hairline gap as still
% "overlapping" once floating-point noise (or the axis-limit rescale between
% rounds) puts the gap a fraction of a micron on the wrong side of zero -
% that is not a visible defect, but it does make the fixed-point loop below
% churn forever chasing it. tol is metres, far below anything a print/screen
% could resolve, so it only swallows that noise - a real overlap is always
% orders of magnitude larger.
tol = 1e-3;
tf = (A(1)+tol)<(B(1)+B(3)) && (B(1)+tol)<(A(1)+A(3)) && (A(2)+tol)<(B(2)+B(4)) && (B(2)+tol)<(A(2)+A(4));
end

function moved = decollidePass(lb, nC, margin)
% One de-collision sweep over every label pair: for each overlapping pair,
% move BOTH labels apart vertically by half the overlap depth (+ margin),
% splitting the separation rather than pinning one end - see the section-B
% comment above for why that matters. Returns true iff any pair moved.
moved = false;
for a = 1:nC-1
    for b = a+1:nC
        % re-fetch both extents every pair: either label may have moved
        % earlier in this same pass (as the 'a' OR the 'b' of a prior pair),
        % so a stale cached extent would push against where the box used to
        % be, not where it is now.
        ea = get(lb(a),'Extent');
        eb = get(lb(b),'Extent');
        if boxOverlap(ea, eb)
            oy   = min(ea(2)+ea(4), eb(2)+eb(4)) - max(ea(2), eb(2));   % y-overlap depth
            push = 0.5*oy*(1+margin);
            sgnv = sign((eb(2)+eb(4)/2) - (ea(2)+ea(4)/2));  if sgnv==0, sgnv = 1; end
            pa = get(lb(a),'Position');  pb = get(lb(b),'Position');
            pa(2) = pa(2) - sgnv*push;
            pb(2) = pb(2) + sgnv*push;
            set(lb(a),'Position',pa);  set(lb(b),'Position',pb);
            moved = true;
        end
    end
end
end

function tf = labelOverlapAny(lb, nC)
% True iff any pair of labels still overlaps at the CURRENT axis scale.
tf = false;
for a = 1:nC-1
    ea = get(lb(a),'Extent');
    for b = a+1:nC
        if boxOverlap(ea, get(lb(b),'Extent')), tf = true; return; end
    end
end
end

function segs = local_detect(sA, kA, knobs)
% Geometry-only corner detection. Returns struct array (i1,i2 index range, wrap flag).
Rgate=knobs.Rgate; minGap=knobs.minGap; minAng=knobs.minAng;
valleyFrac=knobs.valleyFrac; peakProm=knobs.peakProm;

n    = numel(sA);
absk = abs(kA);
mask = absk >= 1/Rgate;

% -- 1. raw contiguous masked runs (each is single-sign: a zero crossing breaks the mask) --
dd = diff([false; mask; false]);
i1 = find(dd==1);  i2 = find(dd==-1)-1;
raw = [i1, i2];
if isempty(raw), segs = struct('i1',{},'i2',{},'wrap',{}); return; end
sgn = zeros(size(raw,1),1);
for m = 1:size(raw,1)
    r = raw(m,1):raw(m,2);  [~,loc] = max(absk(r));  sgn(m) = sign(kA(r(loc)));
end

% -- 2. merge same-sign neighbours across a short gap --
merged = raw(1,:);  msg = sgn(1);
for m = 2:size(raw,1)
    gap = sA(raw(m,1)) - sA(merged(end,2));
    if sgn(m)==msg(end) && gap < minGap
        merged(end,2) = raw(m,2);
    else
        merged(end+1,:) = raw(m,:); %#ok<AGROW>
        msg(end+1,1)    = sgn(m);   %#ok<AGROW>
    end
end

% -- 3. split same-sign segment at a deep valley between distinct apexes --
final = zeros(0,2);
for m = 1:size(merged,1)
    r = merged(m,1):merged(m,2);  p = absk(r);
    if numel(r) < 3, final(end+1,:) = merged(m,:); continue; end %#ok<AGROW>
    lm = find(islocalmax(p,'MinProminence',peakProm*max(p)));
    if numel(lm) < 2, final(end+1,:) = merged(m,:); continue; end %#ok<AGROW>
    cut = [];
    for q = 1:numel(lm)-1
        [vmin,vloc] = min(p(lm(q):lm(q+1)));
        if vmin < valleyFrac*min(p(lm(q)),p(lm(q+1))), cut(end+1) = lm(q)-1+vloc; end %#ok<AGROW>
    end
    if isempty(cut)
        final(end+1,:) = merged(m,:); %#ok<AGROW>
    else
        b = [1, cut(:).', numel(r)];
        for c = 1:numel(b)-1
            a = b(c);  z = b(c+1);  if c>1, a = a+1; end
            final(end+1,:) = [r(a), r(z)]; %#ok<AGROW>
        end
    end
end

% -- 4. gate on total heading change --
keep = false(size(final,1),1);
for m = 1:size(final,1)
    r = final(m,1):final(m,2);
    keep(m) = abs(trapz(sA(r), kA(r)))*180/pi >= minAng;
end
final = final(keep,:);
if isempty(final), segs = struct('i1',{},'i2',{},'wrap',{}); return; end

% -- 5. wrap: first & last touch grid ends with the same sign -> one corner over s=0 --
wrapMerge = false;
if size(final,1) >= 2 && final(1,1)==1 && final(end,2)==n
    r1 = final(1,1):final(1,2);    [~,l1]=max(absk(r1));  s1=sign(kA(r1(l1)));
    r2 = final(end,1):final(end,2);[~,l2]=max(absk(r2));  s2=sign(kA(r2(l2)));
    wrapMerge = (s1==s2);
end
segs = struct('i1',{},'i2',{},'wrap',{});
if wrapMerge
    segs(1) = struct('i1',final(end,1),'i2',final(1,2),'wrap',true);
    for m = 2:size(final,1)-1
        segs(end+1) = struct('i1',final(m,1),'i2',final(m,2),'wrap',false); %#ok<AGROW>
    end
else
    for m = 1:size(final,1)
        segs(end+1) = struct('i1',final(m,1),'i2',final(m,2),'wrap',false); %#ok<AGROW>
    end
end
end
