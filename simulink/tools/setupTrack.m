function info = setupTrack(matPath, varargin)
%SETUPTRACK Point the whole Simulink/3D sim at a solved MLTP lap.
%   info = SETUPTRACK(matPath) takes the raw sidecar of ANY solved lap --
%   the `data` struct Scripts/solveLap.m (public) or Scripts/
%   runComparisonBatch.m (private) writes as
%   solutions/report/<TRK>/raw/run_<TRK>_<cfg>_<dv>_data.mat -- and does
%   everything the closed-loop sim and both 3D routes need to run that track:
%
%     1. buildDriverRef  : the six DriverPath reference arrays (X, Y, Psi,
%                          Kap, Vraw, N) in the PLANT frame.
%     2. buildSpeedPlan  : the driver's grip-limited speed plan, built with
%                          exactly the arguments simulink/tools/
%                          rebuildDriverPlan.m reads back out of DriverPath's
%                          own model workspace -- nothing is re-tuned here.
%     3. simulink/data/trackRibbon_<TAG>.mat : the geometry-only sidecar the
%                          3D ribbon is drawn from (track + ribbonOrigin +
%                          plantStart, no solved states).
%     4. simulink/data/activeTrack.mat : the pack every default-argument sim
%                          tool resolves through activeTrack.m.
%     5. applyTrackPack  : the arrays into DriverPath's model workspace, in
%                          memory (see that file for why, and for the three
%                          delivery routes that were weighed).
%
%   After it returns, buildRefPath(), buildTrackRibbon(), buildReplayInput(),
%   unrealPlayback, runDemoLap and the in-model Unreal3D actors all follow the
%   new track with no further arguments and no hand edits. Nothing tracked is
%   modified unless 'Persist', true is passed.
%
%   matPath  path to the solved sidecar, relative to the repo root or
%            absolute. Required.
%
%   Name-value options
%     'Tag'        char, short track tag used in file names. Default: parsed
%                  from a run_<TAG>_... filename, else data.lawTrack, else the
%                  file's base name.
%     'Persist'    logical, default FALSE. TRUE also saves DriverPath.slx and
%                  ARFWr_Sim.slx with the new bake -- i.e. writes TRACKED
%                  models. Only for deliberately re-baking the shipped track.
%     'Apply'      logical, default TRUE. FALSE writes the pack and the ribbon
%                  sidecar but does not touch any model workspace.
%     'PlanOpts'   struct of buildSpeedPlan overrides, merged ON TOP of the
%                  knobs read back from DriverPath (see below). Empty default.
%     'StartAt'    where the lap starts, passed to buildDriverRef. Default
%                  'auto': keep the solved lap's own s = 0 unless it sits in a
%                  corner, in which case the start moves into the longest
%                  straight, because the driver launches with no steer or
%                  preview history and cannot hold a corner from a standing
%                  reference. Measured at Spa (start line on the exit of La
%                  Source): every off-track sample of the lap was inside the
%                  first 26 m. Pass 'solved' to keep s = 0 regardless, or a
%                  distance in metres to place the start by hand. Barcelona
%                  and Nurburgring are unaffected by 'auto' -- their starts
%                  are already straight, and their bakes are bit-identical.
%     'SmoothWindow', 'ChordHalfLength'  passed to buildDriverRef; leave alone
%                  unless you are re-deriving the reference recipe.
%
%   info fields: tag, matPath, ribbonPath, packPath, N, lapOffline,
%   stopTimeHint, drv, vPlan, planInfo, applied, persisted, startShift.
%
%   =====================================================================
%   WHAT IS AND IS NOT RE-TUNED
%   =====================================================================
%   The speed plan's arguments are READ BACK from DriverPath's model
%   workspace, exactly as rebuildDriverPlan.m:81-118 does: drvPlanRH (the
%   MEASURED ride-height-vs-speed table), drvPlanFzOffN, drvPlanAyCap and
%   drvPlanBrkMargin. Only the two per-track inputs, kap and vRaw, come from
%   the new lap. So a new track inherits the plan calibration that was
%   validated at Barcelona rather than silently regenerating it at
%   buildSpeedPlan's own defaults -- the failure mode rebuildDriverPlan's
%   header records for drvPlanBrkMargin (shipped 0.6, default 0.7).
%
%   The driver GAINS (drvKp/drvKi/drvKff/drvTau/drvPosGain*/drvYawGain/
%   drvPreviewL/drvSpeedLook/drvVScale/drvNlim) are NOT touched at all. They
%   were hand-tuned on Barcelona; a track the car cannot hold at those gains
%   is a tuning result to report, not something this function should paper
%   over.
%
%   =====================================================================
%   WHY drvN MATTERS
%   =====================================================================
%   drvRefX/drvRefY feed Constant blocks, so the reference length is a
%   compile-time signal dimension, and the lap manager's own copy of that
%   length was a LITERAL 4586 (Barcelona) inside DriverPath/Lap Manager/Par
%   until 2026-09-21. It now reads the model-workspace variable drvN, which
%   applyTrackPack writes together with the arrays. That single literal was
%   the last thing pinning the sim to one track.
%
%   See also applyTrackPack, activeTrack, buildDriverRef, buildSpeedPlan,
%   rebuildDriverPlan, runDemoLap.

narginchk(1, inf);

p = inputParser;
p.FunctionName = 'setupTrack';
addParameter(p, 'Tag',             '',    @(v) ischar(v) || isstring(v));
addParameter(p, 'Persist',         false, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'Apply',           true,  @(v) islogical(v) || isnumeric(v));
addParameter(p, 'PlanOpts',        struct(), @isstruct);
addParameter(p, 'SmoothWindow',    5,     @(v) isscalar(v) && isnumeric(v));
addParameter(p, 'ChordHalfLength', 10,    @(v) isscalar(v) && isnumeric(v));
addParameter(p, 'StartAt',         'auto', @(v) (ischar(v) || isstring(v)) || ...
    (isscalar(v) && isnumeric(v) && isfinite(v)));
parse(p, varargin{:});
o = p.Results;
o.Persist = logical(o.Persist);
o.Apply   = logical(o.Apply);

toolsDir = fileparts(mfilename('fullpath'));          % ...\simulink\tools
simDir   = fileparts(toolsDir);
repoRoot = fileparts(simDir);

matPath = char(matPath);
if ~java.io.File(matPath).isAbsolute()
    matPath = fullfile(repoRoot, matPath);
end
assert(isfile(matPath), 'setupTrack:matNotFound', ...
    'setupTrack: solved-lap sidecar not found: %s', matPath);

S = load(matPath);
assert(isfield(S, 'data'), 'setupTrack:noData', ...
    'setupTrack: %s has no ''data'' variable. Variables found: {%s}', ...
    matPath, strjoin(fieldnames(S)', ', '));
data = S.data;

% ---- tag ---------------------------------------------------------------
[~, base] = fileparts(matPath);
tag = char(o.Tag);
if isempty(tag)
    tok = regexp(base, '^run_([A-Za-z0-9]+)_', 'tokens', 'once');
    if ~isempty(tok)
        tag = tok{1};
    elseif isfield(data, 'lawTrack') && (ischar(data.lawTrack) || isstring(data.lawTrack))
        tag = char(data.lawTrack);
    else
        tag = base;
    end
end
assert(~isempty(regexp(tag, '^\w+$', 'once')), 'setupTrack:badTag', ...
    'setupTrack: track tag ''%s'' is not a bare word; pass a valid ''Tag''.', tag);

fprintf('setupTrack: %s\n', tag);
fprintf('  sidecar   : %s\n', matPath);

% ---- 1. the DriverPath reference bake ---------------------------------
drv = buildDriverRef(matPath, 'SmoothWindow', o.SmoothWindow, ...
                              'ChordHalfLength', o.ChordHalfLength, ...
                              'StartAt', o.StartAt);
% psi1 is unwrapped once the start has been shifted (see buildDriverRef), so
% it is folded into +-180 deg for the report only -- the stored value is the
% measured one.
fprintf('  reference : N = %d pts (1 m grid), start [%.3f %.3f] m, psi1 %.4f deg\n', ...
    drv.N, drv.origin(1), drv.origin(2), rad2deg(atan2(sin(drv.psi1), cos(drv.psi1))));
fprintf('              vRaw %.2f .. %.2f m/s | |kap| max %.5f 1/m (R_min %.1f m)\n', ...
    min(drv.Vraw), max(drv.Vraw), max(abs(drv.Kap)), 1/max(abs(drv.Kap)));
% The start shift is the one thing a reader of the pack cannot infer from the
% arrays, so it is printed and stored rather than left implicit.
fprintf('  start     : %+d m from the solved s = 0 -- %s\n', drv.startShift, drv.startWhy);
fprintf('              R over the first 100 m %.0f m, entry speed %.2f m/s\n', ...
    1/max(max(abs(drv.Kap(1:min(100, drv.N)))), eps), drv.Vraw(1));

% ---- 2. the speed plan, on the SHIPPED planner calibration -------------
load_system('DriverPath');
mw = get_param('DriverPath', 'ModelWorkspace');

vp = baseVar('vp');
pt = baseVar('pt');

opts = o.PlanOpts;
rh = mwGet(mw, 'drvPlanRH');
assert(~isempty(rh) && size(rh, 2) == 3, 'setupTrack:noRH', ...
    ['setupTrack: DriverPath''s model workspace has no drvPlanRH [v RHf RHr] table. It is ' ...
     'the measured ride-height input the live-aero planner needs; see rebuildDriverPlan.m.']);
if ~isfield(opts, 'rhTable'), opts.rhTable = rh; end
opts = inheritKnob(opts, mw, 'fzFrontOffsetN', 'drvPlanFzOffN', 0);
opts = inheritKnob(opts, mw, 'ayCap',          'drvPlanAyCap',  inf);
opts = inheritKnob(opts, mw, 'brkMargin',      'drvPlanBrkMargin', []);

[vPlan, planInfo] = buildSpeedPlan(drv.Kap, drv.Vraw, vp, pt, opts);

fprintf('  speed plan: %s\n', planInfo.rhSource);
fprintf('              fzFrontOffsetN = %g N | ayCap = %g m/s^2 | brkMargin = %g\n', ...
    planInfo.opts.fzFrontOffsetN, planInfo.opts.ayCap, planInfo.opts.brkMargin);
fprintf('              plan %.2f .. %.2f m/s, cut below ref.vx at %d of %d pts (mean %.2f, max %.2f)\n', ...
    min(vPlan), max(vPlan), planInfo.nCut, numel(vPlan), planInfo.meanCut, planInfo.maxCut);
fprintf('              PROFILE-ONLY lap time (sum ds/v, no driver): %.3f s\n', ...
    sum(planInfo.opts.ds ./ vPlan));

% ---- 3. geometry-only ribbon sidecar ----------------------------------
ribData = geometrySidecar(data, drv);
ribbonRel = fullfile('simulink', 'data', sprintf('trackRibbon_%s.mat', tag));
ribbonAbs = fullfile(repoRoot, ribbonRel);
dataDir = fileparts(ribbonAbs);
if ~isfolder(dataDir), mkdir(dataDir); end
% Skip the write when an identical sidecar is already there. trackRibbon_BCN
% is TRACKED, and re-saving a .mat produces different bytes for the same
% content -- which would show up as a model-data change in git every time
% anyone set the sim back to Barcelona.
ribNote = 'written';
if isfile(ribbonAbs)
    Sr = load(ribbonAbs);
    if isfield(Sr, 'data') && isequaln(Sr.data, ribData)
        ribNote = 'unchanged, left alone';
    end
end
if strcmp(ribNote, 'written')
    saveSidecar(ribbonAbs, ribData);
end
fprintf('  ribbon    : %s (%d stations, %s)\n', ribbonRel, numel(ribData.track.s), ribNote);

% ---- 4. the pack ------------------------------------------------------
lapOffline = NaN;
if isfield(data, 't_opt') && ~isempty(data.t_opt), lapOffline = double(data.t_opt(end)); end
if isfinite(lapOffline)
    stopHint = ceil(1.5*lapOffline) + 20;
else
    stopHint = 150;                      % runDemoLap's own shipped default
end

active = struct( ...
    'tag',          tag, ...
    'matPath',      relPath(matPath, repoRoot), ...
    'ribbonPath',   ribbonRel, ...
    'drv',          drv, ...
    'vPlan',        vPlan(:), ...
    'planRW',       planInfo.alphaRW(:), ...
    'planFW',       planInfo.alphaFW(:), ...
    'lapOffline',   lapOffline, ...
    'stopTimeHint', stopHint, ...
    'startShift',   drv.startShift, ...
    'startWhy',     drv.startWhy, ...
    'builtOn',      datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<TNOW1,DATST>

packRel = fullfile('simulink', 'data', 'activeTrack.mat');
packAbs = fullfile(repoRoot, packRel);
save(packAbs, 'active', '-v7');
fprintf('  pack      : %s (offline lap %.3f s, StopTime hint %g s)\n', ...
    packRel, lapOffline, stopHint);

% ---- 5. into the models ------------------------------------------------
applied = false;
if o.Apply
    ai = applyTrackPack('Pack', active, 'Persist', o.Persist);
    applied = ai.applied;
    fprintf('  applied   : %d variable(s)%s\n', numel(ai.changed), ...
        ternaryChar(o.Persist, ' -- SAVED into DriverPath.slx / ARFWr_Sim.slx', ' (in memory)'));
else
    fprintf('  applied   : skipped (''Apply'', false)\n');
end

info = struct( ...
    'tag',          tag, ...
    'matPath',      matPath, ...
    'ribbonPath',   ribbonAbs, ...
    'packPath',     packAbs, ...
    'N',            drv.N, ...
    'lapOffline',   lapOffline, ...
    'stopTimeHint', stopHint, ...
    'drv',          drv, ...
    'vPlan',        vPlan(:), ...
    'planInfo',     planInfo, ...
    'applied',      applied, ...
    'persisted',    o.Persist, ...
    'startShift',   drv.startShift);

end

% =========================================================================
function ribData = geometrySidecar(data, drv)
%GEOMETRYSIDECAR The {track, ribbonOrigin, plantStart} struct buildTrackRibbon
% reads when no solved racing line is on hand. Exactly the three fields
% simulink/validation/test_trackRibbon.m pins for the shipped BCN one.
%
% track is cut down to the six fields buildTrackRibbon actually reads --
% s, k, x, y, Xl, Xr. That is not just size: data.track also carries
% xopt/yopt, which IS the solved racing line, and this sidecar's whole point
% is to carry geometry without solved output.
assert(isfield(data, 'track'), 'setupTrack:noTrack', ...
    'setupTrack: data has no ''track'' field. data fields: {%s}', ...
    strjoin(fieldnames(data)', ', '));
keep = {'s', 'k', 'x', 'y', 'Xl', 'Xr'};
track = struct();
for i = 1:numel(keep)
    if isfield(data.track, keep{i}), track.(keep{i}) = data.track.(keep{i}); end
end

if isfield(track, 'x') && isfield(track, 'y') && ~isempty(track.x) && ~isempty(track.y)
    xc = track.x(:);  yc = track.y(:);
else
    [xc, yc] = curv2cart(track.s, track.k);
    xc = xc(:);  yc = yc(:);
end
M = numel(xc);

% ribbonOrigin: the racing line's own first point -- what buildTrackRibbon's
% DEFAULT 'Origin' branch computes from x_full. Precomputed here so the
% geometry-only sidecar reproduces it with no states on board.
%
% It is deliberately NOT moved by a 'StartAt' shift. This is the SOLVE frame's
% anchor -- the translation-only frame test_trackRibbon section 6 pins and
% section 9 requires the geometry sidecar and the full sidecar to agree on --
% and buildTrackRibbon's default branch recomputes it from x_full(4,1), which
% knows nothing about a start shift. Moving it here would make the two
% sidecars disagree for a shifted track while helping nothing: the frame the
% car is actually drawn in is the PLANT frame below, and that one does carry
% the shift.
assert(isfield(data, 'x_full') && size(data.x_full, 1) >= 4 && size(data.x_full, 2) == M, ...
    'setupTrack:noRacingLine', ...
    ['setupTrack: data.x_full is absent or does not match the %d-point track grid, so the ' ...
     'racing line (and therefore the ribbon origin) cannot be computed.'], M);
[xL1, yL1] = cartPath(xc, yc, data.x_full(4, :));

% plantStart: the pose buildTrackRibbon('PlantFrame', true) needs. Taken from
% the buildDriverRef result the caller ALREADY computed, so the rigid
% transform keeps its single owner rather than being re-derived here.
ribData = struct('track', track, ...
                 'ribbonOrigin', [xL1(1) yL1(1)], ...
                 'plantStart',   [drv.origin(:).' drv.psi1]);
end

% =========================================================================
function saveSidecar(pathAbs, data)
%SAVESIDECAR Write the geometry-only sidecar with the variable name
% buildTrackRibbon expects ('data').
save(pathAbs, 'data', '-v7');
end

% =========================================================================
function v = baseVar(nameArg)
%BASEVAR Read vp/pt out of the base workspace, with the actionable error.
try
    v = evalin('base', nameArg);
catch
    error('setupTrack:noParams', ...
        ['setupTrack: ''%s'' is not in the base workspace. The speed planner needs the ' ...
         'vehicle/powertrain structs -- open simulink/ARFWr_RT.prj (projStartup loads ' ...
         'vp/pt/sus/act/inrt) or call loadVehicleParams yourself.'], nameArg);
end
end

% =========================================================================
function opts = inheritKnob(opts, mw, optName, mwName, dflt)
%INHERITKNOB Take a planner knob from DriverPath's model workspace unless the
% caller overrode it. Mirrors rebuildDriverPlan.m:91-118 -- every knob whose
% shipped value differs from buildSpeedPlan's default must round-trip through
% the model workspace or the regenerated plan is not the validated one.
if isfield(opts, optName), return, end
stored = mwGet(mw, mwName);
if ~isempty(stored)
    opts.(optName) = stored;
elseif ~isempty(dflt)
    opts.(optName) = dflt;
end
end

% =========================================================================
function v = mwGet(mw, nameArg)
%MWGET Model-workspace read that returns [] for an absent variable
% (getVariable errors on a name it does not hold).
if mw.hasVariable(nameArg)
    v = mw.getVariable(nameArg);
else
    v = [];
end
end

% =========================================================================
function r = relPath(pathAbs, repoRoot)
%RELPATH Store repo-root-relative paths in the pack so it survives a move.
if strncmpi(pathAbs, repoRoot, numel(repoRoot))
    r = pathAbs(numel(repoRoot)+2:end);
else
    r = pathAbs;
end
end

% =========================================================================
function s = ternaryChar(c, a, b)
if c, s = a; else, s = b; end
end
