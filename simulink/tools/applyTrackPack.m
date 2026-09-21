function info = applyTrackPack(varargin)
%APPLYTRACKPACK Push the active track's reference bake into DriverPath.
%   info = APPLYTRACKPACK() reads the pack setupTrack.m wrote to
%   simulink/data/activeTrack.mat (via activeTrack.m) and assigns its arrays
%   into the DriverPath model workspace IN MEMORY -- drvRefX, drvRefY,
%   drvRefPsi, drvRefKap, drvRefVraw, drvRefVplan, drvN, drvVx0, drvPlanRW,
%   drvPlanFW -- plus simLapStop in ARFWr_Sim. Nothing is written to disk.
%
%   With no pack on disk it is a NO-OP: the models keep their committed
%   Barcelona bake. That is what makes it safe to call unconditionally from
%   project startup and from runDemoLap.
%
%   Name-value options
%     'Pack'     struct, apply this pack instead of reading activeTrack.mat.
%     'Persist'  logical, default FALSE. TRUE also save_system's the models,
%                i.e. BAKES the track into the tracked .slx files. Only pass
%                it deliberately -- see WHY NOTHING IS SAVED below.
%     'Quiet'    logical, default FALSE. Suppress the one-line report.
%
%   info fields: applied, tag, matPath, src, n (array length), changed (cell
%   list of variables actually written), persisted, models.
%
%   =====================================================================
%   WHY THE MODEL WORKSPACE, AND WHY NOTHING IS SAVED
%   =====================================================================
%   Three delivery routes were available for per-track arrays:
%
%     (a) Simulink.SimulationInput.setVariable(name, val, 'Workspace',
%         'DriverPath') inside runDemoLap -- the idiom already used for
%         plantVx0 on Plant. Nothing is ever dirtied. REJECTED as the primary
%         route because it only reaches sim() calls made through that one
%         harness: the green Run button, validateClosedLoop's own runLap, the
%         Unreal3D live variant launched from the toolstrip and any bare
%         sim('ARFWr_Sim') would all still run Barcelona while every offline
%         tool drew the new track. A per-track setup that half the routes
%         ignore is worse than none.
%
%     (b) in-memory model-workspace assignment (THIS FILE). Every route --
%         green Run included -- sees the same arrays, because they live where
%         the blocks resolve them. The cost is that it must be re-applied
%         after DriverPath is closed and reloaded, which is exactly what
%         projStartup.m (project open) and runDemoLap.m (every lap) do, both
%         warn-not-error. The pack on disk is the single source of truth, so
%         a MATLAB restart re-derives the whole state from it.
%
%     (c) save_system, the route simulink/tools/rebuildDriverPlan.m takes.
%         REJECTED as a default: it rewrites a TRACKED .slx, so merely
%         looking at another track would show up as a model change in git and
%         a second person's checkout would silently inherit it. Available on
%         demand as 'Persist', true.
%
%   Route (b) makes the model differ from its file, so the DIRTY FLAG is
%   restored to whatever it was before this function ran (normally 'off').
%   That is deliberate and it is the safety property: an accidental
%   save_system or a "save changes?" prompt can no longer bake a foreign
%   track into DriverPath.slx, and closing the model simply drops the pack --
%   which projStartup then re-applies on the next open. Pass 'Persist', true
%   when you actually want the bake on disk.
%
%   drvN is the reason the lap length had to stop being a literal: it was
%   hardcoded as 4586 in DriverPath/Lap Manager/Par until 2026-09-21, so a
%   track of any other length ran the lap manager past the end of its own
%   reference arrays. See simulink/tools/buildDriverRef.m.
%
%   See also setupTrack, activeTrack, buildDriverRef, rebuildDriverPlan.

p = inputParser;
p.FunctionName = 'applyTrackPack';
addParameter(p, 'Pack',    [],    @(v) isempty(v) || isstruct(v));
addParameter(p, 'Persist', false, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'Quiet',   false, @(v) islogical(v) || isnumeric(v));
parse(p, varargin{:});
o = p.Results;
o.Persist = logical(o.Persist);
o.Quiet   = logical(o.Quiet);

info = struct('applied', false, 'tag', '', 'matPath', '', 'src', 'none', ...
              'n', 0, 'changed', {{}}, 'persisted', false, 'models', {{}});

% ---- the pack ----------------------------------------------------------
if isempty(o.Pack)
    [~, src, pack] = activeTrack();
    info.src = src;
else
    pack = o.Pack;
    info.src = 'caller';
end
if isempty(pack)
    if ~o.Quiet
        fprintf(['applyTrackPack: no active track pack -- DriverPath keeps its committed ' ...
                 'Barcelona bake. Run setupTrack(<solved lap .mat>) to change track.\n']);
    end
    return
end

req = {'drv', 'vPlan'};
missing = req(~isfield(pack, req));
assert(isempty(missing), 'applyTrackPack:badPack', ...
    'applyTrackPack: the active pack is missing field(s) {%s}. Re-run setupTrack.', ...
    strjoin(missing, ', '));
drv = pack.drv;
drvReq = {'X', 'Y', 'Psi', 'Kap', 'Vraw', 'N'};
missing = drvReq(~isfield(drv, drvReq));
assert(isempty(missing), 'applyTrackPack:badDrv', ...
    'applyTrackPack: pack.drv is missing field(s) {%s}. Re-run setupTrack.', ...
    strjoin(missing, ', '));

N = double(drv.N);
lens = [numel(drv.X) numel(drv.Y) numel(drv.Psi) numel(drv.Kap) numel(drv.Vraw) numel(pack.vPlan)];
assert(all(lens == N), 'applyTrackPack:lengthMismatch', ...
    ['applyTrackPack: pack.drv.N = %d but the arrays are [%s] long. The lap manager reads ' ...
     'drvN as the length of drvRefX/Y, so these must agree exactly.'], N, num2str(lens));

info.tag = getfielddefault(pack, 'tag', '<untagged>');
info.matPath = getfielddefault(pack, 'matPath', '');

% ---- DriverPath --------------------------------------------------------
load_system('DriverPath');
d0 = get_param('DriverPath', 'Dirty');
mw = get_param('DriverPath', 'ModelWorkspace');

vals = struct( ...
    'drvRefX',     drv.X(:), ...
    'drvRefY',     drv.Y(:), ...
    'drvRefPsi',   drv.Psi(:), ...
    'drvRefKap',   drv.Kap(:), ...
    'drvRefVraw',  drv.Vraw(:), ...
    'drvRefVplan', pack.vPlan(:), ...
    'drvN',        N);
if isfield(pack, 'planRW') && numel(pack.planRW) == N
    vals.drvPlanRW = pack.planRW(:);
end
if isfield(pack, 'planFW') && numel(pack.planFW) == N
    vals.drvPlanFW = pack.planFW(:);
end

% drvVx0 is the lap's entry speed as the solver left it, drvRefVraw(1) -- the
% convention the committed Barcelona bake uses (82.812 m/s = vRaw(1) exactly,
% measured). It is a RECORD, not a live input: no block dialog in any of the
% four models resolves drvVx0 (verified by a full DialogParameters sweep), and
% the rolling start that IS used is computed by runDemoLap and
% validateClosedLoop themselves as drvVScale*vProf(1) off the arrays above --
% so it already follows the active track without this variable.
vals.drvVx0 = vals.drvRefVraw(1);

changed = assignChanged(mw, vals);

if o.Persist
    save_system('DriverPath');
    info.persisted = true;
    info.models{end+1} = 'DriverPath';
else
    set_param('DriverPath', 'Dirty', d0);
end

% ---- ARFWr_Sim stop time ----------------------------------------------
% StopTime is the model-workspace variable simLapStop, shipped at 130 s for
% Barcelona. runDemoLap and the gates override StopTime themselves, so this
% only affects the green Run button -- but a track whose lap does not fit in
% 130 s would never close one from the toolstrip.
if isfield(pack, 'stopTimeHint') && isfinite(pack.stopTimeHint) && pack.stopTimeHint > 0
    load_system('ARFWr_Sim');
    dS = get_param('ARFWr_Sim', 'Dirty');
    mwS = get_param('ARFWr_Sim', 'ModelWorkspace');
    chS = assignChanged(mwS, struct('simLapStop', double(pack.stopTimeHint)));
    changed = [changed, chS];
    if o.Persist
        save_system('ARFWr_Sim');
        info.models{end+1} = 'ARFWr_Sim';
    else
        set_param('ARFWr_Sim', 'Dirty', dS);
    end
end

info.applied = true;
info.n       = N;
info.changed = changed;

if ~o.Quiet
    fprintf('applyTrackPack: track %s, N = %d, %d variable(s) updated%s [%s]\n', ...
        info.tag, N, numel(changed), ternaryChar(o.Persist, ' and SAVED to .slx', ' (in memory only)'), info.src);
end

end

% =========================================================================
function changed = assignChanged(mw, vals)
%ASSIGNCHANGED Write only the variables whose value actually differs.
% Keeps a repeat call a genuine no-op, so applyTrackPack can be called from
% runDemoLap on every lap without churning the model.
changed = {};
fn = fieldnames(vals);
for i = 1:numel(fn)
    nameArg = fn{i};
    newVal  = vals.(nameArg);
    if mw.hasVariable(nameArg) && isequaln(mw.getVariable(nameArg), newVal)
        continue
    end
    mw.assignin(nameArg, newVal);
    changed{end+1} = nameArg; %#ok<AGROW>
end
end

% =========================================================================
function v = getfielddefault(s, nameArg, dflt)
if isfield(s, nameArg) && ~isempty(s.(nameArg)), v = s.(nameArg); else, v = dflt; end
end

% =========================================================================
function s = ternaryChar(c, a, b)
if c, s = a; else, s = b; end
end
