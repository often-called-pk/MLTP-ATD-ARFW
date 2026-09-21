function out = runDemoLap(varargin)
%RUNDEMOLAP  One ClosedLoop lap of ARFWr_Sim, logged for the viz demos.
%
%   out = runDemoLap()                        unpaced full lap, 0.2 ms step, StopTime from the active track
%   out = runDemoLap('StopTime', 20, 'Pace', 1)   Simulation Pacing ON at 1x for 20 s of sim
%   out = runDemoLap('Save', 'work/lap.mat')  also save the result for re-use
%
%   Shared harness behind BOTH visualisation demos -- dashboardApp and
%   the sim3d/Unreal scene. Promoted out of the git-ignored simulink/work/
%   tree so the demos survive a fresh clone.
%
%   Mirrors validateClosedLoop.m's runLap EXACTLY on everything that affects
%   the result: the same in-memory 0.2 ms fixed step on all three models (the
%   models are now COMMITTED at 0.2 ms, so this is normally a no-op; the 1 ms
%   step they used to ship is numerically unstable above ~25 m/s with any steer
%   and unstable at low speed with the real tyres), the same rolling-start
%   plantVx0 read from DriverPath's own shipped
%   speed plan, the same SimulationInput. Fixed steps and dirty flags are
%   restored by an onCleanup, so this harness never writes to an .slx.
%
%   WHICH TRACK IT DRIVES
%   ---------------------
%   Whichever one simulink/tools/activeTrack.m resolves. The reference bake is
%   re-applied here through applyTrackPack (a no-op when nothing has changed,
%   and when no track has been selected at all), so a lap is correct even if
%   DriverPath was reloaded after setupTrack ran, or the project was opened
%   before it. A failure there is a WARNING, not an error: the lap then runs
%   on whatever bake the model already carries, which is the shipped
%   Barcelona one. Every per-track quantity below -- stop time, rolling start
%   speed, lap-completion index -- follows from that, with no BCN constant
%   left in this file.
%
%   Name-value options
%     'StopTime'  [s]  simulation stop time. Default [] = the active track's
%                 own hint (ceil(1.5 x the offline lap) + 20 s, written by
%                 setupTrack), falling back to 150 s when no track pack is on
%                 disk -- Barcelona's lap ends at ~139.4 s and the counter
%                 needs headroom to register the crossing.
%     'Step'      [s]  fixed step for all three models. Default 2e-4.
%     'Pace'      []   pacing rate; [] (default) disables Simulation Pacing.
%     'Save'      ''   path (repo-root relative or absolute) to save `out` to.
%                 Default '' = do not save.
%
%   out fields: logsout (the raw simLog Dataset), wall, simT, t, sLap,
%   lapCnt, lastLap, n, offT, completed, lapTime, tEndLap, maxAbsN, nOff,
%   step, paceRate.
%
%   Reference result (re-measured on the shipped Barcelona configuration):
%   lap 139.394 s, max |n| 2.301 m, 0 off-track samples.

p = inputParser;
p.FunctionName = 'runDemoLap';
addParameter(p, 'StopTime', [],   @(v) isempty(v) || (isscalar(v) && isnumeric(v) && v > 0));
addParameter(p, 'Step',     2e-4, @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'Pace',     [],   @(v) isempty(v) || (isscalar(v) && isnumeric(v)));
addParameter(p, 'Save',     '',   @(v) ischar(v) || isstring(v));
parse(p, varargin{:});
stopT    = p.Results.StopTime;
step     = p.Results.Step;
paceRate = p.Results.Pace;

mdl = 'ARFWr_Sim';
load_system('DriverPath'); load_system('Plant'); load_system('AeroECU'); load_system(mdl);

% ---- the active track, re-derived from disk every lap ------------------
trackTag = 'shipped bake';
try
    ai = applyTrackPack('Quiet', true);
    if ai.applied, trackTag = ai.tag; end
catch ME
    warning('runDemoLap:trackPack', ...
        ['runDemoLap: could not apply the active track pack (%s: %s). The lap will run on ' ...
         'whatever reference bake DriverPath already carries.'], ME.identifier, ME.message);
end
if isempty(stopT)
    stopT = 150;
    try
        [~, ~, pk] = activeTrack();
        if ~isempty(pk) && isfield(pk, 'stopTimeHint') && isfinite(pk.stopTimeHint) && pk.stopTimeHint > 0
            stopT = double(pk.stopTimeHint);
        end
    catch
        % no pack, or an unreadable one -- 150 s stands (see activeTrack.m)
    end
end
s0 = {get_param('Plant','FixedStep'), get_param('DriverPath','FixedStep'), get_param(mdl,'FixedStep')};
d0 = {get_param('Plant','Dirty'),     get_param('DriverPath','Dirty'),     get_param(mdl,'Dirty')};
cleanupObj = onCleanup(@() restoreSteps(mdl, s0, d0));   % held to scope end
set_param('Plant',      'FixedStep', num2str(step,'%.10g'));
set_param('DriverPath', 'FixedStep', num2str(step,'%.10g'));
set_param(mdl,          'FixedStep', num2str(step,'%.10g'));

mwD = get_param('DriverPath','ModelWorkspace');
if mwD.getVariable('drvUsePlan') > 0.5
    vProf = mwD.getVariable('drvRefVplan');
else
    vProf = mwD.getVariable('drvRefVraw');
end
v0 = mwD.getVariable('drvVScale')*vProf(1);

si = Simulink.SimulationInput(mdl);
si = si.setModelParameter('StopTime', num2str(stopT));
si = si.setVariable('plantVx0', v0, 'Workspace', 'Plant');
if isempty(paceRate)
    si = si.setModelParameter('EnablePacing','off');
else
    si = si.setModelParameter('EnablePacing','on');
    si = si.setModelParameter('PacingRate', num2str(paceRate));
end

fprintf('runDemoLap: track %s (N = %d), stop %g s, step %g s, pacing %s, vx0 %.3f m/s\n', ...
    trackTag, numel(vProf), stopT, step, ...
    ternary(isempty(paceRate),'OFF',sprintf('ON @ %gx',paceRate)), v0);
tw = tic; so = sim(si); out.wall = toc(tw);
out.simT = so.SimulationMetadata.ModelInfo.StopTime;

out.logsout = so.get('simLog');
Lp = so.get('simLap');
LI = reshape(Lp.signals.values, 12, []).';
out.t       = Lp.time;
out.sLap    = LI(:,1);
out.lapCnt  = LI(:,3);
out.lastLap = LI(:,4);
out.n       = LI(:,5);
out.offT    = LI(:,6);
out.completed = any(out.lapCnt >= 1);
if out.completed
    i1 = find(out.lapCnt >= 1, 1);
    out.lapTime = out.lastLap(i1);
    out.tEndLap = out.t(i1);
else
    out.lapTime = NaN; out.tEndLap = out.t(end);
end
out.maxAbsN = max(abs(out.n(1:find(out.t <= out.tEndLap, 1, 'last'))));
out.nOff    = sum(out.offT(1:find(out.t <= out.tEndLap, 1, 'last')) > 0);
out.paceRate = paceRate;
out.step     = step;
out.track    = trackTag;
out.stopTime = stopT;

fprintf('runDemoLap: wall %.1f s / sim %.1f s (%.2fx) | lap %s | max|n| %.3f m | off-track %d\n', ...
    out.wall, out.tEndLap, out.tEndLap/out.wall, ...
    ternary(out.completed, sprintf('%.3f s', out.lapTime), 'NOT COMPLETED'), out.maxAbsN, out.nOff);

if ~isempty(p.Results.Save)
    savePath = char(p.Results.Save);
    if ~java.io.File(savePath).isAbsolute()
        vizDir   = fileparts(mfilename('fullpath'));                 % ...\simulink\viz
        repoRoot = fileparts(fileparts(vizDir));
        savePath = fullfile(repoRoot, savePath);
    end
    d1 = fileparts(savePath);
    if ~isempty(d1) && ~isfolder(d1), mkdir(d1); end
    save(savePath, 'out', '-v7.3');
    fprintf('runDemoLap: saved to %s\n', savePath);
end
end

% =========================================================================
function restoreSteps(mdl, s0, d0)
try
    set_param('Plant','FixedStep',s0{1});       set_param('Plant','Dirty',d0{1});
    set_param('DriverPath','FixedStep',s0{2});  set_param('DriverPath','Dirty',d0{2});
    set_param(mdl,'FixedStep',s0{3});           set_param(mdl,'Dirty',d0{3});
catch
    % best effort: a model closed since the lap started cannot be restored
end
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
