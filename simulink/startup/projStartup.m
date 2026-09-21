function projStartup()
%PROJSTARTUP  Second startup file for the ARFWr_RT project: load the vehicle,
%powertrain, suspension and actuator parameter structs into the BASE workspace.
%
%   Registered as a Startup File on ARFWr_RT.prj, so it runs automatically when
%   the project is opened. Simulink blocks and mask expressions resolve unknown
%   identifiers against the base workspace, so vp/pt/sus/act must live there --
%   hence assignin('base',...) rather than plain assignments (a function's own
%   workspace is discarded on return, and a project startup file always executes
%   as a function).
%
%   ---- ORDERING: this file deliberately does NOT touch the MATLAB path ----
%   Path setup is owned by simulink/startup/arfwr_startup.m, which adds the
%   out-of-root repo folders (Parameters/, Functions/, Circuits/) that
%   loadVehicleParams() needs. R2025a rejects addPath(proj, ...) for folders
%   outside the project root, which is why that startup-file approach exists;
%   duplicating its logic here would create a second owner of the same path
%   state. Project startup files run in REGISTRATION order, and arfwr_startup.m
%   was registered first, per the project's startup-file registration order --
%   verify with:
%       proj = currentProject; disp(proj.StartupFiles)
%   arfwr_startup.m must appear ABOVE projStartup.m in that list. If it ever
%   does not, this file will fail to resolve the parameter code and report it
%   through the warning below rather than silently loading nothing.
%
%   Measured 2026-08-31, by rmpath-and-call on the live session: the
%   hard path dependency of that load is Functions/, NOT Parameters/.
%   loadVehicleParams() reaches vehParams.m / Powertrain.m by ABSOLUTE path
%   built from its own mfilename('fullpath'), so removing Parameters/ from the
%   path leaves the load working; removing Functions/ breaks it at the first
%   bare helper call (setupValue), because vehParams.m calls getfielddef,
%   setupValue, aeroCollapse, rwAeroMap2D, ... by bare name. Either way
%   arfwr_startup.m adds both folders, so the two stay wired together.
%
%   ---- FAILURE BEHAVIOUR ----
%   A parameter-load failure is reported as a WARNING, not an error, and the
%   MException is parked in the base workspace as projStartupError. Rationale:
%   an error thrown from a startup file makes the project itself hard to open,
%   which is exactly when you need it open to debug the parameter code. A caller
%   (or a startup smoke test) detects failure by the ABSENCE of vp/pt/sus/act
%   in the base workspace, or by the PRESENCE of projStartupError -- both are
%   loud, neither bricks the project. On success projStartupError is cleared, so
%   a stale one from a previous open cannot be mistaken for a fresh failure.
%
%   Sources of the four structs (all under simulink/params/, Tasks 0.2-0.3):
%     vp, pt  - loadVehicleParams()   : Parameters/vehParams.m + Powertrain.m,
%                                       pinned to the ARFWr identity
%                                       (vp.ActAero=1, vp.rwMandate=7).
%                                       vp carries the CONFIDENTIAL Zenvo tyre
%                                       parameter structs vp.tyre_f/vp.tyre_r --
%                                       usable in simulation, never
%                                       reproduced in the report.
%     sus     - suspensionBorrowed()  : borrowed damper + ARB rates.
%     act     - actuatorParams()      : wing-actuator lag, rate limit, clamps.
%     inrt    - inertiaBorrowed()     : roll/pitch/yaw inertia tensor for the
%                                       6-DOF body. Despite the file
%                                       name the values are Zenvo SUPPLIER
%                                       data; see its header.
%
%   It then applies the ACTIVE TRACK pack (applyTrackPack), which is what
%   makes a setupTrack selection survive a MATLAB restart. When no pack is on
%   disk that call returns before touching Simulink at all, so project open is
%   unchanged; when one IS on disk it loads DriverPath and ARFWr_Sim to write
%   their model workspaces, which is the price of having the green Run button
%   drive the selected track.

% Everything in base BEFORE this file runs. Anything that appears while it runs
% and is not one of the structs it owns is scratch dropped by a model callback,
% and is swept at the end -- see localSweepBase below.
baseBefore = evalin('base', 'who');
OWNED      = {'vp', 'pt', 'sus', 'act', 'inrt', 'hudGear', 'projStartupError'};

try
    [vp, pt] = loadVehicleParams();
    sus      = suspensionBorrowed();
    act      = actuatorParams();
    inrt     = inertiaBorrowed();

    assignin('base', 'vp',  vp);
    assignin('base', 'pt',  pt);
    assignin('base', 'sus', sus);
    assignin('base', 'act', act);
    assignin('base', 'inrt', inrt);

    evalin('base', 'clear projStartupError');

    % hudGear -- the DISPLAY-ONLY indicated-gear table. ARFWr_Sim's InitFcn
    % builds it too, but an InitFcn only fires when a simulation starts, so
    % until the first lap the live Dashboard panel and anything else reading
    % hudGear find nothing. It is derived from pt alone and costs nothing, so
    % it is built here as well and the two agree by construction (both call
    % indicatedGearParams, its single owner). Its own try: a viz folder that is
    % not yet on the project path must not cost us vp/pt/sus/act/inrt.
    try
        assignin('base', 'hudGear', indicatedGearParams(pt));
    catch MEhud
        warning('ARFWr_RT:startup:hudGearFailed', ...
            ['projStartup could not build hudGear (%s: %s). It is display-only, so the ' ...
             'simulation is unaffected; ARFWr_Sim''s InitFcn builds it at the first ' ...
             'simulation start.'], MEhud.identifier, MEhud.message);
    end

    fprintf(['projStartup: base workspace loaded -- vp (%s, rwMandate=%d), ' ...
             'pt, sus, act, inrt, hudGear.\n'], vp.aeroSetting, vp.rwMandate);

    % ---- the active track -------------------------------------------
    % simulink/data/activeTrack.mat is the single record of which solved lap
    % the sim is set up for; applyTrackPack puts its reference arrays into
    % DriverPath's model workspace IN MEMORY, so the selection survives a
    % MATLAB restart without any .slx ever being rewritten. With no pack on
    % disk this is a no-op and the models keep their committed Barcelona
    % bake. WARN, never error: a bad pack must not make the project hard to
    % open, for the same reason the parameter load below does not.
    try
        applyTrackPack('Quiet', true);
    catch MEtrk
        warning('ARFWr_RT:startup:trackPackFailed', ...
            ['projStartup could not apply the active track pack (%s: %s). DriverPath keeps ' ...
             'its committed reference bake; re-run setupTrack(<solved lap .mat>) to fix it.'], ...
            MEtrk.identifier, MEtrk.message);
    end

catch ME
    assignin('base', 'projStartupError', ME);
    warning('ARFWr_RT:startup:paramLoadFailed', ...
        ['projStartup could not load the parameter structs (%s: %s). ' ...
         'vp/pt/sus/act are NOT in the base workspace; the MException is ' ...
         'parked there as projStartupError. Check that arfwr_startup.m ran ' ...
         'first (proj.StartupFiles order) so Parameters/ and Functions/ are ' ...
         'on the path.'], ME.identifier, ME.message);
end

localSweepBase(baseBefore, OWNED);

end

% =========================================================================
function localSweepBase(before, owned)
%LOCALSWEEPBASE Drop the scratch a model callback left in the base workspace.
%
%   A Simulink model callback is evaluated IN THE BASE WORKSPACE, so its
%   locals stay there. ARFWr_Sim's PreLoadFcn is the one that matters here: it
%   pushes the rolling-start speed into Plant's model workspace and leaves
%   mwD, vProf, v0, mwP and dirty0 behind, which happens during project open
%   because applyTrackPack loads ARFWr_Sim to set its stop time. Opening the
%   project then appeared to "leak" five variables nobody had heard of.
%
%   The sweep is written against the DIFFERENCE between before and after, not
%   against a list of names, so a callback that changes its scratch cannot
%   quietly start leaking again. It is scoped to project open: a bare
%   load_system('ARFWr_Sim') later in an open session drops the same names
%   once more, until the model is unloaded and reloaded.
%
%   A sweep that fails is not worth failing project open over.
try
    added = setdiff(evalin('base', 'who'), [before(:); owned(:)]);
    if ~isempty(added)
        evalin('base', ['clear ' strjoin(added(:)', ' ')]);
    end
catch
    % best effort, deliberately silent
end
end
