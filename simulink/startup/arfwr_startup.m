function arfwr_startup()
%ARFWR_STARTUP  Project startup file for the ARFWr_RT MATLAB project.
%
%   Registered as a Startup File on the project (see ARFWr_RT.prj), so it runs
%   automatically when the project is opened.
%
%   Purpose: put the MLTP repo folders that the real-time model depends on
%   (vehicle/powertrain parameters, shared helper functions, track data) onto
%   the MATLAB path. These live OUTSIDE the project root (simulink/), and a
%   MATLAB project's own ProjectPath can only contain folders inside its root
%   -- addPath(proj, ...) errors with "not a folder in the project root
%   folder" -- so they are added here instead.
%
%   Scripts/ was NOT on this list until 2026-09-21, for no better reason than
%   that nothing under simulink/ needed it when the list was written: the
%   folder then held only the run-style MLTP entry points, which the real-time
%   model does not call. Scripts/solveLap.m and Scripts/exportLapSidecar.m --
%   the front door of the whole solve -> sim workflow -- arrived later, and
%   without this folder they simply did not resolve for anyone who opened
%   simulink/ARFWr_RT.prj on its own. There is no shadowing reason to leave it
%   out: no file name under Scripts/ collides with any under simulink/.
%
%   The repo root is derived from this file's own location
%   (<repo>/simulink/startup/arfwr_startup.m), never from PWD or a bare-name
%   path lookup: the repo is on the user's saved MATLAB path, so a bare name
%   would silently resolve against another checkout. Same anchoring rule as
%   Scripts/userOpts.m and Parameters/vehParams.m use for runOverride.mat.

thisDir  = fileparts(mfilename('fullpath'));            % <repo>\simulink\startup
repoRoot = fileparts(fileparts(thisDir));               % <repo>

extFolders = {'Parameters', 'Functions', 'Circuits', 'Scripts'};

for k = 1:numel(extFolders)
    f = fullfile(repoRoot, extFolders{k});
    if isfolder(f)
        addpath(f);
    else
        warning('ARFWr_RT:startup:missingFolder', ...
            'Expected repo folder not found, not added to path: %s', f);
    end
end

end
