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
%   The repo root is derived from this file's own location
%   (<repo>/simulink/startup/arfwr_startup.m), never from PWD or a bare-name
%   path lookup: the repo is on the user's saved MATLAB path, so a bare name
%   would silently resolve against another checkout. Same anchoring rule as
%   Scripts/userOpts.m and Parameters/vehParams.m use for runOverride.mat.

thisDir  = fileparts(mfilename('fullpath'));            % <repo>\simulink\startup
repoRoot = fileparts(fileparts(thisDir));               % <repo>

extFolders = {'Parameters', 'Functions', 'Circuits'};

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
