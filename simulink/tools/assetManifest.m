function M = assetManifest(name)
%ASSETMANIFEST  Resolve a car-asset name to its manifest struct.
%   M = ASSETMANIFEST('Zenvo')  -> simulink/viz/assets/zenvo_tur/manifest.m
%   Every consumer (zenvoCarActors, unrealPlayback, the Unreal3D init script,
%   test_carAsset) resolves through here, so the folder name lives in ONE place.
%
%   The returned struct is the folder's own manifest() with four paths resolved:
%     M.folder       the manifest's folder (the body asset's folder)
%     M.body.file    absolute path to the body mesh
%     M.donorFolder  the donor asset's folder (a sibling of M.folder)
%     M.donor.file   absolute path to the donor mesh
%   Everything else is left exactly as the manifest wrote it.
switch upper(char(name))
    case 'ZENVO', slug = 'zenvo_tur';
    otherwise
        error('assetManifest:unknown', 'assetManifest: no asset registered as ''%s''.', char(name));
end
here = fileparts(mfilename('fullpath'));                    % simulink/tools
folder = fullfile(here, '..', 'viz', 'assets', slug);
mf = fullfile(folder, 'manifest.m');
assert(isfile(mf), 'assetManifest:missing', ...
    ['assetManifest: %s not found. Download the asset (see the LICENSE.md ' ...
     'in that folder) -- nothing in the repo ships the mesh itself.'], mf);
oldDir = cd(folder); c = onCleanup(@() cd(oldDir));
M = manifest();                                            % the folder's own manifest.m
M.folder      = folder;
M.body.file   = fullfile(folder, M.body.file);
M.donor.file  = fullfile(folder, M.donor.file);
M.donorFolder = fileparts(M.donor.file);
end
