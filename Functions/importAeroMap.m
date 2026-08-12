function map = importAeroMap(xlsxFile, outFile)
%IMPORTAEROMAP One-off import of the Zenvo ride-height aero map.
%
%   map = importAeroMap()                       % default paths (repo layout)
%   map = importAeroMap(xlsxFile, outFile)
%
%   Reads sheet 'Lift_Downforce' of SportsCar_AeroBalance_ver1_0.xlsx and
%   saves a clean struct to Parameters/aeroMap_Tur.mat:
%       RHf    [10x1]  front ride height grid [mm]   (41..140, 11 mm steps)
%       RHr    [1x11]  rear  ride height grid [mm]   (40..150, 11 mm steps)
%       CLf    [10x11] front-axle lift coeff (negative = downforce), MID wings
%       CLr    [10x11] rear-axle  lift coeff (negative = downforce), MID wings
%       BALref [10x11] sheet's "Balance front %" table - REFERENCE ONLY:
%                      does NOT equal CLf./(CLf+CLr) (offset ~ +3.5-4 %),
%                      definition queried with Zenvo.
%
%   CONFIDENTIAL: source workbook is marked "NOT for any publication or
%   public consumption; only for simulation tools". The output .mat is
%   gitignored - re-run this importer on a fresh clone.

thisDir  = fileparts(mfilename('fullpath'));            % .../Functions
repoRoot = fileparts(thisDir);
if nargin < 1 || isempty(xlsxFile)
    % Moved out of the repo root into reference\aero\ on 2026-08-03. The root is
    % still accepted so a checkout on an older branch (or a worktree that has not
    % rebased) keeps importing rather than failing an assert three layers down.
    xlsxFile = fullfile(repoRoot, 'reference', 'aero', 'SportsCar_AeroBalance_ver1_0.xlsx');
    if ~isfile(xlsxFile)
        xlsxFile = fullfile(repoRoot, 'SportsCar_AeroBalance_ver1_0.xlsx');
    end
end
if nargin < 2 || isempty(outFile)
    outFile = fullfile(repoRoot, 'Parameters', 'aeroMap_Tur.mat');
end
assert(isfile(xlsxFile), 'importAeroMap: workbook not found: %s', xlsxFile);

sheet = 'Lift_Downforce';
RHr    = readmatrix(xlsxFile, 'Sheet', sheet, 'Range', 'C10:M10');   % 1x11
RHf    = readmatrix(xlsxFile, 'Sheet', sheet, 'Range', 'B11:B20');   % 10x1
CLf    = readmatrix(xlsxFile, 'Sheet', sheet, 'Range', 'C11:M20');   % 10x11
CLr    = readmatrix(xlsxFile, 'Sheet', sheet, 'Range', 'C24:M33');   % 10x11
BALref = readmatrix(xlsxFile, 'Sheet', sheet, 'Range', 'C37:M46');   % 10x11, reference only

% --- hard checks (sizes / finiteness / monotone grids) ---
assert(isequal(size(RHr), [1 11]),  'importAeroMap: RHr size %s, expected 1x11',  mat2str(size(RHr)));
assert(isequal(size(RHf), [10 1]),  'importAeroMap: RHf size %s, expected 10x1',  mat2str(size(RHf)));
assert(isequal(size(CLf), [10 11]), 'importAeroMap: CLf size %s, expected 10x11', mat2str(size(CLf)));
assert(isequal(size(CLr), [10 11]), 'importAeroMap: CLr size %s, expected 10x11', mat2str(size(CLr)));
assert(all(isfinite(CLf(:))), 'importAeroMap: CLf contains non-finite entries');
assert(all(isfinite(CLr(:))), 'importAeroMap: CLr contains non-finite entries');
assert(all(isfinite(RHf)) && all(diff(RHf) > 0), 'importAeroMap: RHf must be finite, strictly increasing');
assert(all(isfinite(RHr)) && all(diff(RHr) > 0), 'importAeroMap: RHr must be finite, strictly increasing');

% --- soft checks (warn if the sheet layout drifted) ---
if abs(RHf(1) - 41) > 1e-9 || abs(RHf(end) - 140) > 1e-9
    warning('importAeroMap:gridDrift', 'RHf spans %.1f..%.1f mm (expected 41..140)', RHf(1), RHf(end));
end
if abs(RHr(1) - 40) > 1e-9 || abs(RHr(end) - 150) > 1e-9
    warning('importAeroMap:gridDrift', 'RHr spans %.1f..%.1f mm (expected 40..150)', RHr(1), RHr(end));
end

map = struct('RHf', RHf, 'RHr', RHr, 'CLf', CLf, 'CLr', CLr, 'BALref', BALref, ...
    'source',     'SportsCar_AeroBalance_ver1_0.xlsx / Lift_Downforce', ...
    'importedOn', char(datetime('now','Format','yyyy-MM-dd HH:mm')), ...
    'note',       ['MID wing position baseline; negative = downforce. ' ...
                   'BALref is the sheet''s Balance front %% table, reference only ' ...
                   '(does not equal CLf./(CLf+CLr); definition queried with Zenvo).']);

save(outFile, '-struct', 'map');
fprintf('importAeroMap: %s -> %s\n', xlsxFile, outFile);
fprintf('  CLf range [%.3f, %.3f] | CLr range [%.3f, %.3f]\n', ...
    min(CLf(:)), max(CLf(:)), min(CLr(:)), max(CLr(:)));
end
