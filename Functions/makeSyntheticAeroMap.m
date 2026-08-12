function map = makeSyntheticAeroMap(outFile)
%MAKESYNTHETICAEROMAP Build a SYNTHETIC ride-height aero map.
%
%   map = makeSyntheticAeroMap()          % writes Parameters/aeroMap_Synthetic.mat
%   map = makeSyntheticAeroMap(outFile)
%
% =========================================================================
%  THIS MAP IS INVENTED. It is not CFD, not wind-tunnel data, and does not
%  describe any real vehicle.
% =========================================================================
%
% It replaces the confidential ride-height workbook that Functions/importAeroMap.m
% reads, so this repository can run without it. The shape is a generic smooth
% ground-effect model: downforce grows as either ride height falls, with a
% cross-term so rake shifts aero balance the way a real car's does.
%
%     CLf(hf,hr) = -( a0f + a1f*exp(-kf*uf) + a2f*exp(-kc*ur) )
%     CLr(hf,hr) = -( a0r + a1r*exp(-kf*ur) + a2r*exp(-kc*uf) )
%
% with uf, ur the ride heights normalised onto [0,1] over their grids.
% Exponentials are used deliberately: aeroCollapse.m solves a fixed point on
% this map and then fits it piecewise-polynomially against a residual gate, so
% the map has to be smooth and monotone or the collapse will not converge.
%
% Output matches importAeroMap.m's schema exactly:
%     RHf    [10x1]  front ride-height grid [mm]  (41..140)
%     RHr    [1x11]  rear  ride-height grid [mm]  (40..150)
%     CLf    [10x11] front-axle lift coeff (negative = downforce)
%     CLr    [10x11] rear-axle  lift coeff (negative = downforce)
%     BALref [10x11] front balance %, reference only
%
% Calibrated so that at the nominal ride heights (vp.RHf0 = 85, vp.RHr0 = 95)
% the map gives CLf ~ -1.04 and CLr ~ -1.31, i.e. total downforce coefficient
% ~2.4 split ~45% front - a plausible winged sports car, and nothing more.

if nargin < 1 || isempty(outFile)
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    outFile  = fullfile(repoRoot, 'Parameters', 'aeroMap_Synthetic.mat');
end

% Grids - must match importAeroMap.m's expected spans exactly.
RHf = (41:11:140)';        % 10x1
RHr =  40:11:150;          % 1x11
assert(isequal(size(RHf), [10 1]) && isequal(size(RHr), [1 11]), ...
    'makeSyntheticAeroMap: grid size mismatch');

% Normalised ride heights on [0,1], broadcast to the 10x11 grid.
uf = (RHf - RHf(1)) / (RHf(end) - RHf(1));      % 10x1
ur = (RHr - RHr(1)) / (RHr(end) - RHr(1));      % 1x11
UF = repmat(uf, 1, numel(RHr));
UR = repmat(ur, numel(RHf), 1);

kf = 2.2;      % own-axle ride-height sensitivity
kc = 1.2;      % cross-axle (rake) sensitivity

a0f = 0.85; a1f = 0.35; a2f = 0.20;
a0r = 1.05; a1r = 0.42; a2r = 0.22;

CLf = -( a0f + a1f*exp(-kf*UF) + a2f*exp(-kc*UR) );
CLr = -( a0r + a1r*exp(-kf*UR) + a2r*exp(-kc*UF) );

BALref = 100 * CLf ./ (CLf + CLr);

map = struct('RHf', RHf, 'RHr', RHr, 'CLf', CLf, 'CLr', CLr, ...
             'BALref', BALref, 'synthetic', true);

% Same hard checks importAeroMap.m applies, so a bad edit fails here.
assert(all(isfinite(CLf(:))) && all(isfinite(CLr(:))), 'makeSyntheticAeroMap: non-finite lift');
assert(all(diff(RHf) > 0) && all(diff(RHr) > 0), 'makeSyntheticAeroMap: grids must be increasing');
assert(all(CLf(:) < 0) && all(CLr(:) < 0), 'makeSyntheticAeroMap: expected downforce everywhere');

if ~isempty(outFile)
    save(outFile, '-struct', 'map');
    fprintf('makeSyntheticAeroMap: wrote %s\n', outFile);
end
end
