function v = setupValue(name, defaultValue)
%SETUPVALUE Raw-input value for a parameter, honouring a loaded setup sheet.
%
%   v = setupValue('brkB', 0.65)
%
%   Returns the value from the setup override if the sheet supplied one for this
%   parameter, otherwise `defaultValue` unchanged. Used at the RAW-INPUT
%   assignment sites in Parameters/vehParams.m and Parameters/Powertrain.m.
%
%   WHY AT THE ASSIGNMENT SITE, not as a post-hoc patch. vehParams.m interleaves
%   raw inputs with quantities derived from them (vp.kwf on line ~52 feeds
%   vp.ksf_rad on line ~54; vp.l and vp.wB feed vp.l_f; the masses feed vp.ms/m
%   and the static corner loads). Overwriting raw fields AFTER the script has run
%   would leave every derived quantity computed from the OLD value - a silently
%   self-inconsistent vehicle. Substituting at the assignment keeps the existing
%   derivation order intact and correct, which is also why locked decision 10
%   makes derived rows read-only: they are recomputed, never set.
%
%   The override is read ONCE per MATLAB session from <repoRoot>/setupOverride.mat
%   and cached, so this is cheap to call from dozens of assignment sites. Call
%   setupValue('__reset__') to drop the cache (the tests do this between cases).
%
%   PATH ANCHORING is deliberate and load-bearing: the file is resolved from this
%   function's OWN location, never from pwd and never by bare filename. A bare
%   name resolves through the whole MATLAB search path, so a stray override in
%   another checkout silently hijacks runs started here - the exact failure mode
%   documented for runOverride.mat too, and one that cost several hours
%   of wrong-configuration solves in this repo already.

persistent OVR
if nargin >= 1 && ischar(name) && strcmp(name, '__reset__')
    OVR = []; v = [];
    return
end

% CACHE VALIDITY IS KEYED ON THE FILE, NOT MERELY ON "have I looked yet".
% An existence+mtime check per call is free (one dir() against ~38 calls per
% vehParams run) and it closes a silent-wrong-configuration hole that an
% "isempty(OVR)" cache cannot:
%   1. run MLTP        -> cache records "no override"
%   2. write the sheet -> setupOverride.mat appears, manifest records the values
%                         as APPLIED
%   3. run MLTP again  -> a stale cache would still say "no override", so the run
%                         uses code defaults while the manifest certifies the
%                         sheet's values.
% That is precisely the failure the manifest exists to prevent, so the manifest
% asserting the opposite is worse than silence. The reverse case matters too:
% deleting setupOverride.mat mid-session must return to code defaults.
f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setupOverride.mat');  % Functions/ -> repo root
d = dir(f);
if isempty(d), stamp = -1; else, stamp = d.datenum; end
if isempty(OVR) || ~isfield(OVR,'stamp') || OVR.stamp ~= stamp
    OVR = struct('loaded', false, 'vals', struct(), 'stamp', stamp);
    if isfile(f)
        try
            S = load(f);
            if isfield(S, 'setup') && isstruct(S.setup)
                OVR.vals   = S.setup;
                OVR.loaded = true;
                when = '';
                if isfield(S,'setupMeta') && isfield(S.setupMeta,'written')
                    when = sprintf(' [written %s from %s]', S.setupMeta.written, S.setupMeta.sheet);
                end
                fprintf('setupValue: setup sheet override active (%d parameters) from %s%s\n', ...
                        numel(fieldnames(S.setup)), f, when);
            end
        catch ME
            warning('setupValue:badOverride', ...
                'setupOverride.mat could not be read (%s) - using code defaults.', ME.message);
        end
    end
end

if nargin < 2
    error('setupValue:usage', 'setupValue(name, defaultValue) requires both arguments.');
end

v = defaultValue;
if ~OVR.loaded || ~isfield(OVR.vals, name)
    return
end
o = OVR.vals.(name);

% Type/shape guard. The sheet loader already validates everything it
% writes, so this only fires on a hand-made or truncated setupOverride.mat - and
% it warns and keeps the code default rather than erroring, for the same reason
% the loader does (locked decision 11): a bad sheet must not stop a solve. It
% matters because these values are consumed by CasADi expression building, where
% a [] or a NaN surfaces hundreds of lines later as an unrelated dimension error.
if isnumeric(defaultValue)
    if ~(isnumeric(o) && isscalar(o) && isreal(o) && isfinite(o))
        warning('setupValue:badType', ...
            'override for ''%s'' is not a finite real scalar - keeping the code default %g.', ...
            name, defaultValue);
        return
    end
elseif ischar(defaultValue) || isstring(defaultValue)
    if ~(ischar(o) || isstring(o)) || isempty(char(o))
        warning('setupValue:badType', ...
            'override for ''%s'' is not a non-empty string - keeping the code default ''%s''.', ...
            name, char(defaultValue));
        return
    end
    o = char(o);
end
v = o;
end
