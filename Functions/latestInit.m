function f = latestInit(circuit, aeroSetting, nu)
%LATESTINIT Path to the newest USABLE cached MLTP_initial warm start, or '' if none.
%   f = latestInit(circuit, aeroSetting)        any cache for this config
%   f = latestInit(circuit, aeroSetting, nu)    ...that also fits an nu-input model
%
%   Files are named init_<circuit>_<aeroSetting>[_<dv>]_<token>_<yyyyMMdd_HHmmss>.mat
%   under Data\<circuit>\initialisation\. The timestamp format sorts
%   lexicographically == chronologically, so a name sort beats file mtime
%   (which a copy or a checkout would perturb).
%   <token> is the MODEL VERSION, not a date: caches built under an older tyre or
%   aero model deliberately fail to match the glob and are regenerated, because a
%   stale warm start is slow and risks IPOPT infeasibility. Bump the token
%   whenever the tyre or aero model changes.
%
%   Most ActiveRW variants share one token. Two do not, and must not: AFWd and
%   ARFWd/ARFWr are different aero models (a 3-node front-wing axis and a two-wing
%   map) rather than variants of the 4-node rear-wing sweep, so their caches must
%   never satisfy each other's glob. The literals live at the bottom of this file
%   and are mirrored in Scripts/MLTP_initial.m - keep the two in step.
%
%   THE nu FILTER (why it exists). vp.aeroSetting does NOT encode the drivetrain:
%   an ARWm run is 'ARWm' whether it is AWD (nu=4) or ATD (nu=8). Without this
%   filter the glob happily returns an AWD cache for an ATD run, and MLTP.m then
%   dies on "init u_opt has 4 rows - expected 3 or nu = 8" - or, before that
%   assert existed, warm-started from a control vector of the wrong width. The
%   optional `nu` makes the choice by CONTENT rather than by filename, so it is
%   robust to whatever suffix convention a writer happens to use: a cache is
%   usable if its u_opt has 3 rows (the simplified MLTP_initial model, valid for
%   any config) or exactly nu rows. Caches that fit neither are skipped, and the
%   caller falls back to a cold start rather than failing.

if nargin < 3, nu = []; end

initDir = fullfile('Data', circuit, 'initialisation');
% AFWd's model (Functions/fwAeroDelta.m's 3-node combined FW+RW map) is not a
% version bump of the RW map every other ActiveRW variant shares, so it gets
% its own token - see the header note above.
token = 'zenvoMF52rw4n';
if strcmp(aeroSetting, 'AFWd'), token = 'zenvoMF52fw3n'; end
if strcmp(aeroSetting, 'ARFWd'), token = 'zenvoMF52arfw'; end   % two-wing model (nu 5/9), own caches
if strcmp(aeroSetting, 'ARFWr'), token = 'zenvoMF52arfwr'; end   % reactive law variant, own caches
d = dir(fullfile(initDir, sprintf('init_%s_%s_%s_*.mat', circuit, aeroSetting, token)));
% Also admit a drivetrain-tagged variant, init_<circuit>_<setting>_<dv>_<token>_*
dTag = dir(fullfile(initDir, sprintf('init_%s_%s_*_%s_*.mat', circuit, aeroSetting, token)));
if ~isempty(dTag), d = [d(:); dTag(:)]; end

if isempty(d)
    f = '';
    return
end

names = unique({d.name});
names = sort(names);                    % name sort == chronological, see above

for k = numel(names):-1:1               % newest first
    cand = fullfile(initDir, names{k});
    if isempty(nu) || initFits(cand, nu)
        f = cand;
        return
    end
end
f = '';                                 % none usable -> caller cold-starts
end

% ---------------------------------------------------------------------------
function tf = initFits(file, nu)
%INITFITS true when this cache's control width suits an nu-input model.
%  3 rows = the simplified MLTP_initial solve (Tdrive, Tbrake, delta), which is
%  valid for every config; nu rows = a full solution of a config with this exact
%  ladder. Anything else belongs to a different drivetrain/aero combination.
tf = false;
try
    S = load(file, 'data');
    if ~isfield(S,'data'), return, end
    dat = S.data;
    if isfield(dat,'init') && isstruct(dat.init) && isfield(dat.init,'u_opt')
        r = size(dat.init.u_opt, 1);
    elseif isfield(dat,'u_opt')
        r = size(dat.u_opt, 1);
    else
        return                          % no controls to judge - treat as unusable
    end
    tf = (r == 3) || (r == nu);
catch
    tf = false;                         % unreadable/corrupt cache is not usable
end
end
