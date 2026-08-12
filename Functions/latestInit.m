function f = latestInit(circuit, aeroSetting, nu)
%LATESTINIT Path to the newest USABLE cached MLTP_initial warm start, or '' if none.
%   f = latestInit(circuit, aeroSetting)        any cache for this config
%   f = latestInit(circuit, aeroSetting, nu)    ...that also fits an nu-input model
%
%   Files are named init_<circuit>_<aeroSetting>[_<dv>]_<token>_<yyyyMMdd_HHmmss>.mat
%   under Data\<circuit>\initialisation\. The timestamp format sorts
%   lexicographically == chronologically, so a name sort beats file mtime
%   (which a copy or a checkout would perturb).
%   <token> is zenvoMF52rw4n for every ActiveRW variant EXCEPT AFWd
%   (zenvoMF52fw3n) and ARFWd (zenvoMF52arfw, two-wing model) - see below.
%   The zenvoMF52rw4n token is the model version: caches generated under an older
%   model (generic tyre, the pre-RW-angle 'zenvoMF52' aero, or the five-node
%   'zenvoMF52rw' ActiveRW map) deliberately no longer match the glob (a stale
%   warm start is slow and risks IPOPT infeasibility) - they are regenerated
%   instead. Bumped 2026-08-05 from 'zenvoMF52rw' to 'zenvoMF52rw4n' when the
%   ActiveRW map was cut from five nodes to four and the free wing's control
%   bound was capped at +15 deg (was +20): the four surviving five-node caches
%   (BCN/NUR x ARW/ARWv) carry alpha knots up to +20, outside the new bound, so
%   they must not warm-start the new model - see
%   docs/superpowers/specs/2026-08-05-four-node-map-and-report-fixes-design.md.
%   Bump the token again if the tyre or aero model changes again.
%   zenvoMF52fw3n (added 2026-08-07) is AFWd's OWN token, not a bump of the RW
%   one: AFWd's vp.aeroARW is a 3-node map over Functions/fwAeroDelta.m's
%   combined FW+RW "unload axis" ([-25 -20 0] deg), an entirely different model
%   to the 4-node rear-wing sweep every other ActiveRW variant shares, so an
%   AFWd cache must never satisfy an ARW/ARWv/ARWd glob or vice versa - see
%   docs/superpowers/plans/2026-08-07-afwd-front-wing.md.
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
