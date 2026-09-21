function [matPath, src, pack] = activeTrack()
%ACTIVETRACK Which solved-lap sidecar the sim tools are currently pointed at.
%   [matPath, src, pack] = ACTIVETRACK() returns the ABSOLUTE path of the .mat
%   file every default-argument sim tool should read -- the one place that
%   answers "which track is the sim set up for". buildRefPath(),
%   buildTrackRibbon(), buildReplayInput() and runDemoLap() all take their
%   default from here, so pointing the whole sim at another track is a single
%   file write (setupTrack.m) rather than an edit in four places.
%
%   Outputs
%     matPath  char, absolute path. ALWAYS returned, even when nothing on the
%              list exists (see the last resolution step) -- the caller's own
%              "file not found" error then names the familiar default path,
%              exactly as it did before this resolver existed.
%     src      char, which rule produced it:
%                'activeTrack:matPath'   the solved sidecar named by the pack
%                'activeTrack:ribbonPath' the pack's geometry-only sidecar
%                                         (the solved one is not on this
%                                         machine -- e.g. a public checkout)
%                'default:private'  solutions/report/BCN/raw/...  (private)
%                'default:laps'     simulink/data/laps/...        (shipped)
%                'default:ribbon'   simulink/data/trackRibbon_BCN.mat
%                'default:missing'  nothing exists; the private path is
%                                   returned so the error message is familiar
%     pack     struct loaded from simulink/data/activeTrack.mat, or [] when
%              there is no active track. Fields (written by setupTrack.m):
%              tag, matPath, ribbonPath, drv, vPlan, planRW, planFW,
%              lapOffline, stopTimeHint, builtOn.
%
%   RESOLUTION ORDER
%     1. simulink/data/activeTrack.mat -> pack.matPath, else pack.ribbonPath
%     2. solutions/report/BCN/raw/run_BCN_ARFWr_ATD_data.mat   (private repo)
%     3. simulink/data/laps/run_BCN_ARFWr_ATD_data.mat         (shipped slim)
%     4. simulink/data/trackRibbon_BCN.mat                     (geometry only)
%   The first path that EXISTS wins. Steps 2-4 are exactly the fallback chain
%   buildTrackRibbon.m used to carry inline, so with no activeTrack.mat on
%   disk every tool behaves bit-identically to before this file existed.
%
%   Paths stored in the pack are kept RELATIVE to the repo root and anchored
%   here via mfilename('fullpath') -- never pwd, never a bare-name load: a
%   folder holding old runs can sit on the MATLAB path (genpath) and shadow a
%   bare filename with a stale one.
%
%   A pack whose two paths BOTH point at files that are not on this machine is
%   reported through 'activeTrack:packMissing' (a warning, not an error) and
%   the default chain is used instead, so a stale pack degrades to the shipped
%   behaviour rather than breaking every tool at once.
%
%   See also setupTrack, applyTrackPack, buildRefPath, buildTrackRibbon.

toolsDir = fileparts(mfilename('fullpath'));          % ...\simulink\tools
simDir   = fileparts(toolsDir);                       % ...\simulink
repoRoot = fileparts(simDir);                         % repo root

packPath = fullfile(simDir, 'data', 'activeTrack.mat');

defPrivate = fullfile(repoRoot, 'solutions', 'report', 'BCN', 'raw', 'run_BCN_ARFWr_ATD_data.mat');
defLaps    = fullfile(simDir, 'data', 'laps', 'run_BCN_ARFWr_ATD_data.mat');
defRibbon  = fullfile(simDir, 'data', 'trackRibbon_BCN.mat');

pack = [];

% ---- 1. an explicitly selected track ----------------------------------
if isfile(packPath)
    S = load(packPath);
    if isfield(S, 'active') && isstruct(S.active)
        pack = S.active;
        cand = {};
        if isfield(pack, 'matPath') && ~isempty(pack.matPath)
            cand(end+1, :) = {anchor(pack.matPath, repoRoot), 'activeTrack:matPath'};
        end
        if isfield(pack, 'ribbonPath') && ~isempty(pack.ribbonPath)
            cand(end+1, :) = {anchor(pack.ribbonPath, repoRoot), 'activeTrack:ribbonPath'};
        end
        for i = 1:size(cand, 1)
            if isfile(cand{i,1})
                matPath = cand{i,1};
                src     = cand{i,2};
                return
            end
        end
        tagTxt = '<untagged>';
        if isfield(pack, 'tag'), tagTxt = char(pack.tag); end
        warning('activeTrack:packMissing', ...
            ['activeTrack: %s selects track ''%s'' but neither of its sidecars is on this ' ...
             'machine; falling back to the shipped defaults. Re-run setupTrack, or delete ' ...
             'the pack to go back to Barcelona.'], packPath, tagTxt);
        pack = [];
    else
        warning('activeTrack:packMalformed', ...
            ['activeTrack: %s has no ''active'' struct (variables: {%s}); ignoring it. ' ...
             'Re-run setupTrack to rewrite it.'], packPath, strjoin(fieldnames(S)', ', '));
    end
end

% ---- 2-4. the shipped chain -------------------------------------------
chain = {defPrivate, 'default:private'; defLaps, 'default:laps'; defRibbon, 'default:ribbon'};
for i = 1:size(chain, 1)
    if isfile(chain{i,1})
        matPath = chain{i,1};
        src     = chain{i,2};
        return
    end
end

% ---- nothing exists ----------------------------------------------------
% Return the private default anyway: the caller raises its own
% <fcn>:matNotFound naming that path, which is the message this chain
% produced before the resolver existed.
matPath = defPrivate;
src     = 'default:missing';

end

% =========================================================================
function p = anchor(p, repoRoot)
%ANCHOR Make a repo-root-relative path absolute; leave an absolute one alone.
p = char(p);
if ~java.io.File(p).isAbsolute()
    p = fullfile(repoRoot, p);
end
end
