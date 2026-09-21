function [track, meta] = resolveCircuit(name)
%RESOLVECIRCUIT Resolve a circuit name to a track struct, and report its geometry.
%
%   track        = resolveCircuit(name)
%   [track,meta] = resolveCircuit(name)
%
%   THE SINGLE OWNER OF "which track does this circuit name mean". Scripts\userOpts.m
%   used to answer it with a hardcoded switch over a handful of names, which made the
%   solver track-agnostic everywhere EXCEPT at its own entry point: adding a circuit
%   meant editing a script. Everything downstream (init-cache token, apex CSV name,
%   solutions\ folder, report paths) already interpolates the circuit NAME, so moving
%   the resolution here is all it takes to make a new track a pure data drop.
%
%   RESOLUTION ORDER
%     1. the four built-in VIRTUAL tracks - 'Hairpin', 'Straight', 'Sturn',
%        'VirtualTrack' - synthesised here from the curvature arrays that used to
%        live in userOpts.m, verbatim (same numbers, same simpleMA smoothing, same
%        2 m assumed sample spacing, same field creation order k-then-s).
%     2. a name ALIAS for a shipped circuit file ('BCN', 'NUR', 'BCN_S1'..'BCN_S3').
%     3. Circuits\<name>_circuit.mat
%     4. Circuits\<name>.mat
%     5. `name` used as an explicit path to a .mat file (with or without the
%        extension), resolved as given - absolute, or relative to the caller's pwd.
%   The NAME the caller passed stays the token downstream; meta.tag records the
%   basename with any trailing '_circuit' removed, for callers that want a file tag.
%
%   ANCHORED ON THIS FILE, NOT ON pwd. Circuits\ is located from
%   mfilename('fullpath') (...\Functions -> repo root), never from the working
%   directory and never by a bare-name load(): run() cd's into a script's own folder
%   whenever the name it is given has a directory component, so pwd is not reliably
%   the repo root inside a run()-invoked script, and a bare name would resolve
%   through the whole MATLAB search path - picking up another checkout's Circuits\.
%   This is the same reasoning that pins runOverride.mat in Scripts\userOpts.m and
%   Parameters\vehParams.m.
%
%   OUTPUT
%     track  struct with at least .s (cumulative distance, m, strictly increasing)
%            and .k (signed curvature, 1/m), plus whatever else the file holds
%            (usually .x/.y cartesian centreline, used only for plotting). For a
%            file-backed circuit this is EXACTLY what load() returns, with nothing
%            added or removed, so
%              isequal(resolveCircuit('BCN'), load('Circuits/Barcelona_circuit.mat'))
%            is true. All diagnostics go in `meta`, never in `track`.
%     meta   .name .tag .source ('virtual'|'file') .file
%            .N .length_m .ds_median .ds_max .ds_min
%            .gap_m .gap_source .R_min .dkds_max .netTurn
%            .warnings (cellstr of the warnings that were raised)
%
%   CHECKS. Structural faults are errors raised immediately - there is no useful
%   geometry report for a track with no s/k, a length mismatch, a non-finite sample
%   or a non-increasing s. Everything else is reported first and judged after, so a
%   bad track produces its FULL diagnosis in one run rather than one complaint per
%   re-run:
%     ERROR   missing s or k; non-numeric/complex/empty s or k; numel(s) ~= numel(k);
%             fewer than 3 samples; any non-finite value; any diff(s) <= 0;
%             max(diff(s)) > 25 m (raised AFTER the report and the warnings below)
%     WARNING max(diff(s)) > 10 m       resolveCircuit:coarseGrid
%             R_min < 10 m              resolveCircuit:tightRadius
%             endpoint gap > 10 m       resolveCircuit:openLoop
%             max|dk/ds| > 0.02 1/m^2   resolveCircuit:roughCurvature
%             |net turn| outside [0.9, 1.1] rev   resolveCircuit:netTurn
%   The collocation mesh is OPT_ds = 10 m (Scripts\userOpts.m), so a circuit sampled
%   more coarsely than that is being interpolated up, not resolved - hence the 10 m
%   warning; beyond 25 m the interpolation is fiction and it is refused. The
%   curvature-roughness and radius thresholds are convergence advice, not physics:
%   IPOPT on this fixed mesh copes poorly with a k(s) that steps.
%
%   EXPECTED NOISE, so it is not mistaken for a defect:
%     - The four virtual tracks are OPEN test geometries by construction, not laps,
%       so they legitimately raise the endpoint-gap and net-turn warnings. A note
%       line is printed alongside saying so.
%     - Spa raises resolveCircuit:tightRadius (R_min 7.98 m at the La Source hairpin)
%       and nothing else. That is a correct report of a real hairpin, not a fault.
%     - Barcelona and Nurburgring raise nothing.
%   Every warning carries its own identifier, so a caller that has read and accepted
%   one can silence exactly that one with warning('off', <id>).
%
%   See also SIMPLEMA, CURV2CART.

narginchk(1, 1);
if isstring(name) && isscalar(name), name = char(name); end
if ~(ischar(name) && ~isempty(name) && isrow(name))
    error('resolveCircuit:badName', ...
        'circuit name must be a non-empty character row vector or scalar string (got a %s).', class(name));
end

repoRoot   = fileparts(fileparts(mfilename('fullpath')));   % ...\Functions -> repo root
circuitDir = fullfile(repoRoot, 'Circuits');

meta = struct('name', name, 'tag', name, 'source', '', 'file', '', ...
              'N', NaN, 'length_m', NaN, 'ds_median', NaN, 'ds_max', NaN, 'ds_min', NaN, ...
              'gap_m', NaN, 'gap_source', '', 'R_min', NaN, 'dkds_max', NaN, ...
              'netTurn', NaN, 'warnings', {{}});

% ---------------------------------------------------------------------------
% 1) the four built-in virtual tracks - MOVED VERBATIM from Scripts\userOpts.m
%    (curvature arrays, simpleMA(...,10,2) smoothing, and the linspace that
%    assumes each element of the curvature array is spaced by 2 m). Do not
%    "tidy" these expressions: an archived run that names one of them must
%    reproduce bit-identically.
% ---------------------------------------------------------------------------
switch name
    case 'Hairpin'
        track.k = [zeros(1,75) 0.0975*ones(1,16) zeros(1,73)];                                      % Hairpin
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
        meta.source = 'virtual';
    case 'Straight'
        track.k = zeros(1,300);                                                                     % Straight line
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
        meta.source = 'virtual';
    case 'Sturn'
        track.k = [zeros(1,100) pi/45*ones(1,20) zeros(1,30) -pi/45*ones(1,20) zeros(1,100)];       % S-turn
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
        meta.source = 'virtual';
    case 'VirtualTrack'
        track.k = [zeros(1,150) pi/25*ones(1,20) zeros(1,50) -pi/125*ones(1,90) -pi/50*ones(1,35) pi/50*ones(1,35) -pi/500*(1:0.05:6) zeros(1,200) -pi/20*ones(1,20) zeros(1,80)]; %Virtual track
        track.k = simpleMA(track.k,10,2);                                                           % Smoothen curvature signal
        track.s = linspace(0,2*length(track.k),length(track.k));                                    % (assume each element of the curvature array is spaced by 2m)
        meta.source = 'virtual';
    otherwise
        [file, tag] = locateCircuitFile(name, circuitDir);
        track       = load(file);          % EXACTLY what load() returns - see the header
        meta.source = 'file';
        meta.file   = file;
        meta.tag    = tag;
end

% ---------------------------------------------------------------------------
% Structural checks - these must pass before any diagnostic can be computed
% ---------------------------------------------------------------------------
where = describeSource(meta);
if ~isfield(track, 's') || ~isfield(track, 'k')
    missing = {};
    if ~isfield(track,'s'), missing{end+1} = 's'; end
    if ~isfield(track,'k'), missing{end+1} = 'k'; end
    error('resolveCircuit:missingFields', ...
        ['circuit ''%s'' (%s) has no %s. A circuit .mat must hold at least s (cumulative ' ...
         'distance along the centreline, m, strictly increasing) and k (signed curvature, ' ...
         '1/m), each N-by-1; x and y (cartesian centreline) are optional and used only for ' ...
         'plotting. It holds: %s.'], ...
        name, where, strjoin(missing,' and '), strjoin(sort(fieldnames(track))', ', '));
end
s = track.s(:).';
k = track.k(:).';
if ~isnumeric(s) || ~isnumeric(k) || ~isreal(s) || ~isreal(k)
    error('resolveCircuit:notNumeric', ...
        'circuit ''%s'' (%s): s and k must be real numeric arrays (got %s and %s).', ...
        name, where, class(track.s), class(track.k));
end
if numel(s) ~= numel(k)
    error('resolveCircuit:lengthMismatch', ...
        'circuit ''%s'' (%s): numel(s) = %d but numel(k) = %d - they must be the same length.', ...
        name, where, numel(s), numel(k));
end
if numel(s) < 3
    error('resolveCircuit:tooShort', ...
        'circuit ''%s'' (%s): only %d sample(s) - need at least 3.', name, where, numel(s));
end
if any(~isfinite(s)) || any(~isfinite(k))
    error('resolveCircuit:nonFinite', ...
        ['circuit ''%s'' (%s): %d non-finite sample(s) in s and %d in k. NaN/Inf in the ' ...
         'track data propagates straight into the NLP''s curvature parameter and IPOPT ' ...
         'fails with no useful diagnosis.'], ...
        name, where, sum(~isfinite(s)), sum(~isfinite(k)));
end
ds = diff(s);
if any(ds <= 0)
    bad = find(ds <= 0, 1);
    error('resolveCircuit:nonIncreasingS', ...
        ['circuit ''%s'' (%s): s is not strictly increasing - s(%d) = %.6f, s(%d) = %.6f. ' ...
         's is the independent variable of the whole optimal-control problem; it must be a ' ...
         'monotonically increasing distance along the centreline.'], ...
        name, where, bad, s(bad), bad+1, s(bad+1));
end

% ---------------------------------------------------------------------------
% Geometry diagnostic
% ---------------------------------------------------------------------------
meta.N         = numel(s);
meta.length_m  = s(end) - s(1);
meta.ds_median = median(ds);
meta.ds_max    = max(ds);
meta.ds_min    = min(ds);
kmax           = max(abs(k));
if kmax > 0, meta.R_min = 1/kmax; else, meta.R_min = Inf; end
meta.dkds_max  = max(abs(diff(k)./ds));
meta.netTurn   = trapz(s, k) / (2*pi);          % revolutions; -1 = one clockwise lap

if isfield(track,'x') && isfield(track,'y') && isnumeric(track.x) && isnumeric(track.y) ...
        && numel(track.x) == meta.N && numel(track.y) == meta.N
    x = track.x(:).'; y = track.y(:).';
    meta.gap_m      = hypot(x(end)-x(1), y(end)-y(1));
    meta.gap_source = 'x,y';
else
    % No cartesian centreline: integrate the heading and close the loop numerically.
    % This is an ESTIMATE and will not agree exactly with Functions\curv2cart.m
    % (which reconstructs segment by segment); it is used for the closure warning
    % only, never for anything the solver or the plots consume.
    psi             = cumtrapz(s, k);                     % heading along the centreline (rad)
    meta.gap_m      = hypot(trapz(s, cos(psi)), trapz(s, sin(psi)));   % end point vs (0,0)
    meta.gap_source = 's,k (estimated)';
end

fprintf('resolveCircuit: ''%s'' -> %s\n', name, where);
fprintf(['resolveCircuit:   N = %d | length = %.1f m | ds median %.3f / min %.3f / max %.3f m | ' ...
         'endpoint gap %.2f m (%s)\n'], ...
    meta.N, meta.length_m, meta.ds_median, meta.ds_min, meta.ds_max, meta.gap_m, meta.gap_source);
fprintf('resolveCircuit:   R_min = %.2f m | max|dk/ds| = %.5f 1/m^2 | net turn = %+.4f rev\n', ...
    meta.R_min, meta.dkds_max, meta.netTurn);
if strcmp(meta.source, 'virtual')
    fprintf(['resolveCircuit:   (built-in virtual track - an OPEN test geometry, not a lap, ' ...
             'so the closure warnings below are expected)\n']);
end

% ---------------------------------------------------------------------------
% Warnings - all of them, every time, so one run gives the full diagnosis
% ---------------------------------------------------------------------------
meta = addWarning(meta, meta.ds_max > 10, 'resolveCircuit:coarseGrid', ...
    ['circuit ''%s'': sample spacing reaches %.1f m, coarser than the %g m collocation ' ...
     'step (Scripts/userOpts.m OPT_ds) - the curvature the solver sees between samples ' ...
     'is interpolated, not measured.'], name, meta.ds_max, 10);
meta = addWarning(meta, meta.R_min < 10, 'resolveCircuit:tightRadius', ...
    ['circuit ''%s'': minimum centreline radius is %.2f m. That is inside this vehicle''s ' ...
     'comfortable range and the corner may dominate convergence; check it is a real ' ...
     'hairpin and not a digitising artefact.'], name, meta.R_min);
meta = addWarning(meta, meta.gap_m > 10, 'resolveCircuit:openLoop', ...
    ['circuit ''%s'': start and finish are %.1f m apart (%s), so this is not a closed lap. ' ...
     'The solve still runs - the boundary condition is an entry-speed window, not a ' ...
     'periodicity constraint - but the vi closure loop has nothing physical to converge ' ...
     'on and the lap time is a point-to-point time.'], name, meta.gap_m, meta.gap_source);
meta = addWarning(meta, meta.dkds_max > 0.02, 'resolveCircuit:roughCurvature', ...
    ['circuit ''%s'': max|dk/ds| = %.4f 1/m^2. A stepping curvature signal is the single ' ...
     'most common cause of non-convergence here; pre-smooth digitised data (see ' ...
     'Functions/simpleMA.m) before solving.'], name, meta.dkds_max);
meta = addWarning(meta, abs(meta.netTurn) < 0.9 || abs(meta.netTurn) > 1.1, ...
    'resolveCircuit:netTurn', ...
    ['circuit ''%s'': net turn is %+.3f revolutions, not the ~+/-1 of a single closed lap. ' ...
     'Expected for a sector or an open test geometry; otherwise the curvature sign ' ...
     'convention or the sample spacing is wrong.'], name, meta.netTurn);

% ---------------------------------------------------------------------------
% Deferred hard stop - raised last so the report and every warning above are
% already on screen (a track this coarse usually has several other faults too).
% ---------------------------------------------------------------------------
if meta.ds_max > 25
    error('resolveCircuit:gridTooCoarse', ...
        ['circuit ''%s'' (%s) is too coarsely sampled to solve: max(diff(s)) = %.1f m ' ...
         'against a %g m collocation step. Re-sample the centreline to <= 10 m (ideally ' ...
         'uniform) before using it. See the warnings above for the rest of the diagnosis.'], ...
        name, where, meta.ds_max, 10);
end
end


% ===========================================================================
function [file, tag] = locateCircuitFile(name, circuitDir)
%LOCATECIRCUITFILE Alias -> <name>_circuit.mat -> <name>.mat -> explicit path.
%  Errors listing both the built-in names and what is actually in Circuits\.

% Name aliases for the shipped files whose basename is not the token used
% downstream. Everything else resolves by convention and needs no entry here -
% 'Spa' finds Spa_circuit.mat, 'Jarama' finds Jarama_circuit.mat.
ALIAS = { 'BCN',    'Barcelona_circuit.mat'
          'BCN_S1', 'Barcelona_circuit_s1.mat'
          'BCN_S2', 'Barcelona_circuit_s2.mat'
          'BCN_S3', 'Barcelona_circuit_s3.mat'
          'NUR',    'Nurburgring_circuit.mat' };

hit = find(strcmp(name, ALIAS(:,1)), 1);
if ~isempty(hit)
    file = fullfile(circuitDir, ALIAS{hit,2});
    tag  = name;                       % the alias IS the token downstream
    if ~isfile(file)
        error('resolveCircuit:aliasFileMissing', ...
            'circuit alias ''%s'' maps to %s, which is not in this checkout.\n%s', ...
            name, ALIAS{hit,2}, availability(circuitDir));
    end
    return
end

cand = { fullfile(circuitDir, [name '_circuit.mat'])
         fullfile(circuitDir, [name '.mat'])
         name
         [name '.mat'] };
for i = 1:numel(cand)
    if isfile(cand{i})
        file = cand{i};
        [~, base] = fileparts(file);
        tag  = regexprep(base, '_circuit$', '');
        return
    end
end

error('resolveCircuit:notFound', ...
    ['unknown circuit ''%s''. Tried, in order: the built-in virtual tracks, the name ' ...
     'aliases, Circuits\\%s_circuit.mat, Circuits\\%s.mat, and ''%s'' as a path.\n%s'], ...
    name, name, name, name, availability(circuitDir));
end


% ===========================================================================
function txt = availability(circuitDir)
%AVAILABILITY What this checkout can actually offer, for an error message.
d = dir(fullfile(circuitDir, '*.mat'));
if isempty(d)
    have = '(none)';
else
    have = strjoin(sort({d.name}), ', ');
end
txt = sprintf([ ...
    'Built-in virtual tracks: Hairpin, Straight, Sturn, VirtualTrack.\n' ...
    'Name aliases: BCN, BCN_S1, BCN_S2, BCN_S3, NUR.\n' ...
    'Circuits\\ holds: %s\n' ...
    'To add a track, drop a .mat holding s (m, strictly increasing) and k (1/m), ' ...
    'optionally x and y, into Circuits\\ and call it by its basename.'], have);
end


% ===========================================================================
function txt = describeSource(meta)
if strcmp(meta.source, 'virtual')
    txt = 'built-in virtual track';
else
    txt = meta.file;
end
end


% ===========================================================================
function meta = addWarning(meta, cond, id, fmt, varargin)
%ADDWARNING Raise a warning when cond holds, and record it on meta.warnings.
if ~cond, return, end
msg = sprintf(fmt, varargin{:});
warning(id, '%s', msg);
meta.warnings{end+1} = sprintf('%s: %s', id, msg);
end
