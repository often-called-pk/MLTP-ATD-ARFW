function info = solveLap(circuit, varargin)
%SOLVELAP Solve a minimum-lap-time problem end to end and save the lap .mat.
%
%   info = solveLap(circuit)
%   info = solveLap(circuit, config)
%   info = solveLap(circuit, config, drivetrain)
%   info = solveLap(..., Name, Value, ...)
%
%   THE ENTRY POINT. Everything else in Scripts\ is a script that expects a
%   hand-prepared workspace; this is the one function you call. It drives the
%   whole chain - circuit resolution, warm starts, the entry-speed closure loop,
%   the MLTP solve itself - and leaves behind exactly the same `data` .mat file
%   that the Simulink / Unreal replay tools read:
%
%       solutions\report\<circuit>\raw\run_<circuit>_<config>_<drivetrain>_data.mat
%
%   A fresh checkout needs nothing but CasADi on the MATLAB path (this project
%   does not add it - see PREREQUISITES below) and a circuit .mat in Circuits\.
%
%   ---------------------------------------------------------------------------
%   ARGUMENTS
%   ---------------------------------------------------------------------------
%   circuit      Circuit NAME, resolved by Functions\resolveCircuit.m: one of the
%                built-in virtual tracks (Hairpin, Straight, Sturn, VirtualTrack),
%                a shipped alias (BCN, NUR, BCN_S1..BCN_S3), or the basename of
%                any .mat in Circuits\ holding s (m, strictly increasing) and k
%                (1/m) - e.g. 'Spa' for Circuits\Spa_circuit.mat. Optional x and y
%                are used for plotting only.
%                It must be a plain NAME, not a path: the name is the token every
%                downstream artefact is keyed on (init cache folder, apex CSV,
%                solutions\ folder, output filename). Drop the .mat into Circuits\
%                and call it by its basename.
%
%   config       Run configuration. Default 'ARFWr'.
%                  Low | Mid | High | RWp15   fixed rear wing at -10 / 0 / +10 / +15 deg
%                  ARW                        free continuous rear wing, [-10,+15] deg
%                  ARWd                       ARW + braking airbrake floor, station audit
%                  AFWd                       front-wing unload axis, [-25,0] deg
%                  ARFWd                      both wings free (nu +1)
%                  ARFWr                      both wings pinned to the reactive law
%                ('ARWv', the velocity-scheduled wing, is retired - infeasible at
%                the 25 deg/s slew rate - and is not offered here.)
%
%   drivetrain   'ATD' (torque vectoring, default) or 'AWD' (fixed split).
%
%   ---------------------------------------------------------------------------
%   NAME-VALUE OPTIONS
%   ---------------------------------------------------------------------------
%   'Vi'         Entry-speed seed for the closure loop, m/s, applied to EVERY
%                ladder stage. Left unset, only a stage with nothing to learn from
%                starts at 75: each later stage inherits the previous stage's own
%                exit speed, which usually closes it on the first pass instead of
%                the second. See CLOSURE below.
%   'ViTol'      Closure tolerance, m/s. Default 0.5.
%   'ViMaxIter'  Maximum closure iterations per stage. Default 4.
%   'Ladder'     'auto' (default) solves the warm-start chain leading to `config`,
%                skipping any stage whose lap .mat already exists. 'direct' solves
%                `config` alone, warm-started from the simplified single-track
%                model (Scripts\MLTP_initial.m). 'auto' is the default because it
%                is the path every shipped result was produced on; 'direct' is
%                faster when it converges and is not guaranteed to.
%   'Force'      true re-solves the REQUESTED config even if its .mat already
%                exists (prerequisite stages are still skipped when satisfied).
%                Default false.
%   'DryRun'     true resolves the circuit, prints the plan and returns without
%                writing anything or invoking the solver. Default false.
%   'Setup'      true calls setupTrack(info.matPath,'Persist',false) afterwards, to
%                point the Simulink / 3D sim at the solved lap. Default false.
%                Requires the Simulink project's tools on the path (open
%                simulink\ARFWr_RT.prj first); warns and skips if they are not.
%                'Persist',false is deliberate - no tracked model is ever saved as a
%                side effect of solving a lap.
%   'OutDir'     Root for the output tree. Default solutions\report (relative paths
%                are anchored to the repo root, never to pwd). The lap lands in
%                <OutDir>\<circuit>\raw\.
%
%   ---------------------------------------------------------------------------
%   RETURNS
%   ---------------------------------------------------------------------------
%   info  .circuit .config .drivetrain .ladder
%         .matPath   the lap .mat (this is what setupTrack / buildRefPath want)
%         .source    'solved' | 'existing'
%         .status    IPOPT return status of the final stage
%         .iters .lap .vi .vend .miss .closed
%         .stages    1-by-n struct array, one per ladder stage: .name .action
%                    .status .iters .lap .miss .closed .viIters .file .solvedOn
%         .track     the geometry report from resolveCircuit (N, length, ds, gap,
%                    R_min, max|dk/ds|, net turn, warnings raised)
%         .solvedOn  when the lap being returned was SOLVED. For a lap solved in
%                    this call it is the stamp written into the .mat; for one
%                    resolved off disk it is that file's own data.solvedOn, and
%                    '' when the file predates the stamp. Never a fabricated
%                    "now" - a lap from last year must not read as a fresh solve.
%         .resolvedOn when THIS call ran, always set.
%
%   ---------------------------------------------------------------------------
%   PREREQUISITES
%   ---------------------------------------------------------------------------
%   CasADi 3.x must be on the MATLAB path before the first solve; nothing in this
%   repo adds it:
%       addpath('D:\...\casadi-3.7.2-windows64-matlab2018b')
%   solveLap itself puts the repo's own folders on the path (repo root, Scripts,
%   Parameters, genpath(Functions)) and sets the working directory to the repo
%   root, which is the convention every script here assumes. It deliberately does
%   NOT genpath the whole repo: that would put archive\ and solutions\ on the path,
%   where a bare-name load() can resolve an archived run instead of the live one.
%   A run with no solve to do (everything already on disk) needs no CasADi.
%
%   ---------------------------------------------------------------------------
%   CLOSURE (the vi loop)
%   ---------------------------------------------------------------------------
%   The lap is NOT periodic: the initial state is an entry-speed WINDOW, vi +/- 1
%   m/s in physical units (Scripts\MLTP.m builds it as Xi./x_s +/- OPT_e and
%   vx_s = 100), and the final speed is free. A lap is "closed" when the solved
%   entry and exit speeds agree, so solveLap iterates the fixed point exactly as
%   the private comparison batch does: start at 'Vi', accept when
%   |vx(1) - vx(end)| <= 'ViTol', otherwise reseed vi = vx(end) - 1 and re-solve,
%   up to 'ViMaxIter' times.
%   Reseeding with vend (not vend - 1) stalls: the solver always takes the fastest
%   admissible entry, so vx(1) lands on vi + 1 every time and the miss sticks at
%   ~1.0 forever. The -1 is the slack, not a fudge.
%   Running out of iterations is a warning, not an error - the lap is still a
%   converged optimum for its own boundary conditions; it is the closure claim
%   that fails. info.closed records it.
%
%   Stages after the first do not start from scratch. Every rung of the ladder is
%   the same car on the same track, so their closed entry speeds agree to a few
%   tenths, and the seed is carried forward: a stage starts at vend - 1 of the
%   nearest already-solved stage - just solved, or read back out of its lap .mat
%   when it was skipped - and falls back to 75 only when there is no such stage.
%   Without it every rung repeated the same discovery: start at 75, miss closure
%   by ~6 m/s, re-solve. On Spa that was one wasted full solve on each of the four
%   rungs. Passing 'Vi' explicitly turns the carry-forward off, because an explicit
%   seed is a deliberate choice and is honoured on every stage.
%
%   ---------------------------------------------------------------------------
%   THE WARM-START LADDER
%   ---------------------------------------------------------------------------
%   Each active-aero config is warm-started from the one below it, because a cold
%   start on a pinned or floored wing rarely converges:
%       ARW  ->  ARWd  ->  ARFWd  ->  ARFWr
%       ARW  ->  AFWd
%   'auto' walks that chain from the bottom and SKIPS any stage whose lap .mat is
%   already on disk, looking in two places:
%       <OutDir>\<circuit>\raw\run_<circuit>_<cfg>_<dv>_data.mat   (solved here)
%       simulink\data\laps\run_<circuit>_<cfg>_<dv>_data.mat       (shipped)
%   so on a fresh clone the two shipped circuits resolve in seconds and only a new
%   track actually solves.
%   Each stage's init cache is written under Data\<circuit>\initialisation\ with
%   its OWN model-version token - zenvoMF52rw4n (ARW/ARWd and the static settings),
%   zenvoMF52fw3n (AFWd), zenvoMF52arfw (ARFWd), zenvoMF52arfwr (ARFWr). The tokens
%   MUST match Functions\latestInit.m and Scripts\MLTP_initial.m: they are what
%   stops one config's cache satisfying another's warm-start glob (the aero models
%   behind them are different maps with different control bounds, and a silently
%   mismatched warm start produces a lap that started somewhere it could never
%   have gone).
%
%   ---------------------------------------------------------------------------
%   CONVERGENCE IS NOT GUARANTEED
%   ---------------------------------------------------------------------------
%   This is a non-convex optimal-control problem solved by an interior-point
%   method on a fixed mesh, with tolerances, regularisation, rate limits and vi
%   hand-calibrated at Barcelona. The shipped circuits converge on the shipped
%   settings; another track may not, and that is normal rather than a defect. In
%   order, the remedies are: let the vi loop run its iterations; raise OPT_ds from
%   10 to 12-15 m; relax opts.ipopt.tol to 1e-5; solve ARW first and let the ladder
%   carry it up. The first two are reachable without editing anything - solveLap
%   writes runOverride.mat, and Scripts\userOpts.m reads OPT_ds and ipoptTol from
%   it - but they change the problem, so say so when quoting the result.
%   A stage that does not return Solve_Succeeded raises MLTP:notConverged naming
%   the stage. Such a lap time is not an optimum and must not be quoted.
%   Curvature must be smooth: resolveCircuit reports max|dk/ds| before the solve.
%
%   ---------------------------------------------------------------------------
%   HOW IT DRIVES MLTP
%   ---------------------------------------------------------------------------
%   Scripts\MLTP.m is a SCRIPT that starts with `clear; clear global;` and warm-
%   starts through importfile(), which assignin()s into the BASE workspace. Calling
%   it from inside a function would leave that `data` in base, unreachable from the
%   caller, and MLTP would die on data.init.x_opt. So it is invoked with
%   evalin('base','MLTP;') and its result is recovered from the global `data` -
%   re-fetched through a helper with a fresh workspace each time, because MLTP's
%   own `clear global` severs any binding held across the call.
%   Running a solve therefore CLEARS THE BASE WORKSPACE, exactly as MLTP.m always
%   has, and MLTP.m's leading clc wipes the command window - which is why the
%   summary table is printed at the end rather than as it goes.
%
%   ---------------------------------------------------------------------------
%   THE BASE WORKSPACE
%   ---------------------------------------------------------------------------
%   Because MLTP.m clears it, solveLap SNAPSHOTS the base workspace before the
%   first stage that really solves and puts it back on the way out - on a normal
%   return and on an error alike, through onCleanup. After the ladder the base
%   workspace holds what it held before, not the several hundred variables the
%   solver left behind.
%   This is not cosmetic. With the Simulink project open, base carries act, inrt,
%   sus, vp, pt and hudGear, which mask expressions inside Plant resolve by name;
%   without the restore the first runDemoLap after a solve dies on 'Error
%   evaluating parameter Iveh' - reported by Simulink only as "Error due to
%   multiple causes" - and the cure looked like reopening the project.
%   Two limits, both deliberate. GLOBALS are recorded but not re-created: MLTP's
%   own `clear global` destroys the global itself, so re-assigning the value
%   would leave an ordinary variable wearing a global's name. And anything
%   evalin cannot hand over by value is listed in a solveLap:baseNotRestored
%   warning rather than failing the solve - MLTP would have cleared it anyway.
%   A run with nothing to solve takes no snapshot and touches nothing.
%   The RESULT is not in the base workspace and never was: it is the returned
%   `info` and the saved .mat at info.matPath. Driving this from a script
%   (matlab -batch "info = solveLap('Spa')") is unaffected - `info` is assigned
%   in base after the function returns, which is after the restore.
%   MLTP's own trailing post-processing (SDI logging, plotSDI, the circuit-map
%   figure, apexSpeeds) runs inside that call. If any of it throws - the usual
%   cause is a headless -batch session with no display - the solve is NOT
%   discarded: the exception is caught, and as long as the global `data` already
%   carries solver_status and t_opt the stage continues with the solved lap and a
%   solveLap:postSolve warning. That is the same seam the private comparison batch
%   uses, and it is what makes an unattended `matlab -batch` run survive a
%   plotting failure.
%   runOverride.mat is written at the repo root (anchored on this file's own
%   location, never on pwd) and deleted by onCleanup on every exit path, including
%   an error - a stray one silently reconfigures every later run in this checkout.
%
%   ---------------------------------------------------------------------------
%   SCOPE
%   ---------------------------------------------------------------------------
%   Lap-time differences below roughly 0.05 s are not resolved by this method
%   (fixed mesh, no ph refinement, interior-point tolerances). Rank configurations
%   with it; do not quote small absolute deltas from it.
%
%   EXAMPLES
%       addpath('D:\...\casadi-3.7.2-windows64-matlab2018b')
%       info = solveLap('BCN');                       % shipped lap, resolves at once
%       info = solveLap('Spa');                       % real solve, ARFWr + ATD
%       info = solveLap('Spa','ARW','ATD');           % just the free-wing stage
%       info = solveLap('Spa', 'DryRun', true);       % what would it do?
%       info = solveLap('Spa','ARW','ATD','Vi',70,'ViMaxIter',6);
%       info = solveLap('BCN','ARFWr','ATD','Force',true);   % re-solve, ignore the .mat
%       info = solveLap('Spa','ARFWr','ATD','Setup',true);   % solve, then load into Simulink
%
%   See also RESOLVECIRCUIT, MLTP, MLTP_INITIAL, LATESTINIT, NUFORCONFIG.

%% ---------------------------------------------------------------------------
%  Closure-loop constants. Same values the shipped laps were solved with.
%% ---------------------------------------------------------------------------
VI_SEED  = 75;      % entry-speed seed (m/s) - both shipped circuits started here
VI_TOL   = 0.5;     % accept |vx(1)-vx(end)| <= this (m/s)
VI_MAXIT = 4;       % closure iterations per stage
VI_DBACK = 1;       % reseed vi = vend - VI_DBACK; == OPT_e*vx_s = 1e-2*100, the
                    % width of the entry-speed window. See CLOSURE in the header.
RW_SLEW  = 25;      % rear-wing slew rate (deg/s) used when station-rounding a seed.
                    % Mirrors c.ub.RW in Scripts\userOpts.m; userOpts is not run
                    % here, so the value is restated rather than re-derived.

%% ---------------------------------------------------------------------------
%  Repo bootstrap - locate the repo from THIS file, never from pwd
%% ---------------------------------------------------------------------------
repoRoot = fileparts(fileparts(mfilename('fullpath')));      % ...\Scripts -> repo root
addpath(repoRoot, fullfile(repoRoot,'Scripts'), fullfile(repoRoot,'Parameters'));
addpath(genpath(fullfile(repoRoot,'Functions')));            % genpath: PolyfitnTools is nested

%% ---------------------------------------------------------------------------
%  Arguments
%% ---------------------------------------------------------------------------
OPTNAMES = {'Vi','ViTol','ViMaxIter','Ladder','Force','DryRun','Setup','OutDir'};

if isstring(circuit) && isscalar(circuit), circuit = char(circuit); end
if ~(ischar(circuit) && ~isempty(circuit) && isrow(circuit))
    error('solveLap:badCircuit', 'circuit must be a non-empty name (got a %s).', class(circuit));
end
if any(circuit == '/') || any(circuit == '\') || any(circuit == ':')
    error('solveLap:circuitIsPath', ...
        ['circuit must be a NAME, not a path (got ''%s''). The name is the token every ' ...
         'artefact of this run is keyed on - the init cache folder Data\\<name>\\, the apex ' ...
         'CSV, the solutions\\ folder and the output filename - so a path would produce ' ...
         'nonsense paths downstream. Copy the .mat into Circuits\\ and pass its basename.'], ...
        circuit);
end

% Positional config/drivetrain, then Name-Value. Split them by name: no config is
% also an option name, so this is unambiguous and solveLap('BCN','DryRun',true)
% works without repeating the defaults.
args = varargin;
pos  = {};
while ~isempty(args) && (ischar(args{1}) || (isstring(args{1}) && isscalar(args{1}))) ...
        && ~any(strcmpi(char(args{1}), OPTNAMES))
    pos{end+1} = char(args{1}); %#ok<AGROW>
    args(1)    = [];
end
if numel(pos) > 2
    error('solveLap:tooManyPositional', ...
        ['expected at most solveLap(circuit, config, drivetrain) before any Name-Value ' ...
         'pair, got %d positional arguments: %s. Valid option names are: %s.'], ...
        numel(pos), strjoin(pos, ', '), strjoin(OPTNAMES, ', '));
end
config     = 'ARFWr';  if numel(pos) >= 1, config     = pos{1}; end
drivetrain = 'ATD';    if numel(pos) >= 2, drivetrain = pos{2}; end

opt = struct('Vi', VI_SEED, 'ViTol', VI_TOL, 'ViMaxIter', VI_MAXIT, ...
             'Ladder', 'auto', 'Force', false, 'DryRun', false, 'Setup', false, ...
             'OutDir', fullfile('solutions','report'));
if mod(numel(args), 2) ~= 0
    error('solveLap:badPairs', 'Name-Value arguments must come in pairs.');
end
% An EXPLICIT 'Vi' is honoured on every stage; only an unset one is carried
% forward from the previous stage's exit speed (see CLOSURE in the header). The
% flag is raised here rather than inferred later by comparing opt.Vi with the
% default, which would silently disable the carry-forward for anyone who passed
% 'Vi', 75.
viGiven = false;
for i = 1:2:numel(args)
    nm = char(args{i});
    j  = find(strcmpi(nm, OPTNAMES), 1);
    if isempty(j)
        error('solveLap:unknownOption', 'unknown option ''%s'' (expected one of: %s).', ...
            nm, strjoin(OPTNAMES, ', '));
    end
    opt.(OPTNAMES{j}) = args{i+1};
    viGiven = viGiven || strcmp(OPTNAMES{j}, 'Vi');
end
validateattributes(opt.Vi,        {'numeric'}, {'scalar','real','finite','positive'}, mfilename, 'Vi');
validateattributes(opt.ViTol,     {'numeric'}, {'scalar','real','finite','positive'}, mfilename, 'ViTol');
validateattributes(opt.ViMaxIter, {'numeric'}, {'scalar','integer','positive'},       mfilename, 'ViMaxIter');
opt.Force  = logical(opt.Force);
opt.DryRun = logical(opt.DryRun);
opt.Setup  = logical(opt.Setup);
opt.Ladder = lower(char(opt.Ladder));
if ~any(strcmp(opt.Ladder, {'auto','direct'}))
    error('solveLap:badLadder', '''Ladder'' must be ''auto'' or ''direct'' (got ''%s'').', opt.Ladder);
end

drivetrain = upper(char(drivetrain));
if ~any(strcmp(drivetrain, {'AWD','ATD'}))
    error('solveLap:badDrivetrain', ...
        'drivetrain must be ''AWD'' or ''ATD'' (got ''%s'').', drivetrain);
end
if strcmp(drivetrain,'ATD'), ATDstr = 'On'; else, ATDstr = 'Off'; end

config = char(config);
configDef(config);                 % validate now, before anything is written

% Output root. A RELATIVE root is anchored to the repo, never to pwd: every
% skip-if-exists decision below goes through isfile(), and a relative path would
% follow the caller's working directory rather than this checkout.
reportRoot = char(opt.OutDir);
if ~isAbsPath(reportRoot), reportRoot = fullfile(repoRoot, reportRoot); end

%% ---------------------------------------------------------------------------
%  Working directory + runOverride.mat lifetime
%% ---------------------------------------------------------------------------
% cwd == repo root is a hard convention here: Scripts\MLTP.m does run('Parameters\
% vehParams.m'), Functions\latestInit.m globs a relative Data\<circuit>\..., and
% both break from anywhere else. Restored on every exit path.
startDir   = pwd;
restoreDir = onCleanup(@() cd(startDir));
cd(repoRoot);

% Anchored on this file, exactly like the reader (Scripts\userOpts.m and
% Parameters\vehParams.m build the same path from their own mfilename).
ovrFile    = fullfile(repoRoot, 'runOverride.mat');
ovrCleanup = onCleanup(@() safeDelete(ovrFile));

%% ---------------------------------------------------------------------------
%  Circuit + plan
%% ---------------------------------------------------------------------------
[~, tmeta] = resolveCircuit(circuit);      % errors / warns on the geometry, prints the report

if strcmp(opt.Ladder,'auto')
    chain = ladderFor(config);
else
    chain = {config};
end

info = struct('circuit', circuit, 'config', config, 'drivetrain', drivetrain, ...
              'ladder', opt.Ladder, 'matPath', '', 'source', '', 'status', '', ...
              'iters', NaN, 'lap', NaN, 'vi', NaN, 'vend', NaN, 'miss', NaN, ...
              'closed', false, ...
              'solvedOn', '', ...
              'resolvedOn', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
              'dryRun', opt.DryRun);
% Assigned rather than passed to struct(): a struct-valued (and especially an
% EMPTY struct-array-valued) field inside a struct() call is exactly the kind of
% thing that quietly produces a struct array instead of one struct.
info.stages = emptyStageArray();
info.track  = tmeta;

fprintf('solveLap: %s / %s / %s | ladder ''%s'': %s\n', ...
    circuit, config, drivetrain, opt.Ladder, strjoin(chain, ' -> '));
fprintf('solveLap: output root %s\n', reportRoot);

% SHORT CIRCUIT. The ladder exists to PRODUCE the requested lap; if that lap is
% already on disk, none of its rungs is needed - not even a missing one. This is
% what makes a fresh clone resolve a shipped circuit in seconds: the distribution
% ships the final lap only, so walking the chain blindly would solve three stages
% nobody asked for before discovering the answer was already there.
finalFound  = findSidecar(repoRoot, reportRoot, circuit, config, drivetrain);
resolveOnly = ~isempty(finalFound) && ~opt.Force;
if resolveOnly
    fprintf('solveLap: the %s lap already exists - nothing to solve (''Force'',true overrides).\n', config);
end

%% ---------------------------------------------------------------------------
%  Walk the ladder
%% ---------------------------------------------------------------------------
casadiChecked = false;

% Base-workspace protection. Taken LAZILY, immediately before the first stage
% that really solves, and put back by an onCleanup on every exit path - see
% THE BASE WORKSPACE in the header. A run with nothing to solve never touches
% the base workspace at all, which is what keeps solveLap('BCN') sub-second.
baseSnap    = [];
baseRestore = [];  %#ok<NASGU> the onCleanup object must outlive this line

% Entry-speed seed carried down the ladder. It starts at 'Vi' (the 75 m/s default
% unless the caller said otherwise) and, when 'Vi' was NOT given, is replaced after
% every stage by that stage's own exit speed stepped back one window width - the
% same fixed-point step the closure loop takes inside a stage. viSrc is only for
% the log line, so a reader can see where a seed came from.
viSeed = opt.Vi;
viSrc  = 'default';

for i = 1:numel(chain)
    cfg     = configDef(chain{i});
    isFinal = (i == numel(chain));
    st      = newStage();
    st.name = cfg.name;

    dataFile = fullfile(reportRoot, circuit, 'raw', ...
                        sprintf('run_%s_%s_%s_data.mat', circuit, cfg.name, drivetrain));
    found    = findSidecar(repoRoot, reportRoot, circuit, cfg.name, drivetrain);

    if resolveOnly || (~isempty(found) && ~(isFinal && opt.Force))
        % ---- already on disk, or not needed because the target already is --
        %      'Force' applies to the REQUESTED config only: a satisfied
        %      prerequisite is never re-solved just because the final stage is.
        st.action = 'skip';
        st.file   = found;
        if isempty(found)
            fprintf('solveLap: [%d/%d] %-6s skip - not needed (the %s lap already exists)\n', ...
                i, numel(chain), cfg.name, config);
        else
            fprintf('solveLap: [%d/%d] %-6s skip - %s\n', i, numel(chain), cfg.name, found);
        end
        % Read the headline numbers back for the final stage (so a fully skipped
        % run still reports what it resolved to) and for any skipped prerequisite
        % of a stage that will actually solve (so its exit speed can seed it).
        % When nothing is going to solve, prerequisites are left unread: that is
        % what keeps `solveLap('BCN')` on a complete checkout sub-second.
        if ~isempty(found) && (isFinal || ~resolveOnly)
            st = fillFromFile(st, found);
        end

    elseif opt.DryRun
        % ---- dry run: report the intent and move on -----------------------
        st.action = 'solve';
        st.file   = dataFile;
        st.vi     = viSeed;      % no solve, so this is the plan, not a result
        fprintf('solveLap: [%d/%d] %-6s WOULD SOLVE at vi = %.3f m/s (%s) -> %s\n', ...
            i, numel(chain), cfg.name, viSeed, viSrc, dataFile);

    else
        % ---- the base workspace is about to be destroyed ------------------
        %      MLTP.m opens with `clc; clear; clear global;`, so the first real
        %      solve wipes whatever the caller had in base - including the
        %      Simulink project's act/inrt/sus/hudGear and its own vp/pt, whose
        %      absence makes the next runDemoLap die on a mask expression.
        %      Snapshot now, restore on the way out.
        if isempty(baseSnap)
            baseSnap    = snapshotBase();
            baseRestore = onCleanup(@() restoreBase(baseSnap));
        end

        % ---- CasADi is only needed once there is real work ----------------
        if ~casadiChecked
            assert(exist('casadi.SX','class') == 8, 'solveLap:noCasADi', ...
                ['CasADi is not on the MATLAB path, and nothing in this repo adds it. Run\n' ...
                 '    addpath(''<your>\\casadi-3.x-windows64-matlabXXXX'')\n' ...
                 'and call solveLap again. (Stages whose lap .mat already exists need no ' ...
                 'CasADi; this one does.)']);
            casadiChecked = true;
        end

        % ---- warm-start cache for this stage ------------------------------
        % 'direct' deliberately skips this: MLTP falls back to Scripts\
        % MLTP_initial.m, whose simplified 3-row control cache is accepted by
        % latestInit for every config, and MLTP seeds a flat 0 deg wing from it.
        if strcmp(opt.Ladder,'auto') && ~strcmp(cfg.seedMode,'none')
            msg = ensureSeed(repoRoot, reportRoot, circuit, drivetrain, cfg, RW_SLEW);
            if ~isempty(msg)
                error('solveLap:noSeed', ...
                    ['cannot start stage %s for %s/%s: %s\nSolve %s for this track and ' ...
                     'drivetrain first, or pass ''Ladder'',''direct'' to cold-start from the ' ...
                     'simplified model.'], cfg.name, circuit, drivetrain, msg, cfg.seedFrom);
            end
        end

        % ---- solve --------------------------------------------------------
        %      Tolerance and iteration cap come from `opt` (the user's
        %      overrides), not from the VI_* constants - those are only the
        %      defaults `opt` was seeded with. VI_DBACK is not exposed: it is the
        %      width of the entry-speed window built into MLTP.m, not a knob.
        fprintf('solveLap: [%d/%d] %-6s entry-speed seed %.3f m/s (%s)\n', ...
            i, numel(chain), cfg.name, viSeed, viSrc);
        st = solveStage(cfg, circuit, drivetrain, ATDstr, ovrFile, dataFile, viSeed, ...
                        opt.ViTol, opt.ViMaxIter, VI_DBACK, i, numel(chain));
    end

    info.stages(end+1) = st;
    if ~viGiven
        [viSeed, viSrc] = carryVi(st, viSeed, viSrc, VI_DBACK);
    end
end

%% ---------------------------------------------------------------------------
%  Summary
%% ---------------------------------------------------------------------------
last = info.stages(end);
info.matPath = last.file;
if strcmp(last.action,'skip'), info.source = 'existing'; else, info.source = 'solved'; end
info.status = last.status;  info.iters = last.iters;  info.lap = last.lap;
info.vi     = last.vi;      info.vend  = last.vend;   info.miss = last.miss;
info.closed = last.closed;  info.solvedOn = last.solvedOn;

printSummary(info);

%% ---------------------------------------------------------------------------
%  Optional hand-off to the Simulink model
%% ---------------------------------------------------------------------------
if opt.Setup && ~opt.DryRun
    if exist('setupTrack','file') == 2
        % 'Persist',false is setupTrack's own default and is passed EXPLICITLY:
        % true would re-bake and save the tracked DriverPath.slx / ARFWr_Sim.slx,
        % which solving a lap must never do as a side effect. Stated here so a
        % future change of that default cannot silently make solveLap write models.
        fprintf('solveLap: setupTrack(''%s'', ''Persist'', false)\n', info.matPath);
        setupTrack(info.matPath, 'Persist', false);
    else
        warning('solveLap:noSetupTrack', ...
            ['''Setup'' was requested but setupTrack is not on the MATLAB path. Open the ' ...
             'Simulink project (simulink\\ARFWr_RT.prj) - which puts simulink\\tools on the ' ...
             'path - and run  setupTrack(''%s'')  yourself.'], info.matPath);
    end
end
end


%% ===========================================================================
%  One stage: the vi closure loop around a single MLTP solve
%% ===========================================================================
function st = solveStage(cfg, circuit, dv, ATDstr, ovrFile, dataFile, viStart, tol, maxit, dback, i, n)
st        = newStage();
st.name   = cfg.name;
st.action = 'solve';
st.file   = dataFile;

vi     = viStart;
closed = false;

for it = 1:maxit
    st.viIters = it;
    fprintf('solveLap: [%d/%d] %-6s solve, vi iteration %d/%d at vi = %.3f m/s\n', ...
        i, n, cfg.name, it, maxit, vi);

    o = overrideFor(cfg, circuit, ATDstr, vi);
    save(ovrFile, '-struct', 'o');

    d = runMLTP(sprintf('%s/%s/%s', circuit, cfg.name, dv));
    if ~(isstruct(d) && isfield(d,'solver_status') && isfield(d,'t_opt') && isfield(d,'x_opt'))
        error('MLTP:notConverged', ...
            ['stage %s (%s/%s) produced no solution: the global `data` left by MLTP has no ' ...
             'solver_status/t_opt/x_opt. Run MLTP by hand with this runOverride.mat to see ' ...
             'where it stops.'], cfg.name, circuit, dv);
    end

    st.status = d.solver_status;
    st.iters  = getfielddef(d, 'iter_count', NaN);
    st.lap    = d.t_opt(end);
    vx        = d.x_opt(1,:);
    st.vi     = vx(1);
    st.vend   = vx(end);
    st.miss   = abs(vx(1) - vx(end));

    if ~strcmp(st.status, 'Solve_Succeeded')
        if isempty(cfg.seedFrom)
            extra = '';
        else
            extra = sprintf(' Or solve %s for this circuit first and let the ladder carry it up.', ...
                            cfg.seedFrom);
        end
        error('MLTP:notConverged', ...
            ['stage %s (%s/%s, vi iteration %d): IPOPT returned %s after %d iterations. ' ...
             'The lap time it printed is NOT a converged optimum and must not be quoted. ' ...
             'Remedies, in order: let the vi loop run more iterations (''ViMaxIter''); ' ...
             'coarsen the mesh (runOverride field OPT_ds, 10 -> 12-15 m); relax the solver ' ...
             '(runOverride field ipoptTol, 1e-6 -> 1e-5).%s'], ...
            cfg.name, circuit, dv, it, st.status, st.iters, extra);
    end

    % One fixed-point step. Closed when entry and exit speeds already agree;
    % otherwise reseed one window-width below the exit speed (see CLOSURE).
    closed = (st.miss <= tol);
    vi     = st.vend - dback;
    if closed, break, end
end
st.closed = closed;

if ~closed
    warning('solveLap:notClosed', ...
        ['stage %s (%s/%s) did not close after %d vi iterations: |vx(1) - vx(end)| = ' ...
         '%.3f m/s against a %.2f m/s tolerance. The lap is a converged optimum for its ' ...
         'own boundary conditions, but entry and exit speeds do not match, so it is not a ' ...
         'representative flying lap. Raise ''ViMaxIter'' or reseed with ''Vi'', %.2f.'], ...
        cfg.name, circuit, dv, maxit, st.miss, tol, st.vend - dback);
end

% Annotate and archive. Variable name `data` and the raw\ subfolder are both
% load-bearing: every reader in this repo and in simulink\tools does
% S = load(f); S.data.<...>, and the report tooling globs run_<track>_*.mat one
% directory UP, which a *_data.mat sitting beside it would wrongly match.
% `d` is the solution from the LAST vi iteration, reused rather than re-read from
% the global - the global is the one thing MLTP's own `clear global` can pull out
% from under us.
d.vi         = st.vi;
d.vend       = st.vend;
d.miss       = st.miss;
d.closed     = st.closed;
d.circuit    = circuit;
d.config     = cfg.name;
d.drivetrain = dv;
d.solvedOn   = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
st.solvedOn  = d.solvedOn;      % the same stamp that goes into the file

outDir = fileparts(dataFile);
if ~exist(outDir,'dir'), mkdir(outDir); end
data = d;
save(dataFile, 'data');
fprintf('solveLap: [%d/%d] %-6s saved -> %s  [%s, lap %.3f s, miss %.3f, closed %d]\n', ...
    i, n, cfg.name, dataFile, st.status, st.lap, st.miss, st.closed);

closeCircuitMap();
end


function [vi, src] = carryVi(st, vi, src, dback)
%CARRYVI Hand the entry-speed seed down to the next rung of the ladder.
%  Every rung is the same car on the same track, so their closed entry speeds
%  agree to a few tenths: a rung that has been solved (or was read back off disk)
%  already knows where the fixed point is, and the next one should start there
%  rather than rediscover it from 75 m/s at the cost of a whole extra solve.
%  The step is the closure loop's own - vend - dback, never vend - because MLTP
%  always takes the fastest admissible entry in the vi +/- dback window, so
%  reseeding with vend leaves the miss stuck at dback for ever.
%  A stage that did not close is still used: its exit speed belongs to a converged
%  optimum and is far nearer the fixed point than the cold seed. A stage with no
%  usable exit speed - a dry run, or a lap .mat too old or too damaged to read one
%  out of - leaves the seed exactly as it was.
v = st.vend;
if ~(isnumeric(v) && isscalar(v) && isfinite(v)) || v <= dback
    return
end
vi  = v - dback;
src = sprintf('from %s, vend %.3f', st.name, v);
end


%% ===========================================================================
%  Driving MLTP
%% ===========================================================================
function d = runMLTP(label)
%RUNMLTP Invoke MLTP in the BASE workspace and recover its global `data`.
%
%  Why evalin: MLTP.m is a script that opens with `clear; clear global;` and warm-
%  starts through importfile(), which assignin()s into the base workspace. Invoked
%  from inside a function, that `data` would land in base and be unreachable in the
%  caller, and MLTP would die on data.init.x_opt.
%
%  Why the tolerant catch: MLTP's trailing post-processing (SDI logging, plotSDI,
%  the circuit-map figure, apexSpeeds) runs inside this call, and it is the part
%  most likely to fail on a headless machine. As long as the solve itself completed
%  - solver_status and t_opt are already on `data` - the exception is downgraded to
%  a warning and the lap is kept. A genuine crash (nothing usable on `data`) is
%  rethrown untouched.
try
    evalin('base', 'MLTP;');
catch ME
    d = runMLTPResult();
    if ~(isstruct(d) && isfield(d,'solver_status') && isfield(d,'t_opt'))
        rethrow(ME);
    end
    warning('solveLap:postSolve', ...
        ['%s: MLTP threw AFTER the solve completed (%s: %s) - keeping the solved lap. ' ...
         'This is normally a display/SDI call failing in a headless session.'], ...
        label, ME.identifier, ME.message);
    return
end
d = runMLTPResult();
end

function snap = snapshotBase()
%SNAPSHOTBASE Copy the base workspace by value, so it can be put back after MLTP.
%  Only PLAIN variables are captured. A name `whos` reports as GLOBAL is
%  recorded but not copied: MLTP.m's own `clear global` destroys the global
%  itself, and assigning the old value back would create an ordinary base
%  variable wearing a global's name - bound to nothing, and indistinguishable
%  from the real thing until something wrote through it. Anything evalin cannot
%  hand over is listed in .skipped and reported once on restore rather than
%  failing the solve; it would have been wiped by MLTP either way.
snap = struct('names', {{}}, 'vals', {{}}, 'globals', {{}}, 'skipped', {{}});
try
    w = evalin('base', 'whos');
catch
    return
end
for i = 1:numel(w)
    nm = w(i).name;
    if w(i).global
        snap.globals{end+1} = nm;
        continue
    end
    try
        v = evalin('base', nm);
    catch
        snap.skipped{end+1} = nm;
        continue
    end
    snap.names{end+1} = nm;
    snap.vals{end+1}  = v;
end
end

function restoreBase(snap)
%RESTOREBASE Put the snapshot back and drop everything the solve left behind.
%  Order matters: clear first, then assign. MLTP leaves several hundred
%  variables in base, and a caller who looks at `who` after a solve should see
%  their own workspace, not the solver's scratch.
if isempty(snap) || ~isstruct(snap), return, end
try
    present = evalin('base', 'who');
    drop    = setdiff(present(:)', snap.names);
    % Cleared in blocks: `clear` takes names as arguments, and a single command
    % naming ~700 of them is a needlessly long string to build and parse.
    for k = 1:100:numel(drop)
        j = k:min(k + 99, numel(drop));
        evalin('base', ['clear ' strjoin(drop(j), ' ')]);
    end
    for i = 1:numel(snap.names)
        assignin('base', snap.names{i}, snap.vals{i});
    end
    if ~isempty(snap.skipped)
        warning('solveLap:baseNotRestored', ...
            ['these base-workspace variables could not be copied before the solve and are ' ...
             'gone (MLTP.m clears the base workspace): %s.'], strjoin(snap.skipped, ', '));
    end
catch ME
    warning('solveLap:baseRestoreFailed', ...
        ['could not restore the base workspace after the solve (%s: %s). The lap itself is ' ...
         'unaffected - it is in the returned info and in the saved .mat - but the base ' ...
         'workspace still holds MLTP''s leftovers. Re-open the Simulink project (or re-run ' ...
         'its startup) before using the sim tools.'], ME.identifier, ME.message);
end
end

function d = runMLTPResult()
%RUNMLTPRESULT Read the global `data` MLTP leaves behind.
%  Deliberately a separate function so the `global` declaration is made in a FRESH
%  workspace every time: MLTP.m's own `clear global` severs any binding held across
%  the call, and a stale one would silently read the previous stage's lap.
global data %#ok<GVMIS>
d = data;
end

function closeCircuitMap()
%CLOSECIRCUITMAP Drop the Circuit Map figure MLTP opens, so a multi-stage run does
%  not accumulate one window per stage. MLTP recreates it on the next solve.
global figures %#ok<GVMIS>
try %#ok<TRYNC>
    if isstruct(figures) && isfield(figures,'track') && isfield(figures.track,'fig') ...
            && ~isempty(figures.track.fig) && all(isgraphics(figures.track.fig))
        close(figures.track.fig);
    end
end
end


%% ===========================================================================
%  The config matrix
%% ===========================================================================
function cfg = configDef(name)
%CONFIGDEF Everything one config needs: the runOverride payload, its warm-start
%  parent, and its init-cache token.
%
%  .initToken MUST agree with Functions\latestInit.m and Scripts\MLTP_initial.m.
%  The tokens are not decoration: an AFWd cache must never satisfy an ARW glob and
%  vice versa, because the aero models behind them are different maps with
%  different control bounds, and MLTP would warm-start from wing angles its own
%  model cannot reach.
%
%  .seedMode is how the parent's solved lap is reshaped into this config's cache:
%     none      no pre-seed; MLTP falls back to the simplified MLTP_initial model
%     round     round the parent's wing trace onto this config's station set
%     zeroWing  flatten the wing row to 0 deg (its max-downforce end)
%     insertFW  insert a zero front-wing row at nu-2 (this config has one more control)
%     copy      take the parent's controls unchanged (identical control layout)
STATIC = {'Low','Mid','High','RWp15'};
if any(strcmp(name, STATIC))
    cfg = mkcfg(name, 'Static', 'Off', 0.9, name, '', 'zenvoMF52rw4n', 'none');
    return
end
switch name
    case 'ARW'    % free continuous rear wing, [-10,+15] deg. The root of the ladder.
        cfg = mkcfg('ARW',   'ActiveRW', 'Off',       0.9, 'ARW',   '',      'zenvoMF52rw4n',  'none');
    case 'ARWd'   % ARW + a lagged braking airbrake floor; discreteness is a post-
                  % processing audit, so the damping is raised to sit nearer the stations.
        cfg = mkcfg('ARWd',  'ActiveRW', 'Discrete',  3.0, 'ARWd',  'ARW',   'zenvoMF52rw4n',  'round');
    case 'AFWd'   % front-wing unload axis [-25,0] deg - a different surface, its own map.
        cfg = mkcfg('AFWd',  'ActiveRW', 'FrontWing', 3.0, 'AFWd',  'ARW',   'zenvoMF52fw3n',  'zeroWing');
    case 'ARFWd'  % both wings free: the first config with an extra control (nu +1).
        cfg = mkcfg('ARFWd', 'ActiveRW', 'Combined',  3.0, 'ARFWd', 'ARWd',  'zenvoMF52arfw',  'insertFW');
    case 'ARFWr'  % ARFWd's layout, both wings pinned to the reactive law.
        cfg = mkcfg('ARFWr', 'ActiveRW', 'Reactive',  3.0, 'ARFWr', 'ARFWd', 'zenvoMF52arfwr', 'copy');
    otherwise
        error('solveLap:unknownConfig', ...
            ['unknown config ''%s''. Expected one of: %s. (''ARWv'', the velocity-scheduled ' ...
             'wing, is retired - its schedule is infeasible at the 25 deg/s wing slew rate - ' ...
             'and is not offered here.)'], ...
            name, strjoin([STATIC, {'ARW','ARWd','AFWd','ARFWd','ARFWr'}], ', '));
end
end

function cfg = mkcfg(name, aeroCfg, mandate, rdu2RW, aeroSetting, seedFrom, token, seedMode)
cfg = struct('name', name, 'AeroConfig', aeroCfg, 'RWMandate', mandate, ...
             'RWDiscrete', 'Off', 'rwSnapRho', 0, 'rdu2RW', rdu2RW, ...
             'aeroSetting', aeroSetting, 'seedFrom', seedFrom, ...
             'initToken', token, 'seedMode', seedMode);
end

function chain = ladderFor(name)
%LADDERFOR The warm-start chain ending at `name`, root first.
chain = {name};
cfg   = configDef(name);
while ~isempty(cfg.seedFrom)
    chain = [{cfg.seedFrom}, chain]; %#ok<AGROW>
    cfg   = configDef(cfg.seedFrom);
end
end

function o = overrideFor(cfg, circuit, ATDstr, vi)
%OVERRIDEFOR The complete runOverride.mat payload for one (config, circuit, dv, vi).
%  Field names are Scripts\userOpts.m's and Parameters\vehParams.m's, which read
%  them through getfielddef. Every field this run depends on is written
%  EXPLICITLY rather than left to a default, so editing a default cannot silently
%  change what a run actually solved.
%  aeroSetting is inert for the ActiveRW family - vehParams derives the run
%  identity from vp.rwMandate there and overrides any aeroSetting given - but it
%  is written uniformly so the payload is the same shape for every config.
o = struct('AeroConfig',  cfg.AeroConfig, ...
           'ATD',         ATDstr, ...
           'circuit',     circuit, ...
           'RWMandate',   cfg.RWMandate, ...
           'RWDiscrete',  cfg.RWDiscrete, ...
           'rwSnapRho',   cfg.rwSnapRho, ...
           'rdu2RW',      cfg.rdu2RW, ...
           'aeroSetting', cfg.aeroSetting, ...
           'vi',          vi);
end


%% ===========================================================================
%  Warm-start seeding
%% ===========================================================================
function msg = ensureSeed(repoRoot, reportRoot, circuit, dv, cfg, slewRate)
%ENSURESEED Make sure an init cache exists for this config/circuit/drivetrain.
%  Returns '' on success, or a reason string the caller turns into an error.
%
%  The source lap must be from the SAME circuit and the SAME drivetrain. Both
%  halves matter: another circuit's lap is the wrong trajectory, and an AWD lap is
%  the wrong SHAPE under ATD (4 or 5 controls against 8 or 9), which would either
%  fail MLTP's row-count assert or be silently padded.
msg = '';
if strcmp(dv,'ATD'), dvTag = '_ATD'; else, dvTag = ''; end
initDir = fullfile(repoRoot, 'Data', circuit, 'initialisation');
if ~exist(initDir,'dir'), mkdir(initDir); end

% Already seeded, or already solved once here? Match both filename shapes
% Functions\latestInit.m accepts, on THIS config's own token.
have = [dir(fullfile(initDir, sprintf('init_%s_%s_%s_*.mat',   circuit, cfg.name, cfg.initToken))); ...
        dir(fullfile(initDir, sprintf('init_%s_%s_*_%s_*.mat', circuit, cfg.name, cfg.initToken)))];
if ~isempty(have), return, end

[src, tried] = warmSource(repoRoot, reportRoot, circuit, dv, cfg.seedFrom);
if isempty(src)
    msg = sprintf('no %s init cache and no %s lap to seed it from (looked in: %s)', ...
                  cfg.name, cfg.seedFrom, strjoin(tried, ', '));
    return
end
S = load(src, 'data');
if ~isfield(S,'data') || ~isfield(S.data,'x_opt') || ~isfield(S.data,'u_opt')
    msg = sprintf('%s warm source %s holds no full solution (no data.x_opt/u_opt)', cfg.seedFrom, src);
    return
end

switch cfg.seedMode
    case 'round'
        % The wing is the second-to-last control in every ActiveRW ladder (delta is
        % always last). ARWd's discreteness is applied in post-processing anyway, so
        % starting already on the station set is strictly closer than the parent's
        % smooth trace. There is no saved knot abscissa (MLTP keeps only the finer
        % data.s_full), so it is reconstructed the way MLTP itself treats an init of
        % a different length - a linspace between the lap's first and last distance.
        if ~isfield(S.data,'s_full')
            msg = sprintf('%s warm source %s holds no data.s_full (no distance axis to round on)', ...
                          cfg.seedFrom, src);
            return
        end
        wingRow = size(S.data.u_opt,1) - 1;
        vxRow   = S.data.x_opt(1,:);
        sRow    = linspace(S.data.s_full(1), S.data.s_full(end), numel(vxRow));
        S.data.u_opt(wingRow,:) = rwDiscretize(sRow, S.data.u_opt(wingRow,:), ...
                                               [-10 0 10 15], slewRate, vxRow);
        note = sprintf('wing row %d rounded onto [-10 0 10 15] deg', wingRow);

    case 'zeroWing'
        % AFWd's axis is [-25, 0] deg and the parent's free wing lives in [-10, +15]:
        % almost disjoint, with no shared station set to round onto. 0 deg is both
        % in-bound and IS this axis's max-downforce end, so a flat 0 deg row is a
        % physically sane, always-admissible guess rather than a transplant.
        [wingRow, msg] = wingRowOf(S, cfg.seedFrom, src);
        if ~isempty(msg), return, end
        S.data.u_opt(wingRow,:) = 0;
        note = sprintf('wing row %d zeroed', wingRow);

    case 'insertFW'
        % ARFWd is its parent plus ONE control - the front-wing flap at row nu-2,
        % ahead of the rear wing - so the closest admissible guess is the parent's
        % lap with a zero FW row INSERTED there. Without the insert the cache keeps
        % the parent's row count and latestInit's width filter rejects it silently,
        % and the run cold-starts.
        [iRW, msg] = wingRowOf(S, cfg.seedFrom, src);
        if ~isempty(msg), return, end
        if iRW ~= size(S.data.u_opt,1) - 1
            msg = sprintf(['%s warm source %s: aeroARW.uRow = %d but u_opt has %d rows ' ...
                           '(the wing must be second-to-last)'], ...
                          cfg.seedFrom, src, iRW, size(S.data.u_opt,1));
            return
        end
        S.data.u_opt = [S.data.u_opt(1:iRW-1,:); zeros(1,size(S.data.u_opt,2)); S.data.u_opt(iRW:end,:)];
        S.data.aeroARW.uRow = iRW + 1;   % keep the stored metadata true to the new matrix
        note = sprintf('zero FW row inserted at %d', iRW);

    case 'copy'
        % Identical control layout to the parent - the reactive law only re-targets
        % the two wing rows, it neither adds nor removes one - so nothing is reshaped.
        [iRW, msg] = wingRowOf(S, cfg.seedFrom, src);
        if ~isempty(msg), return, end
        nRow = size(S.data.u_opt,1);
        if iRW ~= nRow - 1 || ~ismember(nRow, [5 9])
            msg = sprintf(['%s warm source %s: aeroARW.uRow = %d with %d control rows - ' ...
                           'expected the wing second-to-last and 5 (AWD) or 9 (ATD) rows'], ...
                          cfg.seedFrom, src, iRW, nRow);
            return
        end
        note = 'controls copied unchanged';

    otherwise
        msg = sprintf('unknown seedMode ''%s'' for config %s', cfg.seedMode, cfg.name);
        return
end

data = struct(); data.init = S.data;
outF = fullfile(initDir, sprintf('init_%s_%s%s_%s_%s.mat', circuit, cfg.name, dvTag, ...
    cfg.initToken, char(datetime('now','Format','yyyyMMdd_HHmmss'))));
save(outF, 'data');
fprintf('solveLap: seeded %s init cache for %s/%s from %s (%s)\n', ...
    cfg.name, circuit, dv, src, note);
end

function [row, msg] = wingRowOf(S, srcCfg, src)
%WINGROWOF The rear-wing control row, read from the stored metadata rather than
%  re-derived, so it stays correct if a future ladder change moves the wing.
row = NaN; msg = '';
if ~isfield(S.data,'aeroARW') || ~isfield(S.data.aeroARW,'uRow')
    msg = sprintf('%s warm source %s holds no aeroARW.uRow (wing row unknown)', srcCfg, src);
    return
end
row = S.data.aeroARW.uRow;
end


%% ===========================================================================
%  Where laps live
%% ===========================================================================
function [p, tried] = findSidecar(repoRoot, reportRoot, circuit, cfgName, dv)
%FINDSIDECAR The solved lap .mat for one run, or '' if there is none.
%  Two locations, in order:
%    1) <reportRoot>\<circuit>\raw\   laps solved in this checkout
%    2) simulink\data\laps\           laps shipped with the distribution
%  Both are drivetrain-tagged, so there is no way to hand a 4-control AWD lap to
%  an 8-control ATD model by accident.
f1 = fullfile(reportRoot, circuit, 'raw', sprintf('run_%s_%s_%s_data.mat', circuit, cfgName, dv));
f2 = fullfile(repoRoot, 'simulink', 'data', 'laps', sprintf('run_%s_%s_%s_data.mat', circuit, cfgName, dv));
tried = {f1, f2};
p = '';
if isfile(f1), p = f1; return, end
if isfile(f2), p = f2; return, end
end

function [p, tried] = warmSource(repoRoot, reportRoot, circuit, dv, cfgName)
%WARMSOURCE The lap a new stage may be warm-started from.
%  findSidecar first; then, AWD only, a legacy confirm-run archive if this checkout
%  has one. The archive filename carries no drivetrain tag, so it is skipped under
%  ATD outright - an untagged lap is never safe for the wider ATD control vector,
%  and a silently padded warm start is worse than a cold one.
%  Tie-break is newest by NAME, never by mtime: the yyyyMMdd_HHmmss stamp is in the
%  filename and sorts chronologically, whereas mtime does not survive a checkout or
%  a file copy.
[p, tried] = findSidecar(repoRoot, reportRoot, circuit, cfgName, dv);
if ~isempty(p) || strcmp(dv,'ATD'), return, end

g = fullfile(repoRoot, 'solutions', cfgName, sprintf('%s_%s_solve_*.mat', cfgName, circuit));
tried{end+1} = g;
d = dir(g);
if isempty(d), return, end
names  = sort({d.name});
newest = names{end};
k      = find(strcmp({d.name}, newest), 1);   % index BY NAME, not d(1)
p      = fullfile(d(k).folder, d(k).name);
end


%% ===========================================================================
%  Reporting helpers
%% ===========================================================================
function st = newStage()
%NEWSTAGE One stage record, all fields present and in a fixed order - which is
%  what makes info.stages(end+1) = st legal struct-array concatenation.
st = struct('name', '', 'action', '', 'status', '', 'iters', NaN, 'lap', NaN, ...
            'vi', NaN, 'vend', NaN, 'miss', NaN, 'closed', false, ...
            'viIters', 0, 'file', '', 'solvedOn', '');
end

function s = emptyStageArray()
%EMPTYSTAGEARRAY A 0-element stage array with newStage's exact field set/order.
s      = newStage();
s(1)   = [];
end

function st = fillFromFile(st, f)
%FILLFROMFILE Read the headline numbers back out of an existing lap .mat, so a
%  fully-skipped run still reports what it resolved to. Degrades to NaNs rather
%  than failing: a lap that cannot be read is still a lap the caller asked for,
%  and the path is the useful output.
try
    S = load(f, 'data');
    d = S.data;
    st.status = getfielddef(d, 'solver_status', '');
    st.iters  = getfielddef(d, 'iter_count', NaN);
    if isfield(d,'t_opt'), st.lap = d.t_opt(end); end
    if isfield(d,'x_opt')
        vx = d.x_opt(1,:);
        st.vi = vx(1); st.vend = vx(end); st.miss = abs(vx(1)-vx(end));
    end
    st.closed = getfielddef(d, 'closed', st.miss <= 0.5);
    % When this lap was SOLVED, straight out of the file. A lap saved before
    % solveLap existed carries no stamp, and '' is the honest answer: inventing
    % "now" here is how a file from last year gets read as a fresh solve.
    st.solvedOn = getfielddef(d, 'solvedOn', '');
    if ~(ischar(st.solvedOn) || isstring(st.solvedOn)), st.solvedOn = ''; end
    st.solvedOn = char(st.solvedOn);
catch ME
    warning('solveLap:unreadableLap', ...
        'could not read %s (%s: %s) - reporting the path only.', f, ME.identifier, ME.message);
end
end

function printSummary(info)
fprintf('\n');
fprintf('solveLap: %s / %s / %s\n', info.circuit, info.config, info.drivetrain);
fprintf('  %-6s %-6s %-22s %7s %10s %8s %7s\n', ...
        'stage','action','status','iters','lap [s]','miss','closed');
for i = 1:numel(info.stages)
    s = info.stages(i);
    fprintf('  %-6s %-6s %-22s %7s %10s %8s %7s\n', ...
        s.name, s.action, blankIfEmpty(s.status), fmtnum(s.iters,'%d'), ...
        fmtnum(s.lap,'%.3f'), fmtnum(s.miss,'%.3f'), yesno(s.closed));
end
fprintf('  lap file: %s\n', info.matPath);
if ~info.dryRun && ~isempty(info.matPath) && ~info.closed
    fprintf(['  NOTE: entry and exit speeds do not match within tolerance - this lap is not ' ...
             'a closed flying lap.\n']);
end
if info.dryRun
    fprintf('  (dry run - nothing was written and no solve was started)\n');
end
fprintf('\n');
end

function s = blankIfEmpty(s), if isempty(s), s = '-'; end, end
function s = yesno(v), if v, s = 'yes'; else, s = 'no'; end, end
function s = fmtnum(v, fmt)
if isempty(v) || ~isnumeric(v) || ~isfinite(v), s = '-'; else, s = sprintf(fmt, v); end
end


%% ===========================================================================
%  Small utilities
%% ===========================================================================
function safeDelete(f)
if exist(f,'file'), delete(f); end
end

function tf = isAbsPath(p)
%ISABSPATH true for a Windows drive path (C:\ or C:/), a UNC path, or a POSIX root.
%  Built from character tests rather than a regex on purpose: escaping backslashes
%  inside a MATLAB regex literal is exactly what makes this silently never match,
%  which would send an already-absolute path through fullfile() and produce
%  nonsense.
tf = false;
if isempty(p) || ~(ischar(p) || isstring(p)), return, end
p = char(p);
if numel(p) >= 1 && (p(1) == '/' || p(1) == '\')
    tf = true;                                   % POSIX root, or UNC \\host\share
elseif numel(p) >= 3 && isletter(p(1)) && p(2) == ':' && (p(3) == '\' || p(3) == '/')
    tf = true;                                   % drive-qualified
end
end
