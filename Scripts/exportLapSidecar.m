function info = exportLapSidecar(src, dst)
%EXPORTLAPSIDECAR Write a slim, shippable copy of a solved lap .mat.
%
%   info = exportLapSidecar(src, dst)
%
%   A lap .mat written by solveLap (or by the private comparison batch) is ~11-16
%   MB, and about 90 % of that is scaffolding nothing downstream ever reads: the
%   warm-start guess the solve began from, the raw NLP decision vector, and a
%   Simulink Data Inspector payload. This strips exactly those three and re-saves,
%   which is what makes a solved lap small enough to ship inside the repository so
%   the Simulink / Unreal side runs on a fresh clone with no solve.
%
%   src   the lap .mat to read; must hold a single struct variable `data`.
%   dst   the .mat to write, or a FOLDER (src's own filename is then kept - which
%         is the normal case, because every reader in this repo keys on the
%         run_<circuit>_<config>_<drivetrain>_data.mat filename). Missing folders
%         are created.
%
%   info  .src .dst .bytesIn .bytesOut .ratio .stripped .fields
%
%   ---------------------------------------------------------------------------
%   WHAT IS REMOVED, AND WHY IT IS SAFE
%   ---------------------------------------------------------------------------
%     init   the warm-start solution this lap was seeded FROM - a whole previous
%            lap nested inside this one, and on its own the bulk of the file. It
%            is an input to the solve, never an output: MLTP.m reads data.init to
%            build its initial guess and nothing reads it afterwards. A slim lap
%            can therefore still be replayed, plotted and used as a Simulink
%            reference, but it cannot itself be handed back to MLTP as a warm
%            start - that is what the init caches under Data\<circuit>\ are for,
%            and solveLap builds those from x_opt/u_opt, not from init.
%     sdi    a Simulink Data Inspector run: signal objects plus, in some versions,
%            a live run HANDLE. Handles do not survive being saved and reloaded in
%            another session, so this is the one field that is not merely large
%            but actively misleading once the file leaves the machine that wrote
%            it. Only the private dashboard app ever read it.
%     w_opt  the raw stacked NLP decision vector, in NORMALISED units. Everything
%            in it is already present unscaled and unpacked as x_opt / u_opt /
%            y_opt / xc_opt, which is what every reader actually uses.
%
%   Everything else is kept verbatim - the solved trajectory (x_opt, u_opt, y_opt,
%   the resampled x_full / u_full / y_full and their s_full / t_opt axes), the
%   track geometry the ribbon and reference path are built from (track, track0),
%   the aero and wing-station metadata (aeroARW, aeroAFW, rwDisc, fwDisc,
%   lawTrack), the vehicle and constraint records, and the solver's own verdict
%   (solver_status, iter_count, duration) plus solveLap's closure annotations
%   (vi, vend, miss, closed, circuit, config, drivetrain, solvedOn) when the lap
%   was solved by solveLap. Laps solved before solveLap existed carry no closure
%   annotations; readers derive them from x_opt instead, so they are not
%   synthesised here - this function strips, it does not invent.
%
%   The result is saved -v7 (compressed, and readable by every MATLAB since
%   R14) rather than -v7.3, which for structs of this size is larger and slower.
%
%   EXAMPLE
%       exportLapSidecar( ...
%           'solutions\report\BCN\raw\run_BCN_ARFWr_ATD_data.mat', ...
%           'simulink\data\laps');
%
%   See also SOLVELAP, SETUPTRACK, ACTIVETRACK.

% The three fields removed. Kept as a named constant because it is also the
% verification invariant below: the output must differ from the input by exactly
% this set and nothing else.
STRIP = {'init', 'sdi', 'w_opt'};

% What a slim lap must still be able to answer for. These are the fields the
% Simulink tools (buildDriverRef, buildTrackRibbon, buildRefPath, setupTrack),
% solveLap's ladder seeding and the report readers actually touch. Checked on the
% INPUT, so a source file that was already incomplete is rejected here rather
% than silently shipped.
REQUIRED = {'s_full','t_opt','x_opt','u_opt','y_opt','x_full','u_full', ...
            'track','aeroARW','solver_status'};

%% ---- read ---------------------------------------------------------------
src = char(src);
if ~isfile(src)
    error('exportLapSidecar:noSource', 'source lap .mat not found: %s', src);
end
S = load(src, 'data');
if ~isfield(S, 'data') || ~isstruct(S.data) || ~isscalar(S.data)
    error('exportLapSidecar:noData', ...
        ['%s does not hold a single struct variable `data`. Every lap reader in this ' ...
         'repo does S = load(f); S.data.<...>, so the variable name is load-bearing.'], src);
end
data = S.data;

missing = REQUIRED(~isfield(data, REQUIRED));
if ~isempty(missing)
    error('exportLapSidecar:incompleteLap', ...
        ['%s is missing fields the sim and solver tools need: %s. This is not a full ' ...
         'solved lap (a partial or already-slimmed file?), so slimming it further would ' ...
         'ship something that cannot be replayed.'], src, strjoin(missing, ', '));
end

%% ---- strip --------------------------------------------------------------
fieldsIn = fieldnames(data)';
stripped = STRIP(isfield(data, STRIP));
data     = rmfield(data, stripped);

% The whole contract, asserted rather than assumed: the output differs from the
% input by the stripped set and by nothing else, in the original field order.
expect = setdiff(fieldsIn, stripped, 'stable');
assert(isequal(fieldnames(data)', expect), 'exportLapSidecar:fieldDrift', ...
    'internal error: field set changed by more than the stripped fields.');

%% ---- write --------------------------------------------------------------
dst = char(dst);
if isfolder(dst) || isempty(regexp(dst, '\.mat$', 'once'))
    [~, base, ext] = fileparts(src);
    dst = fullfile(dst, [base ext]);
end
outDir = fileparts(dst);
if ~isempty(outDir) && ~isfolder(outDir), mkdir(outDir); end

save(dst, 'data', '-v7');

%% ---- report -------------------------------------------------------------
a = dir(src);  b = dir(dst);
info = struct('src', src, 'dst', dst, 'bytesIn', a.bytes, 'bytesOut', b.bytes, ...
              'ratio', b.bytes / a.bytes, 'stripped', {stripped}, 'fields', {expect});
fprintf('exportLapSidecar: %s\n', src);
fprintf('  -> %s\n', dst);
fprintf('  %.2f MB -> %.2f MB (%.1f%%), stripped %s, kept %d fields\n', ...
    a.bytes/1e6, b.bytes/1e6, 100*info.ratio, strjoin(stripped, ' '), numel(expect));
end
