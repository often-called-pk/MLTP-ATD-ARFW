function [alphaDisc, info] = rwDiscretize(s, alpha, stations, slewRate, vx)
%RWDISCRETIZE Nearest-station rounding of a continuous rear-wing trace, with
%  dwells too short for the actuator to hold merged into a longer neighbour.
%
%   [alphaDisc, info] = rwDiscretize(s, alpha, stations, slewRate, vx)
%
%   Pure-MATLAB post-processing helper (no CasADi): takes a continuous
%   optimal wing-angle trace alpha(s) - e.g. from a free-ActiveRW solve -
%   and rounds it onto a discrete set of `stations` (the four Static run
%   angles [-10 0 10 15] in the live model), the way an actuator with only a
%   handful of physical detents would actually fly it. A knot-by-knot
%   nearest-station round can leave single-knot "spike" dwells the actuator
%   physically cannot reach and leave again before the next commanded
%   change; the dwell-merge pass folds each such dwell into its longer
%   neighbour (repeating until every surviving dwell clears the threshold)
%   and reports how many merges it took plus a set of audit metrics on the
%   final (post-merge) trace.
%
%   INPUTS
%     s         independent-variable samples [m] (e.g. distance), strictly
%               increasing, any shape. numel(s) must equal numel(alpha) and
%               numel(vx).
%     alpha     continuous wing-angle trace [deg], same length as s.
%     stations  discrete angle set to round onto [deg]. Sorted ascending
%               internally; need not be sorted or unique on input.
%     slewRate  actuator slew rate [deg/s], finite positive scalar.
%     vx        longitudinal speed [m/s] at each sample, same length as s -
%               converts the actuator's deg/s rate into a metres-needed
%               distance, evaluated LOCALLY (per dwell / per switch), not as
%               a single lap-average.
%
%   OUTPUTS
%     alphaDisc  1xN row vector, each entry one of `stations` - the nearest-
%                station rounding of `alpha`, after dwell merging.
%     info       struct:
%       stationIdx     1xN index of alphaDisc into (sorted) stations.
%       nTransitions   number of station changes in the FINAL (post-merge)
%                      alphaDisc.
%       dwell          table, one row per surviving dwell: station, sStart,
%                      sEnd, lengthM (= sEnd - sStart). The last dwell's
%                      sEnd is the position of the trace's final sample
%                      (inclusive); every other dwell's sEnd is the
%                      position of the first sample of the NEXT dwell
%                      (exclusive boundary) - this matches the length
%                      convention the merge pass itself audits against, not
%                      a general-purpose interval spec.
%       merged         number of merge operations performed (each merge
%                      absorbs exactly one dwell into a neighbour).
%       roundedSlewOK  true iff every consecutive pair of transitions in the
%                      FINAL trace is far enough apart, in metres, for the
%                      actuator to complete the intervening angle change:
%                      gap(i) >= |alphaDisc(sw(i)+1) - alphaDisc(sw(i))| /
%                      slewRate * vx(sw(i)), where sw are the transition
%                      knots and gap(i) is the distance from switch i to
%                      switch i+1 (or to the trace end, for the last
%                      switch). Vacuously true with 0 or 1 transitions.
%       devRMS         RMS of (alpha - alphaDisc) over all knots [deg].
%       devMax         max |alpha - alphaDisc| over all knots [deg].
%
%   MERGE RULE. A dwell is flagged too short if its length in metres is less
%   than 10/slewRate * mean(vx over the dwell) - a flat "one 10-degree
%   switch each way" assumption. 10 deg is the largest ADJACENT spacing in
%   the shipped 4-station set [-10 0 10 15] (the 10->15 gap is only 5 deg),
%   so this threshold is deliberately conservative rather than exact per
%   transition - it is cheap, symmetric, and does not need to look at which
%   two stations border a dwell. The shortest flagged dwell is absorbed into
%   whichever neighbour is longer (ties favour the left/earlier neighbour),
%   and the pass repeats until no dwell is flagged or only one dwell
%   remains. A trace with only 2 dwells (1 transition) never enters this
%   loop at all - by construction there is nothing to merge it INTO without
%   first picking a direction, and the loop's own exit test (dwell count
%   <= 2) treats that case as already "merged" as far as it can go. One
%   consequence, inherited as-is rather than special-cased here: a single
%   too-short dwell sitting at the very start or end of the WHOLE trace,
%   with only one neighbour in total, is never merge-checked either (its
%   dwell count is also 2). This is a known limitation of the merge rule,
%   not something introduced by this implementation.
%
%   TIE-BREAK. When alpha sits exactly midway between two stations, MATLAB's
%   min() returns the FIRST (lower) station - documented behaviour, not
%   claimed to be "more correct" than rounding up.
%
%   See also RWVELOCITYTARGET, RWAERODELTA.

% ---- validation ----------------------------------------------------------
assert(isnumeric(s) && isreal(s) && ~isempty(s), 'rwDiscretize:s', ...
    's must be a non-empty real numeric array');
assert(isnumeric(alpha) && isreal(alpha) && ~isempty(alpha), 'rwDiscretize:alpha', ...
    'alpha must be a non-empty real numeric array');
assert(isnumeric(vx) && isreal(vx) && ~isempty(vx), 'rwDiscretize:vx', ...
    'vx must be a non-empty real numeric array');
assert(isnumeric(stations) && isreal(stations) && ~isempty(stations), ...
    'rwDiscretize:stations', 'stations must be a non-empty real numeric array');
assert(isnumeric(slewRate) && isreal(slewRate) && isscalar(slewRate) && ...
    isfinite(slewRate) && slewRate > 0, 'rwDiscretize:slewRate', ...
    'slewRate must be a finite positive real scalar [deg/s]');
assert(all(isfinite(s(:))) && all(isfinite(alpha(:))) && all(isfinite(vx(:))), ...
    'rwDiscretize:finite', 's, alpha and vx must all be finite');

s = s(:).'; alpha = alpha(:).'; vx = vx(:).'; stations = sort(stations(:)).';
n = numel(alpha);
assert(numel(s) == n, 'rwDiscretize:lenS', ...
    'numel(s) (%d) must equal numel(alpha) (%d)', numel(s), n);
assert(numel(vx) == n, 'rwDiscretize:lenVx', ...
    'numel(vx) (%d) must equal numel(alpha) (%d)', numel(vx), n);
assert(n == 1 || all(diff(s) > 0), 'rwDiscretize:sMonotonic', ...
    's must be strictly increasing');

% ---- nearest-station rounding ---------------------------------------------
[~, idx] = min(abs(alpha(:) - stations(:).'), [], 2);   % ties -> first (lowest) station
idx = idx(:).';

% ---- dwell-merge pass -------------------------------------------------
% Repeatedly find the shortest dwell the actuator cannot both enter and
% leave in time, and fold it into its longer neighbour, until none remain.
merged = 0;
while true
    e = [1, find(diff(idx) ~= 0) + 1, n + 1];        % dwell start indices + sentinel
    if numel(e) <= 3, break, end                      % <= 2 dwells: nothing to merge into
    lenM = s(min(e(2:end), n)) - s(e(1:end-1));        % dwell lengths [m]
    vloc = arrayfun(@(a,b) mean(vx(a:b)), e(1:end-1), e(2:end)-1);   % mean speed per dwell
    need = 10/slewRate .* vloc;                        % one 10-deg switch each way
    bad = find(lenM < need, 1);
    if isempty(bad), break, end
    if bad == 1
        nb = 2;
    elseif bad == numel(lenM)
        nb = bad - 1;
    elseif lenM(bad-1) >= lenM(bad+1)
        nb = bad - 1;
    else
        nb = bad + 1;
    end
    idx(e(bad):e(bad+1)-1) = idx(e(nb));               % absorb into the longer neighbour
    merged = merged + 1;
end
alphaDisc = stations(idx);

% ---- audit metrics on the FINAL (post-merge) trace -------------------
sw    = find(diff(idx) ~= 0);                           % transition knots
gapM  = diff([s(sw), s(end)]);                          % switch-to-switch distance [m]
needM = abs(alphaDisc(sw+1) - alphaDisc(sw)) / slewRate .* vx(sw);
roundedSlewOK = all(gapM >= needM);                     % vacuously true if sw is empty

e = [1, find(diff(idx) ~= 0) + 1, n + 1];
sStart = s(e(1:end-1)).';
sEnd   = s(min(e(2:end), n)).';
dwell  = table(stations(idx(e(1:end-1))).', sStart, sEnd, sEnd - sStart, ...
    'VariableNames', {'station', 'sStart', 'sEnd', 'lengthM'});

info = struct( ...
    'stationIdx',    idx, ...
    'nTransitions',  numel(sw), ...
    'dwell',         dwell, ...
    'merged',        merged, ...
    'roundedSlewOK', roundedSlewOK, ...
    'devRMS',        sqrt(mean((alpha - alphaDisc).^2)), ...
    'devMax',        max(abs(alpha - alphaDisc)));
end
