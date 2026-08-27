function [alphaDisc, info] = rwDiscretize(s, alpha, stations, slewRate, vx)
%RWDISCRETIZE Nearest-station rounding of a continuous wing trace, with dwells
%  too short for the actuator to hold merged into a longer neighbour.
%
%   [alphaDisc, info] = rwDiscretize(s, alpha, stations, slewRate, vx)
%
%   Pure-MATLAB post-processing (no CasADi). Takes a continuous optimal wing
%   angle alpha(s) and rounds it onto a discrete `stations` set, the way an
%   actuator with a handful of physical detents would fly it. A plain knot-by-
%   knot round can leave single-knot "spike" dwells the actuator cannot reach
%   and leave again before the next commanded change; the merge pass folds each
%   such dwell into its longer neighbour, repeating until every survivor clears
%   the threshold, and reports the merge count plus audit metrics on the final
%   trace.
%
%   INPUTS
%     s         independent-variable samples [m], strictly increasing.
%               numel(s) must equal numel(alpha) and numel(vx).
%     alpha     continuous wing-angle trace [deg], same length as s.
%     stations  discrete angle set to round onto [deg]. Sorted ascending
%               internally; need not be sorted or unique on input.
%     slewRate  actuator slew rate [deg/s], finite positive scalar.
%     vx        speed [m/s] at each sample, same length as s. Converts deg/s
%               into a metres-needed distance, evaluated LOCALLY per dwell and
%               per switch, never as a single lap average.
%
%   OUTPUTS
%     alphaDisc  1xN row vector, each entry one of `stations`.
%     info       struct:
%       stationIdx     1xN index of alphaDisc into (sorted) stations.
%       nTransitions   station changes in the FINAL trace.
%       dwell          table per surviving dwell: station, sStart, sEnd,
%                      lengthM. The last dwell's sEnd is the final sample
%                      (inclusive); every other dwell's sEnd is the first
%                      sample of the NEXT dwell (exclusive). That matches what
%                      the merge pass audits against - it is not a general
%                      interval convention.
%       merged         number of merges performed (one dwell absorbed each).
%       roundedSlewOK  true iff every consecutive pair of transitions is far
%                      enough apart, in metres, for the actuator to complete
%                      the intervening change:
%                        gap(i) >= |dAlpha(i)|/slewRate * vx(sw(i))
%                      where sw are the transition knots and gap(i) runs to the
%                      next switch, or to the trace end for the last. Vacuously
%                      true with 0 or 1 transitions.
%       devRMS, devMax RMS and max |alpha - alphaDisc| over all knots [deg].
%
%   MERGE RULE. A dwell is too short if its length in metres is below
%   10/slewRate * mean(vx over the dwell) - a flat "one 10 deg switch each way"
%   assumption. 10 deg is the largest ADJACENT spacing in the 4-station set
%   [-10 0 10 15], so the threshold is deliberately conservative rather than
%   exact per transition: cheap, symmetric, and it need not know which two
%   stations border the dwell. The shortest flagged dwell is absorbed into
%   whichever neighbour is longer (ties favour the earlier one), repeating until
%   nothing is flagged or one dwell remains.
%
%   KNOWN LIMITATION. A trace with only two dwells never enters the loop - there
%   is nothing to merge into without first picking a direction, and the exit
%   test treats that as already merged. So a single too-short dwell at the very
%   start or end of the whole trace, having only one neighbour, is never
%   merge-checked.
%
%   TIE-BREAK. When alpha sits exactly midway between two stations, min()
%   returns the FIRST (lower) station. Documented behaviour, not a claim that it
%   is more correct than rounding up.
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
