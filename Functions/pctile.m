function q = pctile(x, p)
%PCTILE Percentiles of a vector, in base MATLAB.
%
%   q = PCTILE(x, p) returns the p-th percentile(s) of the data in x, with p in
%   [0, 100]. q has the shape of p. NaNs in x are ignored; an x with nothing
%   left after that gives NaN.
%
%   WHY THIS EXISTS. prctile belongs to the Statistics and Machine Learning
%   Toolbox on every release before R2024b, and the only thing this repository
%   ever asked of it was one 90th percentile of a band-width trace - computed on
%   every reactive-wing (ARFWr) solve and on every post-hoc recompute. A whole
%   toolbox dependency on the offline solver for that is not a trade worth
%   making, so the number is computed here instead. One owner, shared by
%   Functions/lawTrackStats.m and the mode-3 block in Scripts/MLTP.m - the same
%   standing rule as Functions/rwBasisWeights.m.
%
%   IT REPRODUCES prctile's DEFAULT METHOD EXACTLY, not approximately:
%     - drop NaNs and sort ascending -> xs(1..n);
%     - xs(k) is the empirical percentile 100*(k-0.5)/n, i.e. the sorted values
%       sit at the MIDPOINTS of n equal probability bins, not at k/n. Inverting
%       that, percentile p sits at the fractional POSITION r = p/100*n + 0.5 in
%       the sorted vector;
%     - interpolate linearly between the two neighbouring sorted values;
%     - clamp r to [1, n], so a p below the first position returns min(x) and
%       one above the last returns max(x), rather than extrapolating.
%
%   The arithmetic is deliberately phrased as an interpolation in POSITION and
%   not in percentile. The two are the same in exact arithmetic and not in
%   floating point: interpolating on a percentile axis 100*(k-0.5)/n reproduces
%   prctile only to about 1e-14, and on one of the shipped laps it does differ
%   in the last two digits. Working in position matches bit for bit, which is
%   what lets the substitution be checked with isequal rather than a tolerance.
%   Verified equal to prctile on the six band traces of the three shipped laps,
%   on the n = 1, n = 2, all-equal and NaN-bearing edge cases, and on 2000
%   random vectors of length 2 to 800.
%
%   See also LAWTRACKSTATS.

x = x(:);
x = x(~isnan(x));
if isempty(x)
    q = NaN(size(p));
    return
end
xs = sort(x);
n  = numel(xs);
if n == 1
    q = repmat(xs, size(p));
    return
end
r = min(max((p(:)/100)*n + 0.5, 1), n);      % fractional position, clamped to the data
q = reshape(interp1(xs, r, 'linear'), size(p));
end
