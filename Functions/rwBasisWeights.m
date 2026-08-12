function w = rwBasisWeights(knots, wb, alpha)
%RWBASISWEIGHTS Tanh switch-stack blend weights for the RW cardinal basis.
%
%   w = rwBasisWeights(knots, wb, alpha)
%
%   Returns a 1x(n-1) cell array, n = numel(knots), where w{j} blends polynomial
%   segment j and sum_j w{j} == 1 for every alpha.
%
%   WHY THIS EXISTS. The same stack was written out longhand in FOUR places -
%   rwAeroMap2D's basisEval, rwAeroMapEvalNum's rwBasisNum, the SX block in
%   vehModel.m, and check 4a of Validation/validateActiveRW.m - and all four
%   hardcoded THREE switches and FOUR weights, which is correct only for a
%   five-node map. At four nodes the old code read knots(4), which is then the
%   ENDPOINT rather than an interior knot, so the last weight became
%   s1*s2*(1-s3) instead of s1*s2 and collapsed to about half at the top of the
%   range. rwAeroMapEvalNum did that silently while the SX twin hard-errored -
%   the twins failing in opposite directions.
%
%   Two of the four now call this owner (basisEval, rwBasisNum). The other two
%   stay independent re-implementations BY DESIGN and were generalised in place:
%   vehModel.m's SX block must remain raw CasADi arithmetic with no cell-array
%   plumbing, and validateActiveRW's check 4a is only worth running because it is
%   a separate implementation whose disagreement trips the gate.
%
%   The switches sit on the INTERIOR knots only, knots(2:end-1): n-2 switches,
%   n-1 segments.
%
%   SX-SAFE. tanh, multiply, subtract - nothing else. alpha may be double or a
%   CasADi SX/MX scalar; the arithmetic is identical either way, which is what
%   lets vehModel.m use the same formula symbolically.
%
%   The knots/wb guards below are on BUILD-TIME doubles, never on alpha, so they
%   do not compromise SX-safety.
assert(isnumeric(knots) && isvector(knots) && all(isfinite(knots)), ...
    'rwBasisWeights:knots', 'knots must be a finite numeric vector');
n = numel(knots);
assert(n >= 3, 'rwBasisWeights:knots', ...
    'need at least 3 knots (2 segments), got %d', n);

% Strictly ascending knots and a positive width are NOT decorative. A descending
% or duplicated knot list still telescopes to a partition of unity - every weight
% is a product of s and (1-s) terms whose sum is 1 regardless of the switch ORDER
% - so the build-time partition-of-unity assert is blind to it, and the basis
% then blends the wrong segments with no other symptom. Guard at the boundary.
assert(all(diff(knots) > 0), 'rwBasisWeights:knotOrder', ...
    'knots must be strictly ascending, got [%s]', num2str(knots(:).', '%g '));
assert(isnumeric(wb) && isscalar(wb), 'rwBasisWeights:wb', ...
    'wb must be a numeric scalar, got a %s', class(wb));
assert(isfinite(wb) && wb > 0, 'rwBasisWeights:wb', ...
    'wb (switch width) must be finite and positive, got %g', wb);

s = cell(1, n-2);
for m = 1:(n-2)
    s{m} = 0.5*(1 + tanh((alpha - knots(m+1))/wb));
end

w = cell(1, n-1);
w{1} = 1 - s{1};
acc  = s{1};                       % running product s1*...*s_{j-1}
for j = 2:(n-2)
    w{j} = acc .* (1 - s{j});
    acc  = acc .* s{j};
end
w{n-1} = acc;
end
