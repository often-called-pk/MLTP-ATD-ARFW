function [ClF, ClR, dCdA] = rwAeroMapEvalNum(aeroARW, alpha, vx, aeroAFW, alphaFW)
%RWAEROMAPEVALNUM Numeric evaluation of the 2-D active-rear-wing aero map.
%
%   [ClF, ClR, dCdA] = rwAeroMapEvalNum(aeroARW, alpha, vx)
%   [ClF, ClR, dCdA] = rwAeroMapEvalNum(aeroARW, alpha, vx, aeroAFW, alphaFW)
%
%   The 5-arg form adds the two-wing front-wing delta layer: aeroAFW is a
%   liftMode='additive' map from Functions/rwAeroMap2D.m, consumed as a DELTA
%   layer only - its per-node speed curves are never evaluated, because the RW
%   map owns the ride-height lift. ClF/ClR gain sum_i L_i(alphaFW)*dCl*add(i)
%   (already divided by vp.A at map build); dCdA gains
%   sum_i L_i(alphaFW)*aeroAFW.dCdA(i), still a Cd*A PRODUCT that the caller
%   divides by vp.A exactly once. alphaFW must be scalar or match size(alpha)
%   after expansion; outside the FW node span it errors rwAeroMapEvalNum:rangeFW.
%
%   aeroARW  struct from Functions/rwAeroMap2D.m
%   alpha    rear-wing angle [deg], within the map's node span ([-10, +15])
%   vx       longitudinal speed [m/s]
%            alpha and vx may be any equal-sized arrays, or either may be a
%            scalar (expanded against the other). Outputs take the common size.
%
%   ClF, ClR  TOTAL effective per-axle lift coefficients at (alpha, vx): the
%             ride-height-collapsed map value with the wing-angle increment
%             already inside it, exactly what aeroEvalNum returns for a static
%             setting. NEGATIVE = DOWNFORCE. They are coefficients on the fixed
%             reference area vp.A, so axle downforce is -0.5*rho*vp.A*vx^2*ClF
%             and the left/right split stays 0.5*(ClF + ClR) per side.
%   dCdA      drag delta PRODUCT Cd*A [m^2] relative to the 0 deg wing, NOT
%             divided by the reference area. The caller does
%                 vp.Cd = vp.Cd0 + dCdA/vp.A;
%             and routes the increment through f_dragRW at vp.hw. Exactly 0 at
%             alpha = 0.
%
%   This is the NUMERIC TWIN of the SX evaluator in Scripts/vehModel.m for
%   vp.ActAero == 1 - KEEP THE FORMULAS IDENTICAL (same rule as
%   aeroEvalNum/aeroEvalSX). The evaluation is
%
%       [clf_i, clr_i] = aeroEvalNum(aeroARW.col{i}, vx)      i = 1..n
%       ClF  = sum_i L_i(alpha)*(clf_i + aeroARW.dClFadd(i))
%       ClR  = sum_i L_i(alpha)*(clr_i + aeroARW.dClRadd(i))
%       dCdA = sum_i L_i(alpha)*aeroARW.dCdA(i)               (alpha only)
%
%   .dClFadd/.dClRadd are the liftMode='additive' constant lift shifts, exactly
%   zero under the default liftMode='collapse', so this reduces to the original
%   formula bit-for-bit for every standard map.
%
%   The cardinal basis L_i is defined by aeroARW.basis. For the default
%   basis.kind = 'hermite3tanh' (knots = alphaNodes, n = numel(knots), wb = 0.3
%   deg, C = (n-1) x 4 x n cubic coefficients, highest order first, in
%   t = alpha - knots(j)):
%
%       s_j     = 0.5*(1 + tanh((alpha - knots(j+1))/wb))   j = 1..n-2
%       w_1     = 1 - s_1
%       w_j     = s_1*...*s_{j-1}*(1 - s_j)                 j = 2..n-2
%       w_{n-1} = s_1*...*s_{n-2}
%       L_i     = sum_{j=1..n-1} w_j * horner(C(j,:,i), alpha - knots(j))
%
%   Switches sit on the INTERIOR knots only, so the stack follows the node
%   count. Functions/rwBasisWeights.m owns these weights for this file; the SX
%   twin re-implements the same formula inline.
%
%   For basis.kind = 'lagrange4' (anorm = 10, L = n x n coefficients of the
%   degree-(n-1) cardinal polynomials, highest order first):
%
%       L_i = horner(basis.L(i,:), alpha/basis.anorm)
%
%   horner(p,x) = ((...(p(1)*x + p(2))*x + ...)*x + p(end)), identical to
%   hornEval in Functions/aeroEvalNum.m.
%
%   At alpha == alphaNodes(i) the basis collapses to the i-th unit vector, so
%   the result IS that static setting's aeroEvalNum(col{i}, vx) - measured to
%   2.9e-15 over v = 10:90, the tanh tails being the only slack. alpha = 0
%   therefore reproduces the static 'Mid' model to ~1e-16, i.e. to Horner
%   roundoff, but NOT to a bit-identical zero: the stored node value
%   aeroARW.dCdA(2) is an exact 0, while the interpolated dCdA(0) is ~3e-17 m^2,
%   which is 1e-13 N of drag at 80 m/s.
%
%   RANGE: alpha outside the map's node span (read from aeroARW.alphaNodes,
%   never a literal) is an error. A slack of 1e-6 deg is tolerated and clamped
%   first, because IPOPT satisfies variable bounds only to its
%   bound_relax_factor and post-processing would otherwise blow up on a solution
%   sitting 1e-9 deg outside its own bound.
%
%   See also RWAEROMAP2D, AEROEVALNUM, RWAERODELTA.

TOL_DEG = 1e-6;     % bound slack tolerated (clamped, not an error) - see RANGE

assert(isstruct(aeroARW) && all(isfield(aeroARW, {'alphaNodes', 'col', 'dCdA', 'basis'})), ...
    'rwAeroMapEvalNum:input', 'aeroARW must come from Functions/rwAeroMap2D.m');

aMin = aeroARW.alphaNodes(1);
aMax = aeroARW.alphaNodes(end);
if any(alpha(:) < aMin - TOL_DEG) || any(alpha(:) > aMax + TOL_DEG)
    bad = alpha(find(alpha(:) < aMin - TOL_DEG | alpha(:) > aMax + TOL_DEG, 1));
    error('rwAeroMapEvalNum:range', ...
        'rear-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        bad, aMin, aMax);
end
alpha = min(max(alpha, aMin), aMax);        % clamp the tolerated bound slack

% scalar expansion
if isscalar(alpha) && ~isscalar(vx),  alpha = alpha + zeros(size(vx));  end
if isscalar(vx) && ~isscalar(alpha),  vx    = vx    + zeros(size(alpha));  end
assert(isequal(size(alpha), size(vx)), ...
    'rwAeroMapEvalNum:size', 'alpha %s and vx %s must be the same size (or scalar)', ...
    mat2str(size(alpha)), mat2str(size(vx)));

% cardinal basis weights: W{i} has the size of alpha
W = rwBasisNum(aeroARW.basis, alpha);

% Constant-Cl additive layering (liftMode='additive' in Functions/rwAeroMap2D.m,
% added 2026-08-07 for override maps like AFWd whose node deltas fall outside
% aeroCollapse's ride-height envelope). Every map built by the current
% rwAeroMap2D.m carries these fields (zeros under the default 'collapse' mode),
% but an aeroARW struct loaded from an ARCHIVED .mat predating this change would
% not - the isfield guard resolves that BEFORE any arithmetic, purely for
% backward compat, so the accumulation below is formula-identical to the SX
% twin in Scripts/vehModel.m either way.
n = numel(aeroARW.col);
if isfield(aeroARW, 'dClFadd'), dClFadd = aeroARW.dClFadd; else, dClFadd = zeros(1,n); end
if isfield(aeroARW, 'dClRadd'), dClRadd = aeroARW.dClRadd; else, dClRadd = zeros(1,n); end

ClF  = zeros(size(alpha));
ClR  = zeros(size(alpha));
dCdA = zeros(size(alpha));
for i = 1:n
    [clf_i, clr_i] = aeroEvalNum(aeroARW.col{i}, vx);
    ClF  = ClF  + W{i}.*(clf_i + dClFadd(i));
    ClR  = ClR  + W{i}.*(clr_i + dClRadd(i));
    dCdA = dCdA + W{i}*aeroARW.dCdA(i);
end

% ARFWd FW additive delta layer (vp.rwMandate == 6): a SECOND map's constant
% lift/drag shifts blended in its OWN angle - per-node speed curves NOT
% consumed. SX TWIN: Scripts/vehModel.m ActAero==1 mode-6 block - KEEP THE
% FORMULAS IDENTICAL.
if nargin >= 4 && ~isempty(aeroAFW)
    assert(nargin >= 5, 'rwAeroMapEvalNum:inputFW', 'alphaFW is required when aeroAFW is given');
    assert(isstruct(aeroAFW) && all(isfield(aeroAFW, {'alphaNodes','col','dCdA','dClFadd','dClRadd','basis'})), ...
        'rwAeroMapEvalNum:inputFW', 'aeroAFW must come from Functions/rwAeroMap2D.m (liftMode=''additive'')');
    fMin = aeroAFW.alphaNodes(1);
    fMax = aeroAFW.alphaNodes(end);
    if any(alphaFW(:) < fMin - TOL_DEG) || any(alphaFW(:) > fMax + TOL_DEG)
        badFW = alphaFW(find(alphaFW(:) < fMin - TOL_DEG | alphaFW(:) > fMax + TOL_DEG, 1));
        error('rwAeroMapEvalNum:rangeFW', ...
            'front-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
            badFW, fMin, fMax);
    end
    alphaFW = min(max(alphaFW, fMin), fMax);    % clamp the tolerated bound slack
    if isscalar(alphaFW) && ~isscalar(alpha), alphaFW = alphaFW + zeros(size(alpha)); end
    assert(isequal(size(alphaFW), size(alpha)), ...
        'rwAeroMapEvalNum:sizeFW', 'alphaFW %s and alpha %s must be the same size (or scalar)', ...
        mat2str(size(alphaFW)), mat2str(size(alpha)));
    Wfw = rwBasisNum(aeroAFW.basis, alphaFW);
    for i = 1:numel(aeroAFW.col)
        ClF  = ClF  + Wfw{i}.*aeroAFW.dClFadd(i);
        ClR  = ClR  + Wfw{i}.*aeroAFW.dClRadd(i);
        dCdA = dCdA + Wfw{i}*aeroAFW.dCdA(i);
    end
end
end

% =========================================================================
function W = rwBasisNum(basis, alpha)
%RWBASISNUM Cardinal basis weights L_i(alpha), returned as a 1xN cell of arrays
% the size of alpha. SX TWIN: the same arithmetic with scalar SX alpha.
switch basis.kind
    case 'hermite3tanh'
        kn = basis.knots;
        n  = numel(kn);
        % Shared owner (Functions/rwBasisWeights.m). The hardcoded s1/s2/s3 stack
        % this replaces read kn(4) as a switch, which is the ENDPOINT on a
        % four-node map - the last weight then halved at the top of the range,
        % silently, while the SX twin hard-errored.
        w  = rwBasisWeights(kn, basis.wb, alpha);
        W  = cell(1, n);
        for i = 1:n
            W{i} = zeros(size(alpha));
            for j = 1:(n-1)
                W{i} = W{i} + w{j}.*hornEval(basis.C(j,:,i), alpha - kn(j));
            end
        end
    case 'lagrange4'
        n = size(basis.L, 1);
        W = cell(1, n);
        for i = 1:n
            W{i} = hornEval(basis.L(i,:), alpha/basis.anorm);
        end
    otherwise
        error('rwAeroMapEvalNum:basis', ...
            'unknown basis kind ''%s'' (expected hermite3tanh|lagrange4)', basis.kind);
end
end

function y = hornEval(p, x)
% Horner evaluation, highest-order coefficient first (polyval-compatible).
% Byte-identical to hornEval in Functions/aeroEvalNum.m / hornSX in vehModel.m.
y = zeros(size(x)) + p(1);
for k = 2:numel(p)
    y = y.*x + p(k);
end
end
