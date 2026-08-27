function aeroARW = rwAeroMap2D(aeroMap, vp, basisKind, nodesOverride, deltasOverride, liftMode)
%RWAEROMAP2D Build the 2-D (rear-wing angle, speed) active-aero map.
%
%   aeroARW = rwAeroMap2D(aeroMap, vp)
%   aeroARW = rwAeroMap2D([], vp)                % loads Parameters/aeroMap_Tur.mat
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind)     % 'hermite3tanh' | 'lagrange4'
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes)            % test seam
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes, deltas)    % test seam
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes, deltas, liftMode)
%
%   Offline builder for the ActiveRW configuration (vp.ActAero == 1), where the
%   rear-wing angle alpha [deg] is a continuous NLP control on [-10, +15].
%
%   The map is separable by construction:
%       CL_f(alpha, v) = sum_i  L_i(alpha) * clf_i(v)
%       CL_r(alpha, v) = sum_i  L_i(alpha) * clr_i(v)
%       dCdA(alpha)    = sum_i  L_i(alpha) * dCdA_i
%   where i runs over the rear-wing angle NODES, alphaNodes = [-10 0 +10 +15] deg
%   (the four static settings), [clf_i(v), clr_i(v)] = aeroEvalNum(col{i}, v) is
%   that node's quasi-static aeroelastic collapse fit (Functions/aeroCollapse.m),
%   and L_i are fixed cardinal basis functions of alpha alone, with
%   L_i(alphaNodes(j)) = delta_ij and sum_i L_i(alpha) == 1.
%
%   Three consequences, all intentional:
%     * At every node angle the map reduces EXACTLY to the corresponding static
%       setting - same collapse struct, same formula, bit-identical coefficients.
%       alpha = 0 therefore reproduces the static 'Mid' model, which is the
%       regression anchor of the whole aero chain.
%     * All speed dependence (ride-height collapse, clamp knees, floored tail)
%       lives inside the node fits and is never re-fitted here.
%     * The alpha dependence is polynomial/tanh in ONE scalar, so the result is
%       CasADi-SX-safe: no interpolant objects, no branching, no lookup tables,
%       C-infinity in both alpha and v.
%
%   Inputs
%     aeroMap    struct as returned by load('Parameters/aeroMap_Tur.mat')
%                (fields RHf, RHr, CLf, CLr), or a path to that .mat, or []
%                for the default path. Passed straight through to aeroCollapse.
%     vp         vehicle parameter struct AFTER the geometry/stiffness block of
%                Parameters/vehParams.m; needs .A .rho .RHf0 .RHr0 .kwf .kwr
%     basisKind  optional, default 'hermite3tanh' (see BASIS below)
%     nodesOverride, deltasOverride
%                TEST SEAMS, not configuration knobs - no shipped caller passes
%                them. They let a gate build maps with different node sets side
%                by side in one session. deltasOverride, when given, is a
%                3 x numel(nodesOverride) matrix [dClfA; dClrA; dCdA] of the same
%                force-equivalent delta products rwAeroDelta returns, used verbatim
%                for every node INSTEAD of calling rwAeroDelta - so nodes outside
%                rwAeroDelta's valid range never reach its range check. Values are
%                the caller's responsibility; only the shape is checked here. To
%                change the shipped node list, edit the literal below, not a call
%                site.
%     liftMode   optional, default 'collapse'; omitting it is byte-identical to
%                the previous behaviour and every RW-sweep map uses it.
%                'additive' exists for override maps whose per-node lift deltas
%                fall OUTSIDE aeroCollapse's ride-height-clamp envelope - either
%                the fixed point never reaches the compression floor on an axle
%                (aeroCollapse:nanFit) or the ride-height jump is too near
%                discontinuous for the piecewise fit (aeroCollapse:fitResidual).
%                Under 'additive' every node reuses ONE aeroCollapse fit, run at
%                the node-0 baseline with zero lift delta, so the ride-height speed
%                dependence is identical across nodes; each node's lift delta is
%                then layered on as a CONSTANT speed-independent shift in
%                .dClFadd/.dClRadd, applied by the evaluators under the same
%                alpha-blend weight as the collapsed curves. That is a constant-Cl
%                bounding approximation, not a re-derivation, and it understates
%                ride-height sensitivity in the deployed states. Drag handling is
%                identical in both modes. Under 'collapse', .dClFadd/.dClRadd are
%                zeros so downstream readers can index them unconditionally.
%
%   Output struct aeroARW   (n = numel(alphaNodes) = 4 as shipped)
%     .alphaNodes  1xn  node angles [deg] = [-10 0 10 15]
%     .col         1xn  cell of aeroCollapse structs, one per node (feed to
%                       aeroEvalNum / vehModel.m's aeroEvalSX unchanged)
%     .dCdA        1xn  force-equivalent drag delta products Cd*A [m^2] vs the
%                       0 deg wing (rwAeroDelta output, NOT divided by vp.A);
%                       dCdA(2) == 0 exactly. Caller does
%                       vp.Cd = vp.Cd0 + dCdA(alpha)/vp.A
%     .dClfA .dClrA 1xn matching lift delta products [m^2]
%     .dClF .dClR  1xn  per-node effective CL increments = dCl*A/vp.A. Handed to
%                       aeroCollapse under 'collapse'; under 'additive' they are
%                       copied to .dClFadd/.dClRadd and applied at eval instead
%     .dClFadd .dClRadd  1xn  constant lift shifts applied at eval time under
%                       'additive' (== .dClF/.dClR there), zeros under 'collapse'
%     .basis       struct of PLAIN NUMERIC ARRAYS - see BASIS
%     .alphaMin .alphaMax  -10 / +15 [deg]
%     .meta        echo of the build inputs + per-node collapse residuals
%
%   ------------------------------------------------------------------------
%   BASIS - the exact evaluation formula (TWIN RULE)
%   ------------------------------------------------------------------------
%   Functions/rwAeroMapEvalNum.m is the numeric twin and Scripts/vehModel.m
%   carries the SX twin. THE FORMULAS BELOW MUST BE IMPLEMENTED CHARACTER-FOR-
%   CHARACTER IDENTICALLY IN BOTH (cf. aeroEvalNum / aeroEvalSX, and the
%   tyre model's own twin pair). Everything needed is a plain double array
%   evaluation path needs neither this file, nor toolboxes, nor interp*.
%
%   basis.kind = 'hermite3tanh'   (DEFAULT - see SHAPE below)
%   -------------------------------------------------------------------------
%   Cardinal piecewise-cubic Hermite on the n-1 node intervals, with the segment
%   kinks smoothed by tanh blends exactly as aeroEvalSX's 3-segment blend does.
%   Fields:  basis.knots (1xn, == alphaNodes), basis.wb (scalar, 0.3 deg),
%            basis.C ((n-1) x 4 x n double).
%   basis.C(j,:,i) holds the 4 coefficients, HIGHEST ORDER FIRST, of the cubic
%   that cardinal function L_i takes on interval j, in the LOCAL variable
%   t_j = alpha - basis.knots(j) (MATLAB ppform convention).
%
%   Switches sit on the INTERIOR knots only, giving n-2 switches and n-1
%   segments, so the stack follows the node count rather than assuming a fixed n:
%
%       s_j     = 0.5*(1 + tanh((alpha - knots(j+1))/wb))    j = 1..n-2
%       w_1     = 1 - s_1
%       w_j     = s_1*...*s_{j-1}*(1 - s_j)                  j = 2..n-2
%       w_{n-1} = s_1*...*s_{n-2}
%       L_i(alpha) = sum_{j=1..n-1} w_j * horner(C(j,:,i), alpha - knots(j))
%
%   with horner(p,x) = ((p(1)*x + p(2))*x + p(3))*x + p(4), identical to
%   hornEval/hornSX. Functions/rwBasisWeights.m is the single owner of these
%   weights - do not hand-inline a second copy.
%
%   basis.kind = 'lagrange4'      (alternative, kept for the shape audit)
%   -------------------------------------------------------------------------
%   The unique degree-(n-1) polynomial through the n nodes, as n Lagrange
%   cardinal polynomials in the normalised angle alpha/basis.anorm (anorm = 10,
%   so the nodes sit at [-1 0 1 1.5] and coefficients stay O(1)). The name is
%   historical - it was coined for a five-node quartic and is kept as the
%   basis-kind STRING so audit call sites and saved maps keep working; at n = 4
%   it builds a cubic.
%   Fields:  basis.anorm (scalar), basis.L (n x n double), highest order first:
%
%       L_i(alpha) = horner(basis.L(i,:), alpha/basis.anorm)
%
%   Both bases satisfy sum_i L_i(alpha) == 1 and L_i(alphaNodes(j)) == delta_ij
%   (hermite3tanh to ~4e-15, the tanh tails; lagrange4 to eps). An SX
%   implementation may implement ONE kind and assert on basis.kind - it must
%   never guess.
%
%   ------------------------------------------------------------------------
%   SHAPE - why hermite3tanh is the default
%   ------------------------------------------------------------------------
%   The wing is linear over -10..+10 deg and SEPARATES between +10 and +15 (Clr
%   recovers from -0.983 to -0.812), and the ride-height collapse adds its own
%   per-node clamp knees, so the node curves fan out hard with speed - the
%   front-lift node spread at 80 m/s is 0.75 in CL. A single global polynomial
%   cannot absorb that. Measured against MATLAB pchip through the same node
%   values, worst deviation as a percentage of node range:
%
%       basis         worst |dev| vs pchip    stall-window dev   node-range
%       lagrange4     0.941  (92.2 %)         29.2 %             88.5 %
%       hermite3tanh  0.122  (12.0 %)         10.5 %              4.0 %
%
%   Both worst cases are CL_f at 80 m/s; below 60 m/s hermite3tanh is within
%   3.3 % of pchip everywhere. The global polynomial also makes dCdA(alpha)
%   NON-MONOTONE, dipping below the -10 deg node value, whereas hermite3tanh
%   keeps drag strictly increasing in alpha across the range - the one
%   qualitative property the drag delta must have. Re-run with 'lagrange4' to
%   reproduce the audit.
%
%   ------------------------------------------------------------------------
%   ACCURACY OFF-NODE - the real modelling error
%   ------------------------------------------------------------------------
%   Node identity is exact by construction, so it says nothing about angles
%   BETWEEN nodes. Measured against a genuine aeroCollapse re-run at off-node
%   angles, the separable map deviates by
%       v <= 60 m/s : |dCL_f| <= 0.019 (82 N), |dCL_r| <= 0.028  -- at or below
%                     aeroCollapse's own fit residuals (0.015-0.026)
%       v ~ 74-84   : |dCL_f| <= 0.085 (520-725 N of front downforce)
%   The high-speed error sits exactly where the front-floor CLAMP ONSET migrates
%   with wing angle (60 m/s at -10 deg, never at +10, 69-72 at +15): the
%   separable form interpolates two curves whose knees are at different speeds
%   rather than re-solving the ride-height fixed point. That is inherent to the
%   separable architecture, worth stating in a report, and not a defect here.
%
%   "Just collapse at every alpha" is NOT an available alternative: aeroCollapse's
%   own 0.03 residual gate FAILS at alpha = +5 and +12.5 deg. The nodes are
%   precisely the angles for which a validated collapse fit exists.
%
%   WHY NOT LITERAL pchip. pchip's slope limiter is a NONLINEAR operator on the
%   node values, so no fixed cardinal basis L_i can reproduce it - and a fixed
%   basis is exactly what the separable form needs, because at build time the
%   node values clf_i(v)/clr_i(v) are symbolic functions of speed, not numbers.
%   hermite3tanh therefore uses parabolic (three-point difference) node slopes,
%   which ARE linear in the node values, reproduce straight lines exactly (so the
%   -10..+10 linear region stays linear), and agree with pchip everywhere except
%   for a small quantified overshoot at the stall kink. A deliberate, documented
%   deviation from literal pchip.
%
%   See also RWAEROMAPEVALNUM, AEROCOLLAPSE, AEROEVALNUM, RWAERODELTA.

% ---- defaults -----------------------------------------------------------
DEFAULT_BASIS = 'hermite3tanh';
DEFAULT_LIFTMODE = 'collapse';
WB_DEG        = 0.3;      % tanh blend width [deg] for the hermite3tanh basis.
                          % Set by the node-exactness budget, NOT by taste: the
                          % blend leaks the neighbouring segment's EXTRAPOLATED
                          % cubic into a node by ~0.5*(1-tanh(dv/wb)) with dv =
                          % 5 deg (the tightest node spacing). wb = 0.5 leaks
                          % 2.1e-9 -> node identity 3.6e-9, which misses the
                          % 1e-12 twin gate; wb = 0.3 leaks 3.4e-15 -> measured
                          % node identity 6e-15 (five-node map; 3.6e-15 on the
                          % shipped four-node one). Narrowing it costs nothing in
                          % smoothness: the two blended cubics are C1-matched at
                          % the knot, so the blend's curvature perturbation is
                          % bounded by ~0.14*(curvature jump) INDEPENDENT of wb.
ANORM_DEG     = 10;       % angle normalisation for the lagrange4 basis
VMAX_COLLAPSE = 125;      % speed-grid top [m/s] - LITERAL, verbatim from
                          % Parameters/vehParams.m (pt.Vmax is not loaded there
                          % either); changing it breaks node bit-identity.

if nargin < 3 || isempty(basisKind),  basisKind = DEFAULT_BASIS;  end
if nargin < 6 || isempty(liftMode),   liftMode  = DEFAULT_LIFTMODE;  end
assert(any(strcmpi(liftMode, {'collapse','additive'})), 'rwAeroMap2D:liftMode', ...
    'unknown liftMode ''%s'' (expected collapse|additive)', char(liftMode));
if nargin < 1 || isempty(aeroMap)
    aeroMap = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                       'Parameters', 'aeroMap_Tur.mat');
end
if ischar(aeroMap) || isstring(aeroMap)
    assert(isfile(aeroMap), 'rwAeroMap2D: aero map file not found: %s', char(aeroMap));
    aeroMap = load(char(aeroMap));
end
assert(isstruct(aeroMap) && all(isfield(aeroMap, {'RHf', 'RHr', 'CLf', 'CLr'})), ...
    'rwAeroMap2D: aeroMap must be the struct from Parameters/aeroMap_Tur.mat (RHf/RHr/CLf/CLr)');
need = {'A', 'rho', 'RHf0', 'RHr0', 'kwf', 'kwr'};
assert(all(isfield(vp, need)), 'rwAeroMap2D: vp is missing %s', ...
    strjoin(need(~isfield(vp, need)), ', '));

alphaNodes = [-10 0 10 15];         % the four rear-wing angles the map spans [deg]
if nargin >= 4 && ~isempty(nodesOverride)
    % Test seam ONLY, for the node-map gate, which must build a
    % five-node and a four-node map side by side to prove the basis below +10 deg
    % is node-5-independent. No production caller passes this - it is not a
    % supported configuration knob, and the shipped node list is the literal above.
    assert(isnumeric(nodesOverride) && isvector(nodesOverride) && numel(nodesOverride) >= 3 ...
           && all(isfinite(nodesOverride)) && all(diff(nodesOverride(:).') > 0), ...
        'rwAeroMap2D:nodesOverride', ...
        'nodesOverride must be a strictly ascending finite numeric vector of >=3 nodes');
    alphaNodes = nodesOverride(:).';
end
nN = numel(alphaNodes);

% ---- per-node collapse fits ---------------------------------------------
% The three lines inside the loop that call rwAeroDelta are a VERBATIM copy of
% the aero block of Parameters/vehParams.m (the [dClfA,dClrA,dCdA]/vp.A division
% and the aeroCollapse argument list). Keep them character-identical: node i must
% come out bit-identical to the static setting at that angle, and Validation
% gates on exactly that. This is bypassed only when deltasOverride is supplied
% (test seam - see header); production callers always take this branch.
useDeltasOverride = nargin >= 5 && ~isempty(deltasOverride);
if useDeltasOverride
    assert(isequal(size(deltasOverride), [3, nN]), 'rwAeroMap2D:deltasOverride', ...
        'deltasOverride must be 3 x %d = [dClfA; dClrA; dCdA] per node, got %s', ...
        nN, mat2str(size(deltasOverride)));
end
col   = cell(1, nN);
dClfA = zeros(1, nN);  dClrA = zeros(1, nN);  dCdA = zeros(1, nN);
dClF  = zeros(1, nN);  dClR  = zeros(1, nN);
isAdditive = strcmpi(liftMode, 'additive');
fprintf('rwAeroMap2D: collapsing %d rear-wing angle nodes [%s] deg (liftMode=%s)\n', ...
        nN, strtrim(sprintf('%+d ', alphaNodes)), liftMode);
if isAdditive
    % Constant-Cl additive layering, used when a node's lift delta falls outside
    % aeroCollapse's ride-height-clamp envelope - the fixed point either never
    % reaches the compression floor on an axle (aeroCollapse:nanFit) or gets there
    % via a jump too near discontinuous for the piecewise fit
    % (aeroCollapse:fitResidual). Every node therefore reuses this ONE collapse at
    % the node-0 baseline, so the speed dependence is identical across nodes by
    % construction, and each node's lift delta is layered on afterwards as a
    % constant shift rather than re-solved through the fixed point. A bounding
    % approximation, not a re-derivation.
    baseCol = aeroCollapse(aeroMap, vp.RHf0, vp.RHr0, 2*vp.kwf, 2*vp.kwr, ...
                           0, 0, vp.rho, vp.A, VMAX_COLLAPSE);
end
for i = 1:nN
    if useDeltasOverride
        dClfA(i) = deltasOverride(1,i);
        dClrA(i) = deltasOverride(2,i);
        dCdA(i)  = deltasOverride(3,i);
    else
        [dClfA(i), dClrA(i), dCdA(i)] = rwAeroDelta(alphaNodes(i));
    end
    dClF(i) = dClfA(i)/vp.A;             % effective front-axle CL increment  (-)
    dClR(i) = dClrA(i)/vp.A;             % effective rear-axle  CL increment  (-)
    if isAdditive
        col{i} = baseCol;                % SAME fit object reused for every node
    else
        col{i}  = aeroCollapse(aeroMap, vp.RHf0, vp.RHr0, 2*vp.kwf, 2*vp.kwr, ...
                  dClF(i), dClR(i), vp.rho, vp.A, VMAX_COLLAPSE);
    end
end
if isAdditive
    dClFadd = dClF;
    dClRadd = dClR;
else
    dClFadd = zeros(1, nN);
    dClRadd = zeros(1, nN);
end

% 0 deg is the model anchor: every delta must be an EXACT zero there
i0 = find(alphaNodes == 0, 1);
assert(~isempty(i0) && dClfA(i0) == 0 && dClrA(i0) == 0 && dCdA(i0) == 0, ...
    'rwAeroMap2D: rear-wing deltas are not exactly zero at 0 deg - Mid anchor broken');

% ---- alpha-direction cardinal basis -------------------------------------
switch lower(basisKind)
    case 'hermite3tanh'
        basis = buildHermiteBasis(alphaNodes, WB_DEG);
    case 'lagrange4'
        basis = buildLagrangeBasis(alphaNodes, ANORM_DEG);
    otherwise
        error('rwAeroMap2D:basis', ...
            'unknown basis kind ''%s'' (expected hermite3tanh|lagrange4)', char(basisKind));
end

% basis.C must carry one cubic per SEGMENT, i.e. n-1 rows. buildHermiteBasis
% already sizes it generically, so this is insurance rather than a fix - but it
% is insurance against two failures of opposite loudness, and the quiet one is
% the reason it is worth the three lines. Both consumers (Scripts/vehModel.m's
% SX block, Functions/rwAeroMapEvalNum.m) loop j = 1 : n-1 over the WEIGHTS and
% index basis.C(j,:,i) with that:
%   too FEW rows  -> out-of-bounds index, loud, but thrown far from here, deep
%                    inside SX construction where the cause is unrecognisable.
%   too MANY rows -> SILENT. The loop simply never reads the extra rows, so the
%                    top of the alpha range is evaluated with the wrong cubics
%                    and every gate downstream still gets a smooth, plausible,
%                    wrong map.
% Checking the shape at the source turns both into one message that names what
% was expected.
if strcmp(basis.kind, 'hermite3tanh')
    assert(isequal(size(basis.C), [nN-1, 4, nN]), 'rwAeroMap2D:basisShape', ...
        'basis.C must be (n-1) x 4 x n = %s for %d nodes; got %s', ...
        mat2str([nN-1, 4, nN]), nN, mat2str(size(basis.C)));
end

% partition of unity + node interpolation are the two properties the whole
% construction rests on - check them here, cheaply, every build
aFine = linspace(alphaNodes(1), alphaNodes(end), 601);
Wf = basisEval(basis, aFine);
Wn = basisEval(basis, alphaNodes);
assert(max(abs(sum(Wf, 2) - 1)) < 1e-10, ...
    'rwAeroMap2D: basis is not a partition of unity (max err %.3g)', max(abs(sum(Wf, 2) - 1)));
assert(max(abs(Wn - eye(nN)), [], 'all') < 1e-6, ...
    'rwAeroMap2D: basis is not node-exact (max err %.3g)', max(abs(Wn - eye(nN)), [], 'all'));

% ---- pack ---------------------------------------------------------------
aeroARW = struct( ...
    'alphaNodes', alphaNodes, ...
    'col',        {col}, ...          % 1xnN cell (braces: keep it a cell field)
    'dCdA',       dCdA, ...
    'dClfA',      dClfA, ...
    'dClrA',      dClrA, ...
    'dClF',       dClF, ...
    'dClR',       dClR, ...
    'dClFadd',    dClFadd, ...
    'dClRadd',    dClRadd, ...
    'basis',      basis, ...
    'alphaMin',   alphaNodes(1), ...
    'alphaMax',   alphaNodes(end));

aeroARW.meta = struct( ...
    'RHf0', vp.RHf0, 'RHr0', vp.RHr0, 'kaxf', 2*vp.kwf, 'kaxr', 2*vp.kwr, ...
    'rho',  vp.rho,  'A',    vp.A,    'vmax', VMAX_COLLAPSE, ...
    'residF', cellfun(@(c) c.residF, col), 'residR', cellfun(@(c) c.residR, col), ...
    'nodeExactErr', max(abs(Wn - eye(nN)), [], 'all'), ...
    'builtOn', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), ...
    'note', ['CL_f/CL_r(alpha,v) = sum_i L_i(alpha)*aeroEvalNum(col{i},v); ' ...
             'dCdA(alpha) = sum_i L_i(alpha)*dCdA(i) [m^2, divide by vp.A]. ' ...
             'Node i == static setting at alphaNodes(i), bit-identical.']);

fprintf(['rwAeroMap2D: basis=%s | node-exactness %.1e | collapse resid F %.4f / R %.4f (worst)\n'], ...
        basis.kind, aeroARW.meta.nodeExactErr, max(aeroARW.meta.residF), max(aeroARW.meta.residR));
end

% =========================================================================
function basis = buildHermiteBasis(xn, wb)
% Cardinal piecewise-cubic Hermite with parabolic (three-point) node slopes.
% C(j,:,i) = cubic coeffs (highest order first) of L_i on interval j, in the
% local variable t = alpha - xn(j).
n  = numel(xn);
ns = n - 1;
C  = zeros(ns, 4, n);
for i = 1:n
    y = zeros(1, n);  y(i) = 1;             % unit data -> cardinal function
    m = parabolicSlopes(xn, y);
    for j = 1:ns
        h  = xn(j+1) - xn(j);
        dd = (y(j+1) - y(j))/h;
        a0 = y(j);
        a1 = m(j);
        a2 = (3*dd - 2*m(j) - m(j+1))/h;
        a3 = (m(j) + m(j+1) - 2*dd)/h^2;
        C(j,:,i) = [a3 a2 a1 a0];
    end
end
basis = struct('kind', 'hermite3tanh', 'knots', xn, 'wb', wb, 'C', C);
end

function m = parabolicSlopes(x, y)
% Derivative at each node of the parabola through the 3 nearest points - the
% LINEAR (un-limited) counterpart of pchip's monotone-limited slopes. Linear in
% y (required: the node values are symbolic functions of speed downstream) and
% exact on straight-line data (keeps the -10..+10 linear wing region linear).
h = diff(x);
d = diff(y)./h;
m = zeros(size(y));
m(2:end-1) = (h(2:end).*d(1:end-1) + h(1:end-1).*d(2:end)) ./ (h(1:end-1) + h(2:end));
m(1)   = ((2*h(1)   + h(2))    *d(1)   - h(1)  *d(2))     /(h(1) + h(2));
m(end) = ((2*h(end) + h(end-1))*d(end) - h(end)*d(end-1)) /(h(end) + h(end-1));
end

function basis = buildLagrangeBasis(xn, anorm)
% The n Lagrange cardinal polynomials (degree n-1) in the normalised angle
% an = alpha/anorm. 'lagrange4' is the historical kind name, from the five-node
% quartic; at the shipped n = 4 these are cubics.
n  = numel(xn);
xa = xn/anorm;
L  = zeros(n, n);
for i = 1:n
    p = poly(xa([1:i-1, i+1:n]));           % monic, roots at the other nodes
    L(i,:) = p / polyval(p, xa(i));
end
basis = struct('kind', 'lagrange4', 'anorm', anorm, 'L', L);
end

function W = basisEval(basis, alpha)
% Build-time check helper only: W(k,i) = L_i(alpha(k)). The SHIPPED evaluator is
% Functions/rwAeroMapEvalNum.m.
%
% WHAT IS STILL INDEPENDENT, AND WHAT IS NOT. This used to be a wholly separate
% second implementation, so a typo in either one tripped the asserts above. That
% is now only PARTLY true: since both this helper and rwAeroMapEvalNum's
% rwBasisNum call Functions/rwBasisWeights.m, the blend WEIGHTS are shared code
% and the asserts above cannot see a bug in them. Only the polynomial
% accumulation (the Horner loop over segments and nodes) is still independently
% written here - polyval against rwAeroMapEvalNum's hand-rolled hornEval.
%
% The gap that leaves: an off-by-one in the shared owner that still TELESCOPES -
% switching on knots(m) instead of knots(m+1), say - keeps partition of unity
% intact and propagates identically into both sides, so nothing here would fire.
% The rwBasisWeights unit test is the detector for that class of bug; keep it
% registered in whatever suite you run so it cannot be missed.
alpha = alpha(:);
switch basis.kind
    case 'hermite3tanh'
        n  = numel(basis.knots);
        % Weights from the shared owner (Functions/rwBasisWeights.m). This used to
        % index s(:,3) unconditionally, which is out of bounds for any map with
        % fewer than five nodes.
        wc = rwBasisWeights(basis.knots, basis.wb, alpha);
        W  = zeros(numel(alpha), n);
        for i = 1:n
            for j = 1:(n-1)
                W(:,i) = W(:,i) + wc{j}.*polyval(basis.C(j,:,i), alpha - basis.knots(j));
            end
        end
    case 'lagrange4'
        W = zeros(numel(alpha), size(basis.L, 1));
        for i = 1:size(basis.L, 1)
            W(:,i) = polyval(basis.L(i,:), alpha/basis.anorm);
        end
    otherwise
        error('rwAeroMap2D:basis', 'unknown basis kind ''%s''', basis.kind);
end
end
