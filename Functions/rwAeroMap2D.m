function aeroARW = rwAeroMap2D(aeroMap, vp, basisKind, nodesOverride, deltasOverride, liftMode)
%RWAEROMAP2D Build the 2-D (rear-wing angle, speed) active-aero map.
%
%   aeroARW = rwAeroMap2D(aeroMap, vp)
%   aeroARW = rwAeroMap2D([], vp)                % loads Parameters/aeroMap_Tur.mat
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind)     % kind = 'hermite3tanh' | 'lagrange4'
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes)          % TEST SEAM - see below
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes, deltas)  % TEST SEAM - see below
%   aeroARW = rwAeroMap2D(aeroMap, vp, kind, nodes, deltas, liftMode)  % see LIFTMODE below
%
%   Offline builder for the ActiveRW configuration (vp.ActAero == 1), where the
%   rear-wing angle alpha [deg] is a continuous NLP control on [-10, +15].
%
%   The map is separable by construction:
%       CL_f(alpha, v) = sum_i  L_i(alpha) * clf_i(v)
%       CL_r(alpha, v) = sum_i  L_i(alpha) * clr_i(v)
%       dCdA(alpha)    = sum_i  L_i(alpha) * dCdA_i
%   where i = 1..4 runs over the four rear-wing angle NODES
%       alphaNodes = [-10  0  +10  +15] deg
%   (the four static pipeline settings Low / Mid / High / RWp15; the +20 deg node
%   was DROPPED on 2026-08-05 - see
%   docs/superpowers/specs/2026-08-05-four-node-map-and-report-fixes-design.md),
%   [clf_i(v), clr_i(v)] = aeroEvalNum(col{i}, v) is that node's already-validated
%   quasi-static aeroelastic COLLAPSE fit (Functions/aeroCollapse.m), and L_i are
%   fixed cardinal (interpolating) basis functions of alpha only, L_i(alphaNodes(j))
%   = delta_ij, sum_i L_i(alpha) == 1.
%
%   Consequences of that structure, all of them intentional:
%     * At every node angle the 2-D map REDUCES EXACTLY to the corresponding
%       static setting - the same collapse struct, the same aeroEvalNum formula,
%       bit-identical coefficients. alpha = 0 therefore reproduces the static
%       'Mid' model, which is the regression anchor of the whole aero chain.
%     * The speed dependence (ride-height collapse, clamp knees, floored tail)
%       lives entirely inside the node fits and is never re-fitted here.
%     * The alpha dependence is a plain polynomial/tanh expression in ONE scalar,
%       so the whole thing is CasADi-SX-safe: no interpolant nodes, no if/else,
%       no lookup tables, C-infinity in both alpha and v.
%
%   Inputs
%     aeroMap    struct as returned by load('Parameters/aeroMap_Tur.mat')
%                (fields RHf, RHr, CLf, CLr), or a path to that .mat, or []
%                (default path). Passed straight through to aeroCollapse.
%     vp         vehicle parameter struct AFTER the geometry/stiffness block of
%                Parameters/vehParams.m; needs .A .rho .RHf0 .RHr0 .kwf .kwr
%     basisKind  optional, default 'hermite3tanh' (see BASIS below)
%     nodesOverride  TEST SEAM ONLY - NOT a configuration knob, and NO production
%                caller passes it. It exists for Validation/validateFourNodeMap.m,
%                which has to build a five-node and a four-node map side by side in
%                ONE session to prove the basis below +10 deg is independent of the
%                top node. Anything that ships must take the node list this file
%                hardcodes; if the shipped node list is meant to change, change the
%                literal below, not the call site.
%     deltasOverride TEST SEAM ONLY, paired with nodesOverride - NOT a configuration
%                knob, and NO production caller passes it. Also for
%                Validation/validateFourNodeMap.m: as of 2026-08-05,
%                Functions/rwAeroDelta.m's valid range is [-10, 15], so it can no
%                longer supply the RETIRED five-node map's +20 deg node (that gate
%                still needs to rebuild the old five-node map, to prove the shipped
%                four-node map reproduces it below +10 deg). When supplied, this is
%                a 3 x numel(nodesOverride) matrix [dClfA; dClrA; dCdA] - the same
%                force-equivalent delta PRODUCTS rwAeroDelta returns - used verbatim
%                for EVERY node instead of calling rwAeroDelta() at all, so a node
%                outside rwAeroDelta's current range never reaches its range check.
%                The caller is responsible for the values; this file does not
%                validate them beyond the shape check.
%     liftMode   optional, default 'collapse' (existing behaviour, byte-identical
%                when omitted - every shipped RW-sweep map uses this). 'additive'
%                is for override maps whose per-node lift deltas fall OUTSIDE
%                Functions/aeroCollapse.m's ride-height-clamp envelope (observed
%                for Functions/fwAeroDelta.m's AFWd nodes: at least one node makes
%                the fixed point never reach the compression floor on either axle,
%                aeroCollapse:nanFit, or produces a near-discontinuous ride-height
%                jump the piecewise fit cannot meet, aeroCollapse:fitResidual - see
%                docs/superpowers/specs/2026-08-07-afwd-front-wing-design.md S3).
%                Under 'additive' every node reuses the SAME single aeroCollapse
%                fit, run ONCE at the node-0 baseline (zero lift delta) - the
%                ride-height speed dependence is therefore identical across nodes -
%                and each node's lift delta is layered on afterwards as a CONSTANT
%                (speed-independent) shift, stored in the new output fields
%                .dClFadd/.dClRadd and added by the evaluators (rwAeroMapEvalNum.m,
%                vehModel.m's SX block) under the same alpha-blend weight the
%                collapsed curves use. This is Zenvo's own dashed bounding-case
%                approximation, not a re-derivation. Drag handling (.dCdA, additive
%                at eval) is IDENTICAL in both modes. Under 'collapse', .dClFadd/
%                .dClRadd are populated with zeros(1,n) so the fields always exist
%                and every downstream reader can index them unconditionally.
%
%   Output struct aeroARW   (n = numel(alphaNodes) = 4 as shipped)
%     .alphaNodes  1xn  node angles [deg]        = [-10 0 10 15]
%     .col         1xn  cell of aeroCollapse structs, one per node (feed to
%                       aeroEvalNum / vehModel.m's aeroEvalSX unchanged)
%     .dCdA        1xn  force-equivalent drag delta PRODUCTS Cd*A [m^2] vs the
%                       0 deg wing (rwAeroDelta output, NOT divided by vp.A);
%                       dCdA(2) == 0 EXACTLY. The caller divides by vp.A:
%                       vp.Cd = vp.Cd0 + dCdA(alpha)/vp.A
%     .dClfA .dClrA 1xn the matching lift delta products [m^2] (provenance)
%     .dClF .dClR  1xn  per-node effective CL increments = dCl*A/vp.A, i.e. the
%                       values actually handed to aeroCollapse under 'collapse'
%                       liftMode (under 'additive' these are NOT handed to
%                       aeroCollapse - they are copied into .dClFadd/.dClRadd
%                       instead and applied as a constant shift at eval time)
%     .dClFadd .dClRadd  1xn  per-node CONSTANT lift shifts applied at eval time
%                       under liftMode 'additive' (== .dClF/.dClR there); exactly
%                       zeros(1,n) under 'collapse', so the fields always exist
%     .basis       struct of PLAIN NUMERIC ARRAYS - see BASIS
%     .alphaMin .alphaMax  -10 / +15 [deg]
%     .meta        echo of the build inputs + per-node collapse residuals
%
%   ------------------------------------------------------------------------
%   BASIS - the exact evaluation formula (TWIN RULE)
%   ------------------------------------------------------------------------
%   Functions/rwAeroMapEvalNum.m is the numeric twin and Scripts/vehModel.m will
%   carry the SX twin. THE FORMULAS BELOW MUST BE IMPLEMENTED CHARACTER-FOR-
%   CHARACTER IDENTICALLY IN BOTH (cf. aeroEvalNum / aeroEvalSX, tyreMFnum /
%   tyreMF). Everything needed is a plain double array in aeroARW.basis; nothing
%   in the evaluation path needs this file, MATLAB toolboxes, or interp*.
%
%   basis.kind = 'hermite3tanh'   (DEFAULT - shipped choice, see SHAPE below)
%   -------------------------------------------------------------------------
%   Cardinal piecewise-cubic Hermite on the n-1 node intervals (3 as shipped),
%   with the segment kinks smoothed by tanh blends exactly like aeroEvalSX's
%   3-segment blend.
%   Fields:  basis.knots (1xn, == alphaNodes) , basis.wb (scalar, 0.3 deg) ,
%            basis.C ((n-1) x 4 x n double, i.e. 3 x 4 x 4 as shipped)
%   basis.C(j,:,i) holds the 4 coefficients, HIGHEST ORDER FIRST, of the cubic
%   that cardinal function L_i takes on interval j = 1..n-1, in the LOCAL
%   variable t_j = alpha - basis.knots(j)  (MATLAB ppform convention).
%
%   With n = numel(knots), the switches sit on the INTERIOR knots only,
%   knots(2:end-1) - n-2 switches and n-1 segments, so the stack follows the
%   node count rather than assuming five nodes:
%
%       s_j     = 0.5*(1 + tanh((alpha - knots(j+1))/wb))    j = 1..n-2
%       w_1     = 1 - s_1
%       w_j     = s_1*...*s_{j-1}*(1 - s_j)                  j = 2..n-2
%       w_{n-1} = s_1*...*s_{n-2}
%       L_i(alpha) = sum_{j=1..n-1} w_j * horner(C(j,:,i), alpha - knots(j))
%
%   At the shipped n = 4 that is a 2-switch/3-weight stack, w = [1-s_1,
%   s_1*(1-s_2), s_1*s_2]; at n = 5 it was the 3-switch/4-weight stack
%   [1-s_1, s_1*(1-s_2), s_1*s_2*(1-s_3), s_1*s_2*s_3].
%   Functions/rwBasisWeights.m is the shared owner of these weights for the
%   numeric side.
%
%   with horner(p,x) = p(1)*x^3 + p(2)*x^2 + p(3)*x + p(4) evaluated as
%   ((p(1)*x + p(2))*x + p(3))*x + p(4)   (identical to hornEval/hornSX).
%
%   basis.kind = 'lagrange4'      (alternative - kept for the shape audit)
%   -------------------------------------------------------------------------
%   The unique degree-(n-1) polynomial through the n nodes, as n Lagrange
%   cardinal polynomials in the NORMALISED angle an = alpha/basis.anorm
%   (anorm = 10, so the shipped four nodes sit at [-1 0 1 1.5] and the
%   coefficients stay O(1)). The name 'lagrange4' is historical - it was coined
%   for the five-node quartic and is kept as the basis-kind STRING so the audit
%   call sites (and any saved map) keep working; at n = 4 it builds a cubic.
%   Fields:  basis.anorm (scalar) , basis.L (n x n double)
%   basis.L(i,:) = coefficients of L_i, HIGHEST ORDER FIRST (degree n-1):
%
%       L_i(alpha) = horner(basis.L(i,:), alpha/basis.anorm)
%
%   Both bases satisfy sum_i L_i(alpha) == 1 and L_i(alphaNodes(j)) == delta_ij
%   (hermite3tanh to ~9e-15 as measured on the FIVE-node map; the shipped
%   four-node build reports 3.6e-15 - the tanh tails either way; lagrange4 to
%   eps). An SX
%   implementation may implement ONE kind and assert on basis.kind - it must
%   never guess.
%
%   ------------------------------------------------------------------------
%   SHAPE - why hermite3tanh is the default (audit 2026-07-27)
%   ------------------------------------------------------------------------
%   MEASURED ON THE FIVE-NODE MAP, before the +20 deg node was dropped
%   (2026-08-05). The numbers below are kept verbatim as the record of WHY the
%   default basis was chosen; they are not a description of the shipped
%   four-node map, whose range now stops at +15. The conclusion carries over
%   unchanged - the +10 -> +15 separation that defeats a single global
%   polynomial is entirely inside the surviving node set.
%   The rear wing is linear over -10..+10 deg and SEPARATES between +10 and +15
%   (Clr recovers from -0.983 at 10 deg to -0.812 at 15 and falls again to -0.877
%   at 20), and the ride-height collapse adds its own per-node clamp knees, so
%   the five node curves fan out hard with speed (the front-lift node spread at
%   80 m/s is 0.75 in CL). A single global quartic cannot absorb that. Measured
%   against MATLAB pchip through the same node values on alpha = -10:0.1:20 at
%   v = 30/45/60/80 m/s:
%
%       basis         worst |dev| vs pchip   worst stall-window (10..20 deg)   node-range
%                     (% of node range)      deviation (% of local range)      overshoot
%       lagrange4     0.941  (92.2 %)        29.2 %   <-- FAILS the 15 % bar    88.5 %
%       hermite3tanh  0.122  (12.0 %)        10.5 %                              4.0 %
%
%   (both worst cases are CL_f at 80 m/s; at 30-60 m/s hermite3tanh is within
%   3.3 % of pchip everywhere and 4.1-5.4 % in the stall window). The quartic
%   also makes dCdA(alpha) NON-MONOTONE - it dips below the -10 deg node value -
%   whereas hermite3tanh keeps drag strictly increasing in alpha over the whole
%   range, which is the one qualitative property the drag delta must have.
%   Re-run rwAeroMap2D(...,'lagrange4') to reproduce the audit.
%
%   ------------------------------------------------------------------------
%   ACCURACY OFF-NODE (audit 2026-07-27) - the real modelling error
%   ------------------------------------------------------------------------
%   ALSO MEASURED ON THE FIVE-NODE MAP (see the SHAPE note above). The
%   mechanism - a separable form interpolating two curves whose clamp knees sit
%   at different speeds - is unchanged by dropping the top node.
%   Node identity is exact by construction, so it says nothing about angles
%   BETWEEN nodes. Measured against a genuine aeroCollapse re-run at off-node
%   angles (pchip deltas, same map, same rates), the separable map deviates by
%       v <= 60 m/s : |dCL_f| <= 0.019 (82 N), |dCL_r| <= 0.028   -- at or below
%                     aeroCollapse's OWN fit residuals (0.015-0.026)
%       v ~ 74-84   : |dCL_f| <= 0.085 (520-725 N of front downforce)
%   The high-speed error sits exactly where the front-floor CLAMP ONSET migrates
%   with wing angle (60 m/s at -10 deg, never at +10, 69-72 at +15/+20): the
%   separable form interpolates two curves whose knees are at different speeds
%   instead of re-solving the ride-height fixed point. Inherent to the locked
%   architecture, worth stating in the report, not a defect of this file.
%
%   Note also that "just collapse at every alpha" is NOT an available
%   alternative: aeroCollapse's own 0.03 residual gate FAILS at alpha = +5 and
%   +12.5 deg. The nodes are exactly the angles for which a validated collapse
%   fit exists.
%
%   NOTE ON pchip: pchip's slope limiter is a NONLINEAR operator on the node
%   values, so no fixed cardinal basis L_i can reproduce it - and a fixed basis
%   is exactly what the separable form above needs, because the node values
%   clf_i(v)/clr_i(v) are symbolic functions of speed, not numbers, at build
%   time. hermite3tanh therefore uses the parabolic (three-point difference)
%   node slopes, which ARE linear in the node values, reproduce straight lines
%   exactly (so the -10..+10 linear wing region stays linear), and agree with
%   pchip everywhere except for a small, quantified overshoot at the stall
%   kink. This is a deliberate, documented deviation from "literal pchip".
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
                                    % +20 deg was dropped 2026-08-05 - see
                                    % docs/superpowers/specs/2026-08-05-four-node-map-and-report-fixes-design.md
if nargin >= 4 && ~isempty(nodesOverride)
    % Test seam ONLY, for Validation/validateFourNodeMap.m, which must build a
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
    % Constant-Cl additive layering (spec docs/superpowers/specs/2026-08-07-
    % afwd-front-wing-design.md S3 amendment): at least one node's lift delta
    % falls outside aeroCollapse's ride-height-clamp envelope (measured for the
    % AFWd nodes: -25 deg never reaches the compression floor on EITHER axle -
    % aeroCollapse:nanFit; -20 deg reaches it via a near-discontinuous jump the
    % piecewise fit cannot meet - aeroCollapse:fitResidual). Every node therefore
    % reuses this ONE collapse, run at the node-0 baseline (zero lift delta) -
    % the speed dependence (ride-height collapse, clamp knees) is identical
    % across nodes by construction - and the per-node lift delta is layered on
    % afterwards as a CONSTANT (speed-independent) shift, not re-solved through
    % the ride-height fixed point. This is Zenvo's own dashed bounding-case
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
% tests/test_rwBasisWeights.m is the detector for that class of bug; it is
% registered in tests/runReportTests.m so it cannot be missed.
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
