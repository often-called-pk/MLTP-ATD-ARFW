function [ClF, ClR, Cd, Fdrag0, FdragRW] = aeroLiveEval(RHf, RHr, alphaRW, alphaFW, vx, vp)
%AEROLIVEEVAL Live (instantaneous ride-height) aero reference for the ARFWr
% 6-DOF real-time plant. This is the reference implementation the Simulink
% Plant subsystem's aero block mirrors.
%
%   [ClF, ClR, Cd, Fdrag0, FdragRW] = aeroLiveEval(RHf, RHr, alphaRW, alphaFW, vx, vp)
%
% ---------------------------------------------------------------------------
% WHY THIS IS A DIFFERENT EVALUATION PATH FROM THE OFFLINE MLTP SOLVER
% ---------------------------------------------------------------------------
% The offline solver (Scripts/vehModel.m's vp.ActAero==1 branch, numeric twin
% Functions/rwAeroMapEvalNum.m) never sees ride height as an independent
% variable at solve time: Functions/aeroCollapse.m resolves the ride-height/
% downforce feedback loop QUASI-STATICALLY, once per rear-wing node angle, by
% its own damped fixed-point iteration over a v-only grid, and fits the result
% to a speed-only curve col{i}. The instantaneous ride height is baked out of
% the model entirely - it is a function of v alone, per node.
%
% A 6-DOF plant has genuine ride-height dynamic states (front/rear spring
% compression from a real suspension model), which can visit (RHf,RHr)
% combinations the quasi-static assumption never explores (transients,
% kerb strikes, combined braking+cornering compression, ...). This function
% therefore interpolates the RAW 2-D ride-height lift map directly at
% whatever (RHf,RHr) it is handed, instead of consulting a pre-collapsed
% speed-only fit. It does NOT rebuild any map (no Functions/aeroCollapse.m or
% Functions/rwAeroMap2D.m call inside this function) - it only interpolates
% the same static grid those offline builders themselves interpolate.
%
% ---------------------------------------------------------------------------
% FORMULA (derivation in the task-3.3 report; ported from, and validated
% against, Scripts/vehModel.m's ActAero==1 block / Functions/rwAeroMapEvalNum.m)
% ---------------------------------------------------------------------------
% Functions/aeroCollapse.m's own per-speed lookup is
%     clfFun(rhf,rhr) = interp2(map.RHr, map.RHf, map.CLf, rhr, rhf, 'linear') + dClF
%     clrFun(rhf,rhr) = interp2(map.RHr, map.RHf, map.CLr, rhr, rhf, 'linear') + dClR
% (Functions/aeroCollapse.m:73-74) where dClF/dClR is a CONSTANT (RH- and
% v-independent) additive shift for the node being collapsed. That functional
% form - base 2-D map plus a constant per-node shift - holds for ANY (rhf,rhr),
% not only the node's own solved equilibrium trajectory, so it generalises
% directly to an arbitrary instantaneous ride height:
%
%     ClF_base(RHf,RHr) = interp2(map.RHr, map.RHf, map.CLf, RHr, RHf, 'linear')  [Mid map]
%     ClR_base(RHf,RHr) = interp2(map.RHr, map.RHf, map.CLr, RHr, RHf, 'linear')
%     dClF_RW(alphaRW)  = sum_i L_i(alphaRW) * vp.aeroARW.dClF(i)
%     dClR_RW(alphaRW)  = sum_i L_i(alphaRW) * vp.aeroARW.dClR(i)
%     dCdA_RW(alphaRW)  = sum_i L_i(alphaRW) * vp.aeroARW.dCdA(i)
%     dClF_FW(alphaFW)  = sum_i L_i^FW(alphaFW) * vp.aeroAFW.dClFadd(i)
%     dClR_FW(alphaFW)  = sum_i L_i^FW(alphaFW) * vp.aeroAFW.dClRadd(i)
%     dCdA_FW(alphaFW)  = sum_i L_i^FW(alphaFW) * vp.aeroAFW.dCdA(i)
%
%     ClF = ClF_base + dClF_RW + dClF_FW
%     ClR = ClR_base + dClR_RW + dClR_FW
%     Cd  = vp.Cd0 + (dCdA_RW + dCdA_FW)/vp.A
%
% vp.aeroARW.dClF/.dClR are "the values actually handed to aeroCollapse under
% liftMode='collapse'" (Functions/rwAeroMap2D.m header) - i.e. exactly the
% constant dClF/dClR that aeroCollapse's clfFun/clrFun add to the base map -
% so reusing them here (instead of vp.aeroARW.dClFadd/.dClRadd, which are
% zeros(1,n) under 'collapse') is correct for the standard RW map. The FW
% layer (vp.aeroAFW) is always built with liftMode='additive'
% (Parameters/vehParams.m mode 5/6/7), where .dClFadd/.dClRadd ARE the
% per-node constant shift by construction (Functions/rwAeroMap2D.m's
% 'additive' branch) - matching Functions/rwAeroMapEvalNum.m's own FW-layer
% formula exactly (its ClF/ClR/dCdA accumulation over aeroAFW.dClFadd/
% .dClRadd/.dCdA). Design decision: the fwOnlyDelta chain is reused
% as vp.aeroAFW encodes it. The constants are already baked into vp.aeroAFW
% by Parameters/vehParams.m (Functions/fwOnlyDelta.m), so this function
% consumes them rather than recomputing fwOnlyDelta.m's own pchip fit itself.
%
% L_i(alpha)/L_i^FW(alpha) are the SAME cardinal hermite3tanh basis weights
% Scripts/vehModel.m's SX block and Functions/rwAeroMapEvalNum.m use on the
% per-node SPEED curves - here they blend the per-node CONSTANT lift/drag
% deltas instead, which is valid because those deltas do not depend on speed
% or ride height in the first place. The weight arithmetic itself is a direct
% port of Functions/rwAeroMapEvalNum.m's local rwBasisNum (hermite3tanh case
% only - the only basis kind vp.aeroARW/vp.aeroAFW ship with, same restriction
% Scripts/vehModel.m's SX block asserts), reusing Functions/rwBasisWeights.m
% (the shared owner of the tanh switch-stack) for the weights and re-
% implementing only the Horner polynomial accumulation locally below, since
% rwBasisNum is a private subfunction of Functions/rwAeroMapEvalNum.m and
% cannot be called from outside that file.
%
% The drag/height force split is IDENTICAL to Scripts/vehModel.m:602-604:
%     Fdrag0  = 0.5*vp.rho*vp.A*vx.^2*vp.Cd0            (body drag, acts at vp.hcg)
%     FdragRW = 0.5*vp.rho*vp.A*vx.^2*(Cd - vp.Cd0)     (wing-angle increment, acts at vp.hw)
% This function returns the two FORCES only; applying them at vp.hcg/vp.hw to
% get pitch moments is the caller's job (the Plant subsystem), exactly
% as Scripts/vehModel.m's own f_drag0/f_dragRW are consumed downstream.
%
% ---------------------------------------------------------------------------
% Inputs
%   RHf, RHr   instantaneous front/rear ride height [mm] - SAME convention as
%              vp.RHf0/vp.RHr0 (Parameters/vehParams.m) and the map grid
%              (Functions/importAeroMap.m: RHf spans 41..140 mm, RHr spans
%              40..150 mm). Clamped to the map's grid edges before
%              interpolation (silently - the map has no data outside its
%              measured envelope; Functions/aeroCollapse.m clamps its own RH
%              iterate to the same edges, aeroCollapse.m:93-94).
%   alphaRW    rear-wing angle [deg]. Clamped to vp.aeroARW.alphaNodes range
%              with a 1e-6 deg bound slack (errors beyond that) - same
%              convention as Functions/rwAeroMapEvalNum.m.
%   alphaFW    front-wing flap angle [deg]. Same range/clamp convention
%              against vp.aeroAFW.alphaNodes. Ignored (zero contribution) if
%              vp has no .aeroAFW field.
%   vx         longitudinal speed [m/s] - used only for the drag forces.
%   vp         vehicle parameter struct (simulink/params/loadVehicleParams.m
%              or Parameters/vehParams.m directly) - needs .A .rho .Cd0
%              .aeroARW (struct from Functions/rwAeroMap2D.m), and .aeroAFW
%              when the front-wing layer is in use.
%
%   RHf, RHr, alphaRW, alphaFW, vx may be scalars or equal-sized arrays; a
%   scalar input is expanded against any array input (Functions/
%   rwAeroMapEvalNum.m's scalar-expansion convention).
%
% Outputs
%   ClF, ClR          per-axle lift coefficients on the fixed reference area
%                      vp.A (negative = downforce) - same sign convention as
%                      vp.Cl_front/vp.Cl_rear in Scripts/vehModel.m.
%   Cd                 total drag coefficient.
%   Fdrag0, FdragRW    drag force split [N] (see FORMULA above).
%
% The raw ride-height grid (RHf/RHr/CLf/CLr) is loaded directly from
% Parameters/aeroMap_Tur.mat - the SAME source file Parameters/vehParams.m
% loads into its own local `aeroMapTmp` (Parameters/vehParams.m:215) - because
% vehParams.m does not persist that grid onto vp; it is `clear`ed
% (Parameters/vehParams.m:337) once vp.aeroARW/.aeroAFW/.aero.sel are built
% from it. Loading the same static asset file here is not "rebuilding a map"
% (no aeroCollapse/rwAeroMap2D call): it is the same interp2 lookup those
% builders themselves perform, at a caller-supplied ride height instead of a
% quasi-statically solved one.
%
% See also FUNCTIONS/AEROCOLLAPSE, FUNCTIONS/RWAEROMAP2D,
% FUNCTIONS/RWAEROMAPEVALNUM, FUNCTIONS/RWBASISWEIGHTS, FUNCTIONS/FWONLYDELTA.

TOL_DEG = 1e-6;     % bound slack tolerated on alphaRW/alphaFW (clamped, not an
                    % error) - same convention as Functions/rwAeroMapEvalNum.m

assert(isstruct(vp) && all(isfield(vp, {'A','rho','Cd0','aeroARW'})), ...
    'aeroLiveEval:vp', ...
    'vp must carry .A .rho .Cd0 .aeroARW (Parameters/vehParams.m or simulink/params/loadVehicleParams.m)');
assert(isstruct(vp.aeroARW) && all(isfield(vp.aeroARW, {'alphaNodes','basis','dClF','dClR','dCdA'})), ...
    'aeroLiveEval:aeroARW', 'vp.aeroARW must come from Functions/rwAeroMap2D.m');

hasFW = isfield(vp, 'aeroAFW') && isstruct(vp.aeroAFW) && ~isempty(vp.aeroAFW);
if hasFW
    assert(all(isfield(vp.aeroAFW, {'alphaNodes','basis','dClFadd','dClRadd','dCdA'})), ...
        'aeroLiveEval:aeroAFW', ...
        'vp.aeroAFW must come from Functions/rwAeroMap2D.m (liftMode=''additive'')');
end

% ---- raw ride-height grid: loaded once, cached across calls -------------
persistent rawMap
if isempty(rawMap)
    here     = fileparts(mfilename('fullpath'));            % ...\simulink\aero
    repoRoot = fileparts(fileparts(here));                  % -> repo root
    mapFile  = fullfile(repoRoot, 'Parameters', 'aeroMap_Tur.mat');
    assert(isfile(mapFile), 'aeroLiveEval:mapNotFound', ...
        'raw ride-height aero map not found: %s', mapFile);
    rawMap = load(mapFile);
    assert(all(isfield(rawMap, {'RHf','RHr','CLf','CLr'})), ...
        'aeroLiveEval:mapFields', ...
        '%s is missing RHf/RHr/CLf/CLr', mapFile);
end

% ---- scalar/array broadcast (Functions/rwAeroMapEvalNum.m convention) ---
sz      = aeroLiveEval_broadcastSize(RHf, RHr, alphaRW, alphaFW, vx);
RHf     = aeroLiveEval_expandTo(RHf,     sz);
RHr     = aeroLiveEval_expandTo(RHr,     sz);
alphaRW = aeroLiveEval_expandTo(alphaRW, sz);
alphaFW = aeroLiveEval_expandTo(alphaFW, sz);
vx      = aeroLiveEval_expandTo(vx,      sz);

% ---- 1) base (Mid, 0 deg wing) lift from the raw 2-D ride-height map ----
RHfMin = min(rawMap.RHf); RHfMax = max(rawMap.RHf);
RHrMin = min(rawMap.RHr); RHrMax = max(rawMap.RHr);
RHfC = min(max(RHf, RHfMin), RHfMax);            % silent clamp at map edges
RHrC = min(max(RHr, RHrMin), RHrMax);
% Note: CLf/CLr are [numel(RHf) x numel(RHr)] (rows=RHf, cols=RHr), so the
% interp2 call is (X, Y, Z, Xq, Yq) = (RHr, RHf, CLf/CLr, RHrC, RHfC) without transposing.
ClF_base = interp2(rawMap.RHr, rawMap.RHf, rawMap.CLf, RHrC, RHfC, 'linear');
ClR_base = interp2(rawMap.RHr, rawMap.RHf, rawMap.CLr, RHrC, RHfC, 'linear');

% ---- 2) rear-wing node-delta blend at alphaRW ----------------------------
aRWmin = vp.aeroARW.alphaNodes(1);
aRWmax = vp.aeroARW.alphaNodes(end);
if any(alphaRW(:) < aRWmin - TOL_DEG) || any(alphaRW(:) > aRWmax + TOL_DEG)
    bad = alphaRW(find(alphaRW(:) < aRWmin - TOL_DEG | alphaRW(:) > aRWmax + TOL_DEG, 1));
    error('aeroLiveEval:rangeRW', ...
        'rear-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        bad, aRWmin, aRWmax);
end
alphaRW = min(max(alphaRW, aRWmin), aRWmax);      % clamp the tolerated bound slack

Lrw = aeroLiveEval_cardinalWeights(vp.aeroARW.basis, alphaRW);
dClF_rw = zeros(size(alphaRW)); dClR_rw = zeros(size(alphaRW)); dCdA_rw = zeros(size(alphaRW));
for i = 1:numel(vp.aeroARW.dClF)
    dClF_rw = dClF_rw + Lrw{i}.*vp.aeroARW.dClF(i);
    dClR_rw = dClR_rw + Lrw{i}.*vp.aeroARW.dClR(i);
    dCdA_rw = dCdA_rw + Lrw{i}.*vp.aeroARW.dCdA(i);
end

% ---- 3) front-wing node-delta blend at alphaFW (if the layer is present) -
if hasFW
    aFWmin = vp.aeroAFW.alphaNodes(1);
    aFWmax = vp.aeroAFW.alphaNodes(end);
    if any(alphaFW(:) < aFWmin - TOL_DEG) || any(alphaFW(:) > aFWmax + TOL_DEG)
        bad = alphaFW(find(alphaFW(:) < aFWmin - TOL_DEG | alphaFW(:) > aFWmax + TOL_DEG, 1));
        error('aeroLiveEval:rangeFW', ...
            'front-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
            bad, aFWmin, aFWmax);
    end
    alphaFW = min(max(alphaFW, aFWmin), aFWmax);  % clamp the tolerated bound slack

    Lfw = aeroLiveEval_cardinalWeights(vp.aeroAFW.basis, alphaFW);
    dClF_fw = zeros(size(alphaFW)); dClR_fw = zeros(size(alphaFW)); dCdA_fw = zeros(size(alphaFW));
    for i = 1:numel(vp.aeroAFW.dClFadd)
        dClF_fw = dClF_fw + Lfw{i}.*vp.aeroAFW.dClFadd(i);
        dClR_fw = dClR_fw + Lfw{i}.*vp.aeroAFW.dClRadd(i);
        dCdA_fw = dCdA_fw + Lfw{i}.*vp.aeroAFW.dCdA(i);
    end
else
    dClF_fw = zeros(size(alphaRW)); dClR_fw = zeros(size(alphaRW)); dCdA_fw = zeros(size(alphaRW));
end

% ---- 4) assemble outputs -------------------------------------------------
ClF = ClF_base + dClF_rw + dClF_fw;
ClR = ClR_base + dClR_rw + dClR_fw;
dCdA_sel = dCdA_rw + dCdA_fw;
Cd  = vp.Cd0 + dCdA_sel/vp.A;

Fdrag0  = 0.5.*vp.rho.*vp.A.*vx.^2.*vp.Cd0;
FdragRW = 0.5.*vp.rho.*vp.A.*vx.^2.*(Cd - vp.Cd0);

end

% =============================================================================
function Li = aeroLiveEval_cardinalWeights(basis, alpha)
%AEROLIVEEVAL_CARDINALWEIGHTS Cardinal basis weights L_i(alpha) for the
% hermite3tanh basis (Functions/rwAeroMap2D.m). Port of Functions/
% rwAeroMapEvalNum.m's local rwBasisNum, hermite3tanh case only (the only
% basis kind vp.aeroARW/vp.aeroAFW ship with - Scripts/vehModel.m's SX block
% asserts on this too). Returns a 1xn cell of arrays the size of alpha.
% Codegen-friendly pure arithmetic: tanh, multiply, add, Horner - no
% interpolant objects, no toolbox calls beyond Functions/rwBasisWeights.m.
assert(strcmp(basis.kind, 'hermite3tanh'), 'aeroLiveEval:basis', ...
    'only the hermite3tanh basis is implemented (got ''%s'')', basis.kind);
kn = basis.knots;
n  = numel(kn);
w  = rwBasisWeights(kn, basis.wb, alpha);     % Functions/rwBasisWeights.m (shared owner)
Li = cell(1, n);
for i = 1:n
    Li{i} = zeros(size(alpha));
    for j = 1:(n-1)
        Li{i} = Li{i} + w{j}.*aeroLiveEval_hornEval(basis.C(j,:,i), alpha - kn(j));
    end
end
end

% =============================================================================
function y = aeroLiveEval_hornEval(p, x)
% Horner evaluation, highest-order coefficient first - identical formula to
% Functions/aeroEvalNum.m / Functions/rwAeroMapEvalNum.m's hornEval.
y = zeros(size(x)) + p(1);
for k = 2:numel(p)
    y = y.*x + p(k);
end
end

% =============================================================================
function sz = aeroLiveEval_broadcastSize(varargin)
% Common output size for scalar-or-equal-sized-array inputs.
sz = [1 1];
for k = 1:numel(varargin)
    s = size(varargin{k});
    if ~isequal(s, [1 1])
        if ~isequal(sz, [1 1]) && ~isequal(sz, s)
            error('aeroLiveEval:size', 'inputs must be scalars or equal-sized arrays');
        end
        sz = s;
    end
end
end

% =============================================================================
function y = aeroLiveEval_expandTo(x, sz)
% Expand a scalar to sz; pass an already-matching array through unchanged.
if isequal(size(x), sz) || isequal(sz, [1 1])
    y = x;
elseif isscalar(x)
    y = x + zeros(sz);
else
    error('aeroLiveEval:size', 'input of size %s cannot expand to %s', ...
        mat2str(size(x)), mat2str(sz));
end
end
