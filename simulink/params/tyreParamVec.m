function v = tyreParamVec(ty)
%TYREPARAMVEC Pack one per-axle MF5.2 tyre struct (vp.tyre_f / vp.tyre_r)
% into the fixed-order numeric vector that Plant.slx's `tyreStep` MATLAB
% Function block indexes.
%
%   v = tyreParamVec(vp.tyre_f)     % 19-by-1
%
% WHY THIS EXISTS. The Zenvo Pacejka coefficients are confidential supplier
% data (Parameters/tyreParams_DoNotPublish.m). They must never be
% written into a Simulink dialog, because the .slx would then CARRY them.
% Plant.slx therefore holds only the EXPRESSION
%     [vp.Rw; vp.Jw; VXLOW; tyreParamVec(vp.tyre_f); tyreParamVec(vp.tyre_r)]
% which resolves against the base workspace at model load, exactly the same
% by-name pattern the model's other Params constants use for vp/pt/sus/act.
%
% ORDER IS LOAD-BEARING -- it is duplicated, by index, inside the tyreStep
% MATLAB Function block's source. Change one and you MUST change the other.
% The order matches the sequence the fields are consumed in
% Functions/tyreMFnum.m:
%
%    1  Fz0     nominal load [N]              dfz = (fz - Fz0)/Fz0
%            Packed RAW, i.e. the plant assumes Fz0_shift_f/r == 1. The offline
%            model scales it (Fz0eff = tyre.Fz0*vp.Fz0_shift, vehModel.m:643-644)
%            and only MLTP_TyreOptim.m ever makes those shifts design variables,
%            where they are 1 in every normal run. This matches the gate's own
%            reference call, tyreMFnum(ty, ty.Fz0, ...), so plant and gate agree
%            by construction. A run that optimises Fz0_shift would have to thread
%            it through here.
%    2  pDx1    peak mu_x at nominal load     mux = pDx1 + pDx2*dfz
%    3  pDx2    load sensitivity of mu_x
%    4  Bx      longitudinal stiffness factor (held constant with load)
%    5  Cx      longitudinal shape factor
%    6  Ex      longitudinal curvature factor
%    7  pDy1    peak mu_y at nominal load     muy = pDy1 + pDy2*dfz
%    8  pDy2    load sensitivity of mu_y
%    9  By      lateral stiffness factor (held constant with load)
%   10  Cy      lateral shape factor
%   11  Ey      lateral curvature factor
%   12  rBx1    combined-slip Gxa stiffness
%   13  rBx2    combined-slip Gxa slip-ratio weighting
%   14  rCx1    combined-slip Gxa shape
%   15  rEx1    combined-slip Gxa curvature
%   16  rBy1    combined-slip Gyk stiffness
%   17  rBy2    combined-slip Gyk slip-angle weighting
%   18  rCy1    combined-slip Gyk shape
%   19  rEy1    combined-slip Gyk curvature
%
% No value is printed, logged or returned in any other form.

names = {'Fz0','pDx1','pDx2','Bx','Cx','Ex', ...
         'pDy1','pDy2','By','Cy','Ey', ...
         'rBx1','rBx2','rCx1','rEx1','rBy1','rBy2','rCy1','rEy1'};

missing = names(~isfield(ty, names));
if ~isempty(missing)
    error('tyreParamVec:missingField', ...
        'tyre struct is missing field(s): %s', strjoin(missing, ', '));
end

v = zeros(numel(names), 1);
for k = 1:numel(names)
    x = ty.(names{k});
    if ~isnumeric(x) || ~isscalar(x) || ~isfinite(x)
        error('tyreParamVec:badField', ...
            'tyre field %s is not a finite numeric scalar', names{k});
    end
    v(k) = x;
end
end
