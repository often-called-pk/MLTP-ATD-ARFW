function [dClfA, dClrA, dCdA] = fwOnlyDelta(alpha)
%FWONLYDELTA Front-wing-ONLY force-equivalent product deltas vs FW 0 deg.
%   Change of (coef x frontal area) products [m2] for front lift, rear lift
%   and drag at FW flap angle alpha [deg], RW contribution EXCLUDED (contrast
%   fwAeroDelta.m, whose deployed nodes carry RW -17.5 implicitly). alpha
%   scalar or vector, valid on [-25, 0].
%   Measured-chain recipe (spec 2026-08-09 S2, measured CFD differences only,
%   never the synthetic "Expected" row):
%     FWonly(-20) = R109 - R105              (CFD_R105_Family, both RW 0)
%     FWonly(-25) = FWonly(-20) + (R131-R130) (CFD_NewBaseline, both RW -17.5)
%     FWonly(0)   = 0 exactly, by construction.
%   Products convention: caller divides by fixed vp.A ONCE (lift at map build,
%   drag at eval - spec S2). Positive lift delta = unload (less downforce).
%   Source: reference/aero/additional info regarding front Aero/
%   the supplier's digitised front-aero workbook (not distributed).

angles = [-25 -20 0];

% workbook absolutes (digit-exact) - node deltas computed below, never pre-rounded
% columns:   R105    R109    R130    R131
Cd_abs  = [ 0.558   0.541   0.451   0.448];
Clf_abs = [-0.466  -0.164  -0.158  -0.097];
Clr_abs = [-0.739  -0.982  -0.320  -0.342];
A_abs   = [ 2.095   2.095   2.061   2.061];

if any(alpha(:) < angles(1)) || any(alpha(:) > angles(end))
    error('fwOnlyDelta:range', ...
        'FW flap angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        alpha(find(alpha(:) < angles(1) | alpha(:) > angles(end), 1)), angles(1), angles(end));
end

% force-equivalent products (coefficient x that run's own frontal area)
CdA  = Cd_abs  .* A_abs;
ClfA = Clf_abs .* A_abs;
ClrA = Clr_abs .* A_abs;

% measured-chain node deltas; columns of *_abs: 1=R105 2=R109 3=R130 4=R131
dClfA_n = [ClfA(2)-ClfA(1) + ClfA(4)-ClfA(3), ClfA(2)-ClfA(1), 0];
dClrA_n = [ClrA(2)-ClrA(1) + ClrA(4)-ClrA(3), ClrA(2)-ClrA(1), 0];
dCdA_n  = [CdA(2)-CdA(1) + CdA(4)-CdA(3), CdA(2)-CdA(1), 0];

% shape-preserving cubic through the delta-product nodes (node-exact)
dClfA = pchip(angles, dClfA_n, alpha);
dClrA = pchip(angles, dClrA_n, alpha);
dCdA  = pchip(angles, dCdA_n,  alpha);

end
