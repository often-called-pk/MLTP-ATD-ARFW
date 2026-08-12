function [dClfA, dClrA, dCdA] = fwAeroDelta(alpha)
%FWAERODELTA Combined FW(2nd-element)+RW unload-axis force deltas vs R128.
%   Force-equivalent product deltas [m2] vs the R128 state (FW 0/RW 0) at
%   FW flap angle alpha [deg]. Deployed nodes carry RW -17.5 deg implicitly
%   (CFD runs R130/R131) - these are NOT FW-only deltas.
%   Source: reference/aero/additional info regarding front Aero/
%   FrontAero_Digitised_DoNotPublish.xlsx (verified digit-exact 2026-08-07).
%   Nodes: -25 (R131), -20 (R130), 0 (R128).

angles = [-25 -20 0];
Cd_abs  = [ 0.448  0.451  0.558];
Clf_abs = [-0.097 -0.158 -0.494];
Clr_abs = [-0.342 -0.320 -0.727];
A_abs   = [ 2.061  2.061  2.094];

if any(alpha(:) < angles(1)) || any(alpha(:) > angles(end))
    error('fwAeroDelta:range', ...
        'FW axis angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        alpha(find(alpha(:) < angles(1) | alpha(:) > angles(end), 1)), angles(1), angles(end));
end

CdA  = Cd_abs  .* A_abs;
ClfA = Clf_abs .* A_abs;
ClrA = Clr_abs .* A_abs;
i0 = find(angles == 0, 1);
dClfA = pchip(angles, ClfA - ClfA(i0), alpha);
dClrA = pchip(angles, ClrA - ClrA(i0), alpha);
dCdA  = pchip(angles, CdA  - CdA(i0),  alpha);
end
