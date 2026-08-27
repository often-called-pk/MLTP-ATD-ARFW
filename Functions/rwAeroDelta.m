function [dClfA, dClrA, dCdA] = rwAeroDelta(alpha)
%RWAERODELTA Force-equivalent aero coefficient*area deltas vs rear-wing angle.
%
%   [dClfA, dClrA, dCdA] = rwAeroDelta(alpha) returns the change, relative to the
%   0 deg setting, of the products (aero coefficient x frontal area) for front
%   lift, rear lift and drag, at rear-wing angle(s) alpha [deg]. alpha may be
%   scalar or vector; every element must lie in [-10, 15].
%
%   Negative Cl is downforce.
%
%   Why products, not coefficients. The source sweep measures a different frontal
%   area at each angle, while the vehicle model holds a FIXED reference area
%   (vp.A) and a fixed baseline vp.Cd0. Differencing raw coefficients would
%   therefore not reproduce the measured forces. Differencing the products does:
%       CdA = Cd_abs.*A_abs;  dCdA = CdA - CdA(0 deg);   (likewise Clf, Clr)
%   The caller divides by vp.A to get effective coefficient increments, so
%   0.5*rho*(coefficient*area increment)*vx^2 recovers the measured force. Only
%   deltas are used - the sweep's absolute Cd/area/Cl baselines are not adopted.
%   At alpha = 0 all three deltas are exactly zero by construction.
%
%   Interpolation is shape-preserving pchip through the four nodes, node-exact,
%   and errors outside [-10, 15].
%
%   Stall. The wing is linear over -10..+10 deg and separates between +10 and
%   +15. That is captured by the +15 data point itself, not by a switch.
%
%   Known data inconsistency at -10 deg. The source prints a rear-lift delta of
%   +0.459 relative to 0 deg, giving Clr_abs(-10) = -0.291, but its own Cl sum
%   and quoted aero balance instead imply Clr_abs(-10) ~ +0.280. The printed
%   delta is the one adopted here; the discrepancy is recorded rather than
%   silently reconciled, because which is correct has never been resolved.

% Digitised absolutes from the supplier rear-wing angle sweep. Products and
% deltas are derived below rather than stored pre-rounded.
angles  = [-10     0      10     15   ];
Cd_abs  = [ 0.479  0.540  0.610  0.631];
Clf_abs = [-0.442 -0.371 -0.330 -0.344];
Clr_abs = [-0.291 -0.750 -0.983 -0.812];
A_abs   = [ 2.072  2.095  2.132  2.150];   % m^2, varies with angle

if any(alpha(:) < angles(1)) || any(alpha(:) > angles(end))
    error('rwAeroDelta:range', ...
        'rear-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        alpha(find(alpha(:) < angles(1) | alpha(:) > angles(end), 1)), angles(1), angles(end));
end

% Force-equivalent products (coefficient x that row's own frontal area).
CdA  = Cd_abs  .* A_abs;
ClfA = Clf_abs .* A_abs;
ClrA = Clr_abs .* A_abs;

% Deltas relative to the 0 deg node, so that node returns exactly zero.
i0 = find(angles == 0, 1);
dCdA_n  = CdA  - CdA(i0);
dClfA_n = ClfA - ClfA(i0);
dClrA_n = ClrA - ClrA(i0);

dClfA = pchip(angles, dClfA_n, alpha);
dClrA = pchip(angles, dClrA_n, alpha);
dCdA  = pchip(angles, dCdA_n,  alpha);

end
