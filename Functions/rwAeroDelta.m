function [dClfA, dClrA, dCdA] = rwAeroDelta(alpha)
%RWAERODELTA Force-equivalent aero coefficient*area deltas vs rear-wing angle.
%
%   [dClfA, dClrA, dCdA] = rwAeroDelta(alpha) returns the change, relative to
%   the 0 deg rear-wing setting, of the products (aero coefficient x frontal
%   area) for front lift, rear lift and drag, at rear-wing angle(s) alpha [deg].
%   alpha may be scalar or a vector; every element must lie in [-10, 15].
%
%   Provenance
%     Zenvo "aerotak" slide 19 (rear-wing angle sweep), shared over WhatsApp on
%     2026-07-23, digitised + verified into reference/aero/whatsapp/. Absolute
%     values per rear-wing angle of attack:
%         angles  = [-10    0     10    15  ]  deg
%         Cd_abs  = [0.479  0.540 0.610 0.631]
%         Clf_abs = [-0.442 -0.371 -0.330 -0.344]
%         Clr_abs = [-0.291 -0.750 -0.983 -0.812]
%         A_abs   = [2.072  2.095 2.132 2.150]  m^2 (frontal area, varies)
%
%     The slide's +20 deg row (Cd_abs 0.676, Clf_abs -0.336, Clr_abs -0.877,
%     A_abs 2.166) was DROPPED as a node here on 2026-08-05, along with the
%     ActiveRW 2-D map's fifth node and the free wing's control bound (both
%     capped at +15) - see
%     docs/superpowers/specs/2026-08-05-four-node-map-and-report-fixes-design.md.
%     The digitised value itself is not lost: the source slide is still at
%     reference/aero/whatsapp/, and this file's git history prior to
%     2026-08-05 carries the row; it is simply no longer a live interpolation
%     node or part of the supported range.
%
%   Sign convention
%     Negative Cl = downforce (slide convention, carried through unchanged).
%
%   The -10 deg rear-lift decision
%     The slide prints a -10 deg rear delta of +0.459 relative to 0 deg, giving
%     Clr_abs(-10) = -0.291. The slide's own Cl sum and its "48.5% balance"
%     bullet instead imply Clr_abs(-10) ~ +0.280 - a known internal slide
%     inconsistency. Per explicit user decision (2026-07-23) the printed delta
%     +0.459 is adopted; the inconsistency is logged, not silently reconciled.
%
%   Force-equivalent product design
%     The vehicle model keeps a FIXED reference area (vp.A = 2.0) and a fixed
%     baseline vp.Cd0 = 0.40 (deltas-only decision: the slide's absolute
%     Cd/Area/Cl baselines are an open Zenvo question and are NOT adopted). To
%     reproduce the slide FORCES exactly despite its per-angle area variation,
%     the deltas are formed in PRODUCTS (coefficient x that row's own area):
%         CdA  = Cd_abs .*A_abs;  dCdA  = CdA  - CdA(0deg)
%         ClfA = Clf_abs.*A_abs;  dClfA = ClfA - ClfA(0deg)
%         ClrA = Clr_abs.*A_abs;  dClrA = ClrA - ClrA(0deg)
%     The caller divides these products by the code reference area vp.A to get
%     effective coefficient increments (dClF = dClfA/vp.A, etc.), so the added
%     downforce/drag = 0.5*rho*(coef*area increment)*vx^2 matches the slide.
%     At alpha = 0 all three deltas are EXACTLY zero by construction.
%
%   Interpolation and valid range
%     Shape-preserving pchip through the four delta-product nodes; valid on
%     [-10, 15] deg, error() outside it. pchip is node-exact, so alpha == 0
%     returns exactly zeros.
%
%   Stall note
%     The rear wing is linear over -10..+10 deg; it separates ("RW separated")
%     between 10 and 15 deg, flagged at the 15 deg point. The stall is
%     captured by the +15 data point itself, not by a switch. (The slide's
%     +20 deg point also showed the separated regime - Clr_abs recovering to
%     -0.877 from -0.812 at 15 deg - but is no longer a node; see the
%     Provenance note above.)

% raw absolutes (digitised) - products and deltas are computed here, no
% pre-rounded magic numbers
angles  = [-10     0      10     15   ];
Cd_abs  = [ 0.479  0.540  0.610  0.631];
Clf_abs = [-0.442 -0.371 -0.330 -0.344];
Clr_abs = [-0.291 -0.750 -0.983 -0.812];
A_abs   = [ 2.072  2.095  2.132  2.150];

if any(alpha(:) < angles(1)) || any(alpha(:) > angles(end))
    error('rwAeroDelta:range', ...
        'rear-wing angle out of range: %g deg requested, valid range is [%g, %g] deg', ...
        alpha(find(alpha(:) < angles(1) | alpha(:) > angles(end), 1)), angles(1), angles(end));
end

% force-equivalent products (coefficient x that row's frontal area)
CdA  = Cd_abs  .* A_abs;
ClfA = Clf_abs .* A_abs;
ClrA = Clr_abs .* A_abs;

% deltas relative to the 0 deg node -> exactly zero at that node
i0 = find(angles == 0, 1);
dCdA_n  = CdA  - CdA(i0);
dClfA_n = ClfA - ClfA(i0);
dClrA_n = ClrA - ClrA(i0);

% shape-preserving cubic through the delta-product nodes (node-exact)
dClfA = pchip(angles, dClfA_n, alpha);
dClrA = pchip(angles, dClrA_n, alpha);
dCdA  = pchip(angles, dCdA_n,  alpha);

end
