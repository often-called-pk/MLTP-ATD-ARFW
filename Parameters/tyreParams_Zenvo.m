%% tyreParams_Zenvo.m - Zenvo supplier Pacejka MF5.2 tyre parameters
%
% Published with permission of Zenvo Automotive, 2026-09.
%
% Populates vp.tyre_f / vp.tyre_r, consumed by tyreMF() in Scripts/vehModel.m,
% the axle-lumped model in Scripts/vehModel_initial.m, and the grip-utilisation
% post-processing in the MLTP entry points. Run from Parameters/vehParams.m.
%
% Notes on the data (verification record 2026-07-18):
%  - Micro sets (pD*1/pD*2 + Fz0) agree with the supplier's macro peak forces
%    within -1.1% (front) / +1.5% (rear); micro is used as the source of truth
%    for D = mu(dfz)*Fz, macro B/C/E are used for the curve shape.
%  - No PKX/PKY (slip-stiffness load laws) were supplied, so B,C,E are held
%    constant with load: slip stiffness K = B*C*D scales ~proportionally with
%    Fz instead of saturating. Grip-limited behaviour is unaffected; treat
%    linear-range balance conclusions (paramOptim) with caution. Data request
%    to Zenvo pending.
%  - Camber sensitivities pDx3/pDy3 are carried for reference but DORMANT (the
%    7DoF model has no camber DOF). Front values have a suspect sign - query
%    Zenvo before ever enabling camber.
%  - The supplier's By values are negative (their slip-angle sign convention);
%    magnitudes are stored here because this model's convention (vehModel.m
%    slip-angle definitions) is positive slip angle -> positive lateral force.
%    Lossless: all shift/vertical-shift MF terms are zero, the tyre is symmetric.

% ---------------- Front axle ----------------
tyf.Fz0  = 5000;                    % nominal load FNOMIN                  (N)

% peak friction, load sensitivity: mu = pD1 + pD2*dfz, dfz = (Fz-Fz0)/Fz0
tyf.pDx1 = 1.485;  tyf.pDx2 = -0.205;  tyf.pDx3 = -0.1;   % longitudinal (pDx3: camber^2, dormant)
tyf.pDy1 = 1.35;   tyf.pDy2 = -0.186;  tyf.pDy3 = -0.1;   % lateral      (pDy3: camber^2, dormant)

% pure-slip curve shape (supplier macro fit at FNOMIN)
tyf.Bx = 21.5;   tyf.Cx = 1.324;   tyf.Ex = -0.725;
tyf.By = 15.5;   tyf.Cy = 1.324;   tyf.Ey = -0.725;       % |By|, see header

% combined-slip weighting (MF5.2 Gxa/Gyk; all shift terms RHX1/RHY1/RHY2 and
% vertical terms RVY1-6 are zero, REX2 = 0)
tyf.rBx1 = 16.38;  tyf.rBx2 = 22.8;   tyf.rCx1 = 1.0;  tyf.rEx1 = -0.5;
tyf.rBy1 = 22.8;   tyf.rBy2 = 16.38;  tyf.rCy1 = 1.0;  tyf.rEy1 = -0.5;

% rolling resistance (QSY2-4 zero): body-force equivalent f_roll = qsy1*Fz
tyf.qsy1 = 0.0075;

% aligning moment, trail-based macro set - NOT used by the 7DoF model (no
% steer-torque path); kept for reference/Simulink use
tyf.Bt = 12.45;  tyf.Ct = 1.11;   tyf.Dt = 213;  tyf.Et = -1.89;   % Dt (Nm)

% ---------------- Rear axle ----------------
tyr.Fz0  = 6050;                    % nominal load FNOMIN                  (N)

tyr.pDx1 = 1.467;  tyr.pDx2 = -0.22;    tyr.pDx3 = 3.55;
tyr.pDy1 = 1.33;   tyr.pDy2 = -0.2017;  tyr.pDy3 = 3.55;

tyr.Bx = 22.9;   tyr.Cx = 1.368;   tyr.Ex = -0.052;
tyr.By = 16.8;   tyr.Cy = 1.368;   tyr.Ey = -0.052;       % |By|, see header

tyr.rBx1 = 18.37;  tyr.rBx2 = 25.0;   tyr.rCx1 = 1.0;  tyr.rEx1 = -0.5;
tyr.rBy1 = 25.0;   tyr.rBy2 = 18.37;  tyr.rCy1 = 1.0;  tyr.rEy1 = -0.5;

tyr.qsy1 = 0.0075;

tyr.Bt = 18.77;  tyr.Ct = 1.102;  tyr.Dt = 311;  tyr.Et = -0.626;  % Dt (Nm)

% ---------------- attach to vp ----------------
vp.tyre_f = tyf;
vp.tyre_r = tyr;
clear tyf tyr
