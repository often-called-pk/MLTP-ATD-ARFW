%% tyreParams_Synthetic.m - SYNTHETIC Pacejka MF5.2 tyre parameters
%
% =========================================================================
%  THESE NUMBERS ARE INVENTED. They do not describe any real tyre, and they
%  are not measurement data from any manufacturer or supplier.
% =========================================================================
%
% They exist so this repository runs end to end without shipping anyone's
% confidential tyre data. They are physically self-consistent and produce a
% plausible lap, but any lap time, grip level or balance conclusion drawn
% from them is a property of THIS FILE, not of a real vehicle. Replace it
% with your own measured set before drawing engineering conclusions.
%
% Structure matches what tyreMF() in Scripts/vehModel.m consumes (numeric
% numeric twin). Per axle:
%   Fz0                     nominal load FNOMIN                       (N)
%   pDx1 pDx2 pDx3          long. peak friction: mu = pDx1 + pDx2*dfz
%   pDy1 pDy2 pDy3          lat.  peak friction: mu = pDy1 + pDy2*dfz
%                           (pDx3/pDy3 are camber terms, dormant: the 7DoF
%                            model has no camber DOF)
%   Bx Cx Ex / By Cy Ey     pure-slip magic-formula shape factors
%   rBx1 rBx2 rCx1 rEx1     combined-slip weighting Gxa
%   rBy1 rBy2 rCy1 rEy1     combined-slip weighting Gyk
%   qsy1                    rolling resistance, f_roll = qsy1*Fz
%   Bt Ct Dt Et             aligning moment - NOT used by this model
%
% dfz = (Fz - Fz0)/Fz0. All MF shift and vertical-shift terms are zero, so
% the tyre is symmetric and G(SH)=1, SVyk=0 drop out of tyreMF().
%
% DESIGN NOTE - load sensitivity. Peak grip FORCE mu(Fz)*Fz turns over at
%   Fz_turn = (Fz0/2)*(1 - pD1/pD2)
% and pD2 is chosen so every turnover sits WELL ABOVE the per-tyre load cap
% vp.Fz_abs (12.5 kN). Below the turnover, added downforce still buys grip.
% If you retune these, keep that property or the optimiser will discover
% that downforce hurts and the solution will stop making physical sense.

% ---------------- Front axle ----------------
tyf.Fz0  = 3400;                 % ~static front corner load               (N)

% peak friction, load sensitivity
tyf.pDx1 = 1.75;  tyf.pDx2 = -0.20;  tyf.pDx3 = 0;   % turnover ~16.6 kN
tyf.pDy1 = 1.65;  tyf.pDy2 = -0.18;  tyf.pDy3 = 0;   % turnover ~17.3 kN

% pure-slip curve shape
tyf.Bx = 12.0;  tyf.Cx = 1.65;  tyf.Ex = -0.40;
tyf.By = 16.0;  tyf.Cy = 1.30;  tyf.Ey = -1.20;

% combined-slip weighting
tyf.rBx1 = 13.0;  tyf.rBx2 = 9.5;  tyf.rCx1 = 1.05;  tyf.rEx1 = -0.20;
tyf.rBy1 = 10.5;  tyf.rBy2 = 7.5;  tyf.rCy1 = 1.05;  tyf.rEy1 = -0.30;

% rolling resistance
tyf.qsy1 = 0.010;

% aligning moment (unused by the MLTP model, kept for structural parity)
tyf.Bt = 9.0;  tyf.Ct = 1.15;  tyf.Dt = 12.0;  tyf.Et = -1.50;

% ---------------- Rear axle ----------------
tyr.Fz0  = 4500;                 % ~static rear corner load                (N)

tyr.pDx1 = 1.72;  tyr.pDx2 = -0.20;  tyr.pDx3 = 0;   % turnover ~21.6 kN
tyr.pDy1 = 1.62;  tyr.pDy2 = -0.18;  tyr.pDy3 = 0;   % turnover ~22.5 kN

tyr.Bx = 11.5;  tyr.Cx = 1.65;  tyr.Ex = -0.40;
tyr.By = 15.0;  tyr.Cy = 1.30;  tyr.Ey = -1.20;

tyr.rBx1 = 12.5;  tyr.rBx2 = 9.5;  tyr.rCx1 = 1.05;  tyr.rEx1 = -0.20;
tyr.rBy1 = 10.0;  tyr.rBy2 = 7.5;  tyr.rCy1 = 1.05;  tyr.rEy1 = -0.30;

tyr.qsy1 = 0.010;

tyr.Bt = 8.5;  tyr.Ct = 1.15;  tyr.Dt = 14.0;  tyr.Et = -1.50;

% ---------------- attach to vp ----------------
vp.tyre_f = tyf;
vp.tyre_r = tyr;
clear tyf tyr
