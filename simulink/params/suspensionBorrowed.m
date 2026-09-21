function sus = suspensionBorrowed()
%SUSPENSIONBORROWED Borrowed damper + anti-roll-bar rates for the ARFWr RT
%model. Parameters/vehParams.m has NO damper data at all and NO anti-roll-
%bar (ARB) data - vp.kwf/vp.kwr (:72-73) are wheel rates only, vp.ks_rad
%(:76) is spring-ONLY roll stiffness, and the file's own comment at :77
%flags ARBs as missing supplier data ("a hypercar will have ARBs, so this
%underestimates total roll stiffness. Flag as a data request."). This
%function fills that gap by SCALING a published formula onto our vehicle's
%own numbers. It does not run vehParams.m (deliberately: this file is usable
%without a live MATLAB session) - every raw input below is a
%frozen literal, hand-copied from Parameters/vehParams.m as it stood on
%2026-08-31, cited by line number.
%
%   sus = suspensionBorrowed()
%
% Returns a struct: cDamp_f, cDamp_r (N.s/m per corner, AT THE WHEEL),
% kArb_f, kArb_r (Nm/rad, per axle), notes (a one-paragraph summary of the
% method below, for programmatic/report access).
%
% =====================================================================
%  SOURCE (step 1)
% =====================================================================
% OptimumG "Tech Tip: Springs & Dampers" series, Parts 1-6, by Matt
% Giaraffa (Part 4 co-authored with Samuel Brisson), OptimumG LLC.
% Publicly hosted PDFs (verified by direct read 2026-08-31):
%   Part 2 "Attack of the Units":        optimumg.com/wp-content/uploads/2020/01/SpringsDampers_Tech_Tip_2.pdf
%   Part 3 "Revenge of the Damping Ratio": optimumg.com/wp-content/uploads/2020/01/SpringsDampers_Tech_Tip_3.pdf
% Chosen because it is a named, citable, publicly-hosted engineering
% source (matches the brief's own candidate list: "OptimumG tech tips")
% that supplies BOTH a critical-damping-ratio formula with a worked SI
% numeric example (Part 3) AND a roll-gradient/ARB-sizing formula with
% published target bands (Part 2) - everything step 2 needs, from one
% coherent source, rather than stitching two unrelated papers together.
% A search for a GT3/FSAE worked-example paper with BOTH damper rates and
% ARB stiffness in one place did not turn up a comparably complete, freely
% -readable source, so OptimumG's own series (candidate #1 in the brief)
% is used for both quantities below.
%
% ---- DAMPERS, verbatim from Part 3 pp.3-4 ----
%   Ccr = 2*sqrt(Kw*msm)              [N.s/m]   (Kw = wheel rate, N/m;
%                                                 msm = corner sprung mass, kg)
%   zeta = C/Ccr
%   worked SI example (p.3): msm=300 kg, Kw=90000 N/m -> Ccr=10392 N.s/m
%   "In racecars, 0.65 to 0.70 is a good baseline [damping ratio]... this
%    provides much better body control than a passenger car (less
%    overshoot), and faster response than critical damping." (p.4)
% We use zeta = 0.65, the bottom (least-damped) end of that band — our
% judgment call, consistent with the ARB roll-gradient choice below.
%
% ---- ANTI-ROLL BARS, verbatim from Part 2 p.2-3 (+ Part 4 correction) ----
%   KphiDES = W*H/(phi/Ay)            [Nm/deg roll]
%     W = vehicle weight (N), H = CG-to-roll-axis height (m),
%     phi/Ay = target roll gradient (deg/g)
%   "0.2 - 0.7 deg/g for stiff higher downforce cars" / "1.0 - 1.8 deg/g
%    for low downforce sedans" (p.2, roll-gradient guidance table)
%   Part 2's ORIGINAL KphiDES formula had an extra "+ KphiF + KphiR" term
%   that Part 4 (p.1) explicitly retracts as an error ("In Spring &
%   Dampers, Part Two this equation is incorrect... It should read:
%   KphiDES = WH/(phi/Ay)"); the corrected, term-free version is used here.
% Our vehicle is this project's active-aero hypercar (high downforce at
% speed), so it belongs in the "stiff higher downforce cars" 0.2-0.7 deg/g band.
%
% =====================================================================
%  STEP 2 DERIVATION - scaled onto OUR vehicle
% =====================================================================

%% ---- frozen literal inputs, hand-copied from Parameters/vehParams.m ----
kwf = 40000;                        % BORROWED(vehParams.m:72): front wheel rate (N/m)
kwr = 50000;                        % BORROWED(vehParams.m:73): rear  wheel rate (N/m)
ksf_rad = 0.5*kwf*1.74^2;           % derived, BORROWED(vehParams.m:74) formula: front
                                     % spring-only roll stiffness (Nm/rad). 1.74 is
                                     % track width (m), cf. vehParams.m:49.
                                     % ~60,552 Nm/rad (matches vehParams.m's inline comment)
ksr_rad = 0.5*kwr*1.67^2;           % derived, BORROWED(vehParams.m:75) formula: rear
                                     % spring-only roll stiffness (Nm/rad). 1.67 is
                                     % track width (m), cf. vehParams.m:49.
                                     % ~69,722 Nm/rad (matches vehParams.m's inline comment)
ks_rad = ksf_rad + ksr_rad;         % derived, BORROWED(vehParams.m:76) formula: combined
                                     % spring-only roll stiffness (Nm/rad). ~130,275 Nm/rad.

mb = 1327;                          % BORROWED(vehParams.m:40): sprung mass excl. driver (kg)
md = 75;                            % BORROWED(vehParams.m:41): driver mass (kg)
ms = mb + md;                       % derived, BORROWED(vehParams.m:42) formula: total
                                     % sprung mass (kg) = 1402 kg

wB = 0.43;                          % BORROWED(vehParams.m:52): front sprung-mass fraction
                                     % (-). vehParams.m:97-100 uses wB directly as the
                                     % FRONT fraction multiplying vp.ms in the static
                                     % sprung wheel-load terms, so wB is the front (not
                                     % rear) mass share despite the l_f formula's (1-wB).
ksD = 0.535;                        % BORROWED(vehParams.m:21): target COMBINED
                                     % (spring+ARB) rear roll-stiffness fraction (-)
                                     % Note: only the numeric value is reused; vp.ksD's downstream
                                     % chain (ksf/ksr/ks, vehParams.m:69-71) is marked DEAD and not built on here.

g   = 9.81;                         % BORROWED(vehParams.m:35)
hcg = 0.44;                         % BORROWED(vehParams.m:58): CG height (m)
hRCf = 0.07;                        % BORROWED(vehParams.m:64): front roll-centre height (m)
hRCr = 0.11;                        % BORROWED(vehParams.m:65): rear  roll-centre height (m)
l   = 2.8;                          % BORROWED(vehParams.m:50): wheelbase (m)

l_f = l*(1-wB);                     % derived, BORROWED(vehParams.m:55) formula: CoG->front axle (m)
l_r = l*wB;                         % derived, BORROWED(vehParams.m:56) formula: CoG->rear  axle (m)
hRC = (l_f*hRCr + l_r*hRCf)/l;      % derived, BORROWED(vehParams.m:66) formula: roll-axis
                                     % height at CoG (m)
d   = hcg - hRC;                    % derived, BORROWED(vehParams.m:67) formula: CoG-to-
                                     % roll-axis height (m). This is exactly OptimumG's "H".

%% =====================================================================
%  DAMPERS - OptimumG Part 3 (see header)
%  Ccr = 2*sqrt(Kw*msm); zeta = C/Ccr; racecar baseline zeta = 0.65-0.70.
%  Corner sprung mass = axle sprung-mass fraction * ms / 2 (per corner).
%  wB / (1-wB) are SPRUNG-mass fractions (same use as vehParams.m:97-100's
%  static sprung-load terms), so no unsprung mass is added here - cDamp is
%  a sprung-mass/wheel-rate quantity by the source formula, matching the
%  "at wheel" interface. Unsprung-mass (bump) damping is a separate mode
%  the source treats independently (Part 3 p.4, "single wheel bump") and
%  is out of scope for this task.
%% =====================================================================
zeta = 0.65;                        % BORROWED(OptimumG Pt.3 p.4): bottom of the
                                     % racecar baseline 0.65-0.70 damping-ratio band

msm_f = wB*ms/2;                    % derived: front corner sprung mass (kg), ~301.4 kg
msm_r = (1-wB)*ms/2;                % derived: rear  corner sprung mass (kg), ~399.6 kg

Ccrit_f = 2*sqrt(kwf*msm_f);        % BORROWED(OptimumG Pt.3 p.3) formula: Ccr = 2*sqrt(Kw*msm)
Ccrit_r = 2*sqrt(kwr*msm_r);
cDamp_f = zeta*Ccrit_f;             % derived: ~4.5 kN.s/m (hand check, 2026-08-31)
cDamp_r = zeta*Ccrit_r;             % derived: ~5.8 kN.s/m (hand check, 2026-08-31)

%% =====================================================================
%  ANTI-ROLL BARS - OptimumG Part 2 (+ Part 4 correction)
%% =====================================================================
rollGrad_degPerG = 0.5;             % BORROWED(OptimumG Pt.2 p.2, judgment call WITHIN the
                                     % published band): target roll gradient for a "stiff
                                     % higher downforce car" (published band 0.2-0.7 deg/g).
                                     % 0.5 deg/g is the round midpoint of that band - no
                                     % absolute Nm/deg figure is published for THIS vehicle
                                     % (Zenvo never supplied ARB data, vehParams.m:77), so
                                     % a band-midpoint pick is the closest defensible
                                     % substitute. Sensitivity: even the softest end of the
                                     % band (0.7 deg/g) still demands roughly 3x the
                                     % spring-only roll stiffness in ARB alone (see below),
                                     % because the spring-only setup computes to ~2.1 deg/g
                                     % (softer than even the "low downforce sedan" 1.0-1.8
                                     % deg/g band) - consistent with vehParams.m:77's own
                                     % flag that the model underestimates roll stiffness
                                     % without ARB data.

W = ms*g;                           % derived: sprung weight (N)
H = d;                              % CG-to-roll-axis height (m) - same quantity vehParams.m
                                     % calls vp.d

KphiDES_deg = W*H/rollGrad_degPerG; % BORROWED(OptimumG Pt.2 p.3, Pt.4-corrected) formula:
                                     % KphiDES = W*H/(phi/Ay), combined (spring+ARB) target
                                     % roll stiffness (Nm/deg)
KphiDES_rad = KphiDES_deg*(180/pi); % derived: Nm/deg -> Nm/rad, to match vp.ks_rad's units

% Combined (spring+ARB) roll stiffness must exceed the spring-only figure -
% if it doesn't, the chosen roll gradient is looser than what the springs
% ALONE already deliver and no positive ARB rate would be needed. Assert
% loudly rather than silently return a negative ARB rate (repo convention:
% the scripts assert on dimension mismatches and error rather than
% silently misbehave).
assert(KphiDES_rad > ks_rad, 'suspensionBorrowed:arbNegative', ...
    ['Target combined roll stiffness (%.0f Nm/rad, from a %.2f deg/g gradient) is ' ...
     'below the spring-only figure (%.0f Nm/rad) - ARB would need to be negative. ' ...
     'Loosen the target roll gradient (rollGrad_degPerG).'], ...
    KphiDES_rad, rollGrad_degPerG, ks_rad);

% Split the COMBINED total by ksD ("keeping rear fraction ksD=0.535 of the
% COMBINED total", per the task brief - NOT a re-split of the ARB alone),
% then back out each axle's ARB contribution as that axle's combined
% target minus its OWN spring contribution (ksf_rad/ksr_rad). This is the
% general, robust construction: it guarantees the FINAL combined rear
% fraction is exactly ksD regardless of the spring-only split. It also
% happens to land close to a proportional ARB-only split here, because
% vehParams.m's own spring-only rear fraction (ksr_rad/ks_rad ~ 0.535, from
% the 1.74/1.67 motion-ratio factors) is itself already ~ksD - a property
% of THIS vehicle's numbers, not assumed by the code below.
combinedRear_rad  = ksD*KphiDES_rad;
combinedFront_rad = (1-ksD)*KphiDES_rad;
kArb_r = combinedRear_rad  - ksr_rad;   % derived: ~223 kNm/rad (hand check, 2026-08-31)
kArb_f = combinedFront_rad - ksf_rad;   % derived: ~194 kNm/rad (hand check, 2026-08-31)

assert(kArb_f > 0 && kArb_r > 0, 'suspensionBorrowed:arbAxleNegative', ...
    'Derived per-axle ARB rate is non-positive (kArb_f=%.0f, kArb_r=%.0f Nm/rad).', ...
    kArb_f, kArb_r);

%% =====================================================================
%  Assemble output struct
%% =====================================================================
sus.cDamp_f = cDamp_f;
sus.cDamp_r = cDamp_r;
sus.kArb_f  = kArb_f;
sus.kArb_r  = kArb_r;
sus.notes = sprintf([...
    'Dampers: Ccr = 2*sqrt(Kw*msm), zeta = %.2f (OptimumG Springs & Dampers Pt.3, ' ...
    'racecar baseline 0.65-0.70), applied at each corner''s wheel rate and sprung ' ...
    'mass; cDamp_f/r are AT-THE-WHEEL rates (N.s/m). ARB: combined (spring+ARB) roll ' ...
    'stiffness set to a %.2f deg/g target roll gradient (OptimumG Pt.2 + Pt.4 ' ...
    'correction, "stiff higher downforce cars" band 0.2-0.7 deg/g), split ' ...
    '(1-ksD)/ksD front/rear (ksD=%.3f, Parameters/vehParams.m:21) of the COMBINED ' ...
    'total, minus this checkout''s spring-only roll stiffness (Parameters/' ...
    'vehParams.m:74-76) per axle; kArb_f/r are Nm/rad. No live link to ' ...
    'Parameters/vehParams.m (HARD RULE: this task does not touch the MATLAB ' ...
    'session) - every input is a frozen literal hand-copied 2026-08-31; re-derive ' ...
    'if vehParams.m''s raw inputs change.'], ...
    zeta, rollGrad_degPerG, ksD);

end
