function act = actuatorParams()
%ACTUATORPARAMS Borrowed wing-actuator dynamics (first-order lag time
%constant) plus this repo's own already-established rate limit and angle
%bounds for the ARFWr RT model's rear-wing (RW) and front-wing (FW)
%actuators.
%
%   act = actuatorParams()
%
% Returns a struct: tau (s, first-order actuator lag), rateLim (deg/s,
% shared RW/FW slew-rate limit), rwClamp ([lo hi] deg, RW angle bound),
% fwClamp ([lo hi] deg, FW angle bound).
%
% Unlike suspensionBorrowed.m, three of these four fields are NOT borrowed
% from an external source: they are already-solved-for MLTP quantities
% (Zenvo-directive rate limit, CasADi-solver bounds), copied here by hand
% (deliberately without a MATLAB session, so nothing here depends on a live
% workspace) so the RT model matches the offline solver's actuator envelope
% exactly. Only tau is a genuinely borrowed value, taken from a published
% aero-actuator/servo spec.

%% ---- tau: BORROWED from a published aero-actuator servo design ----
% Source: NASA CR-2059, "A Fast-Acting Electrical Servo for the Actuation
% of Full-Span, Fowler-Type Wing Flaps in DLC Applications - A Detail
% Design Study", F.O. Smetana, R.J. Montoya, R.K. Carden (North Carolina
% State University, prepared for NASA Langley Research Center), NASA
% Contractor Report, July 1972. Public-domain US government report,
% freely hosted: ntrs.nasa.gov/citations/19720020377 (verified by direct
% PDF read 2026-08-31). Chosen over the GT/motorsport candidates
% (Formula-SAE DRS papers) because it is the only source found that states
% an actual DESIGNED, ANALYSED electromechanical wing-flap servo transfer
% function with a numeric time constant, rather than a qualitative
% "opening time" or bare actuator-type description.
%
% What the source states (quoted, p.6 and p.18 of the PDF):
%   - Abstract: "...an electro-mechanical actuator for Fowler-type wing
%     flaps which have a RESPONSE TIME CONSTANT OF 0.025 SECONDS..."
%   - Design target (Background, p.6): actuator modeled as
%     shaft_position/input_voltage = GAIN/(0.02*S + 1), i.e. tau < 0.02 s
%   - Achieved, after the full gain-scheduled compensator design (Input
%     Compensator section, p.18): "The system therefore appears much like
%     a simple first order system with a 0.022 sec. time constant."
% So the NASA design's ACHIEVED tau sits in 0.022-0.025 s - notably FASTER
% than the 0.03-0.05 s band this task was scoped against.
%
% Scaling to THIS application: the NASA actuator moves a 45 lb (~20 kg)
% light-aircraft flap against loads up to ~322 lbf / 150 ft-lbs at a
% maximum airspeed of 114 ft/s (~35 m/s) (Statement of Problem, p.3). Our
% RW/FW elements are full hypercar wing surfaces under much higher dynamic
% pressure (BCN/NUR corner and straight speeds run to ~85+ m/s, per this
% project's documented ARFWr entry-speed figures) and heavier composite structure
% with a larger angular swing (25 deg vs the NASA design's 40 deg, but at
% far higher hinge moment). A same-class fast-acting electric servo is
% therefore expected to be SLOWER than the light-aircraft baseline, not
% faster: applying a ~1.7x margin over the NASA achieved figure (0.025 s)
% lands in the upper half of the 0.03-0.05 s band this task targets.
tauNASA    = 0.025;                 % BORROWED(NASA CR-2059, abstract): achieved response
                                     % time constant of the source's own fast-acting
                                     % electromechanical flap servo (s)
marginFactor = 1.7;                 % derived: judgment-call scale-up for higher
                                     % aero-hinge-load / heavier composite structure
                                     % (see derivation above) - not itself sourced
tau = tauNASA*marginFactor;         % derived: 0.0425 s, inside the task's 0.03-0.05 s band

%% ---- rateLim, rwClamp, fwClamp: derived from THIS repo's own solver setup ----
% These are NOT borrowed - they are the exact bounds/limits the offline
% CasADi/IPOPT solver already uses for the ARFWr (rwMandate==7) control
% rows, hand-copied here (not run) so the RT model's actuator envelope
% matches the solved trajectories exactly.
rateLim = 25;                       % derived(Scripts/userOpts.m:363): [c.ub.RW,c.lb.RW] =
                                     % deal(25,-25) - Zenvo-directive shared RW/FW slew-rate
                                     % limit (deg/s), superseding an earlier 60 deg/s
                                     % CFD-era placeholder
rwClamp = [-10 15];                 % derived(Scripts/vehModel.m:164): activeAeroRW_lim =
                                     % 1/activeAeroRW_s*[-10 15] - the ARW/ARFWr rear-wing
                                     % angle-of-attack bound (deg)
fwClamp = [-25 0];                  % derived(Scripts/vehModel.m:229): activeAeroFW_lim =
                                     % 1/activeAeroFW_s*[-25 0] - the ARFWd/ARFWr front-wing
                                     % (2nd-element flap) angle bound (deg)

%% ---- assemble output struct ----
act.tau     = tau;
act.rateLim = rateLim;
act.rwClamp = rwClamp;
act.fwClamp = fwClamp;

end
