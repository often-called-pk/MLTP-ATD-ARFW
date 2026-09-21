function tv = tvParams()
%TVPARAMS Simple front-axle torque-vectoring (TV) allocator parameters for
% the ARFWr RT model's AeroECU-adjacent TV block (simulink/ecu/tvAllocStep.m).
%
% Per this project's RT design spec (decisions item 5, "Architecture"): a
% SIMPLE PI yaw-rate-tracking
% L/R torque shift on the FRONT axle - the ONLY axle with independent
% per-side e-motors in this vehicle (Parameters/Powertrain.m:6, "2x
% front-axle e-motors, 6:1 each, torque vectoring on the front axle").
% This is deliberately NOT a replication of the offline solver's ATD
% (front/rear split only, no L/R differential - see Parameters/Powertrain.m
% pt.TdistPower) - the spec states "No battery, no ATD replication" for
% this block.
%
%   tv = tvParams()
%
% Returns a struct:
%   Kp, Ki        PI gains (Nm per rad/s error, Nm per rad accumulated
%                 error), placeholder-tuned - see below
%   Ts            ECU discrete step (s)
%   dTqMax        absolute |dTq| ceiling (Nm) - see derivation below
%   perSideCapNm  one front wheel's max continuous torque (Nm), used by
%                 tvAllocStep.m to compute the TqAxle-dependent axle-
%                 headroom clamp
%   enable        1 = TV active, 0 = TV disabled (dTq == 0). See below.

%% ---- Kp, Ki: placeholder-tuned, never tuned against a real closed loop ----
% These gains were fixed before the Plant/torque-path wiring they drive
% existed, so there was no closed-loop model to tune against, and no closed-
% loop tuning pass has been run since. Chosen only to give a stable, moderately-responsive
% discrete PI at Ts=0.01 s that saturates for a "normal" cornering yaw-
% rate error (order 0.1-0.5 rad/s) without needing many steps to reach
% the physical torque-shift ceiling (see tvAllocStep.m / test_tvAlloc.m).
tv.Kp = 4000;   % derived: initial tuning, never re-tuned closed loop (Nm per rad/s of yaw-rate error)
tv.Ki = 8000;   % derived: initial tuning, never re-tuned closed loop (Nm per rad of integrated yaw-rate error, applied as Ki*e*Ts per step)

%% ---- Ts: ECU discrete step ----
tv.Ts = 0.01;   % measured: AeroECU model-ref rate, 100 Hz

%% ---- dTqMax, perSideCapNm: derived from Powertrain.m front e-motor caps ----
% The front axle is the ONLY axle with independent per-side torque in this
% vehicle (Powertrain.m:6). Powertrain.m does not state a front e-motor
% CONTINUOUS TORQUE figure directly - it gives rated power and a power/
% speed map instead (Powertrain.m:38-42):
%   pt.P_EM   = setupValue('P_EM', 150e3);     % Powertrain.m:39, rated power per motor (W)
%   pt.EM.rpm = [0 10000 25000];               % Powertrain.m:41
%   pt.EM.P   = [0 150 150]*1e3;               % Powertrain.m:42
% i.e. power ramps LINEARLY from 0 to the rated 150 kW over 0-10,000 rpm
% (constant-TORQUE region) then holds constant at 150 kW from 10,000 to
% 25,000 rpm (constant-power region) - the standard EV-motor torque/speed
% envelope. So the motor's peak continuous (motor-shaft) torque is the
% rated power divided by the base-speed angular rate:
%   T_motor_max = P_EM / omega_base,  omega_base = 10,000 rpm in rad/s
% NOTE: pt.P_EM is read through setupValue() (Powertrain.m:39), so a setup-
% sheet override could in principle change it
% at run time; with no override present the committed default (150e3, as
% it stood 2026-08-31 - this task does not touch the MATLAB session, HARD
% RULE) is what this derivation uses, same convention as
% simulink/params/actuatorParams.m and simulink/params/suspensionBorrowed.m.
omegaBase  = 10000*(pi/30);      % derived(Powertrain.m:41): base speed 10,000 rpm -> rad/s = 1047.20 rad/s
Tmotor_max = 150e3/omegaBase;    % derived(Powertrain.m:39,41-42): motor-shaft peak continuous torque = 143.24 Nm

% Wheel-side torque = motor-shaft torque x front reduction x driveline
% efficiency (machine->wheel, same efficiency term the lumped model applies
% throughout Powertrain.m):
%   pt.ratioEMF = 6;      % Powertrain.m:81, front e-motor reduction, each
%   pt.eff      = 0.95;   % Powertrain.m:45, driveline efficiency machine->wheel
Twheel_max = Tmotor_max*6*0.95;  % derived(Powertrain.m:81 ratioEMF=6, :45 eff=0.95): 816.46 Nm

tv.perSideCapNm = Twheel_max;    % derived(Powertrain.m:39,41-42,45,81): one front wheel's max continuous torque (Nm)
tv.dTqMax       = Twheel_max;    % derived, DELIBERATELY CONSERVATIVE (review fix 2026-09-01): absolute
                                  % |dTq| ceiling (Nm), set to 1x perSideCapNm - only HALF of the
                                  % THEORETICAL TIGHT limit the tvAllocStep.m TqAxle/2 -+ dTq/2 split
                                  % algebra actually allows. Under that split, the right wheel's torque is
                                  % TqAxle/2 + dTq/2, so it only reaches perSideCapNm at dTq =
                                  % 2*perSideCapNm - exactly what tvAllocStep.m's own headroom formula
                                  % says (headroom = 2*(perSideCapNm - |TqAxle|/2) = 2*perSideCapNm at
                                  % TqAxle=0). Capping dTqMax AT perSideCapNm instead of 2*perSideCapNm is
                                  % therefore a 50% safety margin below that tight bound, not itself a
                                  % physically-derived per-motor limit - a deliberate conservative choice
                                  % pending a closed-loop tuning pass. tvAllocStep.m's TqAxle-dependent
                                  % axle-headroom clamp (built from tv.perSideCapNm) tightens this further
                                  % as the nominal per-side torque (TqAxle/2) approaches tv.perSideCapNm.

%% ---- enable switch ----
% 1 = the TV allocator runs; 0 = it is bypassed and the front axle reverts to
% the symmetric TqAxle/2 split (dTq == 0, integrator held at 0).
%
% WHERE IT IS APPLIED: in the AeroECU wrapper block (AeroECU/TV, the
% tvStep MATLAB Function), NOT inside tvAllocStep.m - tvAllocStep stays a pure
% PI+clamp function with no mode logic, so simulink/validation/test_tvAlloc.m's
% five scenarios keep testing the allocator itself rather than a switch around
% it. The wrapper zeroes BOTH outputs when disabled, so the integrator cannot
% accumulate while bypassed and re-enabling mid-run starts from a clean state.
%
% CODEGEN: this is a literal in a no-input function, so MATLAB Coder constant-
% folds the whole tvParams() call inside the wrapper (same argument the ECU
% block's header makes for ecuLawParams()) - the generated ERT code contains
% either the allocator or a constant 0, never a run-time branch on a struct
% field. Changing it therefore requires a rebuild, which is the intended
% semantics: it is a BUILD-TIME configuration switch, not a run-time input.
% To make it run-time switchable it would have to become a SensorBus field or a
% Simulink.Parameter, which no current requirement asks for.
%
% WHAT THE A/B ACTUALLY MEASURES (corrected 2026-09-02 after review note I4 --
% the figures previously recorded here came from a CONFIGURATION THAT WAS NEVER
% SHIPPED, a drvVScale = 1.0 / 32 s run in a regime where the car was leaving
% the track by tens of metres, and quoting them as the allocator's value was
% not honest). Re-measured on the SHIPPED configuration, full BCN lap, gate 6's
% own operating point:
%
%     TV ON   lap 139.394 s, max |n| 2.301 m, 0 off-track, dTq RMS 417 Nm
%     TV OFF  lap 139.127 s, max |n| 2.297 m, 0 off-track, dTq identically 0
%
% i.e. AT THE SHIPPED OPERATING POINT THE ALLOCATOR MAKES NO MEASURABLE
% DIFFERENCE -- 0.267 s and 4 mm, both of which are inside this harness's
% run-to-run spread, and TV-off is nominally the faster of the two. That is the
% honest headline and it should be reported as such.
%
% It is NOT evidence that the allocator does nothing. In the high-excursion
% regime (drvVScale = 1.0, first 32 s of BCN) the same A/B measured max |n|
% 21.76 m with TV on against 33.58 m with it off -- a third less excursion --
% because that is a regime in which the car is fighting for yaw and the shipped,
% deliberately conservative plan never enters it. The two results together say:
% the allocator earns its place only near the limit, and the shipped driver
% does not operate there.
%
% It is left ENABLED by default because the ECU-to-plant torque-vectoring path
% is part of the shipped model, it costs nothing measurable, and gate 6 grades
% the TV-on lap. A future driver
% that does run at the limit is where it should start paying.
tv.enable = 1;

%% ---- anti-windup ----
% tvAllocStep.m uses CONDITIONAL INTEGRATION (integrator clamping/freeze)
% anti-windup, not back-calculation - so no extra tracking gain (Kt) is
% needed here. The saturation bound the clamp freezes against is exactly
% min(tv.dTqMax, axle headroom from TqAxle), built entirely from the two
% fields above; see tvAllocStep.m's header for the scheme itself.

end
