function [dTq, iStateOut] = tvAllocStep(vx, r, kappaRef, TqAxle, iState, tv)
%TVALLOCSTEP Discrete PI front-axle torque-vectoring (TV) allocator, one
% ECU step. SIMPLE TV per this project's RT design spec
% (decisions item 5, architecture line 61): PI control on the yaw-rate
% error e = kappaRef*vx - r, output dTq = the front-axle LEFT/RIGHT torque
% shift. Codegen-clean: no assert/isfield/defaulting, pure arithmetic,
% scalars, tv fields fixed and always present (tvParams.m). State in/out
% follows simulink/ecu/snapStationRef.m's pattern - the Simulink wrapper
% closes the loop with a Unit Delay on iStateOut -> iState.
%
%   [dTq, iStateOut] = tvAllocStep(vx, r, kappaRef, TqAxle, iState, tv)
%
%   vx        scalar double: vehicle speed [m/s]
%   r         scalar double: yaw rate [rad/s]. SIGN CONVENTION, checked
%             against Scripts/vehModel.m: r > 0 = LEFT turn. vehModel.m:361
%             defines "kappa > 0 for left turns", and vehModel.m:620-629's
%             slip-angle/wheel-speed terms use vx -/+ r*t/2 for the
%             left/right wheels respectively (v_fl uses vx-r*t/2, v_fr uses
%             vx+r*t/2) - i.e. the RIGHT side is the OUTER (faster) wheel
%             when r>0, the standard ISO/CCW-positive yaw convention.
%   kappaRef  scalar double: reference path curvature [1/m] at the
%             driver's preview point, kappaRef > 0 = left turn (same sign
%             convention as vehModel.m's kappa, confirmed above)
%   TqAxle    scalar double: nominal (pre-TV) TOTAL front-axle torque
%             command [Nm] - both front wheels combined, split
%             symmetrically BEFORE the TV shift is applied:
%               T_fl = TqAxle/2 - dTq/2   (front LEFT,  vehModel.m's T_fl)
%               T_fr = TqAxle/2 + dTq/2   (front RIGHT, vehModel.m's T_fr)
%             (the Plant's Torque Path applies this split; this function only
%             computes dTq.)
%   iState    scalar double: PI integrator state carried in from the
%             previous step [Nm] (torque units - see anti-windup below)
%   tv        struct from simulink/params/tvParams.m
%
%   dTq        scalar double [Nm]: front-axle L/R torque shift for this
%              step. SIGN CONVENTION matches vehModel.m's dr (yaw-rate
%              derivative) equation, lines 697-698, which sums a
%              (fx_rr-fx_rl) rear term and a (fx_fr-fx_fl) front term - both
%              RIGHT MINUS LEFT, both with the SAME positive sign - i.e.:
%                dTq > 0  ->  MORE torque to the RIGHT wheel, LESS to the
%                             LEFT  ->  positive yaw moment  ->  increases r
%                             (more LEFT turn, toward kappaRef*vx)
%              With Kp,Ki > 0 this is the natural-sign PI: e > 0 means "not
%              turning left enough" and directly produces dTq > 0 (more
%              right-side torque, which raises r toward the target) - no
%              sign flip is applied anywhere in this function.
%   iStateOut  scalar double [Nm]: updated integrator state for the next
%              step (anti-windup applied - see below)
%
% ANTI-WINDUP SCHEME: CONDITIONAL INTEGRATION (integrator clamping/
% freeze), NOT back-calculation. Each step computes the tentative
% (unclamped) integrator update and PI output; if the output would exceed
% this step's saturation limit, the integrator update is:
%   - FROZEN (held at iState, the previous value) if this step's error
%     would integrate FURTHER into the same-sign saturation
%   - allowed through normally (i.e. it "unwinds") if this step's error
%     would pull the candidate integrator value back toward zero / the
%     opposite sign
% This needs no extra back-calculation tracking gain (Kt) to tune, and
% guarantees the integrator never accumulates more "charge" than the
% output can immediately use while saturated - so a long saturated period
% followed by an error reversal recovers within a bounded, small number of
% steps (see simulink/validation/test_tvAlloc.m's windup-release scenario,
% which asserts this quantitatively against an unclamped integrator), not
% a Kt-dependent decay.

e = kappaRef*vx - r;

iCand  = iState + tv.Ki*e*tv.Ts;   % tentative (unclamped) integrator update
uUnsat = tv.Kp*e + iCand;          % tentative (unclamped) PI output

% Axle headroom: how far a side can move from its symmetric TqAxle/2
% baseline, in EITHER direction (+dTq/2 on one side, -dTq/2 on the other),
% before hitting one front motor's max wheel torque (tv.perSideCapNm) - see
% tvParams.m for perSideCapNm's derivation from Powertrain.m.
headroom = 2*(tv.perSideCapNm - abs(TqAxle)/2);
if headroom < 0
    headroom = 0;
end

dTqLim = tv.dTqMax;
if headroom < dTqLim
    dTqLim = headroom;
end

% NOTE (review fix 2026-09-01): the iCand>iState / iCand<iState freeze-vs-
% unwind test below is a PROXY for "does this step's error integrate
% further into saturation" - it is only correct because tv.Ki > 0 is a
% fixed, always-positive gain (tvParams.m). With Ki>0, iCand-iState has
% the same sign as e, so the proxy is equivalent to checking sign(e). If
% Ki's sign were ever allowed to go negative, this proxy would need to be
% re-derived (or replaced with an explicit sign(e) check).
if uUnsat > dTqLim
    dTq = dTqLim;
    if iCand > iState
        iStateOut = iState;   % freeze: integrating further worsens the positive saturation
    else
        iStateOut = iCand;    % unwind: this step's error already pulls the integrator back
    end
elseif uUnsat < -dTqLim
    dTq = -dTqLim;
    if iCand < iState
        iStateOut = iState;   % freeze: integrating further worsens the negative saturation
    else
        iStateOut = iCand;    % unwind
    end
else
    dTq = uUnsat;
    iStateOut = iCand;
end

end
