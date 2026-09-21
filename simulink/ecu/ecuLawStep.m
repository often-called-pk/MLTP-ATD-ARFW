function [tRW, tFW, sOn] = ecuLawStep(vx, r, Tbrake, law)
%ECULAWSTEP Reactive dual-wing target for one ECU step: kappa classifier
% station blend + brake overlay. Codegen-clean port of
% Functions/reactiveWingTarget.m (lines 27-44, kap2/zeroTrim g1,g2/station
% blend, ported EXACTLY) with the brake overlay inlined from
% Functions/rwVelocityTarget.m's brakeSwitch() sub-function, ported
% VERBATIM. Pure tanh + arithmetic, no branch, no getfielddef/assert/
% isfield - law fields are fixed and always present (ecuLawParams.m).
%
%   [tRW, tFW, sOn] = ecuLawStep(vx, r, Tbrake, law)
%
%   vx, r, Tbrake   scalar double: speed [m/s], yaw rate [rad/s], SIGNED
%                   brake torque [Nm] (model convention Tbrake<=0, same as
%                   Scripts/vehModel.m's T_brake and rwVelocityTarget.m -
%                   NOT a positive magnitude)
%   law             struct from ecuLawParams()
%
%   tRW, tFW  demanded wing angles [deg]
%   sOn       brake-on blend in [0,1], zeroTrim exact 0 off-brake

vEps = 1;                                          % m/s, avoids 1/0 at vx=0
kap2 = (r*r) / (vx*vx + vEps^2);

% zeroTrim renorm (reactiveWingTarget.m:29-31): the raw gate at kappa=0
% does not saturate to 0 unless w << k - untrimmed, the straight station
% is never reached exactly.
g1r = 0.5*(1 + tanh((kap2 - law.k1^2)/(2*law.k1*law.w1)));
g01 = 0.5*(1 + tanh(-law.k1/(2*law.w1)));
g1  = (g1r - g01)/(1 - g01);
g2r = 0.5*(1 + tanh((kap2 - law.k2^2)/(2*law.k2*law.w2)));
g02 = 0.5*(1 + tanh(-law.k2/(2*law.w2)));
g2  = (g2r - g02)/(1 - g02);

tRWk = law.stRW(1) + (law.stRW(2)-law.stRW(1))*g1 + (law.stRW(3)-law.stRW(2))*g2;
tFWk = law.stFW(1) + (law.stFW(2)-law.stFW(1))*g1 + (law.stFW(3)-law.stFW(2))*g2;

% Brake overlay = rwVelocityTarget.m's brakeSwitch(), verbatim, with its
% shipped defaults baked in as literals (onFrac=0.05, tbSign=-1,
% zeroTrim=true - reactiveWingTarget.m always calls it with
% brakeOpts=struct('zeroTrim',true), leaving onFrac/tbSign at their
% rwVelocityTarget.m defaults). w1frac is the one brake-switch field
% ecuLawParams.m carries (matches rwVelocityTarget.m's own default 0.04).
onFrac = 0.05;
tbSign = -1;
w1b    = law.w1frac * law.TbMax;
Tb_mag = tbSign * Tbrake;                          % linear, not abs() - no Jacobian kink at Tbrake=0
sOnRaw = 0.5*(1 + tanh((Tb_mag - onFrac*law.TbMax)/w1b));
sOn0   = 0.5*(1 + tanh(-onFrac/law.w1frac));       % numeric constant, does not depend on Tbrake
sOn    = (sOnRaw - sOn0)/(1 - sOn0);

tRW = tRWk + sOn*(law.stRW(4) - tRWk);
tFW = tFWk + sOn*(law.stFW(3) - tFWk);

end
