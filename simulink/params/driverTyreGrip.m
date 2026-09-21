function out = driverTyreGrip(vp, what)
%DRIVERTYREGRIP The driver model's view of the tyres: peak friction and
% cornering stiffness, computed FROM THE LIVE TYRE SET at run time.
%
%   out = driverTyreGrip(vp, what)
%     what = 'mu'   -> 2x1 [muX; muY]   peak long./lat. friction, front/rear mean
%     what = 'muX'  -> scalar muX
%     what = 'muY'  -> scalar muY
%     what = 'Cyf'  -> scalar front per-tyre cornering stiffness [N/rad]
%     what = 'Cyr'  -> scalar rear  per-tyre cornering stiffness [N/rad]
%
% WHY THIS FILE EXISTS
% --------------------
% DriverPath.slx needs four numbers that describe the tyres to its own internal
% model: muX/muY for the Traction Pedal Map's friction ellipse, and Cy_f/Cy_r
% for the VDB Predictive Driver's Stanley "include dynamics" mode. Originally
% all four were BAKED AS LITERALS into the block dialogs, read back at build
% time from the MathWorks-default Magic Formula coefficients of the placeholder
% VDB tyre blocks that the real-tyre swap later deleted. The Traction Pedal
% Map's own description already flagged that as a defect to fix: the real Zenvo
% tyres are far grippier, so a pedal ceiling sized against placeholder
% coefficients is needlessly low, and all four numbers have to be read from
% vp.tyre_f/vp.tyre_r AT LOAD TIME rather than baked into a dialog.
%
% The real-tyre swap replaced the tyres but did not recompute those literals,
% so the shipped driver was still sizing its pedal ceilings against placeholder
% grip. That is the single biggest reason the closed loop ran far below the
% traction limit. This function is the "read at load time" mechanism: the
% dialogs now hold CALLS to it, not numbers.
%
% CONFIDENTIALITY. The Zenvo MF5.2 coefficients must
% never be reproduced outside the repo, and in particular must never be stored
% inside an .slx dialog where they would travel with a shared model file. That
% is exactly what returning them from a function avoids: the .slx stores the
% EXPRESSION driverTyreGrip(vp,'mu'), the numbers live only in
% Parameters/tyreParams_Zenvo.m and only in memory. Same design as
% simulink/params/tyreParamVec.m, which exists for the identical reason.
%
% DEFINITIONS
% -----------
% muX, muY: Functions/tyreMFnum.m's own peak-friction outputs 3 and 4,
%   mux = pDx1 + pDx2*dfz and muy = pDy1 + pDy2*dfz, evaluated at each axle's
%   STATIC per-corner load and averaged front/rear -- the SAME definition and
%   the same averaging build_driver.m used for the placeholder tyres, so the
%   pedal map's ellipse formula is unchanged and only its two coefficients move.
%   They are read out of tyreMFnum rather than recomputed here, so the driver's
%   mu can never drift from the plant's tyre law.
%
%   LOAD SENSITIVITY, stated plainly: pDy2 < 0, so muY FALLS as load rises and
%   the static-load value is an OPTIMISTIC estimate at the high downforce loads
%   of a fast corner. The pedal map is a MAP, not a hard clamp, and it carries
%   its own safety factors drvSFa/drvSFb precisely to absorb this; the speed
%   PLANNER (simulink/tools/buildSpeedPlan.m) does NOT use this function's
%   static-load figure -- it evaluates muY at the actual downforce-loaded
%   per-corner load, iteratively. The two consumers therefore differ on purpose.
%
% Cy_f, Cy_r: the per-tyre lateral slope at zero slip angle,
%   dFy/dsa|_{sa=0} = muy * Fz * By * Cy, taken from Functions/tyreMFnum.m's
%   pure-slip form fy0 = muy*fz*sin(Cy*atan(By*sa - Ey*(...))) whose derivative
%   at sa = 0 is muy*fz*Cy*By. The placeholder-era formula was the VDB block's
%   PKY1/PKY2/PKY4 expression; the MF5.2 set in this repo carries By/Cy
%   directly, so the equivalent quantity is formed directly rather than through
%   a parameterisation the live tyre struct does not have.
%
% See also FUNCTIONS/TYREMFNUM, SIMULINK/PARAMS/TYREPARAMVEC,
% SIMULINK/TOOLS/BUILDSPEEDPLAN.

narginchk(2, 2);
assert(isstruct(vp) && isfield(vp,'tyre_f') && isfield(vp,'tyre_r'), ...
    'driverTyreGrip:vp', 'vp must carry .tyre_f and .tyre_r (Parameters/vehParams.m)');

% Static per-corner loads, front and rear (2 tyres per axle).
Fzf = vp.m*vp.g*vp.l_r/(2*vp.l);
Fzr = vp.m*vp.g*vp.l_f/(2*vp.l);

% Peak friction from the tyre law itself (outputs 3 and 4 of tyreMFnum).
[~,~,mxf,myf] = tyreMFnum(vp.tyre_f, vp.tyre_f.Fz0, Fzf, 0, 0);
[~,~,mxr,myr] = tyreMFnum(vp.tyre_r, vp.tyre_r.Fz0, Fzr, 0, 0);

switch what
    case 'mu'
        out = [0.5*(mxf+mxr); 0.5*(myf+myr)];
    case 'muX'
        out = 0.5*(mxf+mxr);
    case 'muY'
        out = 0.5*(myf+myr);
    case 'Cyf'
        out = myf*Fzf*vp.tyre_f.By*vp.tyre_f.Cy;
    case 'Cyr'
        out = myr*Fzr*vp.tyre_r.By*vp.tyre_r.Cy;
    otherwise
        error('driverTyreGrip:what', ...
            'unknown request ''%s'' (expected mu | muX | muY | Cyf | Cyr)', what);
end

end
