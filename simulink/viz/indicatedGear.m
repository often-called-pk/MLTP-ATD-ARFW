function [gInd, omEng, info] = indicatedGear(omRL, omRR, pt, varargin)
%INDICATEDGEAR  DISPLAY-ONLY gear indication derived from driven-axle wheel speed.
%
%   [gInd, omEng, info] = INDICATEDGEAR(omRL, omRR, pt)
%   [...]               = INDICATEDGEAR(..., 'Dwell', 0.25, 'Dt', 0.05)
%
%   omRL, omRR  [N x 1] rear-left / rear-right wheel speeds, rad/s, + forward
%               (VehStateBus.Om_rl / .Om_rr, or rows 3 and 4 of Omega_wheel)
%   pt          the powertrain struct from Parameters/Powertrain.m
%
%   gInd        [N x 1] indicated gear, 1..8, after the dwell filter
%   omEng       [N x 8] candidate crank speeds, rad/s
%   info        struct: omPk, omRedline, omCap, vCross (1x7 m/s), dwell, nHold
%
%   Name-value
%     'Dwell'  [s] a new candidate must persist this long before the digit
%              changes. Default 0.25. A bare nearest-band rule chatters at
%              every crossover on a 5 kHz log.
%     'Dt'     [s] sample period of omRL/omRR, used to turn 'Dwell' into a
%              sample count. Default 0.05.
%     'Rw'     [m] wheel radius, only used to report info.vCross. Default 0.37.
%
% ---------------------------------------------------------------------------
% THE CAVEAT THAT MUST TRAVEL WITH THIS NUMBER
%
%   The gear digit is a display-only indication, not a simulated shift.
%   Plant.slx models ONE fixed effective ratio, vp.gear = pt.OMmax*vp.Rw/pt.Vmax
%   ~ 8.30 (Parameters/Powertrain.m:67), and the supplier's 8-speed table
%   pt.gearbox (Powertrain.m:79) is read by NOTHING in the solver or the plant
%   -- grep confirms pt.gearbox / pt.FD / pt.ratioEMR have no consumer outside
%   Powertrain.m itself. The number this function returns is computed post-hoc
%   in the visualisation layer from measured driven-axle wheel speed, so no
%   shift event, torque interruption, ratio step or inertia change exists
%   anywhere in the simulation and NO REPORTED RESULT DEPENDS ON IT. It must
%   never appear in a lap-time, shift-strategy or powertrain-performance
%   statement -- those stay with the MLTP solver.
%
%   Selection rule: of the eight candidate crank speeds Om_wheel*FD*gearbox(i),
%   take the one closest to the ICE map's OWN peak-power speed (computed from
%   pt.ICE.rpm/pt.ICE.T, never a hardcoded rpm), masked by the ICE map's
%   redline. pt.OMmax/pt.ratioEMR is carried as a second guard but is INACTIVE
%   by construction: pt.OMmax is the e-motor ceiling (25,000 rpm), which
%   referred to the crank through the P2's 1.5:1 permits 16,668 rpm, far above
%   the ICE map's own 9,830 rpm end. Do not claim it binds.
%
%   Every number in that rule (ratios, omPk, omCap, nHold) comes from ONE
%   owner, indicatedGearParams(pt), which the live in-model observer in
%   ARFWr_Sim.slx also reads -- so the burned-in HUD digit and the live
%   Dashboard digit can never follow two different rules.
%
%   The rear pair is used because that is the axle the 8-speed physically
%   feeds (ICE + P2); the front e-motors sit on a fixed 6:1 and never see it.
%
%   See also indicatedGearParams, unrealPlayback, hudOverlay.

p = inputParser;
p.FunctionName = 'indicatedGear';
addParameter(p, 'Dwell', 0.25, @(v) isscalar(v) && isnumeric(v) && v >= 0);
addParameter(p, 'Dt',    0.05, @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'Rw',    0.37, @(v) isscalar(v) && isnumeric(v) && v > 0);
parse(p, varargin{:});
o = p.Results;

P = indicatedGearParams(pt, 'Dwell', o.Dwell, 'Dt', o.Dt);
ratios = P.ratios;  omPk = P.omPk;  omRed = P.omRed;  omP2 = P.omP2;
omCap  = P.omCap;   nHold = P.nHold;

omW    = 0.5*(omRL(:) + omRR(:));              % rad/s at the driven axle
omEng  = omW * ratios;                         % N x 8 candidate crank speeds

dst = abs(omEng - omPk);
dst(omEng > omCap) = inf;
allOut = all(isinf(dst), 2);
[~, raw] = min(dst, [], 2);
raw(allOut) = numel(ratios);                   % every candidate over the cap -> top gear

% dwell filter: hold the digit until a new candidate has persisted
gInd  = raw;
cur   = raw(1);  cand = raw(1);  cnt = 0;
for k = 1:numel(raw)
    if raw(k) == cur
        cnt = 0;  cand = cur;
    else
        if raw(k) == cand, cnt = cnt + 1; else, cand = raw(k); cnt = 1; end
        if cnt >= nHold, cur = cand; cnt = 0; end
    end
    gInd(k) = cur;
end

% crossover road speeds, for the record
vCross = nan(1, numel(pt.gearbox) - 1);
for i = 1:numel(vCross)
    % the two neighbouring candidates are equidistant from omPk when
    % omW*(r_i + r_{i+1})/2 = omPk
    vCross(i) = 2*omPk/(ratios(i) + ratios(i+1)) * o.Rw;
end

info = struct('omPk', omPk, 'omRedline', omRed, 'omCap', omCap, ...
              'omP2Guard', omP2, 'vCross', vCross, 'dwell', o.Dwell, 'nHold', nHold, ...
              'ratios', ratios);
end
