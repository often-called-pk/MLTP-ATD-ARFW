function [stIdxRW, stIdxFW, dwellCntRWout, dwellCntFWout] = snapStationRef( ...
    tRW, tFW, sOn, prevRW, prevFW, dwellCntRW, dwellCntFW, law) %#ok<INUSL>
%SNAPSTATIONREF Hysteresis + minimum-dwell snap of the continuous RW/FW
% targets onto their discrete station sets (law.stRW: 4 entries, law.stFW:
% 3 entries). Codegen-clean: no getfielddef/assert/isfield, law fields
% fixed and always present (ecuLawParams.m).
%
% DESIGN NOTE: this function RETURNS its updated dwell state. A 2-output
% signature (station indices only) cannot carry the dwell counters across
% steps, so they are returned explicitly and the Simulink wrapper closes the
% loop with Unit Delays on dwellCntRWout/dwellCntFWout -> dwellCntRW/dwellCntFW.
%
%   [stIdxRW, stIdxFW, dwellCntRWout, dwellCntFWout] = ...
%       snapStationRef(tRW, tFW, sOn, prevRW, prevFW, dwellCntRW, dwellCntFW, law)
%
%   tRW, tFW          continuous blended targets [deg] (ecuLawStep outputs)
%   sOn               brake-on blend in [0,1]; NOT used by the switch
%                      decision itself - tRW/tFW already carry the brake
%                      overlay from ecuLawStep, so the brake pull-to-
%                      station is expressed purely through the blended
%                      target. Kept in the signature to match the ECU
%                      MATLAB Fn / Stateflow-chart port.
%   prevRW, prevFW    current 1-based station index (1..4 / 1..3)
%   dwellCntRW/FW     ECU steps dwelled at prevRW/prevFW since the last switch
%   law               struct from ecuLawParams()
%
%   stIdxRW, stIdxFW              1-based station index for this step
%   dwellCntRWout, dwellCntFWout  updated dwell counters (0 on switch, else +1)
%
% Switch away from the current station only if BOTH:
%   (a) |target-currentStationDeg| > |target-candidateDeg| + hyst, hyst =
%       law.hystFrac * (minimum adjacent-station gap for that wing)
%   (b) dwellCnt*law.Ts >= law.dwellMin

[stIdxRW, dwellCntRWout] = snapOneWing(tRW, law.stRW, prevRW, dwellCntRW, law.hystFrac, law.dwellMin, law.Ts);
[stIdxFW, dwellCntFWout] = snapOneWing(tFW, law.stFW, prevFW, dwellCntFW, law.hystFrac, law.dwellMin, law.Ts);

end

% =========================================================================
function [stIdx, dwellOut] = snapOneWing(target, stations, prevIdx, dwellCnt, hystFrac, dwellMin, Ts)
%SNAPONEWING Nearest-station snap with hysteresis + minimum dwell, one wing.
% Fixed small station count (3 or 4) - plain loops, codegen-safe.
n = numel(stations);

% nearest station to target
candIdx  = 1;
bestDist = abs(target - stations(1));
for i = 2:n
    d = abs(target - stations(i));
    if d < bestDist
        bestDist = d;
        candIdx  = i;
    end
end

% hysteresis: hystFrac x minimum adjacent-station gap, THIS wing's stations
minGap = abs(stations(2) - stations(1));
for i = 2:n-1
    g = abs(stations(i+1) - stations(i));
    if g < minGap
        minGap = g;
    end
end
hyst = hystFrac * minGap;

distCur  = abs(target - stations(prevIdx));
distCand = abs(target - stations(candIdx));

doSwitch = (distCur > distCand + hyst) && (dwellCnt*Ts >= dwellMin);

if doSwitch
    stIdx    = candIdx;
    dwellOut = 0;
else
    stIdx    = prevIdx;
    dwellOut = dwellCnt + 1;
end

end
