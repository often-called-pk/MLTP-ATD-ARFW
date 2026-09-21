function P = indicatedGearParams(pt, varargin)
%INDICATEDGEARPARAMS  The numbers behind the DISPLAY-ONLY gear indication.
%
%   P = INDICATEDGEARPARAMS(pt)
%   P = INDICATEDGEARPARAMS(pt, 'Dwell', 0.25, 'Dt', 0.05)
%
%   pt   the powertrain struct from Parameters/Powertrain.m
%
%   P    struct, all derived from pt, no literals:
%          ratios   [1 x 8]  overall ratio wheel -> crank, pt.FD * pt.gearbox
%          omPk     [rad/s]  crank speed at the ICE map's OWN peak power
%          omRed    [rad/s]  ICE map end = redline
%          omP2     [rad/s]  e-motor ceiling referred to the crank (inactive guard)
%          omCap    [rad/s]  min(omRed, omP2) -- the mask applied to candidates
%          nHold    [-]      dwell filter length in samples, round(Dwell/Dt)
%          dwell,dt [s]      as given
%
%   ONE owner for the rule shared by the offline HUD (simulink/viz/
%   indicatedGear.m, used by unrealPlayback's burned-in overlay) and the live
%   in-model observer (ARFWr_Sim.slx "Live Instruments" MATLAB Function
%   block, whose parameters are the fields of this struct). Both select, of
%   the eight candidate crank speeds omWheel*ratios, the one closest to omPk
%   among those under omCap, and hold the digit for nHold samples before
%   changing it.
%
%   THE CAVEAT that travels with every consumer: the plant runs ONE fixed
%   effective ratio (vp.gear, Parameters/Powertrain.m) and never shifts;
%   pt.gearbox is read by nothing in the solver or the plant. The digit is a
%   legibility aid reconstructed from driven-axle wheel speed. It must never
%   appear in a lap-time, shift-strategy or powertrain statement.
%
%   See also indicatedGear, unrealPlayback, hudOverlay.

p = inputParser;
p.FunctionName = 'indicatedGearParams';
addParameter(p, 'Dwell', 0.25, @(v) isscalar(v) && isnumeric(v) && v >= 0);
addParameter(p, 'Dt',    0.05, @(v) isscalar(v) && isnumeric(v) && v > 0);
parse(p, varargin{:});
o = p.Results;

for f = {'ICE','gearbox','FD','OMmax','ratioEMR'}
    assert(isfield(pt, f{1}), 'indicatedGearParams:missingField', ...
        'indicatedGearParams: pt has no field ''%s'' (run Parameters/Powertrain.m).', f{1});
end

ratios = pt.FD * pt.gearbox(:).';              % 1 x 8, wheel -> crank

omRed = pt.ICE.rpm(end)*(pi/30);               % ICE map end = redline
omP2  = pt.OMmax / pt.ratioEMR;                % e-motor ceiling at the crank; never binds
omCap = min(omRed, omP2);

% peak-power crank speed FROM THE SHIPPED MAP -- no rpm literal anywhere
wg  = linspace(0, omRed, 2001);
Pg  = interp1(pt.ICE.rpm*(pi/30), pt.ICE.T, wg, 'linear') .* wg;
[~, ip] = max(Pg);
omPk = wg(ip);

nHold = max(1, round(o.Dwell/o.Dt));

P = struct('ratios', ratios, 'omPk', omPk, 'omRed', omRed, 'omP2', omP2, ...
           'omCap', omCap, 'nHold', nHold, 'dwell', o.Dwell, 'dt', o.Dt);
end
