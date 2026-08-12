%% POWERTRAIN - Zenvo Aurora Tur (lumped single-machine equivalent for MLTP)
%
% Architecture (Zenvo website / parameter sheet, cross-checked):
%   - 6.6L quad-turbo V12, ~1,250 bhp claim, 1,200 Nm plateau, ~9,800 rpm redline
%   - P2 e-motor at the 8-speed gearbox (rear path), ratio 1.5 x gear x final drive
%   - 2x front-axle e-motors, 6:1 each, torque vectoring on the front axle
%   - AWD; e-motor maps (sheet): rpm [0 10000 25000] -> power [0 150 150] kW each
%
% The MLTP framework models ONE lumped machine through ONE fixed ratio, with
% path constraints on power (pt.Pmax) and machine speed (pt.OMmax). Gearshifts
% and the ICE torque curve are therefore folded into a power-limited envelope:
%   F_tractive(v) = min( pt.Tmax*vp.gear/vp.Rw , pt.Pmax/v )
% which approximates an ideal (closely-stepped) gearbox held near peak power.
%
% IMPORTANT: this file now READS vp.Rw instead of defining it (single-sourcing
% fix - vp.Rw lives in vehParams.m only). In userOpts.m, swap the order to:
%       run('vehParams.m');
%       run('Powertrain.m');

if ~exist('vp','var') || ~isfield(vp,'Rw')
    error(['Powertrain.m now requires vp.Rw from vehParams.m. ' ...
           'Reorder userOpts.m so vehParams.m runs BEFORE Powertrain.m.']);
end

%% ICE - digitised torque map (from Zenvo sheet image; see ice_torque_map.csv)
% The scalars below read through Functions/setupValue.m so a filled-in setup sheet
% (docs/setup-parameters.csv -> Functions/loadSetupSheet.m) can drive a run; with
% no setupOverride.mat present each call returns the committed default unchanged.
% The two MAPS (pt.ICE.rpm/T, pt.EM.rpm/P) are deliberately NOT wired: they are
% vectors, not scalars, and nothing outside this file reads them - the NLP sees
% only the lumped pt.Pmax / pt.OMmax / pt.Tmax envelope, so overriding a map
% would change nothing. They are sheet class DEAD for exactly that reason.
pt.ICE.rpm = [0   1000 1500 2000 2500 3000 3500 4000 5000 6000 7000 7500 8000 8500 9000 9500 9830];
pt.ICE.T   = [0    522  580  668  798  942 1095 1198 1200 1200 1198 1148 1092 1027  946  845  790];   % (Nm)
pt.P_ICE   = setupValue('P_ICE', 918e3);  % peak ICE power (W), ~8,350 rpm; digitising tolerance ~ +/-10 kW
                                          % (website "1,250 bhp" = 932 kW sits within map-reading tolerance)

%% e-motors (Tur: 2x front + 1x P2 rear, 150 kW / ~200 bhp each per sheet & press)
pt.P_EM    = setupValue('P_EM', 150e3);   % rated mechanical power per motor (W)
pt.nEM     = setupValue('nEM', 3);
pt.EM.rpm  = [0 10000 25000];             % per-motor power map from the sheet
pt.EM.P    = [0 150 150]*1e3;             % (W) linear ramp to 10k rpm, then constant

%% Lumped system limits used by the MLTP path constraints
pt.eff  = setupValue('eff', 0.95);        % driveline efficiency machine->wheel (motor/ICE ratings
                                          % assumed at machine output; set 1.0 for crank-rated cap)
pt.Pmax = pt.eff*(pt.P_ICE + pt.nEM*pt.P_EM);     % = 1.30 MW at the wheels (1.368 MW crank;
                                                  %   marketing "1,850 bhp" = 1.38 MW)

pt.OMmax = setupValue('OMmax', 25000*(pi/30));   % (rad/s, NOT rpm) lumped-machine speed ceiling = e-motor limit.
                                          % Real-car check at 420 km/h: ICE ~8,430 rpm (<9,800),
                                          % front motors ~18,100 rpm, P2 ~12,600 rpm (<25,000). OK.
                                          % Sheet units are rad/s; the loader's range check rejects
                                          % an rpm figure typed here as a unit slip.

pt.Vmax  = setupValue('Vmax', 420/3.6);   % (m/s, NOT km/h) current official top-speed estimate for the Tur.
                                          % Sets the vx state upper bound AND vp.gear below; on BCN /
                                          % Nurburgring GP the bound will not bind, so the exact
                                          % 420-vs-450 km/h question is benign here.

pt.Tmax  = setupValue('Tmax', 1200);      % (Nm) machine-side torque cap. Sized so it never binds
                                          % below the grip or power limits:
                                          %   grip:  mu*m*g*Rw/gear ~ 1.2*1602*9.81*0.37/8.30 ~ 840 Nm
                                          %   power: Pmax*Rw/(gear*v) = 1200 Nm at v ~ 48 m/s
                                          % -> envelope is tyre-limited, then power-limited, as intended.

vp.gear  = (pt.OMmax*vp.Rw)/pt.Vmax;      % effective single ratio (~8.30 with Rw = 0.37)

%% Torque split guidance (set in vehParams.m, kept here for traceability)
% Power-capability split front/rear:
%   rear  = ICE + P2      = (918+150) kW = 1068 kW  -> 0.78 of system
%   front = 2 e-motors    = 300 kW               -> 0.22 of system
% With ATD = 'Off' set vp.Tdist = 0.78 so the fixed split can never demand more
% than 300 kW through the front axle. (pt.TdistPower is informational.)
pt.TdistPower = (pt.P_ICE + pt.P_EM)/(pt.P_ICE + pt.nEM*pt.P_EM);   % = 0.781


%% Real-driveline constants (not used by the lumped model; for report/post-processing)
pt.gearbox  = [3.4 2.35 1.75 1.4 1.2 1.0 0.9 0.8];   % 8-speed ratios (sheet)
pt.FD       = 3.5;                                   % final drive (sheet)
pt.ratioEMF = 6;                                     % front e-motor reduction, each (sheet)
pt.ratioEMR = 1.5;                                   % rear P2: 1.5 x gearbox x FD (sheet)