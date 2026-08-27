%% vehParams.m - Vehicle Parameters

% RAW INPUTS GO THROUGH setupValue(). Every scalar below that a real car can be
% measured for reads its value via Functions/setupValue.m, which returns the
% committed default here unless a filled-in setup sheet
% (a filled-in setup sheet, loaded into setupOverride.mat) supplied one. With no
% setupOverride.mat present the call is a pass-through, so this file's behaviour
% is byte-identical to before the hook existed.
%
% The hook sits at the ASSIGNMENT, not on the finished vp struct, because this
% script interleaves raw inputs with quantities derived from them (vp.kwf feeds
% vp.ksf_rad ~2 lines later; vp.l/vp.wB feed vp.l_f; the masses feed vp.ms/vp.m
% and the static corner loads; vp.Fz_abs is aliased into vp.Fz_lim). Patching raw
% fields after the fact would leave every derived quantity computed from the OLD
% value - a silently self-inconsistent vehicle. That is also why the sheet's
% DERIVED rows are read-only: they are recomputed here, never set.

% vehicle parameter inputs
vp.brkB  = setupValue('brkB',  0.65);                                 % fraction of total brake force going to the front wheels  (-) % sheet: 'mechanical pressure bias front 0.65 nominally'
vp.Tdist = setupValue('Tdist', 0.78);                                 % fraction of total torque going to the rear wheels        (-) % rear torque fraction --
vp.ksD   = 0.535;                                                      % fraction of total roll stiffness for the rear axle       (-)
                                                                       % NOT sheet-wired ON PURPOSE: DEAD on the MLTP.m path. Only
                                                                       % vp.ksf_rad/ksr_rad (from the WHEEL RATES below) reach
                                                                       % vp.lty_dis; vp.ksf/ksr/ks built from ksD are read by nothing
                                                                       % in Scripts/. Sheet class DEAD.

% Aerodynamic input
% (per-surface wing angles for the unrebuilt ActAero 2/3 configs are not set here)

% Constants
vp.g = setupValue('g', 9.81);                                           % gravitational acceleration    (m/s^2)
vp.rho = setupValue('rho', 1.204);                                      % air density                   (kg/m^3)

% Vehicle Parameters
% Masses  (track config: dry 1450 + fuel 55 + coolant 12 + lube 10 + driver 75; no pax/luggage)
vp.mb = setupValue('mb', 1327);                                         % vehicle sprung mass                           (kg) % sprung mass excl. driver  (1602 - 90 - 110 - 75)
vp.md = setupValue('md', 75);                                           % driver mass                                   (kg)
vp.ms = vp.mb + vp.md;                                                  % total sprung mass                             (kg)
vp.muf = setupValue('muf', 90);                                         % unsprung mass front axle                      (kg) % unsprung FRONT AXLE = 2 x 45 kg per corner
vp.mur = setupValue('mur', 110);                                        % unsprung mass front axle                      (kg) % unsprung REAR  AXLE = 2 x 55 kg per corner
vp.m = vp.ms + vp.muf + vp.mur;                                         % total vehicle mass                            (kg)
vp.I_z = setupValue('I_z', 2900);                                       % yaw moment of inertia                         (kg*m^2) % Izz from sheet

vp.A = setupValue('A', 2.0);                                            % vehicle reference area                        (m^2) % frontal area -- see note (d) on Cl reference area
vp.t = setupValue('t', 1.705);                                          % track width                                   (m) % model has ONE track width; avg of 1.74 F / 1.67 R (document this)
vp.l = setupValue('l', 2.8);                                            % wheelbase                                     (m)

vp.wB = setupValue('wB', 0.43);                                         % COG distribution front                        (-) % front mass fraction -> l_f = 2.8*0.57 = 1.596 = Excel 'a'


vp.l_f = vp.l*(1-vp.wB);                                                % distance COG to front axle                    (m)
vp.l_r = vp.l*vp.wB;                                                    % distance COG to rear axle                     (m)

vp.hcg = setupValue('hcg', 0.44);                                       % COG height above ground plane                 (m) % Track Mode CoG height
vp.huf = setupValue('huf', 0.35);                                       % height COG unsprung mass front                (m) % ~ front loaded radius (wheel-centre height)
vp.hur = setupValue('hur', 0.37);                                       % height COG unsprung mass rear                 (m)
vp.hw  = setupValue('hw',  1.28);                                       % height of rear wing                           (m) % need to ask Zenvo


vp.hRCf = setupValue('hRCf', 0.07);                                     % height roll centre front                      (m)
vp.hRCr = setupValue('hRCr', 0.11);                                     % height rolll centre rear                      (m)
vp.hRC = (vp.l_f*vp.hRCr + vp.l_r*vp.hRCf)/vp.l;                        % height roll centre axis at centre of gravity  (m)
vp.d = vp.hcg - vp.hRC;                                                 % distance between COG and roll axis            (m) 

vp.ksf = (1-vp.ksD)*2000;                                               % roll stiffness front axle                     (Nm/deg)
vp.ksr = vp.ksD*2000;                                                   % roll stiffness rear axle                      (Nm/deg)
vp.ks = vp.ksf + vp.ksr;                                                % total roll stiffness                          (Nm/deg)
vp.kwf = setupValue('kwf', 40000);                                      % wheel rate front (N/m) - single source: roll stiffness + aero collapse
vp.kwr = setupValue('kwr', 50000);                                      % wheel rate rear  (N/m)
vp.ksf_rad = 0.5*vp.kwf*1.74^2;   % 60,552 Nm/rad  (1057 Nm/deg)        % roll stiffness front axle                     (Nm/rad)
vp.ksr_rad = 0.5*vp.kwr*1.67^2;   % 69,722 Nm/rad  (1217 Nm/deg)        % roll stiffness rear axle                      (Nm/rad)
vp.ks_rad = vp.ksf_rad + vp.ksr_rad;                                    % total roll stiffness                          (Nm/rad)
% NOTE: no anti-roll-bar data in the sheet; a hypercar will have ARBs, so this underestimates total roll stiffness. Flag as a data request.

% Tyre Parameters
vp.Rw = setupValue('Rw', 0.37);                                         % wheel radius                                  (m)
vp.Jw = setupValue('Jw', 1.7);                                          % wheel rotational inertia                      (kg*m^2) % single value; sheet gives 1.6 F / 1.8 R

% Per-axle Pacejka MF5.2 sets -> vp.tyre_f / vp.tyre_r, including the per-axle
% rolling-resistance qsy1 that replaces the old scalar vp.f. This file is NOT
% distributed; supply your own, or copy the synthetic stand-in over it (see
% README). Plots derived from licensed tyre data should use normalised or
% unlabelled vertical axes.
tyreParams_DoNotPublish

% Sheet-wired, but NOT a physical measurement: these shift the nominal load, i.e.
% they re-tune the tyre MODEL. The coefficients themselves stay unsettable
% (sheet class TYRE); this is the one tyre-side number a sheet may touch.
vp.Fz0_shift_f = setupValue('Fz0_shift_f', 1);                          % front nominal-load shift about tyre_f.Fz0 (TyreOptim design parameter)
vp.Fz0_shift_r = setupValue('Fz0_shift_r', 1);                          % rear  nominal-load shift about tyre_r.Fz0 (TyreOptim design parameter)

% Static wheel loads
vp.Wfl0 = 0.5*vp.g*(vp.muf + vp.wB*vp.ms);                              % static wheel load front left tyre             (N)
vp.Wfr0 = 0.5*vp.g*(vp.muf + vp.wB*vp.ms);                              % static wheel load front right tyre            (N)
vp.Wrl0 = 0.5*vp.g*(vp.mur + (1-vp.wB)*vp.ms);                          % static wheel load rear left tyre              (N)
vp.Wrr0 = 0.5*vp.g*(vp.mur + (1-vp.wB)*vp.ms);                          % static wheel load rear right tyre             (N)

% Brakes
vp.Tbrake_max = 4e3;                                                    % max braking torque from brakes                (Nm)
                                                                        % DEAD ASSIGNMENT: overwritten by the 7e3 line at the very
                                                                        % END of this file, and vehModel.m:119,121 only read
                                                                        % vp.Tbrake_max after this script has finished, so this 4 kNm
                                                                        % never reaches a solve. Left in place (it is the historical
                                                                        % value and the comment at the bottom argues against it); the
                                                                        % sheet hook is on the LIVE assignment, not this one.

% tyre vertical load limits (Zenvo requirement) - kept ahead of the aero
% block (ordering preserved; the High->Shallow shed / aeroVsw that consumed
% vp.Fz_lim has been stripped from the live path)
vp.Fz_abs = setupValue('Fz_abs', 12500);   % absolute structural limit incl. kerb strikes.
                      % THE sheet-editable load cap: vp.Fz_lim below is a read-only
                      % alias of this (sheet class DERIVED), so a sheet can only ever
                      % move the enforced constraint by moving Fz_abs.
vp.Fz_work = 7100;    % per-tyre working limit on smooth ground (7000-7200 band).
                      % NOT sheet-wired: diagnostic only, not an NLP constraint.
                      % As a path constraint it imposed a lateral-g ceiling with no
                      % mu term,
                      %     ay_cap = 2*(Fz_lim - Fz_tot/4)*t/(m*hcg)
                      % which binds at every speed and TIGHTENS as downforce grows
                      % (High aero: 1.41 g at 30 m/s -> 0.58 g at 70 m/s, against a
                      % tyre capable of 1.50 -> 1.99 g; above ~76 m/s the rear tyre
                      % exceeds 7100 N in a straight line). It, not the tyre model,
                      % was setting the lap time, and it is why more downforce made
                      % the car slower. Whether 7100 is a peak dynamic limit or a
                      % static/durability figure is unresolved; if it is genuinely a
                      % peak limit, "the car cannot run High aero at speed" is a
                      % design finding, not a constraint to relax.
vp.Fz_lim = vp.Fz_abs;  % <- ENFORCED value (Scripts/MLTP.m fz_lim_* path constraints).
                      % Post-process against vp.Fz_work to report exceedance.

%% Aerodynamics - ride-height aero map
% Baseline CL_f/CL_r come from the ride-height map (MID wing
% position), quasi-statically collapsed onto speed-dependent coefficient
% curves by Functions/aeroCollapse.m: aero load compresses the springs ->
% ride heights drop -> map returns new CL; clamped at the map edges
% (physically: bump stops / platform on the floor - bump-stop data requested).
% PINNED TO THIS CHECKOUT'S ROOT, deliberately (same reason as Scripts/userOpts.m):
% a bare exist/load('runOverride.mat') resolves through the WHOLE MATLAB search path,
% so a stray runOverride.mat sitting in another checkout on the saved path hijacks
% runs started from this one. NOT pwd: userOpts.m reaches this file through
% run('Parameters\vehParams.m'), and run() cd's into Parameters\ when the name it is
% given has a directory component - a pwd-pinned lookup would look for
% Parameters\runOverride.mat and silently find nothing.
% Distinct variable names (vpOvrFile/vpOvr) because this script executes INSIDE its
% callers' workspaces: the aero validation gates
% both keep their own `ovrFile` alive across a vehParams call, and userOpts.m keeps
% its own `ovr` alive (it reads circuit/vi from it AFTER this script returns) - so
% neither may be shadowed or cleared here.
vpOvrFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'runOverride.mat');   % ...\Parameters -> repo root
vpOvr = struct; if isfile(vpOvrFile), vpOvr = load(vpOvrFile); end
clear vpOvrFile
% vp.ActAero is set by userOpts.m (AeroConfig switch) before this script runs
% in the normal pipeline. Some standalone callers (the aero gates)
% invoke this script directly without going through userOpts.m first, so guard
% with getfielddef rather than a bare vp.ActAero == 1 - undefined must fall back
% to the static (0) path, i.e. today's behaviour, unchanged.
if getfielddef(vp,'ActAero',0) == 1
    % ActiveRW: rear-wing angle is a
    % continuous NLP control. Own run identity so ARW runs never collide with the
    % static Low/Mid/High/RWp15 init-cache/CSV/solutions naming - this
    % OVERRIDES any vpOvr.aeroSetting override (ActiveRW batch overrides must use a
    % different field, e.g. runOverride.mat's AeroConfig, not aeroSetting).
    %
    % The velocity-schedule study splits that identity
    % two ways by vp.rwMandate, set in Scripts\userOpts.m BEFORE this script runs,
    % so each variant gets its own init cache, apex CSV and solutions\ folder and
    % neither can collide with the other or with a static setting. The retired
    % braking/force-optimal mandates (modes 1 'ARWm' and 2 'ARWf',
    % the discrete/mandated variants) were removed and
    % are rejected in userOpts.m, so only 0, 3, 4, 5 and 6 are reachable. RWDiscrete/
    % vp.rwSnap never entered the name: the snap penalty was a homotopy over
    % vp.rwSnapRho on the SAME problem. It was retired 2026-08-04 too and 'Snap'
    % is now rejected in userOpts.m, so vp.rwSnap is always 0 regardless.
    % getfielddef, not a bare vp.rwMandate: standalone callers can run this
    % script without going through userOpts.m and must land on the plain 'ARW'.
    switch getfielddef(vp,'rwMandate',0)
        case 0, vp.aeroSetting = 'ARW';     % continuous rear-wing control (reference)
        case 3, vp.aeroSetting = 'ARWv';    % + velocity schedule (-10/+10) & +15 deg brake trigger
        case 4, vp.aeroSetting = 'ARWd';    % + discrete-station rounding & +15 deg airbrake floor
        case 5, vp.aeroSetting = 'AFWd';    % combined FW+RW unload axis & 0 deg airbrake-analogue floor
        case 6, vp.aeroSetting = 'ARFWd';   % TWO wings: RW control + FW-only flap (nu +1)
        case 7, vp.aeroSetting = 'ARFWr';   % reactive dual-wing schedule (nu +1)
        otherwise
            error('vehParams:rwMandate', ...
                'unknown vp.rwMandate %g (expected 0 | 3 | 4 | 5 | 6 | 7)', vp.rwMandate);
    end
else
    % PRECEDENCE, deliberate: runOverride.mat  >  setup sheet  >  this default.
    % runOverride.mat has to win. It is how a batch driver sweeps the wing
    % setting run-by-run, so if a setup sheet outranked it every run of a sweep
    % would silently collapse onto one wing angle - the batch would still produce
    % a full-looking comparison table of identical aero. The sheet's job here is
    % to state the car's nominal setting, which is exactly what a per-run
    % override is entitled to replace. Note the ActiveRW branch above overrides
    % BOTH (its 'ARW*' run identity is not a setup choice), so a sheet row for
    % aeroSetting is inert under AeroConfig = 'ActiveRW'.
    vp.aeroSetting = getfielddef(vpOvr,'aeroSetting', setupValue('aeroSetting','Mid')); % 'Low'|'Mid'|'High'|'RWp15'  <- the per-run choice
end
clear vpOvr
vp.RHf0 = setupValue('RHf0', 85);  % nominal front ride height at v = 10 m/s   (mm)
vp.RHr0 = setupValue('RHr0', 95);  % nominal rear  ride height at v = 10 m/s   (mm)
                                   % PLACEHOLDERS - not measured; the source sheet left
                                   % these blank. Both feed aeroCollapse directly.
vp.Cd0  = setupValue('Cd0', 0.40); % aero sheet, ~constant with ride height
                                   % (an older sheet said 0.32; unresolved)

% aero map load shared by both the static collapse (below) and the ActiveRW
% 2-D (alpha,v) map builder (Functions/rwAeroMap2D.m) - same source file,
% loaded once.
aeroMapTmp = load(fullfile(fileparts(mfilename('fullpath')), 'aeroMap_Tur.mat'));

if getfielddef(vp,'ActAero',0) == 1
    % Continuous (alpha, v) rear-wing map: each node reduces EXACTLY to the
    % static setting at that angle (bit-identical col{i}/aeroEvalNum), so
    % alpha = 0 reproduces static 'Mid' bit-for-bit. Functions/rwAeroMap2D.m
    % carries the full contract: basis, node set, and the off-node accuracy
    % limits.
    %
    % Two mandates build the same 2-D machinery over a different axis instead
    % of the rear-wing sweep. Mode 5 uses the combined front+rear "unload axis"
    % (3 nodes, Functions/fwAeroDelta.m); mode 6 adds a front-wing-only delta
    % layer (Functions/fwOnlyDelta.m). Both go through rwAeroMap2D's node/delta
    % override inputs - it is node-count generic, so neither needs a new map
    % builder. The empty basis argument selects rwAeroMap2D's own default
    % ('hermite3tanh'), so every branch lands on the identical basis.
    %
    % liftMode = 'additive' is required for the mode-5 nodes: the -25 deg node
    % never reaches aeroCollapse's ride-height clamp on either axle
    % (aeroCollapse:nanFit) and -20 deg fails the fit-residual gate
    % (aeroCollapse:fitResidual). It layers each node's lift delta as a constant
    % shift over one shared baseline collapse instead of re-solving the
    % ride-height fixed point per node - a bounding approximation. aeroCollapse.m
    % and fwAeroDelta.m are both UNCHANGED by it.
    if getfielddef(vp,'rwMandate',0) == 5
        fwNodes = [-25 -20 0];
        [dClfA_fw, dClrA_fw, dCdA_fw] = fwAeroDelta(fwNodes);
        vp.aeroARW = rwAeroMap2D(aeroMapTmp, vp, '', ...
                                 fwNodes, [dClfA_fw; dClrA_fw; dCdA_fw], 'additive');
        clear dClfA_fw dClrA_fw dCdA_fw
    elseif any(getfielddef(vp,'rwMandate',0) == [6 7])
        % ARFWd/ARFWr: standard RW map UNCHANGED, plus an FW-only delta layer
        % (Functions/fwOnlyDelta.m) as a SECOND additive 3-node map on its own
        % axis. Same override seam + liftMode as AFWd above; consumers read only
        % .dClFadd/.dClRadd/.dCdA/.basis of aeroAFW (its speed curves are unused).
        vp.aeroARW = rwAeroMap2D(aeroMapTmp, vp);
        fwNodes = [-25 -20 0];
        [dClfA_fw, dClrA_fw, dCdA_fw] = fwOnlyDelta(fwNodes);
        vp.aeroAFW = rwAeroMap2D(aeroMapTmp, vp, '', ...
                                 fwNodes, [dClfA_fw; dClrA_fw; dCdA_fw], 'additive');
        clear dClfA_fw dClrA_fw dCdA_fw
    else
        vp.aeroARW = rwAeroMap2D(aeroMapTmp, vp);
    end

    if getfielddef(vp,'rwMandate',0) == 7
        % ARFWr: reactive law, single tracked owner. vpOvr pattern N/A here.
        rlF = fullfile(fileparts(mfilename('fullpath')), 'reactiveLaw.mat');
        assert(exist(rlF,'file') == 2, 'vehParams:reactiveLaw', ...
            'Parameters/reactiveLaw.mat missing - it ships with this distribution; restore it.');
        rlS = load(rlF, 'law');
        vp.reactLaw = rlS.law; clear rlF rlS
        if vp.reactLaw.provisional == 1
            warning('vehParams:reactiveLawProvisional', ...
                'reactiveLaw.mat is the provisional default - re-fit before quoting solves');
        end
    end

    % numeric conveniences at the reference corner speed, evaluated at the 0 deg
    % (Mid) node -> the ARW warm-start model is identical to static Mid's
    % (vehModel_initial.m: max(vp.Cl)/max(vp.Cd)).
    v_ref = 45;                                                   % (m/s)
    [clf_ref, clr_ref, dCdA_ref] = rwAeroMapEvalNum(vp.aeroARW, 0, v_ref);
    vp.Cl      = clf_ref + clr_ref;
    vp.Cd      = vp.Cd0 + dCdA_ref/vp.A;
    vp.aeroBal = clf_ref/(clf_ref + clr_ref);
    clear v_ref clf_ref clr_ref dCdA_ref
else
    % Rear-wing angle sweep, digitised from the supplier data. The named
    % pipeline settings select a discrete rear-wing angle of attack; Low/Mid/High
    % are kept as aliases so batch/report code keeps working, and Mid == 0 deg is
    % the force-identical reproduction of the previous Mid model.
    switch vp.aeroSetting
        case 'Low',   vp.rwAngle = -10;    % rear wing shallow / low downforce
        case 'Mid',   vp.rwAngle =   0;    % reference wing (all RW-angle deltas = 0)
        case 'High',  vp.rwAngle =  10;    % rear wing steep / high downforce (pre-stall)
        case 'RWp15', vp.rwAngle =  15;    % RW separated (post-stall data point)
        % NOTE: the +20 deg wing angle is gone at every level, in two steps.
        % 2026-08-04 retired the 'RWp20' RUN setting (this switch). 2026-08-05 then
        % retired the DATA POINT itself: it is no longer a node of the ActiveRW 2-D
        % map (Functions/rwAeroMap2D.m, now the four nodes [-10 0 10 15]) and no
        % longer a row of Functions/rwAeroDelta.m's interpolation table, whose valid
        % range is [-10, 15] - rwAeroDelta(20) is an error, not a value. The raw
        % digitised slide row survives only as prose in that file's header, kept for
        % provenance. See
        % the four-node map.
        otherwise
            error('vehParams:aeroSetting', ...
                'unknown vp.aeroSetting ''%s'' (expected Low|Mid|High|RWp15)', vp.aeroSetting);
    end

    % Force-equivalent coefficient increments at the code reference area vp.A.
    % rwAeroDelta returns coefficient*area DELTAS vs the 0 deg wing, so dividing by
    % vp.A yields effective increments that reproduce the slide FORCES exactly
    % despite the slide's per-angle frontal-area variation (deltas-only decision:
    % absolute Cd/Area/Cl baselines are NOT adopted). All three are zero at Mid.
    [dClfA, dClrA, dCdA] = rwAeroDelta(vp.rwAngle);
    vp.dClF   = dClfA/vp.A;             % effective front-axle CL increment          (-)
    vp.dClR   = dClrA/vp.A;             % effective rear-axle  CL increment          (-)
    vp.dCd_rw = dCdA/vp.A;              % effective drag increment (routes via f_dragRW at vp.hw)
    clear dClfA dClrA dCdA

    % collapse the map for the selected wing setting (speed grid to a literal
    % 125 m/s ~ 450 km/h: Powertrain/pt.Vmax is not loaded yet at this point). The
    % RW-angle increments feed the collapse exactly where the old per-setting dCl
    % placeholders did.
    vp.aero.sel = aeroCollapse(aeroMapTmp, vp.RHf0, vp.RHr0, 2*vp.kwf, 2*vp.kwr, ...
                  vp.dClF, vp.dClR, vp.rho, vp.A, 125);

    % numeric conveniences at a representative corner speed -> consumed by the
    % constant-coefficient init model (vehModel_initial.m: max(vp.Cl)/max(vp.Cd))
    v_ref = 45;                                                   % (m/s)
    [clf_ref, clr_ref] = aeroEvalNum(vp.aero.sel, v_ref);
    vp.Cl      = clf_ref + clr_ref;
    vp.Cd      = vp.Cd0 + vp.dCd_rw;
    vp.aeroBal = clf_ref/(clf_ref + clr_ref);                     % now an output of the map
    clear v_ref clf_ref clr_ref
end
clear aeroMapTmp

% lateral load transfer distribution
vp.lty_dis_f = (vp.huf/vp.t)*(vp.muf/vp.m) + ((vp.l_r*vp.hRCf)/(vp.l*vp.t))*(vp.ms/vp.m) + ((vp.ksf_rad*(vp.d/(vp.ks_rad - vp.ms*vp.g*vp.d)))/vp.t)*(vp.ms/vp.m);   % fraction of load transfer reacted at front axle
vp.lty_dis_r = (vp.hur/vp.t)*(vp.mur/vp.m) + ((vp.l_f*vp.hRCr)/(vp.l*vp.t))*(vp.ms/vp.m) + ((vp.ksr_rad*(vp.d/(vp.ks_rad - vp.ms*vp.g*vp.d)))/vp.t)*(vp.ms/vp.m);   % fraction of load transfer reacted at rear axle
vp.lty_dis = vp.lty_dis_r / (vp.lty_dis_f + vp.lty_dis_r);

vp.Tbrake_max = setupValue('Tbrake_max', 7e3);  % <- THE LIVE ASSIGNMENT (the 4e3 near the top of this file is dead).
                      %the total 4-wheel braking torque is 2×T_brake, so the current 4 kNm cap = 8 kNm total. At the grip limit with downforce (ΣFz up to ~28 kN, μ ≈ 1.2, Rw 0.36–0.37) the tyres can react ~12–13 kNm — the current cap would artificially limit braking. Size it so 2×Tbrake_max comfortably exceeds the grip-limited value (~7 kNm as the input scale works).