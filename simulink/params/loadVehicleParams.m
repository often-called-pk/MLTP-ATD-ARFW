function [vp, pt] = loadVehicleParams()
%LOADVEHICLEPARAMS Run the MLTP repo's vp/pt param scripts inside THIS
%function's workspace and return the resulting structs, pinned to the
%ARFWr (reactive dual-wing) identity the real-time Simulink model targets.
%
%   [vp, pt] = loadVehicleParams()
%
% Parameters/vehParams.m and Parameters/Powertrain.m are `run`-style
% scripts, not functions - they build vp/pt as plain
% assignments in whatever workspace executes them. run() does not open a
% new workspace: it executes the script's statements in the CURRENT one,
% exactly as if they had been typed there. Calling run() from inside this
% function therefore builds vp/pt as ordinary LOCAL VARIABLES of
% loadVehicleParams(), which the `function [vp,pt] = ...` output line then
% returns. This is the same trick Validation/validateActiveRW.m's Check 6
% uses (`vp.ActAero = 1; vehParams;`) to call vehParams.m from inside a
% function body - see that file around line 480-483 for a second working
% example of the pattern. Here the calls use an ABSOLUTE path built from
% this file's own location so cwd never matters; vehParams.m/Powertrain.m
% anchor all of THEIR OWN internal file lookups (aeroMap_Tur.mat,
% reactiveLaw.mat, runOverride.mat, ...) on mfilename('fullpath') rather
% than pwd, so calling them by absolute path from a nested folder like
% simulink/params/ is safe and matches how Scripts/userOpts.m calls them.
%
% ---- WHY vp.ActAero / vp.rwMandate ARE PRE-SET BELOW (design note) ----
% Parameters/vehParams.m's ActiveRW branch (its own lines ~163, ~182-192)
% reads:
%     if getfielddef(vp,'ActAero',0) == 1
%         switch getfielddef(vp,'rwMandate',0)
%             ...
%             case 7, vp.aeroSetting = 'ARFWr';
% i.e. it reads the WORKSPACE STRUCT FIELDS vp.ActAero / vp.rwMandate
% directly - NOT a variable called `ovr`, and NOT runOverride.mat (that
% file only feeds vp.aeroSetting on the STATIC branch, vp.ActAero==0,
% which ARFWr never takes; see vehParams.m's own comment at its line
% ~194-203). In the normal Scripts/ pipeline these two fields are set by
% Scripts/userOpts.m's `AeroConfig='ActiveRW'` / `RWMandate='Reactive'`
% switches (userOpts.m lines ~44-52 and ~115-125) BEFORE it calls
% run('Parameters\vehParams.m'). This function reproduces exactly those
% two resulting numeric values (vp.ActAero=1, vp.rwMandate=7) as plain
% pre-assignments, without going through userOpts.m's track/boundary-
% condition/solver machinery at all - this bridge stops at vp/pt, which is
% all this vp/pt bridge needs to do. It never writes runOverride.mat (forbidden here -
% that file is a shared, solver-side batch-sweep seam).
%
% pt.ATD is deliberately NOT set here: grep of Parameters/vehParams.m and
% Parameters/Powertrain.m shows neither reads pt.ATD - it is consumed only
% by Scripts/MLTP.m's control-dimension (nu/nh) ladder, which is out of
% scope for this vp/pt bridge.
%
% Requires Parameters/ and Functions/ on the MATLAB path (getfielddef,
% setupValue, aeroCollapse, rwAeroMap2D, tyreParams_Zenvo, ... all
% live in Functions/ and are called bare by vehParams.m/Powertrain.m).
% simulink/startup/arfwr_startup.m (registered as the ARFWr_RT project's
% Startup File) adds both folders to the path automatically when the
% project is open; this function does not repeat that path setup so path
% management and param loading stay single-owner. If
% Parameters/vehParams.m is not resolvable, open ARFWr_RT.prj first (or
% addpath the two folders manually).
%
% Depends on Parameters/reactiveLaw.mat existing (the vp.rwMandate==7
% branch, Parameters/vehParams.m lines ~264-275, asserts on it and names
% the fitReactiveLaw command to regenerate it if missing) - present in
% this checkout.
%
% NOTE: after any change to Parameters/vehParams.m's identity switch, verify
% that vp.aeroSetting == 'ARFWr' and vp.rwMandate == 7 still hold after the
% calls below, and update the two preset fields if the switch has moved. The
% presets were last checked against vehParams.m as it stood on 2026-08-31.

here = fileparts(mfilename('fullpath'));           % ...\simulink\params
root = fileparts(fileparts(here));                  % ...\simulink\params -> repo root

if ~isfile(fullfile(root,'Parameters','vehParams.m'))
    error('loadVehicleParams:repoNotFound', ...
        'Parameters/vehParams.m not found under %s -- unexpected repo layout.', root);
end

% Pre-set the two identity fields vehParams.m's ActiveRW branch switches
% on (see design note above). Mirrors what Scripts/userOpts.m sets for
% AeroConfig='ActiveRW', RWMandate='Reactive':
vp.ActAero   = 1;   % -> ActiveRW branch                  (Parameters/vehParams.m:163)
vp.rwMandate = 7;   % -> vp.aeroSetting = 'ARFWr'          (Parameters/vehParams.m:188)

run(fullfile(root,'Parameters','vehParams.m'));     % builds/extends vp (adds ~300 fields)
run(fullfile(root,'Parameters','Powertrain.m'));    % builds pt; requires vp.Rw, so must run
                                                     % after vehParams.m (Powertrain.m:20-23
                                                     % asserts vp.Rw exists)

% ---- two things vehParams.m builds but does not keep on vp -------------
% Plant.slx's live ride-height aero block needs BOTH, and neither can be
% recovered from vp as vehParams.m leaves it.
%
% (1) THE RAW RIDE-HEIGHT LIFT GRID. Parameters/vehParams.m:215 loads
%     Parameters/aeroMap_Tur.mat into a LOCAL variable and `clear`s it at
%     :337 once vp.aeroARW/.aeroAFW are built, so the grid the offline
%     builders interpolate is not reachable from vp. The RT plant indexes
%     that same grid at its INSTANTANEOUS (RHf,RHr) instead of at a
%     quasi-statically collapsed one -- see simulink/aero/aeroLiveEval.m's
%     header for why the two evaluation paths differ. Loading it here, ONCE
%     at project startup, is what keeps `load` out of the compiled MATLAB
%     Function block: Plant.slx's aero Constant blocks hold only the
%     EXPRESSIONS vp.aeroMapRaw.CLf / .CLr / .RHf / .RHr, resolved against
%     the base workspace at model load -- the same by-name pattern every
%     other Params constant in that model uses.
vp.aeroMapRaw = load(fullfile(root,'Parameters','aeroMap_Tur.mat'));
assert(all(isfield(vp.aeroMapRaw, {'RHf','RHr','CLf','CLr'})), ...
    'loadVehicleParams:aeroMapFields', ...
    'Parameters/aeroMap_Tur.mat is missing RHf/RHr/CLf/CLr');

% (2) THE UNLOADED (v = 0) RIDE HEIGHTS. vp.RHf0/vp.RHr0 are the nominal
%     heights AT v = 10 m/s (Functions/aeroCollapse.m:76-79 anchors its
%     fixed point there), whereas Plant.slx's suspension defines zero spring
%     compression as the STATIC, no-aero state. aeroCollapse already solves
%     the difference and stores the result as col{i}.raw.RHfU/.RHrU, so the
%     plant reads it back from the alpha = 0 node rather than recomputing
%     the anchor formula in a second place. The offset is small (~0.4 mm
%     front, ~0.3 mm rear) but it is the one build_plant.m's Suspension
%     description flagged as an open gap for the live-aero plant to close.
iMid = find(vp.aeroARW.alphaNodes == 0, 1);
assert(~isempty(iMid), 'loadVehicleParams:noMidNode', ...
    'vp.aeroARW.alphaNodes has no 0 deg node -- cannot locate the ride-height anchor');
vp.RHfU = vp.aeroARW.col{iMid}.raw.RHfU;   % front ride height at v = 0 [mm]
vp.RHrU = vp.aeroARW.col{iMid}.raw.RHrU;   % rear  ride height at v = 0 [mm]

end
