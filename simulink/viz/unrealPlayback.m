function info = unrealPlayback(src, varargin)
%UNREALPLAYBACK  Replay a logged ARFWr_Sim lap in the sim3d/Unreal engine.
%
%   info = UNREALPLAYBACK(src)
%   info = UNREALPLAYBACK(src, Name, Value, ...)
%
%   Drives the 3D demo the project exists to show: the Zenvo on the Barcelona
%   ribbon with BOTH aero surfaces visibly moving -- rear wing flat for low
%   drag down the straights, both surfaces loaded through the corners, rear
%   wing to its +15 deg airbrake station under braking -- each surface rotated
%   by the LOGGED actuator position and carrying a station-coloured band along
%   its own trailing edge. The four ROAD WHEELS turn with the logged wheel
%   speeds and the front pair steers with the logged road-wheel angle, and an
%   optional HUD burns speed, steer, indicated gear, brake and throttle into
%   every recorded frame.
%
%   ONE CAR, ONE ROUTE (2026-09-17). Every alternative body this function used
%   to carry is gone, and with them the body-selection option, the variant
%   option, the attribution-overlay option and the shipped-mesh branch. What
%   is drawn is built by simulink/viz/zenvoCarActors.m and nothing else -- see
%   ZENVO ROUTE below.
%
%   src  one of
%          - a struct from runDemoLap (uses src.logsout)
%          - a Simulink.SimulationData.Dataset (a logsout)
%          - a char/string path to a .mat holding either of the above
%          - '' or omitted: simulink/work/t53_lap.mat if it exists
%
%   Name-value options
%     'Scene'      char, sim3d prebuilt scene. Default 'Empty scene'. Only
%                  four scenes ship with the R2025a engine (see NOTES).
%     'Window'     [t0 t1] s of LOGGED time to replay. Default [] = the whole
%                  logged SPAN -- which is the model's StopTime (150 s), NOT
%                  the ~139.4 s lap. CORRECTED CLAIM: this header used to say
%                  runDemoLap's trimToLapWindow had already trimmed it. It has
%                  not, and cannot: trimToLapWindow calls getsampleusingtime on
%                  each element inside a try, and a logged BUS element's
%                  .Values is a struct of timeseries with no such method, so
%                  it throws, is caught, and the bus is left whole
%                  (trimToLapWindow.m:15-18 documents the behaviour). VehState
%                  is a bus, so the pose channels this function reads are
%                  untrimmed. Pass 'Window' explicitly if frame 1 must be the
%                  lap start; the HUD lap clock reads LapInfo's own tLap and
%                  so is right either way.
%     'Speed'      playback rate. 1 = real time, 2 = twice as fast. Default 1.
%     'Fps'        engine sample rate AND video frame rate. Default 20.
%     'Record'     path to an .mp4 to write (repo-root relative or absolute).
%                  Default '' = do not record. Recording adds an
%                  sim3d.sensors.IdealCamera and writes each frame as it
%                  arrives, so memory does not grow with lap length.
%     'Stills'     [1xK] LOGGED times at which to also save a PNG next to the
%                  video, named <video>_Zenvo_t<sec>.png. The car's name stays
%                  in the filename even now that there is only one car: a
%                  frame left behind by an older body would otherwise be
%                  indistinguishable from a fresh one at the same logged time,
%                  which is exactly how a render that never ran gets read as
%                  evidence. Default [].
%     'ImageSize'  [h w] of the recorded frames. Default [720 1280].
%     'Camera'     which view is rendered and recorded. All ride with the car;
%                  the four REAR views are aimed by the car's heading alone,
%                  the two added 2026-09 aim their rig back at the car (see
%                  the camera block in the body):
%                    'quarter' (default) close 3/4 rear -- wing and deck
%                    'chase'   11 m behind, 4 m up -- rear wing dominant
%                    'low'     low 3/4 at ground level, most dramatic
%                    'high'    elevated chase, shows the corner ahead
%                    'front'   front 3/4 from the car's RIGHT -- the only view
%                              that shows the STEERED front wheel, the front
%                              flap AND the rear wing over the roof at once
%                    'side'    right-side-on -- both rims face the lens, so
%                              wheel SPIN is legible frame to frame
%                  The four rear views cannot show either: the body occludes
%                  the front wheels and the front flap from every one of them.
%     'ShowWindow' logical, default true. False renders off screen (still
%                  records, no viewer window).
%     'Ribbon'     a struct from buildTrackRibbon, to avoid rebuilding it.
%     'vp'         vehicle-parameter struct, handed straight to zenvoCarActors.
%                  The axle stations (vp.l_f / vp.l_r), the wheel radius and
%                  the track are what the imported body is scaled and placed
%                  onto, and what the builder asserts its own measured hubs
%                  against. Default [] = fetch 'vp' from the base workspace
%                  (see Requires).
%     'Wheels'     logical, default true. Pose the four road wheels: spin from
%                  the logged per-corner wheel speeds, steer (front pair) from
%                  the logged road-wheel angle. False restores the pre-2026-09
%                  behaviour, which rendered the car sliding at 88 m/s on
%                  frozen wheels (measured: 48 m of travel, zero rim rotation).
%     'SpinMode'   'apparent' (default) or 'physical'. See WHEEL SPIN below.
%                  'physical' integrates the logged wheel speed as it is;
%                  'apparent' applies a documented COSMETIC compression so the
%                  rim reads as forward rotation at every speed instead of
%                  strobing. Steer is never scaled, in either mode.
%     'HUD'        logical, default false. Composite a driver-instrument
%                  overlay (speed, road-wheel steer, indicated gear, brake %,
%                  throttle %, both wing angles, lap clock) into every RECORDED
%                  frame -- video and stills alike -- via hudOverlay. Needs
%                  Computer Vision Toolbox; if the licence cannot be checked
%                  out this warns once, before the render, and continues with
%                  the HUD off. Default OFF so the certified bare-render path
%                  is never changed by accident.
%     'pt'         powertrain struct for the HUD's indicated gear. Default
%                  [] = fetch 'pt' from the base workspace; if neither is
%                  available the gear tile draws '--' and everything else in
%                  the HUD still works.
%
%   info  struct: nFrames, tSpan, wall, videoPath, stillPaths, ribbonStats,
%         wheels, spinMode, spinGainInfo, hud, and the per-frame arrays
%         actually sent to the engine (info.frames).
%
%   Requires  a vehicle-parameter struct for the car build, either passed as
%             'vp' (see above) or already sitting in the base workspace as
%             'vp' -- opening simulink/ARFWr_RT.prj loads it via projStartup.
%             Without one of the two, this errors clearly rather than
%             hard-failing inside the builder.
%
% ---------------------------------------------------------------------------
% FRAME CONVENTIONS (the thing most likely to be got wrong)
%
%   The plant, DriverPath and VehStateBus all use ISO 8855: x forward, y LEFT,
%   z up, psi positive counter-clockwise. sim3d actors accept exactly that
%   AXIS convention when their CoordinateSystem property is set to 'ISO8855',
%   which every POSED actor here does -- so x/y/z and the SENSE of
%   roll/pitch/yaw go in unconverted.
%
%   UNITS, and the one thing this file got wrong for its whole first life:
%   Actor.Translation is metres, but Actor.Rotation is DEGREES -- in ISO8855
%   mode too. CoordinateSystem re-maps the AXES, it does not switch the unit
%   to the radians the ISO convention is normally written in. Measured, not
%   assumed: ten identical plates commanded Rotation = [0 v 0] for
%   v = 0,10,...,90 render as a clean monotone protractor fan from flat to
%   vertical, while the same ten fed deg2rad(v) render as ten flat slats.
%   Feeding radians therefore does not mis-pose the car by a little, it
%   scales every angle by 180/pi ~ 57: a whole lap's yaw sweep of -6.4 rad
%   arrives as -6.4 DEG, so the car ploughs straight on while the ribbon
%   turns, and a +15 deg wing arrives as 0.26 deg, i.e. a flat slab. Both
%   were live defects. Colour, being unitless, kept working throughout --
%   which is exactly why the symptom looked like "some writes reach the
%   engine and some do not" rather than like a unit error.
%
%   So: pose the car with rad2deg([roll pitch psi]), and hand the two aero
%   surfaces F.aRW / F.aFW, which the log already carries in DEGREES -- no
%   deg2rad on either path.
%
%   WHEELS: same DEGREES rule, and on a HUB CHAIN rather than a pose matrix.
%   Each corner is its own actor under the car root, so every wheel row is an
%   ordinary ISO [roll pitch yaw] and there is no swapped column order to get
%   wrong (the shipped-mesh body that carried a 5x3 pose, with [spin camber
%   steer] on its wheel rows, is gone). Sign senses, all measured, all
%   DEGREES, all body-frame -- the hubs are children of the root, so they
%   follow the car through a yaw:
%     SPIN    Spin<corner>.Rotation = [0 spin 0]. + rolls the wheel FORWARD
%             (top of the wheel toward the nose). Matches VehStateBus.Om_*,
%             which is + for forward rolling, so the integrated angle goes in
%             with NO sign flip.
%     STEER   Steer<corner>.Rotation = [0 0 delta], front pair only. + turns
%             the wheel LEFT (CCW seen from above), the same sense as
%             DriverCmdBus.steer, so that also goes in with no sign flip --
%             only rad2deg.
%     CAMBER  never written. VehStateBus carries no camber signal and the
%             root's own roll already tilts the car.
%   Spin and steer compose by construction, not by luck: the steer hub is the
%   PARENT of the spin hub, so the wheel PLANE turns and the rim rolls inside
%   it. The builder puts each wheel's mesh on its own hub station, so no
%   translation is written here at all -- this file writes Rotation only.
%
% WHEEL SPIN: PHYSICAL vs APPARENT (a cosmetic choice, stated not hidden)
%
%   The wheel is a rate, the actor wants an angle, and the angle must be
%   integrated on the LOG time base and only then resampled -- integrating on
%   the 20 fps frame grid is wrong by about a revolution per frame.
%
%   Even integrated correctly, HONEST spin cannot read as forward rotation at
%   this frame rate. At 88 m/s and vp.Rw = 0.37 the wheel turns ~1.9
%   revolutions between frames, so a 20 fps camera samples it as a slow
%   BACKWARD crawl -- the wagon-wheel effect, faithful to the log and useless
%   as a demo. Raising Fps to 60 only moves the alias-free ceiling from about
%   4.7 m/s to 14 m/s, nowhere near a 88 m/s lap.
%
%   So 'SpinMode' picks which is wanted, and 'apparent' is the DEFAULT because
%   this function's product is a demo video:
%     'physical'  angle = cumtrapz(tLog, Om). Truthful, strobes, and at racing
%                 speed usually looks like slow reverse rotation.
%     'apparent'  the spin RATE is passed through a monotone compression,
%                 omVis = omCap*tanh(Om/omHalf), before integration. omHalf is
%                 the window's own median in-motion |Om|, so the map is close
%                 to proportional below that and saturates above it -- faster
%                 still looks faster, but never faster than omCap, which is
%                 fixed at 30 deg per FRAME (under the 36 deg that a 5-spoke
%                 rim could alias). It stops when the car stops and reverses
%                 when the wheel reverses. It is NOT a physical wheel speed
%                 and must never be measured off the video; the HUD writes a
%                 standing caveat line into the frame whenever it is active.
%   STEER is never scaled in either mode -- it is small (peak ~11 deg on this
%   lap), it does not alias, and it is the part a viewer reads for real.
%
%   The logged wheel speeds go briefly NEGATIVE in two short bursts
%   (t ~ 67.3-68.1 s and ~115.8-116.7 s, 0.3-0.6 % of in-lap samples). The
%   wheels visibly reverse there in BOTH modes. That is the log, not a bug.
%
%   Actor MESH data is the exception: Actor.Vertices is in the actor's own
%   local frame, which is Unreal's y-RIGHT convention and is NOT re-mapped by
%   CoordinateSystem. buildTrackRibbon's default 'Unreal' mode returns the
%   ribbon already flipped and re-wound for that, so the ribbon actor is added
%   with an identity transform and lines up with ISO-posed actors.
%
%   VISUAL WING CONVENTION (documented, not inferred). The REAR wing is
%   rotated about the vehicle's lateral axis by +alpha, the raw logged angle.
%   In ISO the lateral axis is +y (left), so a positive rotation lifts the
%   TRAILING edge -- i.e. a bigger rear-wing angle visibly stands the wing up,
%   which is the direction that adds downforce and drag, and the +15 deg
%   airbrake is the steepest it ever gets. The delivered donor geometry IS
%   station 0, so that raw angle needs no offset.
%
%   THE FRONT SURFACE REFINES THAT RULE, and only the front. The flap is not
%   a free plate on a stalk: it is the car's own lower-bumper vent band,
%   hinged at its upper edge, so "loaded" and "unloaded" read as a swing of
%   the free edge OUT of the bumper rather than as a tilt of a floating slab.
%   The logged alphaFW_pos is therefore passed through the builder's own
%   HC.fwHingeDeg map before it reaches the hinge (see the write site in
%   onOutput). The two end angles are owned by zenvoCarActors, which also
%   answers the static 'hingeMap' query, so there is exactly one copy of them.
%
% ZENVO ROUTE (the only route)
%
%   simulink/viz/zenvoCarActors.m builds the car under one ISO8855 root and
%   hands back every handle posed below. Three real parts, no box plates:
%     * BODY -- the Zenvo Aurora TUR mesh, whole, in its delivered textures.
%     * REAR WING -- box-extracted from the Aurora AGIL mesh's carbon shell:
%       the PLANE (which rotates, about the donor's own pylon-top pivot line)
%       and the PYLONS (static) as separate actors, transplanted onto the
%       Tur's measured deck.
%     * FRONT FLAP -- the Tur's OWN lower-bumper vent band, split face by face
%       out of the nose shells and re-hung on a hinge, with its slot culled
%       from the bodywork it came out of, so nothing is ever drawn twice.
%   Each moving surface carries a 5 cm STATION BAND along its trailing (rear
%   wing) or lower (flap) edge, and those two bands are the only thing this
%   file recolours.
%
%   Both meshes are resolved through assetManifest('Zenvo'), which is the only
%   place a folder name, node name, extraction box, hub station, pivot line or
%   scale factor is written down. Provenance, licence and the modification
%   list travel with the assets:
%   simulink/viz/assets/zenvo_tur/LICENSE.md, and the donor's own LICENSE.md
%   beside it in assets/zenvo_agil.
%
%   WHAT THIS BODY IS, AND IS NOT -- read this before putting a frame in a
%   report. It is a third-party GAME MESH of the Zenvo Aurora. It is not
%   supplier CAD, not a wind-tunnel shape and not a measurement of any kind.
%   Nothing may be measured off it: not the overhangs, not the frontal area,
%   not the wing chord, not the ride height, and not the flap's real travel.
%   What IS project data on this body is the axle stations (vp.l_f / vp.l_r),
%   the wheel radius and the track it is scaled onto and asserted against, and
%   the two logged actuator ANGLES its surfaces are drawn at.
%
%   Per frame this function writes, in DEGREES (Rotation is DEGREES here as
%   everywhere else in this file -- see the UNITS note above):
%     HC.root                  Translation [x y z] m,
%                              Rotation rad2deg([roll pitch psi])
%     HC.steer.FL/FR           Rotation [0 0 F.delta(k)]
%     HC.spin.FL/FR/RL/RR      Rotation [0 F.spin(k,i) 0]
%     HC.rwPlate (WingRW)      Rotation [0 F.aRW(k) 0] -- the PIVOT actor. The
%                              mesh-carrying child is never written to: the
%                              yaw that puts the donor mesh on ISO and the
%                              pitch about the pivot do not commute, so the
%                              pitch has to live on the parent.
%     HC.fwHinge (WingFWHinge) Rotation [0 HC.fwHingeDeg(F.aFW(k)) 0]
%     HC.rwBand / HC.fwBand    .Color, on a station CHANGE only, out of the
%                              builder's own HC.palRW / HC.palFW -- the same
%                              chips dashboardApp draws, kept in ONE place so
%                              a band and a HUD tile cannot disagree
%   HC.rwPylons is static by construction and is never posed: a fixed strut
%   under a rotating plane is what makes the rotation readable in the frame.
%
% NOTES
%   * The R2025a engine ships four scenes: 'Empty scene', 'Empty Grass',
%     'Double lane change' and 'Black Lake'. 'Open surface' and the larger
%     driving scenes need support packages that are NOT installed here.
%   * First engine start costs ~3 minutes (866 MB Unreal project). Later runs
%     in the same MATLAB session are faster but the engine is still restarted
%     per world.
%   * With 'HUD', true the overlay is burned into the VIDEO **and** into every
%     PNG written by 'Stills' -- one composited image feeds both, so the mp4
%     and the stills can never disagree about what the car was doing. That
%     makes a HUD still no longer an untouched camera frame; wingStateMontage's
%     header records the same thing, and a montage of HUD stills would be
%     double-captioned. Render montage stills with 'HUD', false (the default).
%   * The two aero surfaces are REAL extracted geometry, but WHERE they sit is
%     a likeness judgement owned by the manifest -- the donor wing's pivot line
%     and its rigid offset onto the Tur's deck, the vent band's hinge line and
%     extraction box -- not a vehicle parameter. Their ANGLES are the logged
%     actuator positions; their PLACEMENT is not something to measure.
%
%   See also buildTrackRibbon, zenvoCarActors, assetManifest, runDemoLap,
%   trimToLapWindow, dashboardApp.

vizDir   = fileparts(mfilename('fullpath'));           % ...\simulink\viz
repoRoot = fileparts(fileparts(vizDir));

if nargin < 1, src = ''; end

p = inputParser;
p.FunctionName = 'unrealPlayback';
addParameter(p, 'Scene',      'Empty scene', @(v) ischar(v) || isstring(v));
addParameter(p, 'Window',     [],   @(v) isempty(v) || (isnumeric(v) && numel(v) == 2));
addParameter(p, 'Speed',      1,    @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'Fps',        20,   @(v) isscalar(v) && isnumeric(v) && v > 0);
addParameter(p, 'Record',     '',   @(v) ischar(v) || isstring(v));
addParameter(p, 'Stills',     [],   @(v) isempty(v) || isnumeric(v));
addParameter(p, 'ImageSize',  [720 1280], @(v) isnumeric(v) && numel(v) == 2);
addParameter(p, 'Camera',     'quarter', @(v) any(strcmpi(v, {'chase','quarter','low','high','front','side'})));
addParameter(p, 'ShowWindow', true, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'Ribbon',     [],   @(v) isempty(v) || isstruct(v));
addParameter(p, 'vp',         [],   @(v) isempty(v) || isstruct(v));
addParameter(p, 'Wheels',     true, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'SpinMode',   'apparent', @(v) any(strcmpi(v, {'apparent','physical'})));
addParameter(p, 'HUD',        false, @(v) islogical(v) || isnumeric(v));
addParameter(p, 'pt',         [],   @(v) isempty(v) || isstruct(v));
parse(p, varargin{:});
o = p.Results;
o.Scene    = char(o.Scene);
o.SpinMode = lower(char(o.SpinMode));
o.Wheels   = logical(o.Wheels);
o.HUD      = logical(o.HUD);

% ---- the commanded stations ---------------------------------------------
% The ANGLES live here because the station index below is needed long before
% the car exists; the COLOURS do not -- they are the builder's (HC.palRW /
% HC.palFW, read after the build and shape-checked against these two lists),
% so there is one copy of dashboardApp's chips and not a fourth.
STATIONS_RW = [-10 0 10 15];
STATIONS_FW = [-25 -20 0];

% ---- 1. resolve the source into a logsout Dataset ---------------------
ds = localResolveSource(src, repoRoot);

% ---- 2. pull the pose, actuator and wheel channels --------------------
% Every one of these is a field of the logged VehStateBus, so a single
% element carries the whole pose, both wing positions AND the four wheel
% speeds on one time grid.
veh = localGetBus(ds, 'VehState', ...
    {'x','y','z','psi','roll','pitch','vx','alphaRW_pos','alphaFW_pos', ...
     'Om_fl','Om_fr','Om_rl','Om_rr'});
tRaw = veh.x.Time;

% The steer angle and the two pedals are NOT on VehStateBus -- they are the
% driver's own outputs, on DriverCmdBus (SensorBus is no help either: it
% carries only vx, r, Tbrake, TbMax). This is therefore the first genuine
% CROSS-ELEMENT read this file does, which is why the "one element, one time
% grid" claim that used to sit here is gone: drv rides its OWN .Time and is
% interpolated on it. On the shipped ARFWr_Sim both elements are logged at
% the 0.2 ms plant step and the two time vectors are in fact identical, but
% nothing guarantees that, so it is never assumed below.
%
% Fragility worth knowing: 'DriverCmd' is a LINE LABEL (build_sim.m's
% nameLog), not a Custom logging name, so a cosmetic relabel of that line
% would break this lookup -- exactly the risk build_dashlog.m:186-192 warns
% about. 'VehState' has the same exposure and has held up, so this is a known
% and survivable risk, not a new one. localGetBus fails loudly and lists what
% IS logged if it ever happens.
drv  = localGetBus(ds, 'DriverCmd', {'steer','accel','brake'});
tDrv = drv.steer.Time;

win = o.Window;
if isempty(win), win = [tRaw(1) tRaw(end)]; end
win = [max(win(1), tRaw(1)), min(win(2), tRaw(end))];
if diff(win) <= 0
    error('unrealPlayback:emptyWindow', ...
        'unrealPlayback: requested window [%g %g] does not overlap the logged span [%g %g] s.', ...
        o.Window(1), o.Window(2), tRaw(1), tRaw(end));
end

dt      = 1/o.Fps;                       % engine step AND video frame period
tPlay   = (0:dt:(diff(win)/o.Speed)).';  % wall-clock-paced playback time
tLog    = win(1) + o.Speed*tPlay;        % the logged instant each frame shows
nFrames = numel(tPlay);

F.x     = interp1(tRaw, veh.x.Data,           tLog);
F.y     = interp1(tRaw, veh.y.Data,           tLog);
F.z     = interp1(tRaw, veh.z.Data,           tLog);
F.psi   = interp1(tRaw, unwrap(veh.psi.Data), tLog);   % unwrap BEFORE resampling
F.roll  = interp1(tRaw, veh.roll.Data,        tLog);
F.pitch = interp1(tRaw, veh.pitch.Data,       tLog);
F.vx    = interp1(tRaw, veh.vx.Data,          tLog);
F.aRW   = interp1(tRaw, veh.alphaRW_pos.Data, tLog);   % deg
F.aFW   = interp1(tRaw, veh.alphaFW_pos.Data, tLog);   % deg

% ---- 2a. wheel spin, steer and the pedal channels ---------------------
% Om is a RATE and the actor wants an ANGLE, so it is integrated on tRaw and
% only then resampled -- the same discipline as the unwrap above, and for the
% same reason: doing it the other way round (integrating the 20 fps samples)
% is wrong by roughly a revolution per frame at racing speed.
Om  = [veh.Om_fl.Data(:), veh.Om_fr.Data(:), veh.Om_rl.Data(:), veh.Om_rr.Data(:)];  % rad/s
omHalf = median(abs(Om(abs(Om) > 1)));                 % the window's own scale
if isempty(omHalf) || ~isfinite(omHalf) || omHalf <= 0, omHalf = 1; end
switch o.SpinMode
    case 'physical'
        omUse   = Om;
        spinCap = Inf;
    otherwise    % 'apparent' -- COSMETIC, see the SpinMode note in the header
        % 30 deg per FRAME, expressed as a rate in LOGGED time: one frame
        % advances dt*Speed of log time, so a fast-forward playback tightens
        % the cap rather than aliasing past it.
        spinCap = 30/(dt*o.Speed);                     % deg/s of LOGGED time
        omUse   = deg2rad(spinCap) * tanh(Om/omHalf);
end
spinDeg = rad2deg(cumtrapz(tRaw, omUse, 1));           % unwrapped, on the LOG grid
spinDeg = interp1(tRaw, spinDeg, tLog);
spinDeg = mod(spinDeg, 360);                           % wrap only at the end: raw
F.spin  = spinDeg;                                     % values reach ~2e6 deg/lap
                                                       % and are unreadable in a log
% steer / pedals ride DriverCmd's own time base; clamp rather than extrapolate
tD      = min(max(tLog, tDrv(1)), tDrv(end));
F.delta = rad2deg(interp1(tDrv, drv.steer.Data, tD));  % deg, + = LEFT, ROAD-WHEEL
F.thr   = interp1(tDrv, drv.accel.Data, tD);           % 0..1, post traction ceiling
F.brk   = interp1(tDrv, drv.brake.Data, tD);           % 0..1, post traction ceiling

% Lap clock. LapInfo is a plain 12-wide VECTOR element, not a bus, so
% localGetBus rejects it by design -- read it directly. Column 2 is tLap
% (build_driver.m:496-510). Absent or malformed -> NaN, and hudOverlay
% simply omits the chip.
F.tLap = nan(nFrames,1);
try
    liV = ds.getElement('LapInfo').Values;
    liD = squeeze(liV.Data);
    if size(liD,1) ~= numel(liV.Time), liD = liD.'; end
    F.tLap = interp1(liV.Time, liD(:,2), min(max(tLog, liV.Time(1)), liV.Time(end)));
catch
    % leave NaN
end

% station index for the two BAND colours: nearest commanded station to the
% actual actuator position. Derived from the position rather than read from
% the stationRW/stationFW command so the colour can never disagree with the
% angle the surface is actually drawn at.
[~, F.iRW] = min(abs(F.aRW(:) - STATIONS_RW), [], 2);
[~, F.iFW] = min(abs(F.aFW(:) - STATIONS_FW), [], 2);

% ---- 2b. the vehicle-parameter struct ---------------------------------
% vp is the car builder's own prerequisite: the axle stations it places the
% imported body on, the wheel radius and track it checks itself against.
% Resolved HERE -- the 'vp' name-value argument if supplied, else the base
% workspace -- so a missing vp fails with an unrealPlayback-specific message
% naming this function's own prerequisite rather than failing inside the
% builder.
if ~isempty(o.vp)
    vpUse = o.vp;
else
    try
        vpUse = evalin('base', 'vp');
    catch
        error('unrealPlayback:noVp', ...
            ['unrealPlayback: no ''vp'' available for the car build. Open ' ...
             'simulink/ARFWr_RT.prj (loads vp) or pass ''vp'', vpStruct.']);
    end
end

% ---- 2c. HUD-only derived channel: indicated gear ----------------------
% DISPLAY ONLY. The plant runs ONE fixed ratio and never shifts; this digit is
% reconstructed post-hoc in the viz layer from driven-axle wheel speed and the
% supplier gear table, which nothing in the plant or the solver reads.
% indicatedGear.m carries the full caveat -- do not quote it as a shift model.
% Computed on the FRAME grid on purpose: it is a legibility aid with a dwell
% filter, not a measurement.
F.gear = nan(nFrames,1);
ptUse  = o.pt;
if isempty(ptUse)
    try ptUse = evalin('base', 'pt'); catch, ptUse = []; end
end
if ~isempty(ptUse)
    try
        F.gear = indicatedGear(interp1(tRaw, veh.Om_rl.Data, tLog), ...
                               interp1(tRaw, veh.Om_rr.Data, tLog), ptUse, ...
                               'Dt', dt*o.Speed, 'Rw', vpUse.Rw);
    catch meGear
        warning('unrealPlayback:noGear', ...
            'unrealPlayback: indicated gear unavailable (%s); the HUD draws ''--''.', ...
            meGear.message);
    end
end

% ---- 3. track ribbon ---------------------------------------------------
% 'PlantFrame' is not optional here. The poses below come straight off
% VehStateBus, i.e. from the frame Plant.slx integrates in, and only a
% plant-frame ribbon is registered against that -- a translation-only ribbon
% leaves the car drawn on the grass for about a quarter of the lap (measured:
% 75.4 % of the driven lap inside the corridor, against 96.9 % here). See the
% PLANT FRAME NOTE in buildTrackRibbon.m.
if isempty(o.Ribbon)
    rib = buildTrackRibbon('', 'PlantFrame', true);   % Unreal mesh frame, plant origin
else
    rib = o.Ribbon;
    if ~rib.unrealFrame
        error('unrealPlayback:ribbonFrame', ...
            'unrealPlayback: the supplied ''Ribbon'' is in the ISO frame; rebuild it with ''Unreal'', true.');
    end
    if ~isfield(rib, 'plantFrame') || ~rib.plantFrame
        error('unrealPlayback:ribbonPlantFrame', ...
            ['unrealPlayback: the supplied ''Ribbon'' was not built with ''PlantFrame'', true, so it ' ...
             'is not registered against the logged pose and the car would be drawn off the track. ' ...
             'Rebuild it with buildTrackRibbon('''', ''PlantFrame'', true).']);
    end
end
zGround = rib.V(1,3);                          % the ribbon's ZLift

fprintf('unrealPlayback: %d frames, log t = %.2f..%.2f s, %.0fx speed, %g fps\n', ...
    nFrames, win(1), win(2), o.Speed, o.Fps);
fprintf('unrealPlayback: ribbon %d faces, %.0f m, %.2f m wide, scene "%s"\n', ...
    rib.stats.nFace, rib.stats.length, rib.stats.meanWidth, o.Scene);
if o.Wheels
    fprintf(['unrealPlayback: wheels ON, spin ''%s'' (omHalf %.1f rad/s, cap %.0f deg/s ' ...
             '= %.0f deg/frame), steer %.2f..%.2f deg\n'], ...
        o.SpinMode, omHalf, spinCap, spinCap*dt*o.Speed, min(F.delta), max(F.delta));
else
    fprintf('unrealPlayback: wheels OFF (frozen rims, pre-2026-09 behaviour)\n');
end

% ---- 4. world and actors ----------------------------------------------
world = sim3d.World( ...
    Scene           = o.Scene, ...
    RenderOffScreen = ~logical(o.ShowWindow), ...
    Update          = @onUpdate, ...
    Output          = @onOutput);
cleanupWorld = onCleanup(@() localSafeDelete(world));

% (a0) our own ground plane. The shipped scenes' ground is a FINITE tile --
% 'Empty Grass' runs out well inside BCN's ~1.4 km extent, and past its edge
% the car appears to fly over a pale void (seen at the turn-10 hairpin, ~1 km
% from the origin). A single slab sized to the ribbon's own bounding box plus
% a margin removes the dependence on whatever the scene happens to provide.
% rib.lanes is kept in the ISO frame (it is captured before the y flip), so
% the slab can be sized and placed with ISO numbers throughout.
edgeXY = [rib.lanes.kerbLouter; rib.lanes.kerbRouter];
bb     = [min(edgeXY(:,1)) max(edgeXY(:,1)); min(edgeXY(:,2)) max(edgeXY(:,2))];
gPad   = 250;
ground = sim3d.Actor(ActorName = 'Ground', ...
                     Mobility  = sim3d.utils.MobilityTypes.Static);
createShape(ground, 'box', [diff(bb(1,:)) + 2*gPad, diff(bb(2,:)) + 2*gPad, 1.0]);
ground.Color            = [0.30 0.38 0.22];
ground.CoordinateSystem = 'ISO8855';
ground.Translation      = [mean(bb(1,:)), mean(bb(2,:)), -0.55];
ground.Metallic         = 0;
ground.Specular         = 0.02;
ground.Shadows          = false;
add(world, ground);

% (a) the track
track = sim3d.Actor(ActorName = 'Track', ...
                    Mobility  = sim3d.utils.MobilityTypes.Static);
% NB argument order is (vertices, NORMALS, FACES, tcoords, vcolor) -- normals
% come second, faces third. Swapping them silently produces an invisible mesh.
createMesh(track, rib.V, rib.N, rib.F, [], rib.C);
track.TwoSided   = true;      % a handful of quads fold at the hairpin -- see
track.VertexBlend = 1;        % test_trackRibbon [7]; TwoSided keeps them lit
track.Shadows    = false;
track.Metallic   = 0;
track.Specular   = 0.15;
add(world, track);

% (b) start/finish gate, so the lap has a visible beginning
gate = sim3d.Actor(ActorName = 'StartLine', ...
                   Mobility  = sim3d.utils.MobilityTypes.Static);
createShape(gate, 'box', [0.8, rib.stats.meanWidth, 0.03]);
gate.Color            = [0.95 0.95 0.95];
gate.CoordinateSystem = 'ISO8855';
gate.Translation      = [0 0 zGround + 0.02];
add(world, gate);

% (c) the car, and (d) its two active surfaces.
%
% ONE builder, one route. zenvoCarActors puts the whole car under a single
% ISO8855 root -- the imported body, the steer/spin hub chains, the
% transplanted rear wing on its static pylons, the flap cut out of the nose,
% both station bands and the camera rig -- and hands back the handles posed
% below. Nothing about either asset is written here: assetManifest('Zenvo')
% owns every folder, node name, extraction box, hub, pivot and scale, and the
% builder owns the two end angles of the flap hinge and the station chips.
HC      = zenvoCarActors(world, vpUse, assetManifest('Zenvo'));
car     = HC.root;
rwPlate = HC.rwPlate;    % the PIVOT actor (WingRW), never the mesh child
fwHinge = HC.fwHinge;
rwBand  = HC.rwBand;
fwBand  = HC.fwBand;

% STATION COLOURS come from the builder, which is also what it painted the
% bands with at build time, so a band, a HUD chip and the builder cannot
% disagree. The shape check is the whole guard: an index computed against
% STATIONS_RW would silently wrap onto the wrong chip if the two lists ever
% stopped describing the same stations.
palRW = HC.palRW;
palFW = HC.palFW;
assert(size(palRW,1) == numel(STATIONS_RW) && size(palFW,1) == numel(STATIONS_FW), ...
    'unrealPlayback:palette', ...
    ['unrealPlayback: the builder returned %d rear / %d front station colours, but this ' ...
     'file indexes them with %d / %d commanded stations.'], ...
    size(palRW,1), size(palFW,1), numel(STATIONS_RW), numel(STATIONS_FW));
rwBand.Color = palRW(F.iRW(1), :);
for fwi = 1:numel(fwBand), fwBand(fwi).Color = palFW(F.iFW(1), :); end

fprintf(['unrealPlayback: zenvo car -- body %d tris, wing %d, pylons %d, flap %d; ' ...
         'wheelbase %.3f m, flap hinge %+g..%+g deg\n'], ...
    HC.stats.bodyTris, HC.stats.wingTris, HC.stats.pylonTris, HC.stats.flapTris, ...
    HC.stats.wheelbase, HC.stats.fwTuckDeg, HC.stats.fwDeployDeg);

% (e) camera.
%
% CORRECTED NOTE, twice over.
%
% (i) This block used to record a "measured limitation" that a camera actor's
% own Rotation is silently ignored, and offered only rear, car-aimed views on
% that basis. It was a misreading of the SAME unit bug the header now
% documents: a camera asked to pitch 90 deg was being handed pi/2, which the
% engine read as 1.57 DEG, so of course it still rendered the horizon. Probed
% since: an IdealCamera given Rotation = [0 0 90] yaws a clean 90 deg.
% Cameras honour Rotation like any other actor, in DEGREES.
%
% (ii) The follow-on claim that "adding side-on views is possible but is not
% this function's job" is retracted as well, because it was hiding a real
% defect rather than declining a nicety. All four ORIGINAL views sit BEHIND
% the car, and from behind the body occludes both front wheels and the front
% flap completely -- so the steered wheel could not be judged from the video
% at all, and the front flap was evidenced only by the logged trace. Two
% forward rigs are therefore first-class views now. The four rear offsets are
% unchanged, byte for byte.
%
% GEOMETRY OF A NON-REAR RIG (all ISO8855: +x forward, +y LEFT, +z up;
% Rotation = [roll pitch yaw] in DEGREES, and the rig is parented to the car
% so all of it is body-frame). A rear view needs no rotation -- it already
% looks along the car's own +x. A rig placed AHEAD of the car must be turned
% back toward it:
%     yaw = atan2d(-camT(2), -camT(1))      % aim the rig's +x at the origin
% e.g. camT = [5.5 -3.0 1.1] -> atan2d(3.0, -5.5) = 151.4 deg, and the small
% positive pitch drops the lens onto the car (in ISO, +pitch about the LEFT
% axis is nose-DOWN). Both numbers below were probed, not computed and
% trusted: simulink/work/probe_wheelcam.m rendered eight candidate rigs at
% +/-25 deg of steer and probe_wheelcam2.m re-rendered the survivors at the
% +/-11 deg this lap actually reaches.
camR = [0 0 0];
switch lower(o.Camera)
    case 'chase'      % classic over-the-shoulder: rear wing dominant
        camT = [-11.0  0.0 4.0];
    case 'quarter'    % close 3/4 rear: wing and deck in frame -- the demo still
        camT = [-10.5 -3.2 2.3];
    case 'low'        % low 3/4, ground level, most dramatic for video
        camT = [ -8.0 -2.6 1.0];
    case 'high'       % elevated chase: shows the corner the car is entering
        camT = [-15.0  0.0 9.0];
    case 'front'      % front 3/4 from the car's RIGHT: steered front wheel,
                      % front flap and rear wing all in one frame.
                      % Moved in and down 2026-09-16 (was [5.5 -3.0 1.1] /
                      % [0 5.0 151.0]) because the flap is RECESSED in the
                      % bumper: from the old rig it was a few pixels of station
                      % colour under the nose. 12 % closer and 0.18 m lower
                      % puts the bumper cavity across the frame while still
                      % keeping the rear wing inside the top edge -- checked on
                      % a rendered frame, not computed.
                      % RE-CHECKED on the zenvo body 2026-09-17 and NOT moved:
                      % the whole car sits in frame with the rear wing clear of
                      % the top edge and the flap across the bottom third.
                      % Nothing clips, so the numbers stay as they are.
        camT = [  4.9 -2.7 0.92];
        camR = [  0.0  3.5 151.2];
    case 'side'       % right-side-on: both surfaces in profile, and the view
                      % that shows the steered front wheel PLANE (at the
                      % hairpin its tread band sweeps across a wide arc while
                      % the unsteered rear wheel stays face-on).
                      % The SPIN claim this view used to carry is BODY-
                      % DEPENDENT and is not true of a body with closed
                      % arches. RE-MEASURED on the zenvo body 2026-09-17, two
                      % ways. (a) The write LANDS: two frames rendered at the
                      % same logged instant differing ONLY in 'SpinMode' --
                      % i.e. only in the commanded rim angle -- differ in 854
                      % pixels, and every one of them lies on the two wheel
                      % discs, drawing their tread-lug rings and nothing else.
                      % (b) It is still NOT readable by eye at this distance:
                      % the rim is a ~90 px dark disc, and a rotation matcher
                      % that recovers a known 15/30/50 deg exactly on a
                      % synthetic control finds ~0 deg between consecutive
                      % frames of a 29 deg step. So the wheels do turn, the
                      % video is honest, and a still is not the place to read
                      % the rate. Re-checked and not moved otherwise.
        camT = [  0.5 -5.5 1.0];
        camR = [  0.0  2.0  95.0];
    otherwise
        error('unrealPlayback:camera', ...
            ['unrealPlayback: unknown Camera ''%s''. Valid: chase | quarter | low | high | ' ...
             'front | side. The first four ride BEHIND the car and are aimed by its heading; ' ...
             'from every one of them the body occludes the front wheels and the front flap, ' ...
             'so use ''front'' to see the steer or ''side'' to see the wheels turning.'], ...
            o.Camera);
end

% The world already owns its viewer camera as Viewports.Main -- constructing a
% second sim3d.sensors.MainCamera is not the supported route (its constructor
% takes four undocumented positional arguments). Re-parent the existing one.
% One rig actor carries the offset AND the aim, so the viewer camera and the
% recording camera are guaranteed to share both exactly. (camR is identity for
% the four rear views -- the car's own heading does the aiming -- and carries
% the turn-back yaw/pitch for 'front' and 'side'. It is in DEGREES like every
% other Rotation here.)
% ONE rig actor, the builder's, and the PRESET always owns its pose.
% zenvoCarActors already adds a CamRig under its root -- its actor names are a
% frozen contract with the live Simulink route, which resolves by name -- so
% creating a second actor of that name here would leave two rigs in the world
% and make 'CamRig' ambiguous. Both preset lines are written onto it
% unconditionally, so the camera can only ever be where a preset put it.
camRig = HC.camRig;
camRig.Translation      = camT;
camRig.Rotation         = camR;

mainCam = world.Viewports.Main;
mainCam.Translation = [0 0 0];
mainCam.Rotation    = [0 0 0];
try
    add(world, mainCam, camRig);
catch meCam
    warning('unrealPlayback:mainCamParent', ...
        ['unrealPlayback: could not parent the viewer camera to the rig (%s). ' ...
         'The viewer window will show a fixed view; the RECORDED frames are ' ...
         'unaffected -- they come from the IdealCamera.'], meCam.message);
end

recording = ~isempty(char(o.Record));
vw = []; videoPath = ''; stillPaths = {}; idealCam = [];
iRWlast = F.iRW(1); iFWlast = F.iFW(1);

% kNow is the frame index onOutput actually POSED. onUpdate must draw its HUD
% (and name its stills) from that, not from its own kWrote write counter: the
% empty-frame early return below makes the two diverge permanently the first
% time the camera returns nothing, and a HUD indexed off kWrote would then
% show telemetry from a different instant than the frame it is painted on.
% (The same drift was already a latent bug in the still filename, which used
% tLog(kWrote); it is fixed by the same variable.)
kNow = 1;
wheelsOn = o.Wheels;

% Computer Vision Toolbox gate: checked ONCE, here, before the engine starts.
% A licence failure must never surface per frame inside onUpdate. Repo pattern
% is validateSIL.m:191-194 (ver + license checkout) with a namespaced warning.
hudOn = o.HUD && recording;
if o.HUD && ~recording
    warning('unrealPlayback:hudNoRecord', ...
        ['unrealPlayback: ''HUD'' only affects RECORDED frames and no ''Record'' path was ' ...
         'given, so nothing is composited. Pass ''Record'', ''simulink/work/demo.mp4''.']);
end
if hudOn
    okCvt = ~isempty(ver('vision')) && ~isempty(which('insertText'));
    if okCvt
        try okCvt = license('checkout', 'Video_and_Image_Blockset') == 1; catch, okCvt = false; end
    end
    if ~okCvt
        warning('unrealPlayback:noHudToolbox', ...
            ['unrealPlayback: the HUD needs Computer Vision Toolbox (insertText / ' ...
             'insertShape) and it could not be checked out. Rendering with the HUD OFF; ' ...
             'the video and stills are otherwise unaffected.']);
        hudOn = false;
    end
end
hudNote = '';
if hudOn && strcmp(o.SpinMode, 'apparent')
    hudNote = 'wheel spin: cosmetic scale, not a measurable rate';
end

if recording
    idealCam = sim3d.sensors.IdealCamera(ActorName = 'RecCam', ...
        ImageSize = round(o.ImageSize), HorizontalFieldOfView = 75);
    idealCam.CoordinateSystem = 'ISO8855';
    idealCam.Translation      = [0 0 0];   % the rig carries the pose
    idealCam.Rotation         = [0 0 0];
    add(world, idealCam, camRig);

    videoPath = char(o.Record);
    if ~java.io.File(videoPath).isAbsolute()
        videoPath = fullfile(repoRoot, videoPath);
    end
    vdir = fileparts(videoPath);
    if ~isempty(vdir) && ~isfolder(vdir), mkdir(vdir); end
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = o.Fps;
    open(vw);
end

% still capture: convert requested LOGGED times to frame indices
stillFrames = [];
if ~isempty(o.Stills) && recording
    stillFrames = unique(max(1, min(nFrames, round((o.Stills(:) - win(1))/(o.Speed*dt)) + 1)));
end

% ---- 5. run ------------------------------------------------------------
kSeen  = 0;    % frames the Update callback has actually posed
kWrote = 0;    % frames written to the video
tw = tic;
run(world, dt, tPlay(end));
wall = toc(tw);

if recording
    close(vw);
    fprintf('unrealPlayback: wrote %d frames to %s\n', kWrote, videoPath);
end
fprintf('unrealPlayback: %d/%d frames posed, wall %.1f s for %.1f s of playback (%.2fx)\n', ...
    kSeen, nFrames, wall, tPlay(end), tPlay(end)/wall);

info = struct('nFrames', nFrames, 'nPosed', kSeen, 'nWritten', kWrote, ...
              'tSpan', win, 'speed', o.Speed, 'fps', o.Fps, 'wall', wall, ...
              'videoPath', videoPath, 'stillPaths', {stillPaths}, ...
              'ribbonStats', rib.stats, 'scene', o.Scene, 'camera', o.Camera, ...
              'car', 'Zenvo', ...
              'wheels', wheelsOn, 'spinMode', o.SpinMode, ...
              'spinOmHalf', omHalf, 'spinCapDegPerSec', spinCap, ...
              'hud', hudOn, 'hudNote', hudNote, ...
              'frames', F, 'tLog', tLog);

delete(world);
clear cleanupWorld

% =====================================================================
% nested callbacks -- they close over F, the actors and the writer
% =====================================================================
    function onOutput(w)
        % Pose everything for this step. Runs before the engine renders.
        % Actor HANDLES are used, not w.Actors.<name>: every surface here is
        % a child of the car and need not appear at the top of that struct.
        k = min(nFrames, max(1, round(w.SimulationTime/dt) + 1));
        kSeen = max(kSeen, k);
        kNow  = k;

        % Translation is METRES, Rotation is DEGREES -- see the UNITS note in
        % the header. One root carries the whole car and the wheels hang off
        % it through their own steer/spin hubs, so every write below is a
        % LOCAL angle on its own actor. Spin sign is PLUS -- a positive
        % rotation about the hub's lateral axis rolls the wheel FORWARD,
        % matching VehStateBus Om_*, so the integrated angle goes in unflipped.
        % No wheel TRANSLATION is written: the builder already put each hub on
        % its own measured station.
        car.Translation = [F.x(k), F.y(k), zGround + F.z(k)];
        car.Rotation    = rad2deg([F.roll(k), F.pitch(k), F.psi(k)]);
        if wheelsOn
            HC.steer.FL.Rotation = [0, 0, F.delta(k)];
            HC.steer.FR.Rotation = [0, 0, F.delta(k)];
            HC.spin.FL.Rotation  = [0, F.spin(k,1), 0];
            HC.spin.FR.Rotation  = [0, F.spin(k,2), 0];
            HC.spin.RL.Rotation  = [0, F.spin(k,3), 0];
            HC.spin.RR.Rotation  = [0, F.spin(k,4), 0];
        end

        % REAR WING. +alpha about the lateral axis lifts the trailing edge
        % (header note), and F.aRW is already DEGREES as logged -- do not
        % convert. This is the PIVOT actor: the delivered donor geometry IS
        % station 0, so the raw logged angle goes straight in, and the
        % mesh-carrying child is left alone (the two rotations do not commute).
        rwPlate.Rotation = [0, F.aRW(k), 0];

        % FRONT FLAP. It is the car's own bumper vent band, hinged at its
        % upper edge, so the logged station cannot drive it directly: it
        % drives the HINGE through the builder's own map, which owns the two
        % end angles and clamps outside them. alphaFW = 0 (R128, max
        % downforce) is DEPLOYED -- swung down and forward out of the bumper
        % with the cut slot open behind it; alphaFW = -25 (unloaded) is TUCKED
        % flush, the delivered nose with only the band's thin line showing.
        fwHinge.Rotation = [0, HC.fwHingeDeg(F.aFW(k)), 0];

        % recolour only on a station CHANGE -- a material write every step is
        % both wasteful and, on some builds, ignored mid-frame. The BANDS are
        % what carries the station; the surfaces keep their own textures.
        if F.iRW(k) ~= iRWlast, rwBand.Color = palRW(F.iRW(k), :); iRWlast = F.iRW(k); end
        if F.iFW(k) ~= iFWlast
            for fwj = 1:numel(fwBand), fwBand(fwj).Color = palFW(F.iFW(k), :); end
            iFWlast = F.iFW(k);
        end
    end

    function onUpdate(~)
        % Read the recording camera AFTER the engine has drawn this step.
        if ~recording, return, end
        img = read(idealCam);
        if isempty(img), return, end
        % HUD is burned in BEFORE writeVideo and shared with the still below,
        % so the mp4 and every PNG show the same instruments over the same
        % frame. Indexed by kNow -- the frame onOutput posed -- never kWrote.
        if hudOn
            img = hudOverlay(img, struct( ...
                'vx',       F.vx(kNow),   'steerDeg', F.delta(kNow), ...
                'gear',     F.gear(kNow), 'throttle', F.thr(kNow), ...
                'brake',    F.brk(kNow),  'aRW',      F.aRW(kNow), ...
                'aFW',      F.aFW(kNow),  'colRW',    palRW(F.iRW(kNow), :), ...
                'colFW',    palFW(F.iFW(kNow), :), 'tLap', F.tLap(kNow), ...
                'note',     hudNote));
        end
        writeVideo(vw, img);
        kWrote = kWrote + 1;
        if ismember(kWrote, stillFrames)
            [d1, b1] = fileparts(videoPath);
            % The CAR is in the filename, always. Frames left behind by a
            % superseded body otherwise sit at the same logged time under the
            % same name and are indistinguishable from fresh ones -- which is
            % exactly how a render that never ran gets read as evidence.
            pngPath  = fullfile(d1, sprintf('%s_Zenvo_t%06.2f.png', b1, tLog(kNow)));
            imwrite(img, pngPath);
            stillPaths{end+1} = pngPath;
            fprintf('unrealPlayback: still  %s\n', pngPath);
        end
    end
end

% =========================================================================
function ds = localResolveSource(src, repoRoot)
%LOCALRESOLVESOURCE Accept a path / runDemoLap struct / Dataset.
if isa(src, 'Simulink.SimulationData.Dataset')
    ds = src;  return
end
if isstruct(src) && isfield(src, 'logsout')
    ds = src.logsout;  return
end
if isempty(src)
    src = fullfile(repoRoot, 'simulink', 'work', 't53_lap.mat');
    if ~isfile(src)
        error('unrealPlayback:noSource', ...
            ['unrealPlayback: no source given and %s does not exist. ' ...
             'Run  lap = runDemoLap(''Save'', ''simulink/work/t53_lap.mat'');  first.'], src);
    end
end
src = char(src);
if ~java.io.File(src).isAbsolute()
    src = fullfile(repoRoot, src);
end
if ~isfile(src)
    error('unrealPlayback:sourceNotFound', 'unrealPlayback: %s not found.', src);
end
S = load(src);
if isfield(S, 'out') && isfield(S.out, 'logsout')
    ds = S.out.logsout;
elseif isfield(S, 'logsout')
    ds = S.logsout;
else
    fn = fieldnames(S);
    hit = find(cellfun(@(f) isa(S.(f), 'Simulink.SimulationData.Dataset'), fn), 1);
    if isempty(hit)
        error('unrealPlayback:noDataset', ...
            'unrealPlayback: %s holds no logsout Dataset. Variables: {%s}', ...
            src, strjoin(fn', ', '));
    end
    ds = S.(fn{hit});
end
end

% =========================================================================
function out = localGetBus(ds, elName, fields)
%LOCALGETBUS Fetch one logged BUS element and assert the fields exist.
% Simulink returns a DATASET rather than an element for an ambiguous name
% (the ambiguous-name trap dashboardApp.m documents), so the class is checked explicitly.
try
    el = ds.getElement(elName);
catch
    nm = cell(ds.numElements,1);
    for i = 1:ds.numElements, nm{i} = ds.getElement(i).Name; end
    error('unrealPlayback:missingElement', ...
        'unrealPlayback: no logged element named ''%s''. Logged: {%s}', ...
        elName, strjoin(sort(nm)', ', '));
end
if isa(el, 'Simulink.SimulationData.Dataset')
    error('unrealPlayback:ambiguousElement', ...
        ['unrealPlayback: the name ''%s'' is logged more than once, so Simulink ' ...
         'returns a Dataset rather than an element. Fix the duplicate logging.'], elName);
end
v = el.Values;
if ~isstruct(v)
    error('unrealPlayback:notABus', ...
        'unrealPlayback: logged element ''%s'' is a %s, expected a bus struct.', ...
        elName, class(v));
end
missing = fields(~isfield(v, fields));
if ~isempty(missing)
    error('unrealPlayback:missingField', ...
        'unrealPlayback: %s is missing {%s}. Present: {%s}', ...
        elName, strjoin(missing, ', '), strjoin(fieldnames(v)', ', '));
end
out = struct();
for i = 1:numel(fields)
    out.(fields{i}) = v.(fields{i});
end
end

% =========================================================================
function localSafeDelete(w)
try
    if isvalid(w), delete(w); end
catch
    % already deleted, or the engine handed back a dead handle -- nothing to do
end
end
