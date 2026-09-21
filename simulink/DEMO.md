# Live demo — ARFWr reactive dual-wing sim

Zenvo Aurora (Tur body + Agil rear wing) driven by the closed-loop plant in Unreal, both wings and
their station bands moving live.

## Needs
MATLAB R2025a + Simulink, Simulink 3D Animation (Unreal engine), Vehicle Dynamics Blockset,
Computer Vision Toolbox (HUD only). Windows. **No CasADi needed** — that is only for the offline
MLTP solver.

## Before you start
This repository ships the real Zenvo Aurora tyre and aero data by default (`Parameters/
tyreParams_Zenvo.m`, `Parameters/aeroMap_Tur.mat`, published with permission of Zenvo Automotive)
— there is no setup step. Opening `simulink/ARFWr_RT.prj` loads them automatically.

If you have swapped in the synthetic template or your own vehicle data (see the top-level
README's "Swap in different vehicle data" section), the sim uses whatever `vp`/`pt` were loaded
at project-open time — reopen the project after changing the data files.

The 3D visualisation's track ribbon mesh comes from the active track's solved-lap sidecar (see
"Run on a different track" below). With nothing else set up it falls back to the shipped
Barcelona geometry, `simulink/data/trackRibbon_BCN.mat`, so both routes below work immediately
after opening the project.

## A. Live, in the model (viewer window rides the car)
1. Open `simulink/ARFWr_RT.prj` (loads `vp`, `pt`, paths, and the active track pack if one has
   been set with `setupTrack`).
2. In the MATLAB command window:
   ```matlab
   mdl = 'ARFWr_Sim'; load_system(mdl);
   set_param([mdl '/Visualization'], 'LabelModeActiveChoice', 'Unreal3D');
   runDemoLap('StopTime', 140, 'Pace', 1);   % Pace 1 = real time; [] = as fast as possible
   ```
   `'StopTime'` can be omitted — it then defaults to the active track's own hint (Barcelona:
   150 s is enough headroom for the ~139.4 s lap; another track's hint is written by
   `setupTrack`).
3. Unreal window opens on its own (~30 s first launch). View is a front-3/4 riding the car
   (`CamRig`, ~5.7 m out). The 12 m chase mount is the capture camera for stills only.
   Rear-wing band: blue −10°, grey 0°, red +10°, amber +15° (airbrake).
   Front strips: blue −25°, grey −20°, purple 0° (deployed).
4. Model-editor `Live Instruments` panel shows speed, steer, gear, pedals, wing angles.
5. Reset when done (keeps the committed model clean):
   ```matlab
   set_param([mdl '/Visualization'], 'LabelModeActiveChoice', 'Off');
   ```
   Do not save the model.

## B. Offline replay with HUD (video + stills)
`unrealPlayback` takes the `out` struct `runDemoLap` returns, so the normal route saves nothing:
```matlab
out = runDemoLap();
unrealPlayback(out, 'Camera','front', 'HUD',true, ...
    'Window',[0 139.39], 'Record','simulink/work/demo_unreal.mp4');
```
Only save the lap if you want to replay it in a later MATLAB session. `out` carries the whole
logged Dataset and is **about 500 MB** for a full Barcelona lap, so saving it is a deliberate act,
not a habit. Use `runDemoLap`'s own `'Save'`: it anchors the path to the repository root and
creates the folder, unlike a bare `save 'simulink/work/…'`, which resolves against `pwd` — which
is `<repo>/simulink` once the project is open — into a `simulink/work/` a fresh clone does not
have.
```matlab
out = runDemoLap('Save', 'simulink/work/t53_lap.mat');   % ~500 MB, git-ignored
unrealPlayback('simulink/work/t53_lap.mat', 'Camera','front', 'HUD',true);
```
Cameras: `quarter` (default), `chase`, `low`, `high`, `front` (steered wheel + front flap + wing),
`side` (wheel spin). `'Speed',2` = 2× playback. Stills are **opt-in**: pass `'Stills',[t1 t2 …]`
(logged times in seconds) and one PNG per time lands next to the `.mp4` as
`<video>_Zenvo_t<sec>.png`. Without `'Stills'` only the video is written, and without `'Record'`
nothing is written at all.

## Run on a different track
`simulink/tools/setupTrack.m` points every tool above at a different solved lap in one call —
see the top-level README's "Run the sim on a new track" section for the full mechanism
(`buildDriverRef`, `buildSpeedPlan`, `activeTrack`, `'Persist'`). Short version:
```matlab
info = solveLap('Spa', 'ARFWr', 'ATD', 'Setup', true);   % solve a new track, then set it up
out  = runDemoLap();                                     % drives whatever setupTrack last set
```
`solveLap` runs `MLTP`, which clears the workspace — so `solveLap` snapshots the caller's base
workspace before its first real solve and puts it back afterwards (and on error). `vp`, `pt`,
`sus`, `act`, `inrt` and `hudGear` therefore survive a solve and `runDemoLap` runs straight after,
which is what makes the two lines above work in one session. The result is in the returned `info`
and in the saved `.mat` at `info.matPath`, never in the base workspace.

Nothing tracked is written by default (`'Persist', false`, the default, is in-memory only);
`simulink/startup/projStartup.m` re-applies the last active track automatically on project open.
Opening the project is enough to call `solveLap` and `setupTrack` by name — `arfwr_startup.m`
puts `Scripts/`, `Functions/`, `Parameters/` and `Circuits/` on the path.

`setupTrack` also decides where the sim's lap **starts**. Its default, `'StartAt', 'auto'`,
keeps the circuit file's own `s = 0` whenever the reference stays above R = 200 m over the
first 100 m (Barcelona and Nurburgring both do, unchanged), and otherwise rotates the lap to
begin 30 m into its longest straight — the driver launches at plan speed with no steer or
preview history and cannot hold a corner from a standing start. Pass `'StartAt', 'solved'` to
force the file's own start line, or a distance in metres to place it by hand.

Measured, no re-tuning: Nurburgring completes in 145.113 s, maximum path error 1.962 m, zero
off-track excursions. Spa — whose `s = 0` sits on the exit of La Source, so `'auto'` moves the
start 1242 m onto the Kemmel straight — completes in 177.969 s, maximum path error 2.059 m,
zero off-track excursions; from the file's own start line the same lap runs wide over its
opening 26 m (4492 off-track samples, 4.745 m) and is clean everywhere after.

## Notes
- Lap-time numbers from this sim are NOT active-aero gains; those stay with the MLTP solver. The
  sim is a tracking exercise (Stanley-style path follower + grip-limited speed plan), not a
  re-solve of the optimal-control problem — the shipped Barcelona lap closes at 139.394 s here
  against the MLTP solver's 102.674 s for the same configuration; never difference the two.
- Gear digit and wheel spin are display-only.
- If Unreal fails to launch: `sim3d.engine.Env` requires the engine at
  `<matlabroot>\toolbox\shared\sim3d_projects\...\WindowsNoEditor`.
