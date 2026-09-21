# Live demo — ARFWr reactive dual-wing sim

Zenvo Aurora (Tur body + Agil rear wing) driven by the closed-loop plant in Unreal, both wings and
their station bands moving live.

## Needs
MATLAB R2025a + Simulink, Simulink 3D Animation (Unreal engine), Vehicle Dynamics Blockset,
Computer Vision Toolbox (HUD only). Windows.

## Before you start (this distribution only)
This repository ships synthetic tyre and aero data, not the real supplier data the thesis
project used — see the top-level README's "Synthetic stand-ins" section. Two files need to
exist, at these exact names, before `simulink/ARFWr_RT.prj` is opened:
```matlab
copyfile('Parameters/tyreParams_Synthetic.m', 'Parameters/tyreParams_DoNotPublish.m');
copyfile('Parameters/aeroMap_Synthetic.mat',  'Parameters/aeroMap_Tur.mat');
```
Both are already `.gitignore`d, so this is a one-time local step, not a commit.

The 3D visualisation (both routes below) also needs a solved-lap sidecar at
`solutions/report/BCN/raw/run_BCN_ARFWr_ATD_data.mat` — the track ribbon mesh is built from a
solved racing line, not from the raw circuit file, and that sidecar is not part of this
distribution. Produce your own by running the offline solver on the synthetic data (top-level
README, `Scripts/MLTP.m` with the ARFWr/reactive-wing configuration) and saving its `data`
struct to that exact path. Without it, `runDemoLap` still runs the closed-loop lap correctly
(no visualisation call is on that path), but switching the model to `Unreal3D` or calling
`unrealPlayback` errors with `buildTrackRibbon:matNotFound` until the sidecar exists.

## A. Live, in the model (viewer window rides the car)
1. Open `simulink/ARFWr_RT.prj` (loads `vp`, `pt`, paths).
2. In the MATLAB command window:
   ```matlab
   mdl = 'ARFWr_Sim'; load_system(mdl);
   set_param([mdl '/Visualization'], 'LabelModeActiveChoice', 'Unreal3D');
   runDemoLap('StopTime', 140, 'Pace', 1);   % Pace 1 = real time; [] = as fast as possible
   ```
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
Uses a logged lap (`simulink/work/t53_lap.mat`, git-ignored — regenerate with `out = runDemoLap();`
then `save simulink/work/t53_lap.mat out`).
```matlab
unrealPlayback('simulink/work/t53_lap.mat', 'Camera','front', 'HUD',true, ...
    'Window',[0 139.39], 'Record','simulink/work/demo_unreal.mp4');
```
Cameras: `quarter` (default), `chase`, `low`, `high`, `front` (steered wheel + front flap + wing),
`side` (wheel spin). `'Speed',2` = 2× playback. Frames land next to the .mp4.

## Notes
- Lap-time numbers from this sim are NOT active-aero gains; those stay with the MLTP solver.
- Numbers from this sim also are not comparable to the thesis figures: the shipped tyre and
  aero data here are synthetic stand-ins, not the real supplier data.
- Gear digit and wheel spin are display-only.
- If Unreal fails to launch: `sim3d.engine.Env` requires the engine at
  `<matlabroot>\toolbox\shared\sim3d_projects\...\WindowsNoEditor`.
