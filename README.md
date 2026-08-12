# MLTP-ATD-ARFW

Minimum Lap Time Problem (MLTP) solver for a vehicle with **active aerodynamics** and
**active torque distribution**, in MATLAB + CasADi.

A race lap is posed as an optimal-control problem in the **distance domain**, transcribed
by direct collocation and solved as an NLP with IPOPT. The solver returns the optimal
racing line, the control inputs, and - in two of the entry points - vehicle design
parameters, for a given track.

Based on the MLTP framework of Zarzuelo & Siampis and Jimenez et al., extended with
active aerodynamic surfaces (rear wing, front-wing flap) and active torque distribution.

---

## Read this first: the repository does not run out of the box

The code here is the **complete, unmodified framework**. What is missing is the vehicle
data, which is confidential and has been withheld. Two files the solver requires are
therefore absent:

| Expected path | What it is |
|---|---|
| `Parameters/tyreParams_DoNotPublish.m` | Per-axle Pacejka MF5.2 tyre coefficients |
| `Parameters/aeroMap_Tur.mat` | Ride-height lift map, normally built by `Functions/importAeroMap.m` from a source workbook |

Running `MLTP` without them **will error**, and that is deliberate. The import machinery
has been left exactly as it is rather than stubbed out, so that when you drop in your own
data the framework behaves identically to the original.

### Supplying your own data

Provide both files in the layout above:

- **Tyres** - a script defining `vp.tyre_f` and `vp.tyre_r`. See
  `Parameters/tyreParams_Synthetic.m` for the required field set.
- **Aero map** - either run `Functions/importAeroMap.m` against your own ride-height
  workbook, or save a struct with fields RHf [10x1], RHr [1x11], CLf [10x11],
  CLr [10x11] (negative = downforce) to `Parameters/aeroMap_Tur.mat`.

### Or use the synthetic stand-ins

Two invented datasets are included so you can exercise the framework without any real
vehicle data. Copy the tyre template over the expected filename, generate the map, and
copy that over its expected filename:

    copyfile Parameters/tyreParams_Synthetic.m Parameters/tyreParams_DoNotPublish.m
    makeSyntheticAeroMap
    copyfile Parameters/aeroMap_Synthetic.mat Parameters/aeroMap_Tur.mat

**These numbers are invented.** They are physically self-consistent, but they describe no
real vehicle, and the solver settings in `userOpts.m` are *not* tuned for them - expect to
re-derive the boundary velocity `vi`, and possibly the tolerances, before IPOPT converges
(see the tuning caveat below). They exist to let you read and exercise the code, not to
produce meaningful lap times.

### Aerodynamic coefficient data

The aerodynamic coefficients compiled into `Functions/` - `rwAeroDelta.m` (rear-wing
angle sweep) and `fwAeroDelta.m` / `fwOnlyDelta.m` (front-wing flap) - are **real
measured data**, not synthetic. Only the tyre template and the ride-height map generator
described above are invented.

## Requirements

- MATLAB R2021b or newer (developed against R2021b-R2022b; exercised on R2025a)
- CasADi 3.5.5 or newer on the MATLAB path - https://web.casadi.org/
- Simulink is used only by `plotSDI.m`, which pushes results to the Simulink Data Inspector

## Running

    addpath(genpath(pwd));
    MLTP

Entry points in `Scripts/` - these are scripts, not functions, and take no arguments:

| Script | Optimises |
|---|---|
| `MLTP.m` | Trajectory + control inputs. **Start here.** |
| `MLTP_paramOptim.m` | Also design params: torque distribution, brake bias, roll-stiffness distribution |
| `MLTP_TyreOptim.m` | Also the nominal tyre-load shift |
| `MLTP_initial.m` | Simplified single-track model, used to warm-start the others |

Configure a run by editing `Scripts/userOpts.m` (aero config, ATD on/off, track, boundary
conditions, collocation step, solver tolerances, rate limits, regularisation) and the
parameter files in `Parameters/`.

## Configuration matrix

Two flags in `userOpts.m` drive the dimensionality of the whole problem:

- `AeroConfig` -> `vp.ActAero`: `Static` (fixed wing) or `ActiveRW` (wing is an NLP control)
- `ATD` -> `pt.ATD`: torque distribution off (AWD) or on

Together they set the number of control inputs `nu` (3-9) and path constraints `nh`.
Within `ActiveRW`, `RWMandate` selects the actuator law:

| Setting | Config | What the wing does |
|---|---|---|
| `Off` | `ARW` | Free continuous rear-wing angle - the optimiser chooses |
| `Discrete` | `ARWd` | Continuous solve + cosmetic discretisation onto fixed stations, with an in-NLP braking floor |
| `FrontWing` | `AFWd` | Front-wing flap on an unload axis |
| `Combined` | `ARFWd` | Both wings, independently controlled |
| `Reactive` | `ARFWr` | Both wings pinned to a reactive law driven by onboard yaw rate and speed |

Low-drag-on-straights / high-downforce-in-corners / airbrake-under-braking behaviour is
**emergent** in the free configurations - it is never hard-coded.

## Model

- **Independent variable is distance `s`**, not time. Lap time is the integrated objective.
  Curvature `k(s)` is the only parameter varying along the grid.
- **States** (`nx = 9`): `[vx vy r n eps Om_fl Om_fr Om_rl Om_rr]` - body velocities, yaw
  rate, distance to centreline, heading error, four wheel speeds.
- **Everything symbolic is scaled.** Each variable `foo` has a normalised symbol `foo_n`
  and a scale `foo_s`, with `foo = foo_s*foo_n`. The NLP works in normalised units.
- **Tyres**: per-axle Pacejka MF5.2 with full combined slip (Gxa/Gyk weighting), so there
  are no friction-circle path constraints - grip limiting falls out of the tyre model. A
  per-tyre load cap is enforced instead.
- **Aerodynamics**: the ride-height lift map drives a quasi-static aeroelastic collapse
  (`Functions/aeroCollapse.m`), resolving the downforce/ride-height feedback loop per speed
  and fitting it to a CasADi-differentiable piecewise polynomial. Active configurations
  layer a 2-D (angle, speed) wing map on top, blended by a smooth cardinal basis so the
  whole chain stays C-infinity and SX-safe.

## Repository layout

    Scripts/      entry points, userOpts.m, vehModel.m, post-processing
    Parameters/   vehicle, powertrain, synthetic tyre template
    Functions/    model helpers, aero collapse + wing maps
    Circuits/     track .mat files (need s and k; x,y optional for plotting)

Tracks included: Barcelona-Catalunya (full + three sectors), Jarama, Spa, Nurburgring.

## A caveat on tuning

Solver tolerances, regularisation factors, initial guesses and boundary velocities in
`userOpts.m` were hand-calibrated for one specific vehicle at Barcelona-Catalunya.
Changing the vehicle, track or configuration generally means re-tuning them and
regenerating an initialisation file, or IPOPT will not converge. `userOpts.m` documents a
fixed-point protocol for re-deriving the boundary velocity `vi`. This is the normal cost
of direct collocation on a problem this stiff, not a defect - and it applies in full to
the synthetic stand-ins above.

## Attribution

This work builds on an open MLTP framework. If you use it, please cite the original
authors as well:

1. Zarzuelo, A., & Siampis, E. (2022). *MLTP*. MATLAB Central File Exchange.
   https://www.mathworks.com/matlabcentral/fileexchange/119398-mltp
2. Jimenez Elbal, A., Zarzuelo Conde, A., & Siampis, E. (2024). Simultaneous Optimisation
   of Vehicle Design and Control for Improving Vehicle Performance and Energy Efficiency
   Using an Open Source Minimum Lap Time Simulation Framework. *World Electric Vehicle
   Journal*, 15(8), 366. https://doi.org/10.3390/wevj15080366

## Licence

Not yet chosen - contact the repository owner before redistributing. The upstream
framework is distributed via MATLAB Central File Exchange under its own terms.
