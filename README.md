# MLTP-ATD-ARFW

Minimum Lap Time Problem (MLTP) solver for a vehicle with **active aerodynamics** and
**active torque distribution**, in MATLAB + CasADi.

A race lap is posed as an optimal-control problem in the **distance domain**, transcribed
by direct collocation and solved as an NLP with IPOPT. The solver returns the optimal
racing line and the control inputs for a given track.

Based on the MLTP framework of Zarzuelo & Siampis and Jimenez et al., extended with
active aerodynamic surfaces (rear wing, front-wing flap) and active torque distribution.

---

## Contents

- [Read this first](#read-this-first-the-repository-does-not-run-out-of-the-box)
- [Requirements](#requirements)
- [Setup, step by step](#setup-step-by-step)
- [What you must supply](#what-you-must-supply)
- [Running a solve](#running-a-solve)
- [Configuration reference](#configuration-reference)
- [Things to be wary of](#things-to-be-wary-of)
- [What gets written to disk](#what-gets-written-to-disk)
- [Model](#model)
- [Repository layout](#repository-layout)
- [Attribution](#attribution)
- [Licence](#licence)

---

## Read this first: the repository does not run out of the box

The code here is the **complete, unmodified framework**. What is missing is the vehicle
data, which is confidential and has been withheld. Two files the solver requires are
therefore absent:

| Expected path | What it is |
|---|---|
| `Parameters/tyreParams_DoNotPublish.m` | Per-axle Pacejka MF5.2 tyre coefficients |
| `Parameters/aeroMap_Tur.mat` | Ride-height lift map (front/rear lift coefficient over a ride-height grid) |

Running `MLTP` without them **will error**, and that is deliberate. The first thing you
will see is:

    Unrecognized function or variable 'tyreParams_DoNotPublish'.

The loading machinery has been left exactly as it is rather than stubbed out, so that when
you drop in your own data the framework behaves identically to the original.

**Synthetic stand-ins are included** so you can exercise the framework immediately without
any real vehicle data. [Step 3](#step-3-supply-the-two-data-files) covers both routes.

---

## Requirements

| | Needed | Notes |
|---|---|---|
| MATLAB | R2021b or newer | Developed against R2021b–R2022b; this distribution was last exercised on **R2025a Update 1** |
| CasADi | 3.5.5 or newer | Last exercised on **3.7.2**. Not a MathWorks product — install separately, see below |
| Simulink | Required to finish a run | The optimisation itself does not use it, but the post-processing does. See the [warning below](#simulink-is-required-to-finish-a-run) |

No other MathWorks toolboxes are required. Everything else the code calls (`interp1`,
`timeseries`, `table`, `writetable`, `polyfit`) is base MATLAB.

---

## Setup, step by step

### Step 1 — Install CasADi

CasADi is a third-party symbolic/NLP framework and ships with IPOPT, the solver this
project uses. It is **not** installed by MATLAB and must be downloaded separately.

1. Download the MATLAB binary for your platform from <https://web.casadi.org/get/>.
   Pick the archive matching your OS and MATLAB generation — for example
   `casadi-3.7.2-windows64-matlab2018b`. The `matlab2018b` in the name is a *minimum*
   MATLAB build, not an exact one; it works on far newer releases.
2. Unzip it somewhere permanent — **not** inside this repository, and not in a temp
   folder. For example `D:\Software Installs\casadi-3.7.2-windows64-matlab2018b`.
3. Add it to the MATLAB path:

   ```matlab
   addpath('D:\Software Installs\casadi-3.7.2-windows64-matlab2018b')
   savepath   % optional, makes it stick between MATLAB sessions
   ```

4. Verify:

   ```matlab
   import casadi.*
   disp(casadi.CasadiMeta.version)   % e.g. 3.7.2
   x = SX.sym('x'); disp(jacobian(x^2, x))
   ```

   If `import casadi.*` fails, the folder you added is wrong — you need the directory
   that *contains* the `+casadi` folder, not the `+casadi` folder itself.

`MLTP.m` checks for CasADi up front and stops with a clear message if it is missing, so a
bad path here is not a mystery failure.

### Step 2 — Get the repository

```bash
git clone https://github.com/often-called-pk/MLTP-ATD-ARFW.git
cd MLTP-ATD-ARFW
```

There is nothing to build or compile.

You do **not** need to `addpath` this repository. `MLTP.m` locates the repository from its
own file location and registers `Scripts/`, `Parameters/` and `Functions/` itself, then
sets the working directory to the repository root. You only need MATLAB's working
directory or path to be such that the name `MLTP` resolves — the simplest thing is to
`cd` into the repository root in MATLAB and add it:

```matlab
cd 'C:\path\to\MLTP-ATD-ARFW'
addpath(genpath(pwd))
```

### Step 3 — Supply the two data files

Choose **one** of the two routes below.

#### Route A — the synthetic stand-ins (recommended for a first run)

Two invented datasets ship with the repository. Copy each over the filename the solver
expects. From the repository root, in MATLAB:

```matlab
copyfile('Parameters/tyreParams_Synthetic.m', 'Parameters/tyreParams_DoNotPublish.m');
copyfile('Parameters/aeroMap_Synthetic.mat',  'Parameters/aeroMap_Tur.mat');
```

That is the whole setup. To regenerate the aero map from scratch instead of copying it
(the generator is readable and documents the shape it produces):

```matlab
makeSyntheticAeroMap(fullfile(pwd, 'Parameters', 'aeroMap_Tur.mat'));
```

> **These numbers are invented.** They are physically self-consistent and produce a
> well-posed problem, but they describe no real vehicle, and the solver settings in
> `userOpts.m` are *not* tuned for them. Expect to re-derive the boundary velocity `vi`,
> and possibly the tolerances, before IPOPT converges — see
> [Things to be wary of](#things-to-be-wary-of). They exist to let you read and exercise
> the code, not to produce meaningful lap times.

#### Route B — your own vehicle data

Provide both files at the paths in the table above.

- **Tyres** — a *script* (not a function) that defines `vp.tyre_f` and `vp.tyre_r`, each a
  struct of Pacejka MF5.2 coefficients including the per-axle rolling-resistance term
  `qsy1`. It is called by name from `Parameters/vehParams.m`, so it must be on the MATLAB
  path. Copy `Parameters/tyreParams_Synthetic.m` and edit it — it documents the exact
  field set required.
- **Aero map** — a `.mat` containing a struct with these fields:

  | Field | Size | Meaning |
  |---|---|---|
  | `RHf` | 10×1 | front ride-height grid [mm] |
  | `RHr` | 1×11 | rear ride-height grid [mm] |
  | `CLf` | 10×11 | front-axle lift coefficient (**negative = downforce**) |
  | `CLr` | 10×11 | rear-axle lift coefficient (**negative = downforce**) |

  Grids must be strictly increasing, all lift values finite, and the map smooth and
  monotone — `Functions/aeroCollapse.m` solves a fixed point on it and then fits the
  result piecewise-polynomially against a residual gate, so a noisy or non-monotone map
  will fail to converge. `Functions/makeSyntheticAeroMap.m` shows a valid example.

Both files are listed in `.gitignore`. On an installation with real supplier data these
exact paths hold licensed content, so **do not commit them** from either kind of checkout.

### Step 4 — First run

```matlab
MLTP
```

With no changes, this solves Barcelona-Catalunya in the default `Static` / no-ATD
configuration. On the first run for a given circuit and configuration there is no cached
warm start, so the solver automatically runs the simplified model `MLTP_initial.m` first
and caches its result — you will see:

    MLTP: no cached init for BCN / Mid - running MLTP_initial.m

Subsequent runs of the same configuration reuse that cache and report:

    MLTP: warm start from Data\BCN\initialisation\init_BCN_Mid_...mat

### Step 5 — Verify your setup without a full solve

A full solve is expensive. To confirm the setup is correct in seconds, build the model
without solving — from the repository root:

```matlab
run('userOpts.m');    % loads config, vehicle, powertrain, track, aero collapse
run('vehModel.m');    % builds the symbolic model
fprintf('nx = %d, nu = %d, dx is %s\n', nx, nu, class(dx));
```

A healthy default configuration prints the aero-collapse summary and then
`nx = 9, nu = 3, dx is casadi.SX`. If this works, your data files and CasADi install are
both fine and any later failure is a tuning or convergence problem, not a setup problem.

> Run these as **bare names from the repository root**, exactly as shown. Do not call them
> by full path — see the [`run()` trap](#the-run-cd-trap).

---

## What you must supply

Beyond the two data files, these are the values that decide a run. Everything has a
committed default, so nothing here is mandatory for a first run — but the defaults
describe one specific vehicle at one specific circuit, and are not meaningful for another.

### Decided in `Scripts/userOpts.m`

| Value | Default | What it is |
|---|---|---|
| `AeroConfig` | `'Static'` | Aerodynamic configuration — see the [configuration reference](#configuration-reference) |
| `ATD` | `'Off'` | Active torque distribution on/off |
| `RWMandate` | `'Off'` | Which actuator law the wing follows (`ActiveRW` only) |
| `circuit` | `'BCN'` | Track: `'BCN'`, `'NUR'`, or one of four virtual tracks |
| `vi` | `79.65` | **Initial velocity [m/s]. The one number most likely to need changing.** See below |
| `vf` | `nan` | Final velocity; `nan` lets the solver choose |
| `OPT_ds` | `10` | Collocation step [m]. Smaller = finer mesh, slower, and generally harder to converge |
| `OPT_d` | `3` | Degree of the interpolating polynomials |
| `opts.ipopt.*` | — | IPOPT tolerances and iteration cap (`max_iter = 5000`) |
| `c.ub.*` / `c.lb.*` | — | Actuator rate limits (wing slew is ±25 deg/s, steering ±0.1 rad/s, …) |
| `c.ru/rdu/rdu2.*` | — | Regularisation weights on the inputs and their derivatives |

Boundary states are set as vectors `Xi` and `Xf`; **`nan` means "let the solver choose"**.
Not every combination of fixed boundary conditions is feasible.

### `vi` — the boundary velocity, and how to derive it

`vi` is the speed at the start of the lap. It is not a free parameter you can guess: for a
representative flying lap you want the lap to *close*, i.e. the car to arrive at the finish
line at the same speed it started. `userOpts.m` documents the fixed-point protocol and it
is worth repeating, because getting it wrong is the most common reason IPOPT fails to
converge on a new vehicle, track or configuration:

1. Solve with any plausible `vi`.
2. Read the final speed `vx(end)` from the solution.
3. **Re-seed with `vi = vx(end) - 1`, not `vi = vx(end)`.** The initial state is enforced
   as a *window*, not a pin — ±`OPT_e` in normalised units, which is ±1 m/s on `vx`. The
   solver always takes the fastest admissible entry, so `vx(1)` lands on `vi + 1` every
   time. Seeding `vi = vx(end)` overshoots by exactly the slack and the iteration stalls
   forever at a miss of ~1.0 m/s.
4. Repeat until `|vx(1) - vx(end)| <= 0.5` m/s.

The committed `vi = 79.65` is the converged fixed point for one specific vehicle at
Barcelona in the static configuration. It is reused as the starting guess for the active
configurations rather than re-derived for them, and it is **not** valid for the synthetic
data or for any other vehicle.

### Vehicle and powertrain values

Defaults live in `Parameters/vehParams.m` and `Parameters/Powertrain.m` — masses, inertia,
wheelbase, track width, CoG height and distribution, roll centres, wheel rates, wheel
radius, reference area, drag coefficient, ride heights, brake bias, torque split, power and
speed ceilings.

You can edit those files directly, but there is a cleaner route that avoids touching the
code. Every raw input is read through `Functions/setupValue.m`, which honours an override
file at the repository root. Create `setupOverride.mat` containing a struct named `setup`,
with one field per parameter you want to change:

```matlab
setup = struct();
setup.mb   = 1400;    % sprung mass excl. driver (kg)
setup.l    = 2.65;    % wheelbase (m)
setup.wB   = 0.45;    % front mass fraction (-)
setup.hcg  = 0.40;    % CoG height (m)
setup.A    = 1.95;    % reference area (m^2)
setup.RHf0 = 80;      % nominal front ride height at 10 m/s (mm)
setup.RHr0 = 90;      % nominal rear  ride height at 10 m/s (mm)
save('setupOverride.mat', 'setup');
```

The substitution happens at the *assignment site*, so every quantity derived from an
overridden value is recomputed correctly. Overriding after the fact — patching the finished
`vp` struct — would leave derived quantities computed from the old value and give you a
silently self-inconsistent vehicle. Use the field names exactly as they appear in the
`setupValue('name', default)` calls; an unrecognised name is simply ignored, and a
malformed value warns and falls back to the default rather than stopping the run.

Two values worth singling out, because they feed the aero collapse directly and are
**placeholders in the committed data, not measurements**:

- `RHf0` — nominal front ride height at 10 m/s (mm), default 85
- `RHr0` — nominal rear ride height at 10 m/s (mm), default 95

---

## Running a solve

```matlab
MLTP
```

`MLTP.m` is a script, not a function, and takes no arguments. It clears the workspace when
it starts — save anything you care about first.

| Script | Role |
|---|---|
| `Scripts/MLTP.m` | The solver. **Start here.** |
| `Scripts/MLTP_initial.m` | Simplified single-track model used to warm-start `MLTP.m`. Called automatically when no cache exists; you rarely run it directly |

Expect a long run. IPOPT is capped at 5000 iterations; recorded Barcelona solves converged
in roughly 900–1000 iterations, and wall-clock depends entirely on your machine. Progress
is printed by IPOPT as it goes.

The solution ends up in the workspace struct `data` and is pushed to the Simulink Data
Inspector. **`MLTP.m` does not save `data` to disk** — if you want to keep a solution,
save it yourself:

```matlab
save('my_solution.mat', 'data');
```

---

## Configuration reference

Two flags in `userOpts.m` drive the dimensionality of the whole problem:

- `AeroConfig` → `vp.ActAero`: `'Static'` (fixed wing) or `'ActiveRW'` (wing is an NLP control)
- `ATD` → `pt.ATD`: `'Off'` (AWD, fixed split) or `'On'` (torque distribution is controlled)

Together they set the number of control inputs `nu` and the number of path constraints
`nh`. Within `ActiveRW`, `RWMandate` selects the actuator law:

| `RWMandate` | Name | What the wing does | `nu` (AWD / ATD) |
|---|---|---|---|
| `'Off'` | `ARW` | Free continuous rear-wing angle — the optimiser chooses | 4 / 8 |
| `'Discrete'` | `ARWd` | Continuous solve, then rounding onto fixed stations, with an in-NLP braking floor | 4 / 8 |
| `'FrontWing'` | `AFWd` | Front-wing flap on a combined unload axis | 4 / 8 |
| `'Combined'` | `ARFWd` | Both wings, independently controlled | 5 / 9 |
| `'Reactive'` | `ARFWr` | Both wings pinned to a reactive law driven by onboard yaw rate and speed | 5 / 9 |
| `'Velocity'` | `ARWv` | Speed-scheduled wing + brake trigger. **Retired — see below** | 4 / 8 |

`'Static'` is `nu = 3` with ATD off (drive torque, brake torque, steer) and `nu = 7` with
ATD on.

Low-drag-on-straights / high-downforce-in-corners / airbrake-under-braking behaviour is
**emergent** in the free configurations — it is never hard-coded.

Configuration can also be driven from a `runOverride.mat` at the repository root instead of
by editing `userOpts.m`, which is how batch sweeps are run:

```matlab
AeroConfig = 'ActiveRW'; ATD = 'On'; RWMandate = 'Combined';
save('runOverride.mat', 'AeroConfig', 'ATD', 'RWMandate');
```

Delete the file to return to the committed defaults. Read the
[hijack warning](#stray-override-files-hijack-runs) before you use it.

---

## Things to be wary of

### The `run()` cd trap

MATLAB's `run()` changes the working directory into the target script's folder whenever the
name it is given contains a directory component. Several scripts here call each other with
repository-root-relative paths, for example `run('Parameters\vehParams.m')`. So:

```matlab
run('C:\path\to\repo\Scripts\userOpts.m')   % WRONG - cds into Scripts\, then
                                            % Parameters\vehParams.m resolves to
                                            % Scripts\Parameters\vehParams.m and fails
```

```matlab
cd 'C:\path\to\repo'                        % RIGHT - repository root, bare name
run('userOpts.m')
```

`MLTP.m` handles this itself: it `cd`s to the repository root and then calls bare names.
The trap only bites when you invoke the inner scripts directly.

### Simulink is required to finish a run

The optimisation does not touch Simulink, but the post-processing block does, and it is
**not** guarded by a capability check. Without Simulink a run will solve completely and
then die at the logging stage, and because the last statement of `MLTP.m` is the apex-speed
export, you lose that output too. The solution is still in the workspace variable `data` at
that point.

### Tuning does not transfer

Solver tolerances, regularisation factors, initial guesses and boundary velocities in
`userOpts.m` were hand-calibrated for one specific vehicle at Barcelona-Catalunya.
Changing the vehicle, the track or the configuration generally means re-tuning them and
regenerating an initialisation file, or IPOPT will not converge. This is the normal cost of
direct collocation on a problem this stiff, not a defect — and it applies in full to the
synthetic stand-ins. If a run fails to converge, re-derive `vi` by the fixed-point protocol
above *before* concluding there is a model defect.

### Warm-start caches can silently mismatch

Warm starts are cached under `Data/<circuit>/initialisation/` and selected by circuit, aero
setting, a model-version token, and the control width `nu`.

- The **token** encodes the tyre and aero *model*, not a date. If you change the tyre model
  or the aero map, bump the token in **both** `Functions/latestInit.m` and
  `Scripts/MLTP_initial.m` — they must agree, or a cache written by one becomes permanently
  unreachable by the other. A stale warm start is slow at best and drives IPOPT infeasible
  at worst.
- Swapping in your own tyre data or aero map without bumping the token will reuse caches
  built for the old model. **If in doubt, delete `Data/` and let it regenerate.**
- The `nu` filter exists because the aero setting name does not encode the drivetrain: an
  `ARW` run is `ARW` whether AWD (`nu = 4`) or ATD (`nu = 8`). Caches that fit neither
  3 rows nor exactly `nu` rows are skipped and the run cold-starts.

### Stray override files hijack runs

Both `runOverride.mat` and `setupOverride.mat` are resolved from the *repository root*,
anchored on the source file's own location rather than on the working directory or a bare
filename. This is deliberate: a bare filename resolves through the entire MATLAB search
path, so a forgotten override file in another checkout will silently change your runs. If
results do not match the configuration you think you set, check for stray override files
before anything else — both announce themselves when they load, so read the first few lines
of console output.

### `RWMandate = 'Velocity'` is retired

It is kept for reproducing archived runs and does not currently solve. At the committed
25 deg/s wing slew limit its own commanded slew is 36 deg/s, and the band window it needs
is wider than the enforced ceiling. Both feasibility conditions fail and the gate asserts.
`RWDiscrete = 'Snap'` is likewise retired and raises an explicit error rather than being
silently ignored.

### The reactive law ships pre-fitted for one circuit

`RWMandate = 'Reactive'` (`ARFWr`) reads `Parameters/reactiveLaw.mat`. The shipped law was
fitted from a Barcelona ARFWd/ATD solve. Using it on a different circuit or vehicle is
running outside its calibration. The file carries a `provisional` flag, and `vehParams.m`
warns if it is set.

### Aero collapse has hard gates

`Functions/aeroCollapse.m` resolves the downforce/ride-height feedback loop and fits it to a
differentiable piecewise polynomial. It fails loudly rather than returning a bad fit:

- `aeroCollapse:nanFit` — the ride-height fixed point did not stay inside the map's grid.
  Usually means your `RHf0`/`RHr0` or the map's grid span are inconsistent.
- `aeroCollapse:fitResidual` — the collapse converged but the polynomial fit is worse than
  the tolerance, i.e. the map is not smooth enough.

A healthy run prints a one-line summary (clamp onset speeds, per-axle residuals, iteration
count) before solving.

### Track data must be smooth

The optimisation reads only `s` (distance) and `k` (curvature); `x` and `y` are optional and
used for plotting. **Raw curvature must be pre-processed** — noisy curvature is very
detrimental to the collocation method. The virtual tracks in `userOpts.m` show the pattern:
build `k`, then smooth it with `simpleMA` before use.

To add a circuit, drop a `.mat` with at least `s` and `k` into `Circuits/` and add a `case`
to the switch in `userOpts.m`. An unknown circuit name raises `userOpts:unknownCircuit`
rather than failing obscurely later.

### Do not commit the data files

`Parameters/tyreParams_DoNotPublish.m` and `Parameters/aeroMap_Tur.mat` are gitignored. On
an installation with real supplier data those exact paths hold licensed content. The
synthetic route above tells you to create files at precisely those names, so the ignore
rules protect both kinds of checkout — leave them in place.

---

## What gets written to disk

| Path | Written by | Contents |
|---|---|---|
| `Data/<circuit>/initialisation/` | `MLTP_initial.m` | Cached warm starts, one `.mat` per solve |
| `solutions/apex/` | `apexSpeeds.m` | Apex-speed CSV export, written as the last step of a run |
| Simulink Data Inspector | `plotSDI.m` | States, inputs, tyre forces, slips, aero forces, path constraints, track |

Neither `Data/` nor `solutions/` is tracked by git. The solved trajectory itself is **not**
written to disk automatically — save the workspace `data` struct yourself if you want it.

---

## Model

- **Independent variable is distance `s`**, not time. Lap time is the integrated objective.
  Curvature `k(s)` is the only parameter varying along the grid.
- **States** (`nx = 9`): `[vx vy r n eps Om_fl Om_fr Om_rl Om_rr]` — body velocities, yaw
  rate, distance to centreline, heading error, four wheel speeds.
- **Everything symbolic is scaled.** Each variable `foo` has a normalised symbol `foo_n`
  and a scale `foo_s`, with `foo = foo_s*foo_n`. The NLP works in normalised units.
- **Tyres**: per-axle Pacejka MF5.2 with full combined slip (Gxa/Gyk weighting), so there
  are no friction-circle path constraints — grip limiting falls out of the tyre model. A
  per-tyre vertical load cap is enforced instead.
- **Aerodynamics**: the ride-height lift map drives a quasi-static aeroelastic collapse
  (`Functions/aeroCollapse.m`), resolving the downforce/ride-height feedback loop per speed
  and fitting it to a CasADi-differentiable piecewise polynomial. Active configurations
  layer a 2-D (angle, speed) wing map on top, blended by a smooth cardinal basis so the
  whole chain stays C-infinity and safe to build in CasADi `SX`.
- **Steering** `delta` is the **front road-wheel angle**, not a hand-wheel angle, so its
  magnitude is small (a few degrees). Positive is left.

---

## Repository layout

    Scripts/      entry points, userOpts.m, vehModel.m, post-processing
    Parameters/   vehicle, powertrain, synthetic tyre template, synthetic aero map
    Functions/    model helpers, aero collapse, wing maps, warm-start selection
    Circuits/     track .mat files (need s and k; x, y optional for plotting)

Circuits included: **Barcelona-Catalunya** (`BCN`) and **Nurburgring** (`NUR`), plus four
self-defined virtual tracks (`Hairpin`, `Straight`, `Sturn`, `VirtualTrack`) generated
directly in `userOpts.m`.

### Aerodynamic coefficient data

The aerodynamic coefficients compiled into `Functions/` — `rwAeroDelta.m` (rear-wing angle
sweep) and `fwAeroDelta.m` / `fwOnlyDelta.m` (front-wing flap) — are **real measured data**,
not synthetic. Only the tyre template and the ride-height map generator are invented.

---

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

Not yet chosen — contact the repository owner before redistributing. The upstream
framework is distributed via MATLAB Central File Exchange under its own terms.
