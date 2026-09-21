# Ready-made laps

Three solved minimum-lap-time solutions, shipped so the Simulink and Unreal side
of this repository runs on a fresh clone without solving anything first.

| file | circuit | lap time | notes |
|---|---|---|---|
| `run_BCN_ARFWr_ATD_data.mat`  | Circuit de Barcelona-Catalunya | 102.674 s | the default track everything falls back to |
| `run_NUR_ARFWr_ATD_data.mat`  | Nürburgring GP                 | 108.293 s | law fitted on Barcelona, applied unseen |
| `run_Spa_ARFWr_ATD_data.mat`  | Spa-Francorchamps              | 127.814 s | third track, solved to prove the pipeline is track-agnostic |

All three are `ARFWr` (both wings pinned to the reactive controller law) with
`ATD` (torque vectoring), solved at a 25 deg/s wing slew rate. Every one returned
`Solve_Succeeded` and closed its entry/exit speed to well inside the 0.5 m/s
tolerance.

## What is in them

Each file holds a single struct variable `data` — the same variable, the same
field names and the same filename shape that `Scripts/solveLap.m` writes to
`solutions/report/<circuit>/raw/`. They are **slim** copies: three fields are
stripped by `Scripts/exportLapSidecar.m`, which takes ~11-16 MB down to ~1.3 MB
without touching anything downstream reads.

- `init` — the warm-start solution the solve began from, i.e. a whole previous
  lap nested inside this one. An input to the solve, never an output.
- `sdi` — a Simulink Data Inspector payload, including a run handle that does not
  survive being reloaded in another session.
- `w_opt` — the raw stacked NLP decision vector in normalised units; already
  present, unscaled and unpacked, as `x_opt` / `u_opt` / `y_opt` / `xc_opt`.

Everything else is verbatim: the solved trajectory (`x_opt`, `u_opt`, `y_opt` and
the resampled `x_full` / `u_full` / `y_full` on their `s_full` / `t_opt` axes),
the track geometry (`track`, `track0`), the wing and aero metadata (`aeroARW`,
`aeroAFW`, `rwDisc`, `fwDisc`, `lawTrack`), the vehicle and constraint records and
the solver's verdict (`solver_status`, `iter_count`, `duration`).

The consequence of dropping `init` is worth stating: a slim lap can be replayed,
plotted, and used as the Simulink reference, but it cannot itself be handed back
to `MLTP` as a warm start. That is what the init caches under
`Data/<circuit>/initialisation/` are for, and `solveLap` builds those from
`x_opt`/`u_opt` rather than from `init`.

The Barcelona and Nürburgring laps were solved before `solveLap` existed, so they
carry none of its closure annotations (`vi`, `vend`, `miss`, `closed`, `circuit`,
`config`, `drivetrain`, `solvedOn`); the Spa lap carries all of them. Readers
derive the missing ones from `x_opt` instead, so the difference is invisible in
use.

## Who reads them

- `Scripts/solveLap.m` — looks here as the second of its two lap locations, after
  `solutions/report/<circuit>/raw/`. This is what makes `solveLap('BCN')` on a
  fresh clone resolve in about a second instead of solving four ladder stages.
- `simulink/tools/activeTrack.m` — third step of its fallback chain, after an
  explicitly selected track (`simulink/data/activeTrack.mat`) and your own local
  solves under `solutions/` (git-ignored, created the first time you solve
  something). So with no active track selected and nothing solved locally, the
  sim tools default to the Barcelona lap here.
- `simulink/tools/setupTrack.m` — point the whole sim at one of them directly:

```matlab
setupTrack('simulink/data/laps/run_Spa_ARFWr_ATD_data.mat')
out = runDemoLap();
```

## How they were produced

```matlab
exportLapSidecar('solutions/report/BCN/raw/run_BCN_ARFWr_ATD_data.mat', ...
                 'simulink/data/laps');
```

after the corresponding `solveLap(<circuit>)`. Re-running the exporter on a
freshly solved lap is the whole update procedure.
