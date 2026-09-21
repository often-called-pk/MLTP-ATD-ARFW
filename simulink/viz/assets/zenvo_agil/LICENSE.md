# 2026 Zenvo Aurora Agil — third-party asset

This folder contains a third-party 3D model that is **not** original work of this
project. It is redistributed here under the Creative Commons licence its author
published it under.

| | |
|---|---|
| **Model** | 2026 Zenvo Aurora Agil |
| **Author** | **Ddiaz Design** (Sketchfab user `Ddiaz Design`) |
| **Sketchfab uid** | `7659b0982c9f4550a674bc73e6d3497e` |
| **Source** | <https://sketchfab.com/3d-models/2026-zenvo-aurora-agil-7659b0982c9f4550a674bc73e6d3497e> |
| **Licence** | **CC Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)** — <https://creativecommons.org/licenses/by-nc-sa/4.0/> |
| **Downloaded** | 2026-09-17 (Sketchfab "Download → FBX", `source/FINAL_MODEL (2)/FINAL_MODEL.fbx`) |

The author, licence and provenance below were read from the Sketchfab public API
(`https://api.sketchfab.com/v3/models/7659b0982c9f4550a674bc73e6d3497e`) on
2026-09-17, which returned `user.displayName = "Ddiaz Design"`,
`license.label = "CC Attribution-NonCommercial-ShareAlike"`,
`license.slug = "by-nc-sa"` and
`license.requirements = "Author must be credited. No commercial use. Modified
versions must have the same license."`

## Provenance

The uploader's own model description states, verbatim:

> "Based on a CSR2 3d model"

so the mesh is a game asset re-published by Ddiaz Design, not an engineering
model and not supplied by Zenvo Automotive. Nothing derived from it is a
statement about the real vehicle's geometry.

## Attribution

> "2026 Zenvo Aurora Agil" by **Ddiaz Design**, licensed under
> [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
> Source: <https://sketchfab.com/3d-models/2026-zenvo-aurora-agil-7659b0982c9f4550a674bc73e6d3497e>

**No credit line is rendered on the frames**: the creators were contacted by the
project owner on 2026-09-17 and confirmed that no credit line is required. The
attribution above is kept in the repository as the record of origin, and the
NonCommercial and ShareAlike terms continue to apply to this academic, non-commercial use.

## Files

| File | What it is |
|---|---|
| `model.fbx` | the uploader's `source/FINAL_MODEL (2)/FINAL_MODEL.fbx`, unchanged — **the donor file the pipeline imports** |
| `*.png` | the texture set shipped beside that FBX by the uploader, copied next to `model.fbx` so the file's own relative references resolve |
| `model_textures/`, `model.fbm/` | **generated, not committed.** The FBX loader extracts the model's embedded textures beside it on first load |

There is **no `manifest.m` in this folder**: this model is used only as a donor
for the car whose manifest is `../zenvo_tur/manifest.m`, which addresses this file
through its own `M.donor` fields. `assetManifest.m` registers no separate name for it.

Not kept: the download `.zip`, the uploader's duplicate `textures/` and
`source/FINAL_MODEL (2)/FINAL_MODEL_textures/` copies of the same images, and the
Sketchfab USDZ/USDC export (opaque to `sim3d` — a USD load arrives as a single
`sim3d.usd.Actor` with no addressable geometry, measured 2026-09-17).

## Modifications made by this project

The model is **not modified on disk**. It is loaded into a hidden actor at build
time and only two parts of it are used, both face-extracted from its shells by
bounding box and copied onto this project's own actors:

- the **rear wing plane**, which is then rotated about the measured pylon-top line
  to show the commanded rear-wing angle;
- the **two wing pylons**, placed static on the companion Tur body's rear deck.

The rest of the Agil body is discarded after the extraction. Nothing about this
model contributes to any reported number: the visualisation layer is a likeness,
and every lap-time and aerodynamic result in this project comes from the MLTP
solver and the certified Simulink gates.
