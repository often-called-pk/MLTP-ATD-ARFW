# 2026 Zenvo Aurora Tur — third-party asset

This folder contains a third-party 3D model that is **not** original work of this
project. It is redistributed here under the Creative Commons licence its author
published it under.

| | |
|---|---|
| **Model** | 2026 Zenvo Aurora Tur |
| **Author** | **Ddiaz Design** (Sketchfab user `Ddiaz Design`) |
| **Sketchfab uid** | `a9420f263dac41e99e42112c122987a8` |
| **Source** | <https://sketchfab.com/3d-models/2026-zenvo-aurora-tur-a9420f263dac41e99e42112c122987a8> |
| **Licence** | **CC Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)** — <https://creativecommons.org/licenses/by-nc-sa/4.0/> |
| **Downloaded** | 2026-09-17 (Sketchfab "Download → FBX", `source/Final_Model/Final_Model.fbx`) |

The author, licence and provenance below were read from the Sketchfab public API
(`https://api.sketchfab.com/v3/models/a9420f263dac41e99e42112c122987a8`) on
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

> "2026 Zenvo Aurora Tur" by **Ddiaz Design**, licensed under
> [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
> Source: <https://sketchfab.com/3d-models/2026-zenvo-aurora-tur-a9420f263dac41e99e42112c122987a8>

**No credit line is rendered on the frames**: the creators were contacted by the
project owner on 2026-09-17 and confirmed that no credit line is required. The
attribution above is kept in the repository as the record of origin, and the
NonCommercial and ShareAlike terms continue to apply to this academic, non-commercial use.

## Files

| File | What it is |
|---|---|
| `model.fbx` | the uploader's `source/Final_Model/Final_Model.fbx`, unchanged — **the file the pipeline imports** |
| `*.png` | the texture set shipped beside that FBX by the uploader, copied next to `model.fbx` so the file's own relative references resolve |
| `manifest.m` | this project's import recipe (measurements, extraction boxes, hinge/pivot lines) — original work of this project |
| `model_textures/`, `model.fbm/` | **generated, not committed.** The FBX loader extracts the model's embedded textures beside it on first load |

Not kept: the download `.zip`, the uploader's duplicate `textures/` and
`source/Final_Model/Final_Model_textures/` copies of the same images, the
Arnold `.tx` caches, and the Sketchfab USDZ/USDC export (opaque to `sim3d` —
a USD load arrives as a single `sim3d.usd.Actor` with no addressable geometry,
measured 2026-09-17, so it cannot be split or measured in this pipeline).

## Modifications made by this project

The model is used as the visual likeness of the Zenvo Aurora in the offline
replay and in-model visualisation. It is **not modified on disk** — every change
below is applied at load time from `manifest.m` in this folder:

- yawed 90° so the car's nose lies on ISO 8855 +x (the file is authored
  nose-along-its-own +y);
- imported at scale 1: the measured mesh wheelbase, 2.802 m, is already the
  vehicle's own 2.800 m to within 2 mm, so no wheelbase-matching shrink is applied;
- the four wheel corners' rotating (rim/disc/tyre) and fixed (calliper) parts
  split apart and their geometry copied onto separate actors this project poses
  independently (steered and spun), leaving the originals hidden in place;
- a band of the front bumper's own vent geometry **face-extracted** onto a hinged
  actor so it can be actuated as the front aerodynamic surface, and its slot culled
  from the nose shells so the band does not appear twice;
- the rear wing and its pylons **face-extracted from the companion Agil model**
  (`../zenvo_agil/`, same author and licence) and placed on this body's rear deck —
  the Tur has no rear wing of its own;
- 5 cm colour bands added on the moving surfaces' edges to show the station the
  controller has selected.

Nothing about this model contributes to any reported number: the visualisation
layer is a likeness, and every lap-time and aerodynamic result in this project
comes from the MLTP solver and the certified Simulink gates.
