# GUIBuilder Notes

## Scope

This directory is the plane-first tube-aircraft design tool.

Primary files:
- `guibuilder.py`: Tk window and widget wiring
- `libs/airframe.py`: airframe/2 schema loader and validator (the saved aircraft definition)
- `libs/geometry.py`: `resolve_airframe`: shared geometry, mass points, CG, checks; every consumer reads it
- `libs/aero.py`: aero (lifting-line wings + AeroBuildup bodies), Barrowman cross-check, VLM sweep
- `libs/report.py`: plain-text report shared by the GUI and `analyze_airframe.py`
- `libs/cad.py`: CAD export: FreeCAD-style varsets plus CadQuery solid geometry (STEP/STL, built in mm)
- `libs/graphics.py`: orthographic drawing on Tk canvas
- `libs/library_editors.py`: main-window editor panel for `data/parts.json`
- `libs/airfoil_analysis.py`: NeuralFoil 2D airfoil comparison and DAT import
- `libs/views/configuration_view.py`: configuration main-view layout and definition editors
- `libs/views/aero_analysis_view.py`: Aero Analysis main-view layout and NeuralFoil airfoil comparison
- `libs/views/vlm_view.py`: VLM main-view layout and selectors
- `libs/views/cad_view.py`: CAD main-view with STEP/STL export controls
- `libs/views/part_editor_view.py`: part-editor main-view mount
- `libs/pointed_shapes.py`: pointed-profile geometry helpers
- `libs/simlib2.py`: propulsion test-data helpers
- `libs/motordata.csv`: measured motor test data
- `libs/batterydata.csv`: battery cell data
- `analyze_airframe.py`: non-GUI analysis and STEP/STL export entrypoint
- `aero_check.py`: aero cross-check
- `libs/jsbsim.py`: JSBSim aircraft export and INAV hover + forward mixer profiles
- `sim_inav.py`: flies the JSBSim export in INAV SITL on `inav_development/inav-jsbsim-bench`
- `airfoil_analysis_example.py`: simple non-GUI example of the airfoil comparison
- `aircraft/*.json`: airframe/2 aircraft definitions
- `data/parts.json`: part library data
- `data/airfoils.json`: imported airfoil coordinates
- `data/specs.json`: optional standards catalog

## Rules

- Do not silently invent missing aircraft data at runtime.
- If a field is required, put it in the airframe/2 schema (`libs/airframe.py`, `PLAN.md` section 1) or `data/parts.json` and then use it directly.
- If required data is missing, let it fail visibly.
- Keep UI behavior documented in `docs/ui/ui_layout.md`, do not detail implementation there, only visible UI.
- Keep README about design philosophy and analysis staging, not widget details.
- Keep analysis usable without Tk.
- Keep drawing code in `libs/graphics.py`, not mixed back into `guibuilder.py`.
- Keep library editor UI in `libs/library_editors.py`, mounted into the main window from `guibuilder.py`.

## Save / Load Contract

- Save writes the airframe/2 definition only; nothing computed is saved.
- Load accepts airframe/2 files only and recomputes everything.

## Current Caveats

- The aero path (`libs/aero.py` `analyze_aero`) is lifting-line wings (AeroSandbox `LiftingLine` algorithm, batched over AoA in `lifting_line_sweep`) plus AeroBuildup bodies, with a Barrowman cross-check. Keep `lifting_line_sweep` equal to `asb.LiftingLine`.
- AeroSandbox 3D exists in the main center-pane `ASB` tab.
- VLM sweep exists in the main-window `VLM` view.
- `Aero Analysis` provides NeuralFoil-backed 2D airfoil comparison plots and summary output.
- The right side of the `VLM` view uses the selected solved-point controls and a `Show` button that opens the normal external AeroSandbox/PyVista window.
- The displayed VLM view should come from the actual AeroSandbox draw path, not a custom renderer.
- That transient VLM cache is display-only and must not be saved to disk.
- Orthographic drawing is Tk-canvas-native, not CAD geometry.
- Motor geometry uses measured simlib2 CSV dimensions plus explicit shaft/base drawing constants.

## Remember

- When adding editable definition fields:
  - add them to the schema in `libs/airframe.py` and to `aircraft/*.json`
  - document them in `PLAN.md` section 1
  - expose them in UI if intended
  - update `docs/ui/ui_layout.md`
- Keep `analyze_airframe.py` working as the simplest validation path for definition and analysis changes.
