# GUIBuilder Notes

## Scope

This directory is the modular tube-aircraft configuration frontend.

Primary files:
- `guibuilder.py`: Tk window and widget wiring
- `libs/analysis.py`: headless configuration expansion and analysis
- `libs/cad.py`: CAD export mapping from current design state to FreeCAD-style varsets
- `libs/graphics.py`: orthographic drawing on Tk canvas
- `libs/library_editors.py`: main-window editor panels for `data/modules.json` and `data/parts.json`
- `libs/views/configuration_view.py`: configuration main-view layout and assembly-card rendering
- `libs/views/aero_analysis_view.py`: Aero Analysis main-view layout and NeuralFoil airfoil comparison
- `libs/views/vlm_view.py`: VLM main-view layout and selectors
- `libs/views/cad_view.py`: CAD main-view placeholder
- `libs/views/module_editor_view.py`: module-editor main-view mount
- `libs/views/part_editor_view.py`: part-editor main-view mount
- `libs/pointed_shapes.py`: pointed-profile geometry helpers
- `libs/simlib2.py`: propulsion test-data helpers
- `libs/motordata.csv`: measured motor test data
- `libs/batterydata.csv`: battery cell data
- `analyze_headless.py`: non-GUI validation entrypoint
- `cad_example.py`: simple non-GUI example of CAD export usage
- `library_editors_example.py`: simple launch example for the library editors
- `data/default.json`: built-in aircraft config
- `data/templates.json`: field types and creation templates
- `data/parts.json`: part library data
- `data/modules.json`: module and surface library data
- `data/specs.json`: optional standards catalog

## Rules

- Do not silently invent missing aircraft data at runtime.
- If a field is required, put it in `data/default.json`, `data/templates.json`, or the saved config and then use it directly.
- If required data is missing, let it fail visibly.
- Keep UI behavior documented in `docs/ui/ui_layout.md`, do not detail implementation there, only visible UI.
- Keep README about design philosophy and analysis staging, not widget details.
- Keep analysis usable without Tk.
- Keep drawing code in `libs/graphics.py`, not mixed back into `guibuilder.py`.
- Keep library editor UI in `libs/library_editors.py`, mounted into the main window from `guibuilder.py`.

## Save / Load Contract

- Save:
  - `assembly_name`
  - `config`
  - `analysis`
- Saved `analysis` must not include the expanded `assembly` field.
- Load ignores saved `analysis` and recomputes from `config`.

## Current Caveats

- `material_density_g_cm3` is stored and editable, but not yet used by analysis or drawing.
- `standard_profile` and `apply_standard` exist, but standards are only used to lock dimensions such as fuselage OD/ID. No DRC is applied yet.
- Assembly cards can load a module block directly from `data/modules.json`; this is instance replacement, not a live link.
- The aero path is still:
  - assembled modular geometry
  - Barrowman-like static stability
  - simple linear AoA pass
- AeroSandbox 3D exists in the main center-pane `ASB` tab.
- VLM sweep now exists in the main-window `VLM` view.
- `Aero Analysis` now provides NeuralFoil-backed 2D airfoil comparison plots and summary output.
- The right side of the `VLM` view uses the selected solved-point controls and a `Show` button that opens the normal external AeroSandbox/PyVista window.
- The displayed VLM view should come from the actual AeroSandbox draw path, not a custom renderer.
- That transient VLM cache is display-only and must not be saved to disk.
- Orthographic drawing is Tk-canvas-native, not CAD geometry.
- Motor geometry uses measured simlib2 CSV dimensions plus explicit shaft/base drawing constants.

## Remember

- If saved analysis grows, keep computed results but avoid duplicating expanded config-derived geometry.
- When adding editable config fields:
  - add them to `data/default.json`
  - add type info to `data/templates.json`
  - expose them in UI if intended
  - update `docs/ui/ui_layout.md`
- Keep `analyze_headless.py` working as the simplest validation path for config and analysis changes.
