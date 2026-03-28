# GUIBuilder Notes

## Scope

This directory is the modular tube-aircraft configuration frontend.

Primary files:
- `guibuilder.py`: Tk window and widget wiring
- `analysis.py`: headless configuration expansion and analysis
- `cad.py`: CAD export mapping from current design state to FreeCAD-style varsets
- `graphics.py`: orthographic drawing on Tk canvas
- `analyze_headless.py`: non-GUI validation entrypoint
- `cad_example.py`: simple non-GUI example of CAD export usage
- `libs/default.json`: built-in aircraft config
- `libs/templates.json`: field types and creation templates
- `libs/parts.json`: part library data
- `libs/specs.json`: optional standards catalog

## Rules

- Do not silently invent missing aircraft data at runtime.
- If a field is required, put it in `libs/default.json`, `libs/templates.json`, or the saved config and then use it directly.
- If required data is missing, let it fail visibly.
- Keep UI behavior documented in `docs/ui/ui_layout.md`, do not detail implementation there, only visible UI.
- Keep README about design philosophy and analysis staging, not widget details.
- Keep analysis usable without Tk.
- Keep drawing code in `graphics.py`, not mixed back into `guibuilder.py`.

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
- The aero path is still:
  - assembled modular geometry
  - Barrowman-like static stability
  - simple linear AoA pass
- AeroSandbox 3D exists in the main center-pane `ASB` tab.
- VLM sweep now exists in the main-window `Aero Analysis` view.
- The right side of the `Aero Analysis` view is still a placeholder; no VLM display view is implemented there yet.
- Orthographic drawing is Tk-canvas-native, not CAD geometry.
- Motor geometry uses measured simlib CSV dimensions plus explicit shaft/base drawing constants.

## Remember

- If AeroSandbox 3D is added, make it a distinct center-pane mode instead of replacing the orthographic view path blindly.
- If saved analysis grows, keep computed results but avoid duplicating expanded config-derived geometry.
- When adding editable config fields:
  - add them to `libs/default.json`
  - add type info to `libs/templates.json`
  - expose them in UI if intended
  - update `docs/ui/ui_layout.md`
- Keep `analyze_headless.py` working as the simplest validation path for config and analysis changes.
