# GUIBuilder Notes

## Scope

This directory is the plane-first tube-aircraft design tool.

Primary files:
- `guibuilder.py`: Tk window and widget wiring
- `libs/airframe.py`: airframe/2 schema loader and validator (the saved aircraft definition) and size preset validation
- `libs/project.py`: aircraft project layout (`aircraft/<project>/`), open and copy
- `libs/presets.py`: size presets (`data/presets/<name>.json`) applied to, compared with and saved from a definition
- `libs/geometry.py`: `resolve_airframe`: shared geometry, mass points, CG, checks; every consumer reads it
- `libs/aero.py`: aero (lifting-line wings + AeroBuildup bodies), Barrowman cross-check, VLM sweep
- `libs/report.py`: report shared by the GUI and `analyze_airframe.py`; lines carry a level (`libs/reportline.py`: info, warning, problem) the GUI colours
- `libs/cad.py`: CAD export: CadQuery solid geometry (STEP/STL, built in mm): whole airframe, and printable sections and couplers
- `libs/cutaway.py`: blueprint side cutaway (SVG/PNG) of the printable parts: cut faces hatched, far half hidden-line drawn
- `libs/varsets.py`: FreeCAD varsets, one JSON per varset (Varset_FROM_JSON.FCMacro); every value in exactly one varset
- `libs/graphics.py`: orthographic drawing on Tk canvas
- `libs/library_editors.py`: main-window editor panel for the part library `data/parts/`
- `libs/airfoil_analysis.py`: NeuralFoil 2D airfoil comparison and DAT import
- `libs/views/editor_view.py`: Editor main-view layout and definition editors
- `libs/views/aero_analysis_view.py`: Aero Analysis main-view layout and NeuralFoil airfoil comparison
- `libs/views/vlm_view.py`: VLM main-view layout and selectors
- `libs/views/sim_view.py`: SIM main-view: runs `sim_inav.py` or `sim_script.py`, streams its output, live flight-log plot
- `libs/views/cad_view.py`: CAD main-view with STEP/STL, printable section, cutaway and varset export controls
- `libs/views/part_editor_view.py`: part-editor main-view mount
- `libs/pointed_shapes.py`: pointed-profile geometry helpers
- `libs/motorlib/`: motorlib (rcplanes.online calculator data and formulas: motors, props, cells, ESCs, wires), copied from dronebuildcalc
- `libs/batterydata.csv`: battery cell data (not read by any code)
- `analyze_airframe.py`: non-GUI analysis and STEP/STL export entrypoint
- `aero_check.py`: aero cross-check
- `libs/jsbsim.py`: JSBSim aircraft export (AeroBuildup aero tables, control surfaces) and INAV mixer profiles and servo mix
- `libs/propulsion.py`: motor + prop model on motorlib, wiring resistance from the design, throttle as ESC duty
- `sim_inav.py`: flies the JSBSim export in INAV SITL on `inav_development/inav-jsbsim-bench`
- `sim_script.py`: flies the JSBSim export with a project's controller script, lockstep in simulated time
- `libs/controller.py`: `Controller` script API: simulated SPI IMU (at the first fc part's position and axes), baro, mag, GPS, pitot, rangefinder in raw hardware units; ESC/servo pulses in us; `truth` (`TruthState`): the simulator's own state, for scripting around a flight, not for control
- `libs/simrun.py`: shared by both runners: start conditions, vehicle build, ground rule, electrical model, plots
- `airfoil_analysis_example.py`: simple non-GUI example of the airfoil comparison
- `aircraft/<project>/`: one aircraft project (`aircraft/` is gitignored except `aircraft/example`, a copy of simpleplane): `aircraft.json` (airframe/2 definition), `inav/` (real FC diffs, SITL diffs; `generated/`: the diff written for each generated or real-FC run), `sitl/` (pinned SITL.elf, eeprom.bin, sitl.log), `jsbsim/` (export), `logs/` (SITL and script flight logs and plots), `cad/` (exports; `cutaway.svg/.png`; `sections/`: printable parts, `varsets/`: FreeCAD varsets), `controllers/` (flight controller scripts)
- `data/parts/`: part library: tables `<category>.json` (battery, prop, material, connector, mounting patterns) and role parts, one file each, `<role>/<name>.json` (fc, esc, camera, companion, vtx, receiver, gps, servo)
- `data/airfoils.json`: imported airfoil coordinates
- `data/inav/`: INAV settings templates (`sitl.txt` bench hardware, `plane.txt`, `tailsitter.txt` switches and modes) for generated SITL diffs
- `data/presets/`: size presets (M62, M75), one file each, in the definition's own form (section, joints, rods, spars, limits, CAD)
- `data/specs.json`: shared specs: fasteners, printer, openings, signal wire paths (an early sketch, not authoritative)

## Rules

- Do not silently invent missing aircraft data at runtime.
- If a field is required, put it in the airframe/2 schema (`libs/airframe.py`, `PLAN.md` section 1) or the part library `data/parts/` and then use it directly.
- If required data is missing, let it fail visibly.
- Keep UI behavior documented in `docs/ui/ui_layout.md`, do not detail implementation there, only visible UI.
- Keep README about design philosophy and analysis staging, not widget details.
- Keep analysis usable without Tk.
- INAV always flies a diff file the owner can read and edit. Never compose FC settings from strings in code: templates live in `data/inav/`, the composed diff is written to the project's `inav/generated/`.
- Keep drawing code in `libs/graphics.py`, not mixed back into `guibuilder.py`.
- Keep library editor UI in `libs/library_editors.py`, mounted into the main window from `guibuilder.py`.

## Save / Load Contract

- Load opens a project directory (`aircraft/<project>/`); its `aircraft.json` must be airframe/2; everything is recomputed.
- Save writes the project's `aircraft.json` (the definition only; nothing computed). Save As copies the whole project under a new name, then saves there.

## Current Caveats

- The aero path (`libs/aero.py` `analyze_aero`) is lifting-line wings (AeroSandbox `LiftingLine` algorithm, batched over AoA in `lifting_line_sweep`) plus AeroBuildup bodies, with a Barrowman cross-check. Keep `lifting_line_sweep` equal to `asb.LiftingLine`.
- AeroSandbox 3D exists in the main center-pane `ASB` tab.
- VLM sweep exists in the main-window `VLM` view.
- `Aero Analysis` provides NeuralFoil-backed 2D airfoil comparison plots and summary output.
- The right side of the `VLM` view uses the selected solved-point controls and a `Show` button that opens the normal external AeroSandbox/PyVista window.
- The displayed VLM view should come from the actual AeroSandbox draw path, not a custom renderer.
- That transient VLM cache is display-only and must not be saved to disk.
- Orthographic drawing is Tk-canvas-native, not CAD geometry.
- Motor geometry uses motorlib stator dimensions plus explicit shaft/base drawing constants.

## Remember

- When adding editable definition fields:
  - add them to the schema in `libs/airframe.py` and to `aircraft/*/aircraft.json`
  - document them in `PLAN.md` section 1
  - expose them in UI if intended
  - update `docs/ui/ui_layout.md`
- Keep `analyze_airframe.py` working as the simplest validation path for definition and analysis changes.
