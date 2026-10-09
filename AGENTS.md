# GUIBuilder Notes

## Scope

This directory is the plane-first tube-aircraft design tool.

Primary files:
- `webbuilder.py`: web front-end entry: starts the local aiohttp server on 127.0.0.1 and opens the browser (`--port`, `--no-browser`)
- `webapp/`: Python server over `libs/` for the web front-end. `state.py` (one session definition/report/recompute + background aero), `server.py` (routes, `/ws/events` state push, draw/move/model3d/editor/library APIs), `drawing.py` + `drawing_enrich.py` (record the `libs/graphics.py` drawing and enrich it: airfoil sections, projected spars, front-view thickness, roll detents, drag handles), `editor.py` (editor context, path edits and structural ops, ports of the retired Tk editor with source-line citations), `model3d.py` (triangle meshes for the 3D tab), `library.py` (part library payload/save/delete), `sim.py` (one server-owned simulation run: spawns `sim_inav.py`/`sim_script.py`, streams stdout and tails the flight-log CSV), `serialize.py`; headless suites `test_draw_drag.py`, `test_editor_sync.py`, `test_library.py`, `test_sim.py`
- `web/`: React + TypeScript + Vite front-end (npm; `web/dist` is what the server serves): `src/editor/` (the nine definition editor tabs), `src/components/` (Simple drawing and the Three.js `ModelView`), `src/parteditor/` (part library tree + editor), `src/views/`; three.js for 3D, uPlot for the motor chart
- `libs/airframe.py`: airframe/2 schema loader and validator (the saved aircraft definition) and size preset validation
- `libs/project.py`: aircraft project layout (`aircraft/<project>/`), open and copy
- `libs/presets.py`: size presets (`data/presets/<name>.json`) applied to, compared with and saved from a definition
- `libs/geometry.py`: `resolve_airframe`: shared geometry, mass points, CG, checks; every consumer reads it
- `libs/aero.py`: aero (lifting-line wings + AeroBuildup bodies), Barrowman cross-check, VLM sweep
- `libs/report.py`: report shared by the web UI and `analyze_airframe.py`; lines carry a level (`libs/reportline.py`: info, warning, problem) the UI colours
- `libs/cad.py`: CAD export: CadQuery solid geometry (STEP/STL, built in mm): whole airframe, and printable sections and couplers
- `libs/drawings.py`: blueprint drawings (SVG/PNG) of the printable parts: side cutaway (cut faces hatched), three-view, x-ray (skin see-through)
- `libs/varsets.py`: FreeCAD varsets, one JSON per varset (Varset_FROM_JSON.FCMacro); every value in exactly one varset
- `libs/graphics.py`: orthographic drawing on a duck-typed canvas (recorded server-side for the web)
- `libs/airfoil_analysis.py`: NeuralFoil 2D airfoil comparison and DAT import
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
- `aircraft/<project>/`: one aircraft project (`aircraft/` is gitignored except `aircraft/example`, a copy of simpleplane): `aircraft.json` (airframe/2 definition), `inav/` (real FC diffs, SITL diffs; `generated/`: the diff written for each generated or real-FC run), `sitl/` (pinned SITL.elf, eeprom.bin, sitl.log), `jsbsim/` (export), `logs/` (SITL and script flight logs and plots), `cad/` (exports; drawings `cutaway`, `threeview`, `xray` .svg/.png; `sections/`: printable parts, `varsets/`: FreeCAD varsets), `controllers/` (flight controller scripts)
- `data/parts/`: part library: tables `<category>.json` (battery, prop, material, connector, mounting patterns) and role parts, one file each, `<role>/<name>.json` (fc, esc, camera, companion, vtx, receiver, gps, servo)
- `data/airfoils.json`: imported airfoil coordinates
- `data/inav/`: INAV settings templates (`sitl.txt` bench hardware, `plane.txt`, `tailsitter.txt` switches and modes) for generated SITL diffs
- `data/presets/`: size presets (tube62, tube75), one file each, in the definition's own form (section, joints, rods, spars, limits, CAD)
- `data/specs.json`: shared specs: fasteners, printer, openings, signal wire paths (an early sketch, not authoritative)

## Rules

- Do not silently invent missing aircraft data at runtime.
- If a field is required, put it in the airframe/2 schema (`libs/airframe.py`, `PLAN.md` section 1) or the part library `data/parts/` and then use it directly.
- If required data is missing, let it fail visibly.
- Keep UI behavior documented in `docs/ui/` (one file per view). Update the matching doc when visible UI behavior changes.
- Keep README about design philosophy and analysis staging, not widget details.
- Keep analysis usable headless: `libs/` and the analysis entry points must work without the web server or a browser.
- INAV always flies a diff file the owner can read and edit. Never compose FC settings from strings in code: templates live in `data/inav/`, the composed diff is written to the project's `inav/generated/`.
- Keep drawing code in `libs/graphics.py`, not mixed into the server or front-end.
- The web front-end is the only UI; new UI work goes in `web/`.
- Keep web-specific engine adapters in `webapp/`; prefer adapter code there over changing `libs/` (change `libs/` only when the shared engine itself changes). The retired Tk sources live in git history; `webapp/editor.py` and `webapp/library.py` cite them, and `docs/ui/` documents the behavior those ports follow.
- The 2D scene is recorded from `libs/graphics.py` unchanged and enriched in `webapp/drawing_enrich.py`; do not fork a second drawing implementation in JS.
- Web mutations go through the browser store's queued editor calls; element drags use the move-with-draw fast path (geometry-only recompute, no snapshot/WS push per motion; the release syncs the full state).
- Run the web app under the drone env: `cd web && npm install && npm run build`, then `/home/anon/miniconda3/envs/drone/bin/python webbuilder.py aircraft/example`. Rebuild `web/` after front-end edits and restart the server after `webapp/` edits.
- Keep the headless suites (`webapp/test_draw_drag.py`, `webapp/test_editor_sync.py`, `webapp/test_library.py`) passing when touching drawing, editor or library paths.

## Save / Load Contract

- Load opens a project directory (`aircraft/<project>/`); its `aircraft.json` must be airframe/2; everything is recomputed.
- Save writes the project's `aircraft.json` (the definition only; nothing computed). Save As copies the whole project under a new name, then saves there.
- The web does these project operations server-side (File menu -> `webapp/server.py`).

## Current Caveats

- The aero path (`libs/aero.py` `analyze_aero`) is lifting-line wings (AeroSandbox `LiftingLine` algorithm, batched over AoA in `lifting_line_sweep`) plus AeroBuildup bodies, with a Barrowman cross-check. Keep `lifting_line_sweep` equal to `asb.LiftingLine`.
- The editor's centre tabs are Simple (the enriched recorded scene) and 3D (Three.js, meshes from `webapp/model3d.py`). The Tk ASB tab went with the Tk UI.
- The VLM sweep engine (`libs/aero.py`) has no UI: the web VLM view is not built yet (placeholder). When it is, it renders the solved panel/streamline data in-browser (PyVista cannot run server-side), and the transient VLM cache stays display-only (never saved to disk).
- `Aero Analysis` has no UI: the web view is not built yet (placeholder). The NeuralFoil-backed 2D airfoil comparison runs headless through `libs/airfoil_analysis.py` / `airfoil_analysis_example.py`.
- SIM is ported: `web/src/views/SimView.tsx` with `webapp/sim.py` runs the same engines as a server-owned subprocess (one run at a time, shared by every tab), streams stdout and tails the flight log for the plots and 3D path. The engines stay scriptable headless through `sim_inav.py` and `sim_script.py`.
- Orthographic drawing starts from `libs/graphics.py`, recorded server-side into plain draw data and enriched for the web (`webapp/drawing_enrich.py`): airfoil sections, projected spar geometry, front-view thickness. It is not CAD geometry.
- Motor geometry uses motorlib stator dimensions plus explicit shaft/base drawing constants.

## Remember

- When adding editable definition fields:
  - add them to the schema in `libs/airframe.py` and to `aircraft/*/aircraft.json`
  - document them in `PLAN.md` section 1
  - expose them in the web editor (`web/src/editor/`, structural ops and semantics in `webapp/editor.py`)
  - update the matching `docs/ui/<view>.md`
- Keep `analyze_airframe.py` working as the simplest validation path for definition and analysis changes.
