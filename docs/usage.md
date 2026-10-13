# Usage

## Running it

Dependencies (aerosandbox, neuralfoil, cadquery, pandas, numpy, threadpoolctl) live in the conda env `drone`.

```
cd web && npm install && npm run build                        # once, and after front-end edits
python webbuilder.py aircraft/example                         # web UI
python webbuilder.py aircraft/example --host 0.0.0.0 --no-browser # phone/LAN access on port 8765
python webbuilder.py aircraft/quadlancer62
python analyze_airframe.py aircraft/m62v2                     # same analysis, text report
python analyze_airframe.py aircraft/m62v2 --aoa 5 --airspeed 20 --sweep
python analyze_airframe.py aircraft/m62v2 --export-step out.step --step-mode fused --export-stl out.stl
python analyze_airframe.py aircraft/m62v2 --export-sections   # printable sections and couplers into cad/sections/
python analyze_airframe.py aircraft/m62v2 --export-varsets     # FreeCAD varsets into cad/varsets/
python analyze_airframe.py aircraft/m62v2 --export-drawings    # blueprint drawings: cad/cutaway, threeview, xray (.svg and .png)
python aero_check.py                                          # live lifting line vs asb.LiftingLine, per-surface info
python sim_flight.py aircraft/m62v1 hover --controller inav     # INAV SITL on JSBSim: hover (VTOL)
python sim_flight.py aircraft/quadlancer62 efficiency --controller inav # speed sweep: Wh/km, range, endurance
python sim_flight.py aircraft/m62v1 efficiency --controller inav --start ground # take off and transition first
python sim_flight.py aircraft/m62v1 top-speed --controller inav --inav-config aircraft/m62v1/inav/m62v1_2025-08-20.txt
python sim_flight.py aircraft/example controls --controller inav # stick steps checked against motion
python sim_flight.py aircraft/testglide glide --controller inav # motors off: smallest sink rate and best glide
python sim_flight.py aircraft/example glide --controller inav --alt 600 --heading 90 --wind-from 270 --wind-speed 5
python sim_flight.py aircraft/example acro_climb --controller inav --clock stepped-faster
python sim_flight.py aircraft/example configurator --controller inav # disarmed, TCP 5761
python sim_flight.py aircraft/m62v1 experiment --controller inav
python sim_flight.py aircraft/example stability --controller mcu --clock stepped
python sim_flight.py aircraft/minirocket launch --controller mcu --start rail --launch-pitch 85 # unguided rocket flight
python sim_flight.py aircraft/example level --controller mcu --duration 120
```

Each aircraft is a project directory, opened as an argument or with File → Load; Save As copies the whole project under a new name. The bundled example is `aircraft/example`; the other names above are local projects:

```
aircraft/<project>/
    aircraft.json   the definition (airframe/2)
    inav/           INAV diffs: real FC dumps, and SITL diffs flown as written
      generated/    the diff a run flies, written from the design or a real FC dump (templates in data/inav/); copy into inav/ to edit
    sitl/           eeprom.bin, sitl.log; firmware is shared: inav-claude/inav/build_sitl/bin/SITL.elf (--sitl overrides it)
    jsbsim/         JSBSim export
    logs/           SITL and script flight logs and plots
    cad/            STEP / STL exports, blueprint drawings: cutaway, threeview, xray (.svg / .png)
      sections/     printable sections S01.. and couplers C01.., STEP or STL
      varsets/      FreeCAD varsets, one JSON per varset
    scenarios/      one run(flight) convention; overrides of data/scenarios/<controller>/*.py
      inav/         scripts importing controllers.inav
      mcu/          scripts importing controllers.mcu; raw sensor/PWM scripts use the same flight API
```

Every scenario defines `run(flight)`. Controller commands come from `from controllers import inav as controller` or `mcu`; simulation state, conditions, raw sensors, scheduling and recording are shared (`libs/scenario.py`, `libs/sensors.py`). `experiment` is an INAV example; `stability` is an MCU PWM example; `launch` lights the rocket motors and flies unguided. MCU control logic lives in the script, reading raw sensors and writing PWM; the backend has no autopilot or mixer. The SIM toolbar or CLI sets initial conditions; a scenario's optional `START` (e.g. `START = {"start": "rail", "launch-pitch": 85}`, keys as the CLI names them) supplies defaults the toolbar pre-selects and the CLI fills in where no option is given. Optional `PARAMETERS`, `TELEMETRY` and `LOG_FIELDS` declare parameters and recording.

The GUI's SIM view runs the same simulations with a live plot of the flight log; STEP/STL export is in its CAD view. Tube size presets (tube62, tube75: section, joints, rods) are loaded, saved and created in the Fuselage tab.
