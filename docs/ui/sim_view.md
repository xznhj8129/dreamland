# SIM View

This file covers flying the current project's airframe in simulation, with the selected controller and a scenario script, and watching its flight log.

## Layout

```text
SimView [fill, split:h]

    * LeftPane [split:v]
        SITLRunPanel
        RunOutputPanel

    * MiddlePane [ui:tabs]
        Plots     FlightLogPlot
        3D View   FlightScene
        Scenario  Python editor
        INAV diff Diff editor
        Performance Measured cards and speed-sweep samples

    * RightPane
        XAxisPanel
        YValuesPanel
```

### Split behavior

```text
SimView split behavior
    - startup split is roughly 2/7 left, 4/7 middle, 1/7 right
```

## SITLRunPanel

```text
SITLRunPanel [arrange:column]

    - Flight controller [ui:dropdown, readonly]
        values = available controller modules: inav, mcu
        default = inav
        selected controller chooses the scenarios/<controller>/ script directory; FC settings apply to INAV

    - Scenario [ui:dropdown, readonly]
        values = data/scenarios/<controller>/*.py with project scenarios/<controller>/*.py overrides:
            hover        VTOL: hover motor command vs prediction, position hold
            efficiency   powered: from the upper aero speed grid, a speed sweep down to the slowest level flight; Wh/km, best range and endurance
            top-speed    powered: full-throttle level speed
            controls     any: roll, pitch and yaw stick steps, each checked to move the commanded way
            glide        any: motors off from 1000 m, airspeeds fast to slow until the ground; smallest sink rate, best glide
            launch       MCU, rocket motors: ignite, leave the launcher, no control; rail exit, burnout, top speed and apogee,
                         then the ballistic fall to the ground (no recovery yet). START: rail, launcher pitch 85

    - Start [ui:dropdown, readonly]
        air      held level at 150 m and the operating airspeed unless toolbar or CLI values specify otherwise
        ground   take off and transition first (VTOL); hover always starts here
        catapult the bench's catapult: held and spun up, fired along its rail; forward-flight scenarios then use the air-start hold
        rail     the bench's unpowered launch rail: the motor's own thrust runs it up; forward-flight scenarios then use the air-start hold
        catapult and rail are available to fixed wings and tailsitters; ground requires VTOL; hover starts on the ground

    - Start conditions [ui:collapsed group, directly editable fields]
        Altitude (m), Speed (m/s), Heading (deg), Latitude, Longitude, Pitch (deg), Roll (deg), Wind from (deg), Wind (m/s)
        Launcher pitch (deg), Launcher length (m), Catapult speed (m/s), Launcher height (m)
        defaults: 150 m, operating airspeed, heading 0, the bench's home
        (Sneedville), level, still air; launcher defaults: catapult 10 deg / 2.0 m / 15 m/s / 1.5 m,
        rail 20 deg / 3.0 m / 1.0 m (the rail is unpowered, so Catapult speed is catapult-only)
        defaults come from the runners; edit values directly; an empty field disables Run
        only fields applicable to the selected start are shown
        launcher fields appear for catapult/rail; Catapult speed appears only for catapult
        the resolved values appear in the summary's start line
        a value that is not a number stops Run and says which; a non-positive launcher value stops it too

    - FC settings [ui:dropdown, readonly]
        values = "default", then for every .txt diff in the project's inav/:
            "real FC diff: <file>"   a real FC's dump (it has INAV's version banner)
            "SITL diff: <file>"      any other diff, flown as written
        default = default
        "default" and a real FC diff are written to inav/generated/ (rewritten on every such run), and that file flies;
        to edit one, copy it into inav/ and pick it as a SITL diff; the diff tab deletes a project diff (the generated
        file has no deletable name)

    - Time limit (s) [ui:textbox]
        maximum simulated seconds for any scenario; default 600
        a value that is not a number stops Run and says which

    - MenuRow [top of the run card]: Scenario ▾ Build ▾ (the secondary actions, in the top bar's menu idiom)

        Scenario ▾  acts on the open editor (the Scenario or INAV diff tab)
            New scenario  copy the selected controller's example into scenarios/<controller>/<name>.py
            Save <file>   save the open scenario or diff; needs unsaved edits
            Save copy     save it under the toolbar's file name; needs a saved file
            Discard edits drop the unsaved edits in the open editor

        Build ▾
            Workers      processes solving the aero tables when the model rebuilds (1 = sequential, one after
                         another); default this machine's logical CPUs minus two, at most eight; a value below 1 stops
                         Run and says which; the results do not depend on the count
            Model: ...   the project's last build record: "no record yet", "reused, nothing to solve",
                         "<stage> · m:ss · N workers", or a failure

    - Run [ui:button]: save the airframe file, then run the scenario; disabled while a run is going
    - Abort [ui:button], in the same row at the bottom of the run card: stop the run: the flight log and plots so far
      are written, INAV's SITL is stopped, the summary ends with "Aborted."; a run still going a few seconds later is
      killed; only while a run is going. While the run is solving a model rebuild (its first phase, shown as
      "Building model: <stage> <done>/<total>") the button reads "Abort build": stopping it drops the assembled
      export, keeps the solved tables in the cache for the next run, and marks the build record failed

    - StatusText [ui:text]
        Idle | No valid airframe | Building model: <stage> <done>/<total> (while the run is solving a rebuild, before
        the flight starts) | Running <controller>: <scenario>... | Aborting... | Aborted | Finished (exit code N)
        Abort stops the whole run process, which during that first phase is the model build: the assembled export is
        dropped, the solved tables stay in the table cache for the next run, and the build record is marked failed
```

### Run behavior

```text
Run
    - saves the airframe (File -> Save) first; what flies is the file on disk
    - clears the summary and the plot
    - all controllers: Realtime, Stepped, or Stepped faster; tick rate defaults to 200 Hz; INAV uses the shared build
    - MSP and MAVLink checkboxes enable external configurator/telemetry links; the internal bench MSP link stays active
    - shared scenarios are readonly templates; Save copy creates an editable project override
    - Python scenarios define run(flight); they own arming, modes, launch, control and stop conditions
    - the toolbar owns initial conditions; fields are directly editable and displayed values are passed to the run
    - a scenario may declare START = {"start": ..., <start option>: number, ...} (keys as the CLI names them): opening
      the scenario pre-selects those in the toolbar (once per scenario and START; edits after that stand); its outline
      lists them; a START start the aircraft cannot use shows in the status line; a bad START (unknown key, a value
      that is not a finite number, an option that does not apply to its start) refuses the scenario
    - changing scenarios otherwise preserves the toolbar settings; run output shows the actual start conditions
    - New scenario copies the selected controller's example (INAV experiment, MCU stability) into scenarios/<controller>/<name>.py
    - Set as default writes the chosen controller and scenario to the project's sim.json: another browser, or a
      project this browser has nothing remembered for, opens with that pair; the button's tooltip shows the current one
    - one run(flight) sequence; import the selected controllers module for commands
    - optional PARAMETERS and TELEMETRY (fields and rate_hz, maximum 50) apply to every scenario
    - flight.phase(name, description) gives arbitrary labels in telemetry and the sidebar
    - flight.sensors, truth, state, raw, initial, elapsed, wait, measure and stop have the same API for every controller; print() is freeform status, measure() records a point in the Measurements file
    - LOG_FIELDS declares extra numeric CSV columns written with flight.log(name, value)
    - selected telemetry runs alongside every phase on simulator ticks; CSV includes simulated time and phase
    - telemetry fields: truth (agl_m, roll/pitch/yaw, vn/ve/vd), airspeed_m_s, aoa_deg, sideslip_deg, p/q/r_deg_s, motor1..., servo0..., battery_w/a/v with electric motors, the rocket columns with rocket motors; raw sensor columns are shared; the full INAV CSV also records FC telemetry
    - Scenario sidebar shows the template/saved scenario outline and parameters; Save refreshes it
    - failed conditions, ground impact and Abort retain partial logs and skip the completion manoeuvre
    - experiment template climbs, settles, then hands over to INAV (NAV POSHOLD, then NAV RTH and landing); customize ground/VTOL launch and transitions explicitly
    - Scenario and INAV diff tabs: edit, Save, Save copy, Discard edits; a project diff can be deleted (the default view and the generated file are read only); unsaved edits disable Run
    - generated design diff is shown as the last generated file, readonly; Save copy writes an editable project diff
    - Configurator only holds physics disarmed, exposes MSP on TCP 5761, and runs until Abort
    - MCU transport: configured tick rate, 50 Hz wait callbacks; cached exports, cold control-table exports may take minutes
    - efficiency default sweep stays within the aero speed grid; out-of-grid top-speed points are marked
    - run output reports the geometry-dependent lifting-line AoA limit and AeroBuildup blend; efficiency marks measurements outside that limit
    - MCU: esc[index] and servo[index] PWM in us, or outputs(motors_us, servos_us); arm enables outputs, release handles the simulated launcher;
      ignite(*mount_numbers) fires the igniters of the loaded rocket motors (none given: all), each burns to the end
    - rocket motors: the flight log adds rocketN_thrust_n, rocket_thrust_n, mass_kg (propellant burning off) and accel_x_g;
      run output gives the airspeed the aero tables are solved up to (rocket airframes: 300 m/s), held beyond
    - MCU scripts read shared raw sensors and contain their own control logic; the backend has no autopilot or mixer
    - controller commands own launch and transitions; controller directory determines which scripts are offered
    - Performance separates the editor static estimate from measured JSBSim motion; range/endurance extrapolate measured power
    - stall, sustained turn and ceiling cards remain Not measured until a matching experiment supplies them
    - every run follows the selected clock; return, time limit, stop condition, ground impact or Abort ends it
    - a Model build section (under the run card's menus) shows the model rebuild as it happens: stage, the task detail,
      done/total tasks, workers, elapsed, and the tasks reused from the table cache; it stays as the record of the
      last build, says "model reused" when nothing needed solving, and marks a failed build
    - the scenario, start and FC settings lists refresh whenever the view is shown and after Load
    - the panel's setup (controller, scenario, start, conditions, clock, rate, duration, plot picks and the open tab)
      is remembered per project in the browser: switching to another view and back, or reloading, returns to it; a
      project with nothing remembered opens with its sim.json default (Set as default)
    - a run started on the ground, catapult or rail logs to its own file, next to the air-start one
    - logs/<controller>_<scenario>_<start>.csv; measurements and selected telemetry use the same stem
    - the run belongs to the server: every open tab shows it, and a page reload or a closed browser does not stop it
    - stopping the server aborts a live run, as Abort does
```

## RunOutputPanel

```text
RunOutputPanel [scrollable, copyable, readonly, word-wrapped]

    - before a run: No simulation run yet.
    - first line: the equivalent command line
    - then the simulation's output as it runs: per-second reports, tables, the final result, or the error traceback
    - CSV, Measurements and Selected telemetry download links appear when available; measurement points are saved beside the CSV as .measurements.json, selected telemetry as .telemetry.csv
```

## FlightLogPlot

```text
FlightLogPlot [fill]

    - one subplot per picked Y value, fitted to the pane, all sharing the X axis
    - click a subplot name to maximize it; click again to restore all picked plots
    - updates about twice a second while a run is going
    - redraws at once when the X axis or a Y value changes
    - the last run stays shown until the next Run
    - mouse wheel over a subplot: zoom the X range in (up) or out (down) around the cursor
    - drag across a subplot: the stretch is shaded; on release, zoom to it (a very short drag is a click)
    - double-click: the whole run again; a new X axis or a new Run does the same
    - while zoomed, each Y axis fits the data shown, and live updates keep the zoom
```

## FlightScene

```text
FlightScene [fill]

    - before a run: Run a simulation to see its flight path.
    - the flight path in 3D: east, north (m from home) and height above ground (m), from the log's
      east_m, north_m, agl_m; the path line breaks at gaps, and the aircraft itself marks the current position
      (no start/end markers)
    - the box starts 50 m either side of the start point east and north (one scale for both) and 0-50 m high, also
      before any data, and grows to fit the path; height runs from 0 (the ground) up (a path below the ground is clipped)
    - the resolved airframe flies the path: external geometry only (fuselage, wing skins, control surfaces,
      propulsion), placed at each frame's position and attitude; each control surface follows its logged servo<N>
      output through the surface's deflection limits; each rocket motor shows a red flame cone from its nozzle,
      against the thrust, 0-1 m long in proportion to its logged rocket<N>_thrust_n over the motor's peak thrust
      (hidden at 0 N)
    - a row missing any position or attitude field (t, east_m, north_m, agl_m, roll, pitch, yaw) is a gap: the
      path line breaks there and the aircraft holds the last valid frame
    - a matte ground plane, a grid over the box footprint and a sky gradient surround the path; the ground edge
      fades into the horizon colour with distance as the box grows
    - a resolved airframe is required for the model: without one the path still draws, with a notice
    - camera: Orbit [default] targets the aircraft and carries the user's orbit and zoom along as it moves; Chase
      looks from behind and above along the heading only (roll and pitch do not swing it); Path orbits the whole
      box, fitting the growing bounds until the user interacts; Fit reframes the current mode
    - playback: Live [default] shows the newest frame; Play/Pause replays the log (from the start at the end),
      the slider scrubs and switches to replay, the speed select runs 0.5x, 1x, 2x or 4x, and the readout shows
      the cursor time over the log's total; a new run starts live again
    - drawn when shown, then updated live like the plots
```

## XAxisPanel

```text
XAxisPanel

    - X axis [ui:dropdown, readonly]
        values = the flight log's columns
        default = t
```

## YValuesPanel

```text
YValuesPanel [scrollable]

    - one checkbox per flight log column
    - default checked = agl_m, ias_kts, pitch
    - picks are kept across runs
```

### Flight log columns

```text
Columns (the bench's raw units)
    t                       s since the bench started
    armed                   0 / 1
    fc_roll fc_pitch fc_yaw INAV's attitude estimate, deg
    lat lon                 deg
    north_m east_m          m from home
    agl_m                   m above ground
    roll pitch yaw          true attitude, deg
    vn ve vd                m/s
    ias_kts                 indicated airspeed, knots
    rc1 ... rc8             RC channels, us (INAV runs)
    motor1 ...              motor outputs, us
    servo0 ...              servo outputs, us

What INAV reports (read over MSP every log row)
    fc_stab_roll fc_stab_pitch fc_stab_yaw fc_stab_throttle   stabilised commands, -500..500
    fc_battery_v fc_battery_a fc_battery_w                    battery as INAV measures it (fed from the model below)
    fc_battery_mah fc_battery_wh fc_battery_pct fc_battery_state
    fc_alt_m fc_climb_m_s fc_baro_alt_m                       altitude estimate, climb rate, baro altitude
    fc_airspeed_m_s                                           INAV's airspeed (0 without a pitot)
    fc_gps_fix fc_gps_sats fc_gps_alt_m fc_gps_speed_m_s fc_gps_course_deg
    fc_nav_mode fc_nav_state fc_nav_wp fc_nav_error
    fc_throttle_pct fc_auto_throttle fc_flight_time_s
    fc_cycle_us fc_cpu_load_pct fc_arming_flags fc_mixer_profile
    mode_<name>             1 while that flight mode is active, one column per mode INAV lists

Electrical model (powered aircraft)
    battery_w battery_a     battery power and current
    battery_v               pack voltage, nominal (no sag modelled)
    motor<n>_v motor<n>_a motor<n>_rpm   each motor's voltage, current and prop rpm

Script runs (no fc_ or rc columns)
    imu_ax imu_ay imu_az imu_gx imu_gy imu_gz   the script's IMU, raw counts
    baro_pa mag_x mag_y mag_z pitot_pa range_mm  its baro (Pa), mag (counts), pitot (Pa), rangefinder (mm, -1 out of range)
    gps_lat gps_lon gps_height_mm gps_speed_mm_s gps_heading   its last GPS fix (1e-7 deg, mm, mm/s, 1e-5 deg)
    <name>                  each value the script log()s
```
