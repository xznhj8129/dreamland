# SIM View

This file covers flying the current project's airframe in simulation, with INAV SITL or one of the project's flight controller scripts, and watching its flight log.

## Layout

```text
SimView [fill, split:h]

    * LeftPane [split:v]
        SITLRunPanel
        SummaryPanel

    * MiddlePane [ui:tabs]
        Plots     FlightLogPlot
        3D View   FlightPath3D

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
        values = "INAV SITL", then "script: <name>" for every script in the project's controllers/
        default = INAV SITL
        a script greys out Scenario and FC settings and enables Duration

    - Scenario [ui:dropdown, readonly]
        values = the scenarios this aircraft can fly, out of:
            hover        VTOL: hover motor command vs prediction, position hold
            efficiency   powered: from its top speed, a speed sweep down to the slowest level flight; Wh/km, best range and endurance
            top-speed    powered: full-throttle level speed
            controls     any: roll, pitch and yaw stick steps, each checked to move the commanded way
            glide        any: motors off from 1000 m, airspeeds fast to slow until the ground; smallest sink rate, best glide

    - Start [ui:dropdown, readonly]
        air      held level at 150 m: a fixed-wing at its operating airspeed, a tailsitter from rest (forward-flight scenarios)
        ground   take off and transition first (VTOL); hover always starts here
        catapult the bench's catapult: held and spun up, fired along its rail, then the ordinary air-start hold
        rail     the bench's unpowered launch rail: the motor's own thrust runs it up, then the ordinary air-start hold
        only the starts this aircraft and scenario can use are offered (catapult/rail: fixed wings; a tailsitter takes off standing)

    - Start conditions [ui:textbox x13, blank = default]
        Altitude (m), Speed (m/s), Heading (deg), Latitude, Longitude, Pitch (deg), Roll (deg), Wind from (deg), Wind (m/s)
        Launcher pitch (deg), Launcher length (m), Catapult speed (m/s), Launcher height (m)
        defaults: 150 m (glide 1000 m), the operating airspeed (tailsitter: from rest), heading 0, the bench's home
        (Sneedville), level, still air; launcher defaults: catapult 10 deg / 2.0 m / 15 m/s / 1.5 m,
        rail 20 deg / 3.0 m / 1.0 m (the rail is unpowered, so Catapult speed is catapult-only)
        Altitude, Speed, Pitch and Roll are greyed out on a ground start (standing on the tail, at rest)
        the launcher fields are greyed out unless a launcher start is chosen, and the selected launcher's
        defaults show as placeholders; the resolved values appear in the summary's start line
        a value that is not a number stops Run and says which; a non-positive launcher value stops it too

    - FC settings [ui:dropdown, readonly]
        values = "generated from the design", then for every .txt diff in the project's inav/:
            "real FC diff: <file>"   a real FC's dump (it has INAV's version banner)
            "SITL diff: <file>"      any other diff, flown as written
        default = generated from the design
        "generated" and a real FC diff are written to inav/generated/ (rewritten on every such run), and that file flies;
        to edit one, copy it into inav/ and pick it as a SITL diff

    - Duration (s) [ui:textbox, script only]
        simulated seconds the script flies; default 60
        a value that is not a number stops Run and says which

    - Run [ui:button]
        on:click -> save the airframe file, then run the scenario
        disabled while a run is going

    - Abort [ui:button], next to Run
        on:click -> stop the run: the flight log and plots so far are written, INAV's SITL is stopped, the summary ends
                    with "Aborted."; a run still going a few seconds later is killed
        enabled only while a run is going

    - StatusText [ui:text]
        Idle | No valid airframe | Running <scenario>... | Running script: <name>... | Aborting... | Aborted |
        Finished (exit code N)
```

### Run behavior

```text
Run
    - saves the airframe (File -> Save) first; what flies is the file on disk
    - clears the summary and the plot
    - an INAV run takes real time: the simulation runs at wall-clock speed
    - a script run is in simulated time, as fast as the computer allows; it ends after Duration, when the script stops it, or on the ground
    - the scenario, start and FC settings lists refresh whenever the view is shown and after Load
    - a run started on the ground, catapult or rail logs to its own file, next to the air-start one
    - a script run logs to logs/script_<name>.csv
    - the run belongs to the server: every open tab shows it, and a page reload or a closed browser does not stop it
    - stopping the server aborts a live run, as Abort does
```

## SummaryPanel

```text
SummaryPanel [scrollable, copyable, readonly, word-wrapped]

    - first line: the equivalent command line
    - then the simulation's output as it runs: per-second reports, tables, the final result, or the error traceback
```

## FlightLogPlot

```text
FlightLogPlot [fill]

    - one stacked subplot per picked Y value, all sharing the X axis
    - updates about twice a second while a run is going
    - redraws at once when the X axis or a Y value changes
    - the last run stays shown until the next Run
    - mouse wheel over a subplot: zoom the X range in (up) or out (down) around the cursor
    - drag across a subplot: the stretch is shaded; on release, zoom to it (a very short drag is a click)
    - double-click: the whole run again; a new X axis or a new Run does the same
    - while zoomed, each Y axis fits the data shown, and live updates keep the zoom
```

## FlightPath3D

```text
FlightPath3D [fill]

    - the flight path in 3D: east, north (m from home) and height above ground (m), from the log's
      east_m, north_m, agl_m; start marked green, end red
    - the box starts 50 m either side of the start point east and north (one scale for both) and 0-50 m high, also
      before any data, and grows to fit the path; height runs from 0 (the ground) up (a path below the ground is clipped)
    - drag to rotate; the angle is kept across live updates and runs
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
