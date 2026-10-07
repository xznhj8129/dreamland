# Editor View

This file covers the main aircraft-building view shown by default.

## Layout

```text
EditorView [fill, split:h]

    * LeftPane [fill]
    * CenterPane [fill]
    * RightPane [fill]
```

### Pane behavior

```text
EditorView split behavior
    - left / center / right panes are horizontally split
    - startup split is 25% / 50% / 25%
    - pane dividers remain draggable after startup
```

## Left Pane

```text
LeftPane [arrange:column]

    * RecomputeHeader [arrange:row]
    * LeftTabs [ui:tabs]
        Summary
        Log
```

### RecomputeHeader

```text
RecomputeHeader [arrange:row]

    - RecomputeButton [ui:button]
        label = "Recompute (F5)"
        on:click -> run heavy recompute

    - RecomputeStatus [ui:text]
        values include:
            "Up to date"
            "ASB stale (F5)"
```

### Summary tab

```text
SummaryTab [fill]

    * SummaryPanel [fill, scrollable, copyable, word-wrapped]
        * SummaryText [ui:fixed-width text, bind:analysis.report_text]
            includes dimensional surface-force estimates at the current operating condition
            reports per-surface lift / drag / vertical force using the configured airfoil and surface geometry
            problems in red (layout problems, unstable margin, no hover), warnings in yellow (marginal margin,
            parts not modelled, motorlib warnings)
```

### Log tab

```text
LogTab [fill]

    * LogPanel [fill, scrollable, copyable, word-wrapped]
        * LogText [ui:fixed-width text]
            every error in red, appended with its time: definition errors, motor overloaded (no motor info, no propulsion
            in the report), aero failures, load / save / preset errors, unhandled errors with their traceback
            a new entry brings the Log tab to the front; the status bar says "<what> (see Log)"; the Summary keeps the
            last valid report
```

## Center Pane

```text
CenterPane [arrange:column]

    * CenterTabs [ui:tabs]
        Simple
        ASB
```

### Simple tab

```text
SimpleTab [arrange:column]

    * GraphicsPanel
    * OrthographicView [fill]
```

### GraphicsPanel

```text
GraphicsPanel [arrange:column]

    - MarkerLegend [ui:text]
        "X = Front   Y = Right   Z = Up"

    * ViewSelector [ui:radio, arrange:row, bind:view_mode]
        enum:view_mode -> { Longitudinal, Front }
        on:select -> redraw orthographic view

    - Reset View [ui:button]
        on:click -> reset simple-view pan and zoom

    * RollRail [arrange:column]
        shown only in Longitudinal mode
        docked at right of orthographic view

        - RollView [ui:vertical slider, bind:roll_view_deg]
            range = 0 deg to 90 deg
            resolution = 1 deg
            0 deg = side
            90 deg = top
            on:slide -> redraw longitudinal view
```

### OrthographicView

```text
OrthographicView [fill, switchable, pannable, zoomable]

    - mouse drag -> pan view
    - mouse wheel -> zoom view at cursor
    - horizontal scrollbar -> pan x
    - vertical scrollbar -> pan y

    mode Longitudinal
        - draw fuselage in side view
        - draw ends as curved caps meeting the tube
        - draw lifting surfaces as projected trapezoids through roll angle
        - draw lifting-surface chord angle using aoa_ang, positive = front up
        - draw volumes as dashed outlines, named above
        - draw parts as solid outlines (a cross without library size), named below
        - draw wires battery -> ESC -> each motor as yellow dotted lines (battery -> ESC along the axes; to a wing
          motor inside its panel, out along the mid-chord)
        - draw spars as pink lines (the rod colour), inner end to tip: solid = through spar, dashed = stops where it meets another
        - draw the carry-through as dashed pink outlines (each arm's side profile)
        - mouse drag on a spar -> move that set's spars along the root chord (x offset)
        - draw top-left colored point guide: blue CG, red Neutral point, green Lift @ AoA, volumes, parts, wires, cuts
        - draw pods, motors, shafts, props, CG, CP, CL

    mode Front
        - draw body as circular cross-section
        - draw lifting surfaces as radial lines
        - draw volumes as dashed outlines, parts as solid outlines (a cross without library size)
        - draw wires as yellow dotted lines
        - draw spars as pink lines (solid = through, dashed = stops) and the carry-through as dashed pink outlines
        - draw top-left colored point guide: blue CG, red Neutral point, green Lift @ AoA, volumes, parts, wires, cuts
        - draw motor / pod / prop cross-sections
```

### ASB tab

```text
ASBTab [fill]

    * ASBView [fill]
        - Update View [ui:button]
            on:click -> run heavy recompute and redraw the AeroSandbox model
        - embedded matplotlib 3D canvas
```

### ASB behavior

```text
ASB view
    - uses the current in-memory AeroSandbox airplane
    - light recompute does not redraw ASB
    - heavy recompute rebuilds and redraws ASB
    - starts in left isometric orientation
    - user may reorient the same 3D model interactively
```

## Right Pane

```text
RightPane [arrange:column]

    * RightTabs [ui:tabs]
        Design [ui:tabs]   the airframe itself
            Airframe
            Fuselage
            Sections
            Surfaces
            Volumes
            Motor mounts
        Build [ui:tabs]    what goes in it
            Propulsion
            Parts
            Wiring
```

## Fuselage tab

```text
FuselageTab [scrollable, arrange:column]

    * SizePresetPanel
    * FuselagePanel
    * ReinforcementPanel
```

### SizePresetPanel

```text
SizePresetPanel  (title "Size preset (data/presets/)")

    - aircraft [ui:text]
        none | <preset> | <preset> (modified)
        modified = section, joints or rod pattern differ from the preset; updates after every edit
    - preset [ui:dropdown, readonly]
        values = (none), then the presets in data/presets/
    - Load [ui:button]
        on:click -> fill section, joints (not coupler mass) and rod pattern (not from/to x) from the preset,
                    label the aircraft with it; rods it had keep their from/to x, new rods run nose end to tail start;
                    a preset without rods removes them; (none) clears the label only
    - Save [ui:button]
        on:click -> confirm, then overwrite the picked preset with this aircraft's section, joints and rod pattern;
                    the preset keeps its limits and CAD data; (none) asks to pick a preset or use New
    - New [ui:button]
        on:click -> ask a name, save this aircraft's values as a new preset (no limits, no CAD data), label the aircraft with it;
                    an existing name is refused
    a preset that fails validation is not written; data/presets/ stays as it was and the reason is shown
```

### FuselagePanel

```text
FuselagePanel
    - length
    * section: width, height, shape (superellipse exponent, 2 = circle), wall radial
    * nose, tail: profile [ui:dropdown], length, the profile's parameters
    * joints: type [ui:dropdown], count (screws per section end), screw, insertion depth, coupler wall, fit clearance,
      socket wall, hole edge dist, hole diameter, hole angle offset, coupler mass
```

### ReinforcementPanel

```text
ReinforcementPanel  (title "Reinforcement (longitudinal rods)")

    - rods [ui:checkbox]
        off -> rods removed (kept to restore while the window is open)
        on  -> the removed rods, or the labelled preset's pattern, or 4 single 2.5 mm carbon rods at 45 deg against the inner wall;
               new rods run nose end to tail start
    - type [ui:dropdown]
    - material [ui:dropdown]  (data/parts/material.json)
    - diameter, radial position (rod centre from the axis)
    - polar_n (all rods), polar_offset_angle (deg from the horizontal), polar_split (0 = single, 2 = pair),
      polar_split_angle (deg: a pair sits at offset +/- this)
    - from x, to x
    one rod position per quadrant, mirrored left/right and top/bottom
    drawn as dashed lines in the side view and small circles in the front view
```

## Sections tab

```text
SectionsTab [scrollable]  (group title "Sections (cut lines; drag them in the side view)")

    - Add cut [ui:button]
        on:click -> split the longest section at its middle; both halves keep its solid flag and lose its weighed masses;
                    the new cut's joint is coupler

    columns, one row per section
        section mm   S<n> from-to
        cut mm       the section's end cut [ui:textbox]; none on the last section
        joint        that cut's joint [ui:dropdown]: coupler (separate coupler, its mass at the cut), front_male (this
                     section carries the male end), rear_male (the next section does); none on the last section
        solid        [ui:checkbox]  off = hollow shell wall; on = solid less the volumes inside it
        empty g      weighed printed mass [ui:textbox]; blank = calc
        calc         computed printed mass (hollow: shell wall x material density; solid: x print bulk density)
        complete g   weighed printed mass + everything attached in the section [ui:textbox]; blank = items
        items        itemised mass of everything attached in the section (props with their motor, motors with their mount,
                     wing sets at their root leading edge); a range when it holds a mass range
        Del          [ui:button] delete the section's end cut and its joint: the two sections merge, keeping the left one's solid flag,
                     each weighed mass summed when both had one
    calc and items update after every edit
    the side view labels each section "S<n> <empty> g" (* = computed), its length, "solid", and "<complete> g all" when weighed
```

## Surfaces tab

```text
SurfacesTab [arrange:column]

    - Add surface set [ui:button]
    * one card per surface set: its planform fields, mass (blank = computed), Delete
      control surface [ui:checkbox] with its fields
      spars [ui:checkbox] (ticking fills them from the aircraft's preset): material [ui:dropdown], diameter, count, spacing, x offset, y offset (up),
        carry-through [ui:checkbox]: stem width, length, flare radius, root length, wall fillet
      root base (not built yet) [ui:checkbox]: length, width, height, forward offset, leading cut, trailing cut
```

## Motor mounts tab

```text
MotorMountsTab [arrange:column]

    - Add motor mount [ui:button]
    * one card per mount: on (nose | tail | surface), surface, span position, x offset, direction, spin, thrust axis
      tilt, pod [ui:checkbox] with its fields, Delete
    a surface mount puts one motor on every panel of the surface
```

## Propulsion tab

```text
PropulsionTab [arrange:column]

    * BatteryPanel
    * SelectionPanel
    * MotorInfoPanel   (only while the motor can turn the prop; otherwise the Log says why)
```

### BatteryPanel

```text
BatteryPanel (title "Battery")

    - Battery model [ui:dropdown, readonly]   Commercial | Custom Li-Ion
        switching keeps where the pack sits; the other model's fields come back as they were left
        (first time: the first library pack, or a 4S1P of Molicel INR21700-P45B along x)
    Commercial
        - pack [ui:dropdown]   data/parts/battery.json packs
        - mass (g) [ui:textbox]   blank = library
    Custom Li-Ion
        - S (cells in series), P (in parallel) [ui:textbox]
        - cell (motorlib) [ui:dropdown]   motorlib cells with a cylindrical format (18650, 21700, ...) in the name
        - connector [ui:dropdown]   data/parts/connector.json with a contact resistance
        - cell length along [ui:dropdown]   x | y | z
        - cells along x, y, z [ui:textbox]   their product must be S x P (otherwise a definition error in the Log)
        - lead AWG [ui:dropdown], lead length (mm) [ui:textbox]
    - volume [ui:dropdown, "(none)" first] and offsets, or its own x, y, z (as on a part)
    - the resolved pack: label, size (each axis count x (cell + 2 mm)), mass, capacity
```

### SelectionPanel

```text
SelectionPanel [editable]

    - Manufacturer [ui:dropdown, readonly]
    - Motor [ui:dropdown, readonly]
    - Prop [ui:dropdown, readonly]
    - ESC [ui:dropdown, readonly]

Options
    - ESC: placed esc parts (Parts tab); blank until there is one to pick (a definition error in the Log)
    - manufacturers and motors from motorlib, only motors with a stator size
    - changing manufacturer selects its first motor
    - props: all motorlib props
```

## Volumes tab

```text
VolumesTab [arrange:column]

    - Add volume [ui:button]
    * one card per volume: name, shape, x, y, z (mm), shape sizes, mass (g, blank = space only),
      list of the parts in it, Delete

Rules
    - shapes: box, rounded box (size, axis of its rounded edges, corner radius), cylinder, bore, point
    - deleting a volume leaves its parts (and the battery) where they are, at their own x, y, z
    - renaming a volume renames it in its parts
    - deleting a volume that still holds parts leaves them pointing at it: a definition error in the Log until moved
    - dragging a volume in the side view moves its parts
```

## Parts tab

```text
PartsTab [arrange:column]

    - Add part [ui:button]        new part: first library entry with a mass, in the first volume (no volume: at mid-fuselage)
    * one card per part: name, part (library), volume [ui:dropdown, "(none)" first],
      in a volume: x, y, z offset from its centre (mm, blank = 0); "(none)": its own x, y, z (mm; y, z blank = 0),
      yaw, pitch, roll (deg, blank = 0), mass (g; blank = library, required when the library has none; 0 = counted elsewhere), Delete

Rules
    - renaming a part renames it in the propulsion ESC; the propulsion battery is not a part (Propulsion tab)
    - deleting the propulsion battery or ESC: a definition error in the Log until another is picked
    - a part outside its volume, or overlapping another part, is listed under layout problems
    - volume -> "(none)": the part stays where it was, now at its own x, y, z; "(none)" -> a volume: at its centre
```

## Wiring tab

```text
WiringTab [arrange:column]

    * WireRunsTable  (title "Wire runs (auto-routed: along the axes, wing motors inside their panel; drawn in the side view)")
        one row per run: battery -> ESC, then ESC -> each motor (motor order)
        columns
            run          its ends
            length mm    the auto-routed length ("-" while the definition has an error)
            AWG          [ui:dropdown] motorlib's wire gauges
            wires        [ui:textbox] wires in the cable
            plugs        one [ui:dropdown] per connector along the run, in order; "(remove)" takes it off;
                         a last "+" [ui:dropdown] appends one; connectors: data/parts/connector.json entries with a
                         contact resistance (Part Editor)
    an added motor gets a copy of the last motor run; a removed motor's run goes with it
    gauges and plugs set each motor's resistance in the motor model (Motor info, report, JSBSim)
```

### MotorInfoPanel

```text
MotorInfoPanel [fill]

    - top half = matplotlib plot host
    - bottom half = fixed-width text report

Plot contents (mean per motor)
    - thrust vs INAV motor output
    - RPM / 1000 vs INAV motor output
    - kW vs INAV motor output
    - hover line (VTOL)

Text report
    - motors, prop, battery
    - resistance per motor, by part (a range when the motor runs differ)
    - full throttle, thrust/weight, hover
    - parts not modelled, motorlib warnings

Refreshes after every edit
```

## Field input behavior

```text
Length fields
    - displayed in mm
    - stored internally in meters

Any number field
    - mouse wheel over it: up multiplies by 1.01, down divides by 1.01 (a whole number: +/- 1), applied at once

Mass fields
    - displayed in g
    - stored internally in kg

Numeric entries
    - Up arrow increases by 1 displayed unit
    - Down arrow decreases by 1 displayed unit
    - Return, keypad Enter, and FocusOut apply the value

Type fields
    - readonly dropdowns

incidence_mode (Surfaces tab, per surface set)
    - enum = { symmetric, cant }
    - symmetric: incidence pitches mirror panels alike (lift, like AoA)
    - cant: every panel turned the same way about the body axis (roll); + = right side up, anticlockwise seen from behind

motor_dir
    - enum = { front, back }

pod_shape
    - enum = { elliptical, ogive }
```
