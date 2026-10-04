# Configuration View

This file covers the main aircraft-building view shown by default.

## Layout

```text
ConfigurationView [fill, split:h]

    * LeftPane [fill]
    * CenterPane [fill]
    * RightPane [fill]
```

### Pane behavior

```text
ConfigurationView split behavior
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
        Controls
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

    * SummaryPanel [fill, scrollable, copyable]
        * SummaryText [ui:fixed-width text, bind:analysis.report_text]
            includes dimensional surface-force estimates at the current Configuration operating condition
            reports per-surface lift / drag / vertical force using the configured airfoil and surface geometry
```

### Controls tab

```text
ControlsTab [arrange:column]

    * ControlsPanel
        * SurfaceForceOperatingPoint [ui:group]
            - AoA (deg) [ui:number entry]
            - Airspeed (m/s) [ui:number entry]
            - Altitude (m) [ui:number entry]
            used by the Summary -> Surface Forces block
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
        - draw wires battery -> ESC -> each motor as yellow dotted lines along the axes
        - draw top-left colored point guide: blue CG, red Neutral point, green Lift @ AoA, volumes, parts, wires, cuts
        - draw pods, motors, shafts, props, CG, CP, CL

    mode Front
        - draw body as circular cross-section
        - draw lifting surfaces as radial lines
        - draw volumes as dashed outlines, parts as solid outlines (a cross without library size)
        - draw wires as yellow dotted lines
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
        Airframe
        Fuselage
        Surfaces
        Propulsion
        Volumes
        Parts
```

## Fuselage tab

```text
FuselageTab [scrollable, arrange:column]

    * SizePresetPanel
    * FuselagePanel
    * ReinforcementPanel
    * SectionsTable
```

### SizePresetPanel

```text
SizePresetPanel  (title "Size preset (data/specs.json)")

    - aircraft [ui:text]
        none | <preset> | <preset> (modified)
        modified = section, joints or rod pattern differ from the preset; updates after every edit
    - preset [ui:dropdown, readonly]
        values = (none), then the presets in data/specs.json
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
    a preset that fails validation is not written; data/specs.json stays as it was and the reason is shown
```

### FuselagePanel

```text
FuselagePanel
    - length
    * section: width, height, shape (superellipse exponent, 2 = circle), wall radial
    * nose, tail: profile [ui:dropdown], length, the profile's parameters
    * joints: type [ui:dropdown], count, screw, insertion depth, coupler mass
```

### ReinforcementPanel

```text
ReinforcementPanel  (title "Reinforcement (longitudinal rods)")

    - rods [ui:checkbox]
        off -> rods removed (kept to restore while the window is open)
        on  -> the removed rods, or the labelled preset's pattern, or 4 single 2.5 mm carbon rods at 45 deg against the inner wall;
               new rods run nose end to tail start
    - type [ui:dropdown]
    - material [ui:dropdown]  (data/parts.json materials)
    - diameter, radial position (rod centre from the axis)
    - polar_n (all rods), polar_offset_angle (deg from the horizontal), polar_split (0 = single, 2 = pair),
      polar_split_angle (deg: a pair sits at offset +/- this)
    - from x, to x
    one rod position per quadrant, mirrored left/right and top/bottom
    drawn as dashed lines in the side view and small circles in the front view
```

### SectionsTable

```text
SectionsTable  (title "Sections (cut lines; drag them in the side view)")

    - Add cut [ui:button]
        on:click -> split the longest section at its middle; both halves keep its solid flag and lose its weighed masses

    columns, one row per section
        section mm   S<n> from-to
        cut mm       the section's end cut [ui:textbox]; none on the last section
        solid        [ui:checkbox]  off = hollow shell wall; on = solid less the volumes inside it
        empty g      weighed printed mass [ui:textbox]; blank = calc
        calc         computed printed mass (hollow: shell wall x material density; solid: x print bulk density)
        complete g   weighed printed mass + everything attached in the section [ui:textbox]; blank = items
        items        itemised mass of everything attached in the section (props with their motor, motors with their mount,
                     wing sets at their root leading edge); a range when it holds a mass range
        Del          [ui:button] delete the section's end cut: the two sections merge, keeping the left one's solid flag,
                     each weighed mass summed when both had one
    calc and items update after every edit
    the side view labels each section "S<n> <empty> g" (* = computed), its length, "solid", and "<complete> g all" when weighed
```

## Propulsion tab

```text
PropulsionTab [arrange:column]

    * SelectionPanel
    * MotorInfoPanel
```

### SelectionPanel

```text
SelectionPanel [editable]

    - Battery [ui:dropdown, readonly]
    - Manufacturer [ui:dropdown, readonly]
    - Motor [ui:dropdown, readonly]
    - Prop [ui:dropdown, readonly]
    - ESC [ui:dropdown, readonly]

Options
    - battery, ESC: placed parts of that kind (Parts tab)
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
    - renaming a volume renames it in its parts
    - Delete is refused while parts sit in the volume (status bar says which)
    - dragging a volume in the side view moves its parts
```

## Parts tab

```text
PartsTab [arrange:column]

    - Add part [ui:button]        new part: first library entry with a mass, in the first volume
    * one card per part: name, part (library), volume, x, y, z offset (mm, blank = 0),
      mass (g; blank = library, required when the library has none; 0 = counted elsewhere), Delete

Rules
    - renaming a part renames it in the propulsion battery / ESC
    - Delete is refused for the propulsion battery or ESC (status bar says so)
    - a part outside its volume, or overlapping another part, is listed under layout problems
```

### MotorInfoPanel

```text
MotorInfoPanel [fill]

    - top half = matplotlib plot host
    - bottom half = fixed-width text report

Plot contents
    - thrust vs INAV motor output
    - RPM / 1000 vs INAV motor output
    - kW vs INAV motor output
    - hover line (VTOL)

Text report
    - motors, prop, battery
    - resistance per motor, by part
    - full throttle, thrust/weight, hover
    - parts not modelled, motorlib warnings

Refreshes after every edit
```

## Field input behavior

```text
Length fields
    - displayed in mm
    - stored internally in meters
    - mouse wheel changes by 1 displayed mm

Mass fields
    - displayed in g
    - stored internally in kg

Numeric entries
    - Up arrow increases by 1 displayed unit
    - Down arrow decreases by 1 displayed unit
    - Return, keypad Enter, and FocusOut apply the value

Type fields
    - readonly dropdowns

motor_dir
    - enum = { front, back }

pod_shape
    - enum = { elliptical, ogive }
```
