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
    - compact spreadsheet-style label/value cells stay side by side in narrow panes
    - optional numeric inputs: Auto beside the value (A in Placement cells, pressed = automatic); source under the info control
    - explanatory text under Help; definition errors and status remain visible
    - component pickers: uniform tight vertical rows; name left, detail right; selected row highlighted
      sections show length and Solid/Hollow; guides show x; surfaces show span; volumes show x; parts show volume or x; mounts show target
    - Notes: editable text; blank allowed
```

## Left Pane

```text
LeftPane [arrange:column]

    * RecomputeHeader [arrange:row]
    * LeftTabs [ui:tabs]
        Summary
        Mass
        Aero
        Propulsion
        Log
```

### RecomputeHeader

```text
RecomputeHeader [arrange:row]

    - RecomputeButton [ui:button]
        label = "Recompute (F5)"
        on:click -> run heavy recompute

    - ComputeGeometryButton [ui:button]
        label = "Compute Geometry"
        on:click -> POST /api/geometry/compute (in the CAD worker process; edits go on meanwhile)
            builds every CAD solid of the current design once and measures the exact printed mass
            the exact mass replaces the estimates in Mass, CG, inertia and stability as soon as it is measured
            the rest (airframe pieces, fused shape, drawing projections) follows, so CAD exports only write files
        disabled while computing and when current (this design measured and built)
        any edit makes it a different design: estimates again, button enabled
        returning to the measured design (undo) applies its exact mass again without computing
        failures go to the Log tab

    the analysis and geometry status show in the top bar (main_window.md, ProjectStatus)
```

### Summary tab

```text
SummaryTab [fill]

    * SummaryPanel [fill, scrollable, copyable]
        * Overview [ui:labelled values, bind:analysis.report_pages.summary]
            airframe, mass, CG, thrust/weight, layout problems, propulsion alerts, static stability estimate
            geometry collapsed under Geometry details; layout problems link to the affected inspector
            problems in red (layout problems, unstable margin, no hover), warnings in yellow (marginal margin,
            parts not modelled, motorlib warnings)
```

### Detail tabs

```text
MassTab [fill, scrollable, copyable]
    mass and CG, printed structure basis ("Exact (built solids)" after Compute Geometry, else "Estimated"),
    inertia, mass breakdown table (Source: Exact / Estimated / Given / weighed value with the exact or estimated one),
    printed parts, fuselage section masses

AeroTab [fill, scrollable, copyable]
    static stability first: nominal and ranged-CG margins, verdicts, neutral point, Barrowman cross-check
    operating point, total forces, per-surface forces table
    expandable angle-of-attack sweep table; all points retained

PropulsionTab [fill, scrollable, copyable]
    motors and propellers, spin, battery, full throttle, hover, model limitations, wiring resistance

Detail values [ui:labelled values, fixed-width numbers, tables with units and sticky headings]
    tables scroll horizontally when needed; tabs retain their scroll positions
    tab keyboard navigation: Left/Right, Home/End
```

### Log tab

```text
LogTab [fill]

    * LogPanel [fill, scrollable, copyable, word-wrapped]
        * LogText [ui:fixed-width text]
            every error in red, appended with its time: definition errors, motor overloaded (no motor info, no propulsion
            in the report), aero failures, load / save / preset errors, and unexpected errors in full — the traceback
            with the failing context (for a CAD export, the task and the progress label it failed on), which also goes
            to logs/webapp.log at the repo root as a durable copy
            a new entry brings the Log tab to the front; the status bar says "<what> (see Log)"; the Summary keeps the
            last valid report, explicitly labelled when the definition is invalid
```

## Center Pane

```text
CenterPane [arrange:column]

    * CenterTabs [ui:tabs]
        Simple
        3D

    the open tab is remembered in the browser: leaving the view or reloading comes back to it
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

    - selecting a drawing handle opens its component inspector; inspector selection highlights the handle

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
        - draw top-left colored point guide: blue CG, red Neutral point, green Lift @ AoA, each with its x from the nose
        - draw pods, motors, shafts, props, CG, CP, CL
        - pink projected cut outlines: joint screws, motor bolts, head bores and shaft relief

    mode Front
        - draw body as the front envelope of its lofted cross-sections
        - selected loft guide: actual cross-section at its position highlighted white; selected printable section uses its midpoint
        - draw lifting surfaces as radial lines
        - draw volumes as dashed outlines, parts as solid outlines (a cross without library size)
        - draw wires as yellow dotted lines
        - draw spars as pink lines (solid = through, dashed = stops) and the carry-through as dashed pink outlines
        - draw top-left colored point guide: blue CG, red Neutral point, green Lift @ AoA, each with its x from the nose
        - draw motor / pod / prop cross-sections
        - pink projected cut outlines: joint screws, motor bolts, head bores and shaft relief
```

### 3D tab

```text
3DTab [fill]

    - Separate compact rows: axes/NP legend, visibility toggles, camera buttons
    - Transparent skin [ui:toggle button]: translucent body, pods, panels and controls; internal parts remain visible
    - Sections: toggle fuselage cross-section outlines and surface print-cut planes; off initially
    - Loft guides: independently toggle fuselage cross-section outlines at loft guides; off initially
    - Cuts & holes: pink cut-volume outlines for joint screws, motor bolts, head bores and shaft relief; on initially
    - NP = static estimate; camera retained through transparency changes
    - Three.js model from webapp/model3d.py
    - body-rooted panels start inside the body where the CAD export starts them (no gap over a tapering body)
    - drag to orbit, wheel to zoom
    - camera survives tab switches and form edits
```

## Right Pane

```text
RightPane [arrange:column]

    optional groups with an Enable checkbox (control surface, spars, pod, mounting holes, head bore, ...): expanded
    while enabled (on load and as soon as they are ticked), collapsed while not used; can still be collapsed by hand

    * RightTabs [ui:tabs]
        Design [ui:tabs]   the airframe itself
            Airframe
            Fuselage
            Sections
            Surfaces
            Access           saved holes and hatches (cuts and CAD are later work)
            Volumes
            Motor mounts
            Optimization     placeholder; objectives and part-value ranges deferred
        Build [ui:tabs]    what goes in it
            Propulsion
            Parts
            Wiring

    the group and the open section are remembered in the browser: leaving the view or reloading comes back to them
    (a selection made in the drawing still switches to the tab that edits it)
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
    - airframe length: resize unfixed sections proportionally; fixed sections keep their lengths
      parts, volumes, battery, surface roots, rod/channel endpoints, loft guides and nose/tail stations follow the resized sections
      component dimensions and surface-local hardware offsets remain unchanged
    * section: width, height, shape (superellipse exponent, 2 = circle), wall radial
    * nose, tail: profile [ui:dropdown], length, the profile's parameters
    * Loft guides: independent position mm, width mm, height mm, shape exponent; compact G<n> picker.
      A new guide takes the section at its position from the last valid airframe (also while the definition has an
      error elsewhere).
      One Auto on/off button per profile field inherits the Fuselage default when on.
      Add guide at the entered position, seeded from the interpolated profile; Remove guide deletes only the guide.
      Smooth cubic transitions join guides before nose/tail taper; no guides uses the default section throughout.
      Shape 1 = diamond, 2 = ellipse/circle, 4 = rounded rectangle; wall thickness stays global.
      Position edits reorder guides; side-view orange guide lines drag along x without moving cuts.
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

    vertical section picker labelled S<n>, length mm and Solid/Hollow at right; fields for the selected section
    Add before / Add after below the picker split the selected span in half and select the new section on that side.
    New section inherits Solid/Hollow and Fix length; airframe length, loft and placements stay unchanged.
    Weighed sections require clearing their masses before splitting; the cut starts with joint none.
        start mm     shared with the previous section's end; first section fixed at nose origin 0
        end mm       shared with the next section's start; last section edits fuselage length
        length mm    keep airframe length; move boundaries and scale the other unfixed sections proportionally
        Fix length   preserve length during automatic resizing; explicit length edits still apply
                     refuse changes when no unfixed section can absorb them; rejected changes leave geometry unchanged
        joint        that cut's joint [ui:dropdown]: coupler (separate coupler, its mass at the cut), front_male (this
                     section carries the male end), rear_male (the next section does), none (the sections butt with no
                     coupler, spigot or screws); empty on the last section
        solid        [ui:checkbox]  off = hollow shell wall; on = solid less the volumes inside it
        empty g      Auto = geometry estimate; Auto off = weighed printed mass [ui:textbox]
        calc         computed printed mass (hollow: shell wall x material density; solid: x print bulk density)
        complete g   Auto = itemised components; Auto off = weighed complete mass [ui:textbox]
        items        itemised mass of everything attached in the section (props with their motor, motors with their mount,
                     wing sets at their root leading edge); a range when it holds a mass range
        Remove end cut & merge sections [ui:button] delete the section's end cut and its joint: the two sections merge, keeping the left one's solid flag,
                     each weighed mass summed when both had one; mixed measured/automatic masses require clearing or measuring both first
    * joints: type [ui:dropdown], count (screws per section end), screw, insertion depth, coupler wall, fit clearance,
      socket wall, hole edge dist, hole diameter, hole angle offset, coupler mass
    Cut and section-length edits preserve the body, guides and placements.
    calc and items update after every edit
    the side view labels each section "S<n> <empty> g" (* = computed), its length, "solid", and "<complete> g all" when weighed
```

## Surfaces tab

```text
SurfacesTab [arrange:column]

    - Add surface set [ui:button]
    * compact surface picker and selected surface inspector: its planform fields, mass (blank = computed), Delete
      Sections: compact S<n> span-length list; S1 is the attached root, later fixed segments export separately.
        Add before/after below the list: first cut leaves a 15 mm root (half span on shorter panels), later cuts bisect the selected segment.
        Start and End are cut positions in mm from the root; the first Start is 0 and the last End is the fixed wing span (shown read-only; only its Start moves the last cut).
        Remove end cut merges adjacent segments. Cuts apply to every panel; controls stay unsplit, spar holes stay aligned.
        STEP/STL names W<surface>_P<panel>_S<segment>; Simple shows cyan airfoil cut outlines, 3D Sections shows cut planes.
      collapsed control surface group with Enable checkbox and its fields: kind (split: chord start, span start, span end; all-moving: pivot, span start),
        Align hinge vertically [ui:checkbox]: straight across span, anchored at the inboard hinge position; unchecked follows sweep.
        roles, deflection min/max, rate, gap (blank = 0), pivot rod diameter (blank = no rod), pivot rod x offset (mm, + aft) and y offset (mm, + up), both blank = 0
        Per-control span after gaps and pivot rod length in mm; pivot rods show projected end circles (plus a fixed-pixel centre dot) and dotted cylinder outlines in Simple, cylinders in 3D.
      collapsed spars group with Enable checkbox (ticking fills them from the aircraft's preset): material [ui:dropdown], diameter, count, spacing, x offset, y offset (up),
        carry-through [ui:checkbox]: stem width, length, flare radius, root length, wall fillet
      root base (not implemented) [ui:read-only]: length, width, height, forward offset, leading cut, trailing cut
```

## Access tab

```text
AccessTab [arrange:column]

    - Saved holes and hatches (definition access[]); physical fuselage cuts, separate retained hatch parts.
    - Add Hole / Add Hatch [ui:buttons]; component picker; selected inspector with name, type and Delete
    - name; type Hole | Hatch; profile Slot / circle or Rounded rectangle; width and length (mm, total opening
      extents; equal slot extents = circle; a slot's length is its longest dimension); corner radius (mm, rounded
      rectangle)
    - Hatch: gap (mm, clearance per edge); fasteners and hinges deferred
    - Simple: projected profile, label and cut-direction arrow; select/drag the seed or a radial copy to move the seed
    - 3D: actual openings and curved hatch pieces; Hatches visibility toggle; Cuts & holes hides only overlays
    - No live CAD operations; mass estimates exclude Access removals; exact material is built only on CAD export
    - CAD export reports misses, unsupported seams, overlap ownership and attached partial-depth hatches
    - Placement table: profile centre x/y/z (mm), then roll/pitch/yaw (deg); local X is the length, Y the width,
      +Z the cut direction; roll about X, then pitch about Y, then yaw about Z
    - cut extent: Depth (mm, from the profile plane along its local +Z), Through wall (first wall into its cavity),
      Through all (the whole body)
    - radial pattern: Enable, count (including original), X/Y/Z axis and axis position x/y/z (mm)
      evenly spaced around a full circle; position and orientation rotate together
```

## Motor mounts tab

```text
MotorMountsTab [arrange:column]

    - Add motor mount [ui:button]   a new mount is electric, on the nose
    * compact mount picker (number, motor type, on) and selected mount inspector: Motor type [ui:dropdown] (Electric motor |
      Rocket motor), on (nose | tail | surface; a rocket mount: tail | surface), surface, span position, x offset, direction,
      spin, thrust axis tilt, pod [ui:checkbox] with its fields, Delete
    a surface mount puts one motor on every panel of the surface
    - Motor type rocket: no direction, spin, pod or mounting holes (dropped when the type changes; a nose mount moves to the
      tail); X offset is the loaded motor's aft (nozzle) end from the tail face or surface root LE, positive aft; the motor
      runs forward from there. The motor itself is loaded on the Propulsion tab (Rocket)
    - deleting a mount takes the rocket motor loaded in it; later mounts' motors follow their mount's new number
    - Mount clearance (mm, added to the bell diameter), electric only, new = 1 mm: everything of the carrying wing and pod
      (or fuselage section) on the motor's side of the mount face, within bell + clearance, is cut away
    - Mounting holes [ui:checkbox]: new = round, count 4, angle 0, M3, pattern diameter 19 mm
        Hole type [ui:dropdown] Round | Slot; switching swaps the diameters (round: pattern diameter 19 mm;
          slot: inner 16 mm, outer 19 mm)
        Pattern count, Pattern angle (deg; 0 = +y), Hole diameter (screw) [ui:dropdown] M1.6 M2 M2.5 M3 M4 M5
          (data/parts/screw.json; its clearance bore is cut)
        Center relief diameter/depth; zero disables the relief
        Head bore [ui:checkbox]: Head bore diameter (starts at the screw table's head diameter), Head seat depth from
          mount face; the head bore runs from the seat on through the material in front, the way the screw is inserted
      a surface motor's bores go through its whole wing and pod; a nose/tail motor's through its fuselage section
      pink cut outlines in Simple and 3D; cuts in CAD exports
    - X offset: all mounts; positive aft; nose/tail motor and pod move together from the end face
```

## Propulsion tab

```text
PropulsionTab [arrange:column]

    * [ui:tabs] Electric | Rocket   (opens on Rocket when every mount is a rocket mount)
    Electric
        - Electric system [ui:checkbox]   motor, prop, battery, ESC and wiring for every electric mount; ticking it adds
          the default system (one motor run per electric motor), unticking removes it (electric mounts without it: a
          definition error in the Log)
        * SelectionPanel
        * BatteryPanel
        * MotorInfoPanel   (only while the motor can turn the prop; otherwise the Log says why)
    Rocket
        - Add rocket motor [ui:button]   the first database motor into the first rocket mount without one (mount 1 when
          none is free: a definition error in the Log until a rocket mount is picked)
        * one card per loaded motor: mount [ui:dropdown] (rocket mounts, "Mount N (on)"), motor [ui:search + dropdown]
          (data/rocket_motors/), its data as a labelled list [ui:labelled values] (motor, size, loaded mass, propellant,
          total impulse, average and peak thrust, burn time, delays),
          a thrust (N) against time (s) plot of its published curve [ui:plot], Delete
        a hint while there is no rocket mount (Motor type on the Motor mounts tab)
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
    - collapsed Battery placement & orientation: volume and Placement table (offsets or own x/y/z, roll/pitch/yaw)
    - the resolved pack as a labelled list [ui:labelled values]: label, size (each axis count x (cell + 2 mm)), mass, capacity
```

### SelectionPanel

```text
SelectionPanel [editable]

    - Manufacturer [ui:dropdown, readonly]
    - Motor [ui:search + dropdown, readonly]
    - Prop [ui:search + dropdown, readonly]
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
    * compact volume picker and selected volume inspector: name, shape, x, y, z (mm), shape sizes, mass (g, blank = space only),
      list of parts and battery in it, Delete

Rules
    - shapes: box, rounded box (size, axis of its rounded edges, corner radius), cylinder, bore, point
    - deleting a volume leaves its parts (and the battery) where they are, at their own x, y, z
    - renaming a volume renames it in its parts
    - dragging a volume in the side view moves its parts
```

## Parts tab

```text
PartsTab [arrange:column]

    - Add part [ui:button]        new part: first library entry with a mass, in the first volume (no volume: at mid-fuselage)
    * compact part picker and selected part inspector: name, searchable part (library), volume [ui:dropdown, "(none)" first],
      in a volume: x, y, z offset from its centre (mm, blank = 0); "(none)": its own x, y, z (mm; y, z blank = 0),
      Placement table: x/y/z (mm), roll/pitch/yaw (deg, blank = 0); mass (g; blank = library, required when the library has none; 0 = counted elsewhere), Delete

Rules
    - renaming a part renames it in the propulsion ESC; the propulsion battery is not a part (Propulsion tab)
    - deleting the propulsion battery or ESC: a definition error in the Log until another is picked
    - a part outside its volume, or overlapping another part, is listed under layout problems
    - volume -> "(none)": the part stays where it was, now at its own x, y, z; "(none)" -> a volume: at its centre
```

## Wiring tab

```text
WiringTab [arrange:column]

    a hint instead while there is no electric system (Propulsion tab, Electric)

    * WireRunsPanel  (title "Wire runs (auto-routed: along the axes, wing motors inside their panel; drawn in the side view)")
        one labelled group per run: battery -> ESC, then ESC -> each motor (motor order)
        full-width gauge and conductor fields; connectors wrap in route order
        fields
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

    - uPlot motor chart
    - labelled Full throttle, Hover and Model limitations results
    - complete battery and circuit details in the Propulsion report tab

Plot contents (mean per motor)
    - thrust vs INAV motor output
    - RPM / 1000 vs INAV motor output
    - kW vs INAV motor output
    - hover line (VTOL)

Results
    - full throttle, thrust/weight, hover
    - parts not modelled, motorlib warnings

Refreshes after every edit
```

## Field input behavior

Placement table: shared by Access, Parts and battery; position-only for volumes and radial pattern axes.
Two columns (label/value), six rows (X/Y/Z in mm, Roll/Pitch/Yaw in deg); position-only tables have three rows. Auto is A inside each cell.

```text
Length fields
    - displayed in mm
    - stored internally in meters

Any number field
    - wheel on a focused writable input: +/- 1 mm, +/- 1 whole number, otherwise +/- 0.1 displayed unit

Mass fields
    - displayed in g
    - stored internally in kg

Numeric entries
    - Enter and blur apply the value
    - optional fields: Auto (pressed) shows the effective value; the info control shows its source; toggled off enables entry
    - Auto removes the explicit definition value

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
