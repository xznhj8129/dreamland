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

        - static note:
            "Cheap aero sweep is always recomputed from -45 to 45 deg."
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
        - draw internal objects with known size as dashed outlines
        - draw top-left colored point guide: blue CG, red Static CP, green Lift @ AoA
        - draw pods, motors, shafts, props, CG, CP, CL

    mode Front
        - draw body as circular cross-section
        - draw lifting surfaces as radial lines
        - draw internal objects with known size as dashed outlines
        - draw top-left colored point guide: blue CG, red Static CP, green Lift @ AoA
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
        Parameters
        Propulsion
        Assembly
```

## Parameters tab

```text
ParametersTab [arrange:column]

    * ParametersHeader
    * AssemblyCard
```

### ParametersHeader

```text
ParametersHeader [arrange:column]

    - Assembly Name [ui:textbox, bind:assembly_name]
    - Standard Profile [ui:dropdown, bind:config.standard_profile]
    - Apply [ui:checkbox, bind:config.apply_standard]
    - Fuselage OD (mm) [ui:textbox, bind:config.fuselage_od_mm]
    - Fuselage ID (mm) [ui:textbox, bind:config.fuselage_id_mm]
    - Material Density (g/cm^3) [ui:textbox, bind:config.material_density_g_cm3]
```

### AssemblyCard

```text
AssemblyCard [editable]

    fields from config excluding:
        - parts
        - fuselage_od_mm
        - fuselage_id_mm
        - material_density_g_cm3
        - thrust_axis_tilt_deg
        - standard_profile
        - apply_standard
        - battery
        - motors
        - props

    bool fields
        ui:checkbox

    scalar fields
        ui:textbox
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

    - Battery [ui:dropdown, readonly, bind:config.battery]
    - Motor [ui:dropdown, readonly, bind:config.motors]
    - Propeller [ui:dropdown, readonly, bind:config.props]

Propeller options
    - are filtered from tested motor data
    - are narrowed by selected battery series count when available
```

### MotorInfoPanel

```text
MotorInfoPanel [fill]

    - top half = matplotlib plot host
    - bottom half = fixed-width text report

Plot contents
    - thrust vs throttle
    - RPM / 1000 vs throttle
    - kW vs throttle
```

## Assembly tab

```text
AssemblyTab [arrange:column]

    - Add New [ui:button]
    * ModuleStack [fill, scrollable, arrange:column]
```

### PartCard

```text
PartCard [editable, arrange:column]

    title = "{index}. {module_name}"
    header includes a small + / - collapse toggle beside the title

    * PartControls [arrange:row]
        - Up
        - Down
        - Delete

    * LoadModuleRow
        - module-library dropdown
        - Load button

    * ParametersSection
    * SurfaceSection
    * PartFieldsSection
```

### ParametersSection

```text
ParametersSection [editable]

    preferred module field order:
        name
        type
        len
        mass
        cg
        wings_layout
        wings_fin_pos
        wings_ang
        wings_x
        wings_y_offset
        pointed_shape
        fins
        ballast
        battery
        payload
        esc
        motor
        motor_x_pos
        motor_y_pos
        motor_dir

    hidden module fields:
        diam
        model

    read-only rows
        - wing count, derived from the selected layout (shown for wingmounts
          and for end modules with fins enabled)

    special cases
        - wings_layout is a dropdown with options H, 1, 3, +, X
        - selecting X sets dihedral to 45 if it was 0
        - wings_y_offset meaning depends on layout: for H-type pairs
          -1=bottom, 0=mid, 1=top; for X, -1=mid, 0=45 deg, 1=top
        - wings_fin_pos is only shown for layouts 1 and 3
        - end modules only expose fin placement fields when fins is enabled
        - end modules edit pointed geometry through a nested pointed_shape block
        - inside pointed_shape, only the parameters relevant to the selected shape are shown
        - canonical exposed shape parameters include:
            K' for parabolic
            rho for secant ogive
            L1 and R1 for bi-conic
            r_n for spherically blunted shapes
        - motor placement fields are hidden when motor is false
```

### SurfaceSection

```text
SurfaceSection [editable]

    shown only when the part supports surfaces

    preferred surface field order:
        name
        type
        semispan
        mass
        cg
        chord_root
        chord_tip
        aoa_ang
        sweep_ang
        airfoil
        motor
        motor_x_pos
        motor_y_pos
        motor_dir
        pod_diam
        pod_len_flat
        pod_len_curve
        pod_shape
        ctl_surface
        all_moving

    hidden surface fields:
        model

    special cases
        - shows a derived read-only mid-span chord, computed as the trapezoid chord halfway from root to tip
        - motor placement and pod fields are hidden when motor is false
```

### PartFieldsSection

```text
PartFieldsSection [editable]

    placement fields appear when module.battery, module.payload, or module.ballast is true:
        x
            FRU local frame
            positive = Front
            relative to module geometric center
        y
            FRU local frame
            positive = Right
            relative to module geometric center
        z
            FRU local frame
            positive = Up
            relative to module geometric center
        pitch
        yaw
        roll

    special mass fields
        - payload_mass when module.payload is true
        - ballast_mass when module.ballast is true

    orientation fields
        - are stored in assembly data
        - are exposed in the UI
        - are not yet used by analysis or drawing
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

## Config normalization

```text
Built-in default and loaded saved configs
    - are normalized through templates before editing or rendering
    - module dicts inherit their full module template by type
    - surface dicts inherit their full surface template by type
    - part dicts inherit part_defaults
```
