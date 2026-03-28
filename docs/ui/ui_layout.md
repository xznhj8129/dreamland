# GUIBuilder — UI Layout Spec

This document records the current window layout and widget behavior of `guibuilder.py`.

## Notation

```text
Component [attr, attr]          Layout/behavior properties
    ui:dropdown/radio/textbox/etc
    arrange:column/row
    on:event -> action
    bind:state_key
    enum:name -> { A, B, C }

    * Child                     Container / structural child
    - Item                      Direct interactive control
```

### Attribute glossary

| Attribute             | Meaning                                      |
|-----------------------|----------------------------------------------|
| `docked:edge`         | Pinned to one edge of parent                 |
| `fill`                | Expands to fill remaining space              |
| `split:h`             | Horizontal pane split                        |
| `scrollable`          | Has scrollable content area                  |
| `switchable`          | Multiple modes, one visible at a time        |
| `copyable`            | Text can be selected and copied              |
| `editable`            | User can directly edit values                |
| `readonly`            | Display only; user cannot type arbitrary     |
| `bind:key`            | Reads/writes shared application state        |

---

## Window

```text
GUIBuilderWindow [fill]

    * MenuBar [docked:top, arrange:row]

    * MainArea [fill, split:h]
        LeftPane
        CenterPane
        RightPane

    * BottomBar [docked:bottom, arrange:row]
```

### Theme

```text
Theme

    - Palette = monochrome green on black
    - Window background = black
    - Panels = dark green/black
    - UI outlines = green
    - Airframe drawing = green
    - Semantic markers remain color-coded:
        CG = blue
        CP = red
        CL@AoA = green
```

---

## Menu Bar

```text
MenuBar [docked:top]

    * FileMenu [ui:dropdown, arrange:column]
        - Load
            on:click -> open file picker, load JSON
        - Save
            on:click -> open file picker, save JSON
        - Exit
            on:click -> close application

    * Export [ui:dropdown, arrange:column]
        - Export Body STEP
            on:click -> open file picker, save JSON
        - Export Body STL
            on:click -> open file picker, save STL
        - Export VSP
            on:click -> open file picker, airplane.export_OpenVSP_vspscript(str("{name}.vspscript"))
```

### Load / Save behavior

```text
Save payload
    assembly_name
    config
    analysis
    analysis excludes the expanded assembly field

Load behavior
    If JSON contains:
        - parts                  -> treat file as direct config
        - assembly_name/config   -> use config payload
        - top-level assembly map -> use first key

    analysis from file is ignored after load.
    The app regenerates analysis from config immediately.
```

---

## Main Area

```text
MainArea [fill, split:h]

    * LeftPane [fill]
    * CenterPane [fill]
    * RightPane [fill]
```

### Pane behavior

```text
MainArea split behavior
    - Left / Center / Right are horizontally split panes
    - Pane separators are draggable
    - Window fills available application size
    - Default startup split is 25% / 50% / 25%
    - User may drag separators after startup
```

---

## Left Pane

```text
LeftPane [arrange:column]

    * RecomputeHeader [arrange:row]

    * LeftTabs [ui:tabs]
        Summary
        Controls
```

### Recompute Header

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

### Left Tabs

```text
LeftTabs [fill]

    * SummaryTab [fill]
        * SummaryPanel [fill]

    * ControlsTab [arrange:column]
        * ControlsPanel
```

### ControlsTab

```text
ControlsPanel [arrange:column]

    - AoA eval [ui:slider, bind:aoa_eval_deg]
        range = -45 deg to 45 deg
        resolution = 1 deg
        default = 0 deg
        current value displayed beside slider
        on:slide -> run light recompute

Numeric text entry behavior
    - Up arrow increases numeric field by 1 displayed unit
    - Down arrow decreases numeric field by 1 displayed unit
    - Mouse wheel changes length fields in mm by 1 mm
```

### SummaryTab

```text
SummaryPanel [fill, scrollable, copyable]

    * SummaryText [ui:fixed-width text, bind:analysis.report_text]
        read-only by convention
        user can select/copy text
        horizontal scroll enabled
        vertical scroll enabled
```

### Summary content

```text
SummaryText sections
    - Overall Summary
    - Lifting Surfaces
    - Battery
    - Motors
    - Static Stability
    - Aerodynamic Centers
    - Components
    - Modules
    - Notes
```

---

## Center Pane

```text
CenterPane [arrange:column]

    * CenterTabs [ui:tabs]
        Simple
        ASB
```

### Center Tabs

```text
CenterTabs [fill]

    * SimpleTab [arrange:column]
        * GraphicsPanel
        * OrthographicView [fill]

    * ASBTab [fill]
        * ASBView [fill]
```

### Graphics Panel

```text
GraphicsPanel [arrange:column]

    - MarkerLegend [ui:text]
        "Blue: CG   Red: CP   Green: CL@AoA"

    * ViewSelector [ui:radio, arrange:row, bind:view_mode]
        enum:view_mode -> { Longitudinal, Front }
        on:select -> redraw orthographic view

    * RollRail [arrange:column]
        docked at right of orthographic view

        - RollView [ui:vertical slider, bind:roll_view_deg]
            range = 0 deg to 90 deg
            resolution = 1 deg
            default = 0 deg
            0 deg = side
            90 deg = top
            current value displayed below slider
            on:slide -> redraw longitudinal view
```

### Orthographic View

```text
OrthographicView [fill, switchable]

    mode Longitudinal
        - Draw fuselage in side view
        - Draw ends as half-ellipse arcs meeting the tube
        - Draw lifting surfaces as projected trapezoids through roll angle from side to top
        - Draw pod body behind motor when pod lengths are nonzero
        - Draw motors as outlined squares
        - Draw motor shaft as 5 mm diameter, 15 mm long cylinder projection
        - Draw motor base as 1/2 motor diameter, 5 mm long cylinder projection
        - Draw props edge-on as red lines
        - Prop line meets shaft end
        - Draw CG / CP / CL markers on fuselage centerline

    mode Front
        - Draw body as circular cross-section
        - Draw lifting surfaces as radial lines
        - Draw pod diameter as outer motor circle
        - Draw motors as circular faces
        - Draw shaft as central 5 mm hub circle
        - Draw props as red circles
        - Draw small marker legend
```

### ASB View

```text
ASBView [fill]

    - UpdateViewButton [ui:button]
        label = "Update View"
        on:click -> run heavy recompute and redraw current ASB airplane

    - Embedded matplotlib canvas in Tk
    - Draws one AeroSandbox 3D model from the current in-memory airplane
    - Light recompute does not redraw ASB
    - Heavy recompute rebuilds and redraws ASB
    - Starts in an isometric orientation
    - User may reorient the same 3D model interactively
```

### Config Normalization

```text
Built-in default and loaded saved configs
    - Are normalized through templates before editing/rendering
    - Module dicts inherit their full module template by type
    - Surface dicts inherit their full surface template by type
    - Part-level keys inherit part_defaults
```

### Drawing conventions

```text
Airframe geometry
    - Fuselage OD is global from config.fuselage_od_mm
    - Module-level diameters are not used as editable independent geometry
    - Wing semispan is measured from tube surface / wing root outward
    - Wing mount angle affects top/side/front projection

Markers
    - Blue dot = CG
    - Red dot = CP
    - Green dot = CL

Propulsion drawing
    - Surface motors use motor_x_pos from wing leading edge/root reference
    - Surface motors use motor_y_pos as radial distance from root
    - End-mounted motors ignore x/y offset fields
    - motor_dir enum = { front, back }
    - Surface motor diameter uses pod_diam when present

Motor pod data model
    - pod_diam
    - pod_len_flat
    - pod_len_curve
    - pod_shape enum = { elliptical, ogive }
```

---

## Right Pane

```text
RightPane [arrange:column]

    * RightTabs [ui:tabs]
        Parameters
        Assembly
```

### Right Tabs

```text
RightTabs [fill]

    * ParametersTab [arrange:column]
        * ParametersHeader
        * AssemblyCard

    * AssemblyTab [arrange:column]
        * ModuleStack [fill, scrollable]
```

### Parameters Header

```text
ParametersHeader [arrange:column]

    - Assembly Name [ui:textbox, bind:assembly_name]
        editable
        Enter applies value

    - Standard Profile [ui:dropdown, bind:config.standard_profile]
        enum = { None, standards list }
        on:select -> refresh analysis

    - Apply [ui:checkbox, bind:config.apply_standard]
        on:toggle -> selected standard fixes standard-driven dimensions such as fuselage OD/ID

    - Fuselage OD (mm) [ui:textbox, bind:config.fuselage_od_mm]
        editable
        on:edit -> refresh analysis

    - Fuselage ID (mm) [ui:textbox, bind:config.fuselage_id_mm]
        editable
        on:edit -> refresh analysis

    - Material Density (g/cm^3) [ui:textbox, bind:config.material_density_g_cm3]
        editable
        on:edit -> refresh analysis
```

### Module Stack

```text
ModuleStack [fill, scrollable, arrange:column]

    * PartCard[]                One card per config.parts entry
```

### Assembly Card

```text
AssemblyCard [editable]

    fields from config excluding:
        - parts
        - fuselage_od_mm
        - fuselage_id_mm

    bool fields
        ui:checkbox

    scalar fields
        ui:textbox
```

### Part Card

```text
PartCard [editable, arrange:column]

    title = "{index}. {module_name}"

    * PartControls [arrange:row]
        - Up
            on:click -> move card up
        - Down
            on:click -> move card down
        - Delete
            on:click -> delete card

    * ParametersSection
        module-level editable fields

    * SurfaceSection
        shown only if part has surfaces

    * PropulsionSection
        shown only if module.motor or surfaces.motor is true

    * AuxiliarySection
        remaining part-level fields
```

### Parameters Section

```text
ParametersSection [editable]

    module fields shown in preferred order:
        name
        type
        len
        mass
        cg
        wings_n
        wings_ang
        wings_x
        eo
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

    if motor is false:
        motor placement fields are hidden

    if ballast is true:
        ballast mass is edited in AuxiliarySection
```

### Surface Section

```text
SurfaceSection [editable]

    surface fields shown in preferred order:
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

    if motor is false:
        motor placement and pod fields are hidden
```

### Propulsion Section

```text
PropulsionSection [editable]

    - motor [ui:dropdown, readonly, bind:part.motors]
        source = simlib measured motor names

    - prop [ui:dropdown, readonly, bind:part.props]
        source = propeller names
```

### Auxiliary Section

```text
AuxiliarySection [editable]

    currently includes remaining part-level fields after:
        module
        surfaces
        motors
        props
        parts

    special cases
        battery
            ui:dropdown, readonly
            source = libs/parts.json battery keys

        ballast_mass
            appears when module.ballast is true
            displayed in g
            stored internally in kg
```

### Field input behavior

```text
Length fields
    displayed in mm
    entered in mm
    internal storage remains meters

Mass fields
    displayed in g
    entered in g
    internal storage remains kg

Bool fields
    ui:checkbox

Type fields
    ui:readonly dropdown

motor_dir
    ui:readonly dropdown
    enum:motor_dir -> { front, back }

pod_shape
    ui:readonly dropdown
    enum:pod_shape -> { elliptical, ogive }
```

### Entry interaction rules

```text
Textbox interaction
    - valid numeric edits refresh analysis
    - Enter applies current field value
    - keypad Enter applies current field value

Mouse wheel
    - bound on length-driver textboxes
    - one wheel step = 1 mm
    - applies to:
        fuselage_od_mm
        fuselage_id_mm
        len
        diam
        semispan
        chord_root
        chord_tip
        motor_x_pos
        motor_y_pos
        pod_diam
        pod_len_flat
        pod_len_curve
```

---

## Bottom Bar

```text
BottomBar [docked:bottom, arrange:row]

    - Configuration
    - Aero Analysis
    - CAD
    - Module Editor
    - Part Editor
```

### Bottom bar behavior

```text
Current state
    - Configuration switches the main window to the configuration view
    - Aero Analysis switches the main window to the aero-analysis view
    - CAD button is visible
    - Module Editor button is visible
    - Part Editor button is visible
```

---

## Aero Analysis View

```text
MainArea mode AeroAnalysis [fill, split:h]

    * LeftPane [split:v]
        TopPanel
        BottomPanel

    * RightPane
        DisplayPanel
```

### Aero Analysis split behavior

```text
MainArea mode AeroAnalysis
    - startup split is 1/3 left, 2/3 right
    - left pane is split 50/50 top/bottom
```

### TopPanel

```text
TopPanel [arrange:column]

    - Run [ui:button]
        on:click -> start threaded VLM sweep

    - ProgressBar [ui:progress]
        determinate
        updates while worker thread runs

    - ProgressText [ui:text]
        format = "completed / total"

    - StatusText [ui:text]
        shows current point or final state

    * VLMParameterGroups [arrange:grid]
        AoA
        Airspeed
        Altitude
        Solver
```

### VLMParameterGroups

```text
AoA
    - AoA start (deg)
    - AoA end (deg)
    - AoA step (deg)

Airspeed
    - Airspeed start (m/s)
    - Airspeed end (m/s)
    - Airspeed step (m/s)

Altitude
    - Altitude start (m)
    - Altitude end (m)
    - Altitude step (m)

Solver
    - Worker threads
    - Spanwise resolution
    - Chordwise resolution
```

### BottomPanel

```text
BottomPanel [fill]

    * ResultsText [ui:fixed-width text, scrollable, copyable]
        displays VLM sweep report text
```

### RightPane

```text
DisplayPanel [fill]

    - placeholder text only
    - no VLM display content yet
```

### Aero Analysis worker behavior

```text
Run behavior
    - VLM sweep runs in a background thread
    - main application remains responsive
    - results are pushed back into the window when complete
    - errors are surfaced through the existing traceback dialog

F5 behavior
    - in Configuration view: run heavy recompute
    - in Aero Analysis view: rerun the current VLM sweep with the current sweep settings
```

---
