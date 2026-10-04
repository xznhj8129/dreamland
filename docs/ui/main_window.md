# GUIBuilder Main Window

This file covers the persistent shell of `guibuilder.py`: top bar, central view host, and bottom bar.

## Theme

```text
Theme

    - Palette = monochrome green on black
    - Window background = black
    - Panels = dark green/black
    - UI outlines = green
    - Airframe drawing = green
    - Semantic markers remain color-coded:
        CG = blue
        Neutral point = red
        Lift @ AoA = green
```

## Window

```text
GUIBuilderWindow [fill]

    * MenuBar [docked:top, arrange:row]

    * MainViewHost [fill, switchable]
        ConfigurationView
        AeroAnalysisView
        VLMView
        SITLView
        CADView
        PartEditorView

    * BottomBar [docked:bottom, arrange:row]
```

### Window behavior

```text
Startup
    - window title = "quadrocketbuilder - <project> (<aircraft name>)"
    - default geometry = 1600x980
    - opens the project given on the command line, default aircraft/m62v2
    - default visible view = Configuration
    - F5 triggers heavy recompute in most views
    - F5 reruns the current VLM sweep while VLM view is active
```

## Top Bar

```text
MenuBar [docked:top]

    * FileMenu [ui:dropdown, arrange:column]
        - Load
            on:click -> pick a project directory (starts in aircraft/), load it
        - Save
            on:click -> write the project's aircraft.json
        - Save As
            on:click -> ask a new project name, copy the whole project to aircraft/<name>/, save there, continue there
        - Exit
            on:click -> close application
```

### Load / Save behavior

```text
Project = aircraft/<project>/ with aircraft.json (the airframe/2 definition), inav/, sitl/, jsbsim/, logs/, cad/

Load
    - a directory without aircraft.json -> error dialog, nothing changes
    - aircraft.json not airframe/2 -> error dialog, nothing changes
    - otherwise everything is recomputed from the definition

Save
    - writes the definition only; nothing computed is saved

Save As
    - a name that is not a plain directory name, or an existing project -> error dialog, nothing changes
    - cancel -> nothing changes
    - the old project stays as it was last saved
```

## Central View Host

```text
MainViewHost [fill, switchable]

    - one main view frame exists for each bottom-bar mode
    - all view frames are gridded into the same parent
    - switching view raises the requested frame
```

### Main-view mapping

```text
Configuration  -> configuration_view.md
Aero Analysis  -> aero_analysis_view.md
VLM            -> vlm_view.md
SITL           -> sitl_view.md
CAD            -> cad_view.md
Part Editor    -> part_editor_view.md
```

## Bottom Bar

```text
BottomBar [docked:bottom, arrange:row]

    - Configuration
    - Aero Analysis
    - VLM
    - SITL
    - CAD
    - Part Editor
```

### Bottom-bar behavior

```text
Configuration
    - raises the configuration view

Aero Analysis
    - raises the aero-analysis view

VLM
    - raises the VLM sweep view

SITL
    - refreshes the scenario and INAV config lists
    - raises the SITL simulation view

CAD
    - raises the CAD view

Part Editor
    - refreshes data/parts.json from disk
    - raises the part-library editor view
```

## Error handling

```text
Unhandled Tk callback errors
    - are printed with full traceback
    - open an Error dialog with scrollable traceback text
    - dialog can be closed with the Close button or Escape
```
