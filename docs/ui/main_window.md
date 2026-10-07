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
        EditorView
        AeroAnalysisView
        VLMView
        SimView
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
    - default visible view = Editor
    - F5 triggers heavy recompute in most views
    - F5 reruns the current VLM sweep while VLM view is active
```

## Top Bar

```text
MenuBar [docked:top]

    * FileMenu [ui:dropdown, arrange:column]
        - New
            on:click -> ask a new project name, create aircraft/<name>/ with a blank definition, continue there
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
Project = aircraft/<project>/ with aircraft.json (the airframe/2 definition), inav/, sitl/, jsbsim/, logs/, cad/, controllers/

Load
    - a directory without aircraft.json -> error in the Log, nothing changes
    - aircraft.json not airframe/2 -> error in the Log, nothing changes
    - a definition missing a top-level block the editors need -> error in the Log, nothing changes (an incomplete definition with all blocks still opens, so it can be repaired)
    - otherwise everything is recomputed from the definition

Save
    - writes the definition only; nothing computed is saved

Save As
    - a name that is not a plain directory name, or an existing project -> error in the Log, nothing changes
    - cancel -> nothing changes
    - the old project stays as it was last saved

New
    - a name that is not a plain directory name, or an existing project -> error in the Log, nothing changes
    - cancel -> nothing changes
    - creates aircraft/<name>/ with the standard subdirectories and a blank definition: one default tube, no surfaces, volumes or parts, unpowered
    - the Log starts empty; the summary shows the static report (mass, geometry, layout) and the status reads "No lifting surfaces" instead of failing aero
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
Editor         -> editor_view.md
Aero Analysis  -> aero_analysis_view.md
VLM            -> vlm_view.md
SIM            -> sim_view.md
CAD            -> cad_view.md
Part Editor    -> part_editor_view.md
```

## Bottom Bar

```text
BottomBar [docked:bottom, arrange:row]

    - Editor
    - Aero Analysis
    - VLM
    - SIM
    - CAD
    - Part Editor
```

### Bottom-bar behavior

```text
Editor
    - raises the Editor view

Aero Analysis
    - raises the aero-analysis view

VLM
    - raises the VLM sweep view

SIM
    - refreshes the scenario and INAV config lists
    - raises the SIM view (INAV SITL and script simulations)

CAD
    - raises the CAD view

Part Editor
    - refreshes the part library (data/parts/) from disk
    - raises the part-library editor view
```

## Error handling

```text
Errors (unhandled callback errors with their traceback, aero failures, load / save / preset errors)
    - are appended to the Editor view's Log tab; the status bar says "<what> (see Log)"
    - an unchanged repeat of the last logged error updates the status bar but adds no second line
    - never open a window
```
