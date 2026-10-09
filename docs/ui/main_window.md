# GUIBuilder Main Window

This file covers the persistent shell of the web front end (`web/src/App.tsx`, `web/src/components/TopBar.tsx`): the top bar with the File menu and the main-view buttons, and the central view host.

## Theme

```text
Theme

    - Palette = lighter green on darker greyish
    - Window background = #141517
    - Panels = #1c1e21; cards = #25282c
    - UI outlines = #3b4045
    - UI font = system sans-serif; reports = monospace
    - Menus, bars, boxes and controls = restyled; drawing renderer = unchanged
    - Airframe drawing = green
    - Semantic markers remain color-coded:
        CG = blue
        Neutral point = red
        Lift @ AoA = green
```

## Window

```text
GUIBuilderWindow [fill]

    * TopBar [docked:top, arrange:row]
        - File menu
        - main-view buttons

    * MainViewHost [fill, switchable]
        EditorView
        AeroAnalysisView
        VLMView
        SimView
        CADView
        PartEditorView
```

### Window behavior

```text
Startup
    - window title = "quadrocketbuilder - <project> (<aircraft name>)"
    - opens the project given on the command line (`webbuilder.py <project>`)
    - default visible view = Editor
    - F5 triggers heavy recompute while the Editor view is active
```

## Top Bar

```text
TopBar [docked:top, arrange:row]

    * FileMenu [ui:dropdown, arrange:column]
        - New
            on:click -> ask a new project name, create aircraft/<name>/ with a blank definition, continue there
        - Load
            on:click -> pick a project from the list under aircraft/, load it
        - Save
            on:click -> write the project's aircraft.json
        - Save As
            on:click -> ask a new project name, copy the whole project to aircraft/<name>/, save there, continue there

    * ViewButtons [arrange:row]
        - Editor, Aero Analysis, VLM, SIM, CAD, Part Editor
            on:click -> raise the view; the visible view's button is highlighted
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

    - one main view exists for each top-bar view button
    - only the visible view is rendered
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

Aero Analysis and VLM are placeholder views in the web front end; their engines
(VLM sweep, NeuralFoil airfoil comparison) run headless.

## Error handling

```text
Errors (aero failures, load / save / preset errors)
    - are appended to the Editor view's Log tab
    - an unchanged repeat of the last logged error adds no second line
    - a failed save also shows its message in the top bar; a failed load in its dialog
```
