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
        CADView
        ModuleEditorView
        PartEditorView

    * BottomBar [docked:bottom, arrange:row]
```

### Window behavior

```text
Startup
    - window title = "Builder V3"
    - default geometry = 1800x980
    - default visible view = Configuration
    - F5 triggers heavy recompute in most views
    - F5 reruns the current VLM sweep while VLM view is active
```

## Top Bar

```text
MenuBar [docked:top]

    * FileMenu [ui:dropdown, arrange:column]
        - Load
            on:click -> open file picker, load JSON
        - Save
            on:click -> open file picker, save JSON
        - Exit
            on:click -> close application
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
CAD            -> cad_view.md
Module Editor  -> module_editor_view.md
Part Editor    -> part_editor_view.md
```

## Bottom Bar

```text
BottomBar [docked:bottom, arrange:row]

    - Configuration
    - Aero Analysis
    - VLM
    - CAD
    - Module Editor
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

CAD
    - raises the CAD view

Module Editor
    - refreshes data/modules.json from disk
    - raises the module-library editor view

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
