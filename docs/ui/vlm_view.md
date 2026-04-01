# VLM View

This file covers the VLM sweep UI and the display of solved AeroSandbox VLM points.

## Layout

```text
VLMView [fill, split:h]

    * LeftPane [split:v]
        TopPanel
        BottomPanel

    * RightPane
        DisplayPanel
```

### Split behavior

```text
VLMView split behavior
    - startup split is roughly 1/3 left, 2/3 right
    - left pane is split roughly 50/50 top and bottom
```

## TopPanel

```text
TopPanel [arrange:column]

    - Run [ui:button]
        on:click -> start threaded VLM sweep

    - ProgressBar [ui:progress]
        determinate

    - ProgressText [ui:text]
        format = "completed / total"

    - StatusText [ui:text]

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

### Numeric entry behavior

```text
Numeric entries
    - Return, keypad Enter, and FocusOut apply the value
    - Up arrow increases by 1
    - Down arrow decreases by 1
```

## BottomPanel

```text
BottomPanel [fill]

    * ResultsText [ui:fixed-width text, scrollable, copyable]
        displays VLM sweep report text
```

## RightPane

```text
DisplayPanel [fill]

    * PointSelectors [arrange:column]
        - Altitude [ui:slider]
        - AoA [ui:slider]
        - Airspeed [ui:slider]

    - SelectedPointText [ui:text]
    - Show [ui:button]
    - DisplayMessage [ui:text]
```

### Point selector behavior

```text
Sliders
    - use actual solved values, not row indexes
    - are reconfigured from the solved result set
    - show formatted current values beside each slider

SelectedPointText
    - shows the current altitude, AoA, and airspeed when a solved point exists
    - otherwise shows that no VLM point is selected

Show
    - opens the selected solved VLM point in the external AeroSandbox display
```

## Worker behavior

```text
Run behavior
    - VLM sweep runs in a background thread
    - the Tk window remains responsive
    - progress updates are delivered through a queue polled on the Tk thread
    - results update the text report and selector ranges when complete
    - errors are surfaced through the traceback dialog
    - solved VLM objects are kept transiently in memory for fast display switching
    - transient VLM cache is not saved to disk

Recompute behavior
    - any light recompute invalidates the current VLM cache
    - the view is reset to "VLM cache stale. Press F5 in VLM."
    - F5 while VLM view is active reruns the current sweep
```
