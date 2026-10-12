# Aero Analysis View

This view provides 2D airfoil comparison using NeuralFoil-backed AeroSandbox airfoil analysis. The web view ports
the retired Tk `AeroAnalysisMainView`; the routes live in `webapp/airfoils.py` and the compute is
`libs/airfoil_analysis.py` unchanged. The plots are uPlot (`web/src/views/AeroAnalysisView.tsx`), not matplotlib.

## Layout

```text
AeroAnalysisView [fill, split:h]

    * LeftPane [25%]
    * RightPane [75%]
```

Split behavior:

```text
AeroAnalysisView split behavior
    - startup split is roughly 1/4 left, 3/4 right; the divider drags as in the other views
```

The view is created lazily when its tab is first shown and unmounts when the tab is left, so it reseeds from the
current airframe's surface airfoils on every first show and on every project switch: it reads the definition's
`surfaces[].airfoil` names, sends them as `surfaces` to `GET /api/airfoils`, selects the names the server seeds
back and recomputes. A project switch while the view is open clears the view and reseeds from the new airframe.

## Left Pane

```text
LeftPane [arrange:column]

    * ParametersPanel [~1/3 h]
    * AirfoilSelectionPanel [~1/3 h]
    * SummaryPanel [~1/3 h]
```

### ParametersPanel

```text
ParametersPanel [editable]

    - Recompute [ui:button]
        label = "Recompute (F5)"
        on:click -> rerun selected airfoils through NeuralFoil

    - Flow / Sweep Readout [ui:text]
        after each analysis: "Re = <reynolds> | M = <mach> | alpha sweep = <min>..<max> deg"
        the numbers come from the last analyze response (the sweep range from its alpha series; before the
        first response the sweep range comes from the library payload); a non-finite value shows n/a

    - AoA operating point [ui:slider + textbox]
        range = -20 deg to +20 deg, step 0.5 deg
        the textbox accepts any number, not clamped to the slider range
        dragging the slider updates the textbox and reruns the analysis, debounced ~150 ms
        Enter/blur commits the textbox: a parse failure reverts it to the last valid value and does not rerun;
        any other finite number is committed as-is and reruns
        a committed value outside the slider range moves the thumb to the nearer end, but is analyzed as entered

    - Airspeed (m/s) [ui:textbox]
        on:commit -> refresh plots and summary

    - Altitude (m) [ui:textbox]
        on:commit -> refresh plots and summary

    - Temperature (C) [ui:textbox]
        on:commit -> refresh plots and summary

    - Reference chord (mm) [ui:textbox]
        on:commit -> refresh plots and summary

    - N-Crit [ui:textbox]
        on:commit -> refresh plots and summary

    - Xtr Upper [ui:textbox]
        on:commit -> refresh plots and summary

    - Xtr Lower [ui:textbox]
        on:commit -> refresh plots and summary

    - Model size [ui:dropdown]
        enum -> { xxsmall, xsmall, small, medium, large, xlarge, xxlarge, xxxlarge }
        on:select -> refresh plots and summary

    - Include 360 deg effects [ui:checkbox]
        on:toggle -> refresh plots and summary
```

Numeric-entry semantics for every textbox above (the Tk `bind_numeric_entry`): the text is free; Enter/blur
commits it. A commit that does not parse as a finite number reverts that field to its last committed value and
runs nothing; a valid commit stores the value and reruns the analysis. An uncommitted edit just stays in the
box until it is committed or reverted.

Defaults (seeded by the server from `DEFAULT_AIRFOIL_OPERATING_POINT` / `DEFAULT_NEURALFOIL_SETTINGS`):

```text
alpha_deg 4.0 | airspeed_m_s 25.0 | altitude_m 0.0 | temperature_c 15.0 | chord_mm 100.0
n_crit 9.0 | xtr_upper 1.0 | xtr_lower 1.0 | model_size "large" | include_360_deg_effects true
```

F5 recomputes whatever the active view shows: in this view it is exactly this panel's Recompute. The key
listener exists only while this view is mounted, and the Editor's own F5 listener only while the Editor is
shown, so the two never fire together.

### AirfoilSelectionPanel

```text
AirfoilSelectionPanel [editable]

    - Available list [ui:listbox]
        source = data/airfoils.json plus the DAT files imported this session, plus AeroSandbox airfoil names the
        current airframe's surfaces use and the library file does not hold (the server seeds them through
        asb_airfoil_record, source "asb"; a name AeroSandbox does not know is skipped)

    - Selected list [ui:listbox]
        all selected airfoils are plotted together for comparison

    - Add -> [ui:button]
        adds the selected Available airfoils (without duplicates)

    - <- Remove [ui:button]
        removes the selected Selected airfoils

    - Import DAT [ui:button]
        opens a hidden file input (accept .dat), reads the file text in the browser and posts it; the imported
        airfoil joins both lists and the analysis reruns

    - Clear [ui:button]
        clears the active comparison list

Double-clicking an Available entry adds it; double-clicking a Selected entry removes it.

Import DAT is session-only (imports are lost on a server restart, like the Tk view's). The server's name
deduplication is authoritative: a taken name is imported as "<name>-2", "<name>-3", ... and the returned name
joins the lists. A DAT the server refuses (parse error) puts the server's message in the Summary panel; the
next successful analysis replaces it with the report text.

Empty selection: no analyze request is sent; the Summary shows "No airfoils selected." and the four plots are
blank (titles and axes only).
```

### SummaryPanel

```text
SummaryPanel [fill, scrollable, copyable, word-wrapped]

    * SummaryText [ui:fixed-width text]
        the analyze response's summary_text: a machine-friendly dense key=value report
        includes:
            operating airspeed / altitude / temperature / chord
            computed Reynolds / Mach / atmosphere properties
            operating-point CL / CD / CM / CL/CD
            transition locations
            positive / negative stall point
            best L/D point
            minimum drag point
            confidence values
        a minimum or maximum the sweep did not reach prints as
        "n/a (no point with confidence >= 0.5)"
```

## Right Pane

```text
RightPane [fill]

    * PlotPanel [fill]
        uPlot 2x2 grid
```

## PlotPanel

```text
PlotPanel [fill]

    - top-left  = "Cd vs Cl" (Drag Polar)
    - top-right = "Cl vs Alpha" (Lift Curve)
    - bottom-left = "Cl/Cd vs Alpha" (Efficiency Curve)
    - bottom-right = "Cm vs Alpha" (Pitching Moment Curve)

Plot axes
    - drag polar:      x = Cl,             y = Cd
    - lift curve:      x = Alpha (deg),    y = Cl
    - efficiency curve: x = Alpha (deg),   y = Cl/Cd
    - moment curve:    x = Alpha (deg),    y = Cm

Plot behavior
    - all selected airfoils are overlaid in different colors (the house series palette, cycled)
    - each airfoil shows an operating-point marker on every plot: at (Cl, Cd) on the drag polar and at
      (the analysis' AoA, value) on the other three
    - high-confidence curve segments (analysis confidence >= 0.5) are solid
    - low-confidence curve segments are faded dashed (alpha 0.3) in the same color; the low mask is widened by
      one point on each side so a dashed stretch meets the solid line, and a null (non-finite) value is a gap
      that keeps separate stretches unjoined
    - the drag polar's x is each airfoil's own Cl, which is not monotonic past stall: its curve is stroked
      directly on the canvas in sweep order by a uPlot draw hook (plugin `hooks.draw` via `u.valToPos`), not
      through the joined plot data, so the post-stall branch keeps its place and equal-Cl points are not
      collapsed; solid/dashed/gap semantics are the same as the other three plots
    - each plot's legend lists the airfoils only; the dashed and marker helper series draw without legend rows
    - every plot autoscales to its data (no fixed ranges); the drag polar's scale ranges come from every finite
      sweep point and operating-point mark and are re-read for each response
    - a failed analyze request (400, e.g. an out-of-domain operating point) puts the server's message in the
      Summary and blanks the plots, so the previous parameters' curves are not shown as the current run
```
