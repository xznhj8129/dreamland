# Aero Analysis View

This view provides 2D airfoil comparison using NeuralFoil-backed AeroSandbox airfoil analysis.

## Layout

```text
AeroAnalysisView [fill, split:h]

    * LeftPane [25%]
    * RightPane [75%]
```

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
        shows computed Reynolds number, Mach number, and fixed alpha sweep range

    - AoA operating point [ui:slider + textbox]
        range = -20 deg to +20 deg
        on:change -> refresh plots and summary

    - Airspeed [ui:textbox]
        on:commit -> refresh plots and summary

    - Altitude [ui:textbox]
        on:commit -> refresh plots and summary

    - Temperature [ui:textbox]
        on:commit -> refresh plots and summary

    - Reference chord [ui:textbox]
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

## AirfoilSelectionPanel

```text
AirfoilSelectionPanel [editable]

    - Available list [ui:listbox]
        source = data/airfoils.json plus imported DAT files

    - Selected list [ui:listbox]
        all selected airfoils are plotted together for comparison

    - Add -> [ui:button]
        adds selected available airfoils

    - <- Remove [ui:button]
        removes selected active airfoils

    - Import DAT [ui:button]
        loads a custom DAT file into the available list and selected list

    - Clear [ui:button]
        clears the active comparison list
```

## SummaryPanel

```text
SummaryPanel [fill, scrollable, copyable, word-wrapped]

    * SummaryText [ui:fixed-width text]
        machine-friendly dense report for each selected airfoil
        includes:
            operating airspeed / altitude / temperature / chord
            computed Reynolds / Mach / atmosphere properties
            operating-point CL / CD / CM / CL/CD
            transition locations
            positive / negative stall point
            best L/D point
            minimum drag point
            confidence values
```

## Right Pane

```text
RightPane [fill]

    * PlotPanel [fill]
        matplotlib 2x2 grid
```

## PlotPanel

```text
PlotPanel [fill]

    - top-left  = Cd vs Cl (Drag Polar)
    - top-right = Cl vs Alpha (Lift Curve)
    - bottom-left = Cl/Cd vs Alpha (Efficiency Curve)
    - bottom-right = Cm vs Alpha (Pitching Moment Curve)

Plot behavior
    - all selected airfoils are overlaid in different colors
    - each airfoil shows an operating-point marker on every plot
    - high-confidence curve segments are solid
    - low-confidence curve segments are faded dashed
    - legends identify the selected airfoils
```
