# CAD View

Solid-geometry export of the assembled airframe.

## Layout

```text
CADView [fill]

    * SolidGeometryExportPanel [top]
        - STEP mode radio buttons
        - propeller-disc checkbox
        - Export STEP button
        - Export STL button
        - status line
```

## Behavior

```text
SolidGeometryExportPanel
    - title = "Solid Geometry Export"
    - STEP mode radio options:
        "Component assembly (named parts, for CAD)"
        "Fused OML (single watertight solid, for CFD)"
    - checkbox: "Include propeller discs (actuator-disc CFD markers)"
    - Export STEP opens a save dialog (default <config name>.step), then
      writes the current analysis geometry in the selected mode
    - Export STL opens a save dialog (default <config name>.stl), then
      writes a fused single mesh
    - status line shows progress, the written file path, or the error text
    - if the boolean fuse fails, the export still writes an unfused compound
      and the status line says so
    - both exports require a computed analysis; otherwise the status line
      asks to recompute the configuration first
```
