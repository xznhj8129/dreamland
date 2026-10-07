# CAD View

Solid-geometry export of the assembled airframe and of its printable parts, and FreeCAD varsets.

## Layout

```text
CADView [fill]

    * SolidGeometryExportPanel [top]
        - STEP mode radio buttons
        - propeller-disc checkbox
        - Export STEP button
        - Export STL button
        - Export printable sections button
        - Export cutaway drawing button
        - Export FreeCAD varsets button
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
    - Export STEP opens a save dialog (in the project's cad/, default <project>.step), then
      writes the current analysis geometry in the selected mode
    - Export STL opens a save dialog (in cad/, default <project>.stl), then
      writes a fused single mesh
    - Export printable sections writes every fuselage section (S01.., with the surfaces and pods
      attached in it) and every coupler (C01.., by cut) as STEP and STL into cad/sections/;
      the status line gives the part count; problems (a root chord past a section end, a part
      larger than the printer bed, a part in several pieces) go to the Log tab
    - Export cutaway drawing draws the side cutaway (near half removed, cut faces hatched, far half
      in hidden-line view, parts numbered with a key) into cad/cutaway.svg and cad/cutaway.png;
      the status line gives the paths
    - Export FreeCAD varsets writes one JSON per varset into cad/varsets/ (Airframe, one per
      surface set, Propulsion, one per motor mount); the status line lists them
    - status line shows progress, the written file path, or the error text
    - if the boolean fuse fails, the export still writes an unfused compound
      and the status line says so
    - every export requires a computed analysis; otherwise the status line
      asks to recompute the configuration first
```
