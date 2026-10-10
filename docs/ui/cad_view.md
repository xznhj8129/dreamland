# CAD View

Solid-geometry export of the assembled airframe and of its printable parts, and FreeCAD varsets.

## Layout

```text
CADView [fill]

    * Export cards [two-column grid on wide screens, single column below 1000 px]
        Whole airframe, Printable parts, Drawings, FreeCAD varsets
        - STEP mode radio buttons
        - propeller-disc checkbox
        - Export STEP button
        - Export STL button
        - Export printable sections button
        - Export drawings button
        - Export FreeCAD varsets button
        - status line
```

## Behavior

```text
SolidGeometryExportPanel
    - per-export results show written file links and notes
    - STEP mode radio options:
        "Component assembly (named parts, for CAD)"
        "Fused OML (single watertight solid, for CFD)"
    - checkbox: "Include propeller discs (actuator-disc CFD markers)"
    - Export STEP writes cad/<project>.step in the selected mode
    - Export STL writes cad/<project>.stl as a fused single mesh
    - Export printable sections writes every fuselage section (S01.., with the surfaces and pods
      attached in it) and every coupler (C01.., by cut) as STEP and STL into cad/sections/;
      the status line gives the part count; problems (a root chord past a section end, a part
      larger than the printer bed, a part in several pieces) go to the Log tab
    - Export drawings draws, each as .svg and .png in cad/:
        cutaway: the side cutaway (near half removed, cut faces hatched, far half in hidden-line
          view, parts numbered with a key)
        threeview: side, top and front views as seen from outside, name and a 100 mm scale bar
        xray: from above, ahead and to the left, the printed skin see-through (grey, far side
          faint), the parts, rods, spars and motors inside drawn over it, parts labelled
      the status line gives the paths
    - Export FreeCAD varsets writes one JSON per varset into cad/varsets/ (Airframe, one per
      surface set, Propulsion, one per motor mount); the status line lists them
    - status line shows progress, the written file path, or the error text
    - if the boolean fuse fails, the export still writes an unfused compound
      and the status line says so
    - every export requires a computed analysis; otherwise the status line
      asks to recompute the definition first
```
