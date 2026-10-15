# CAD View

Export the airframe and its printed parts as CAD files, drawings and FreeCAD varsets.

## Layout

```text
CADView [fill, scrollable]

    * Export list [arrange:column, max 900 px wide]
        one row per export:  name | options | [Export]
                                    status (under options and button)

        Whole airframe   - format [ui:dropdown] { STEP, separate parts | STEP, one solid | STL }
                         - Propeller discs [ui:checkbox]
        Printed parts    - format [ui:dropdown] { STEP | STL }
        Drawings
        FreeCAD varsets

    * Footer [arrange:row]
        - CPUs for CAD [ui:number, 1..WORKER_CPUS]
        - Priority: low | lowest [readonly]
```

## Row status

```text
never exported      Not exported yet
running             <step> — <done> of <total>                      e.g. "Building S03 — 3 of 20"
                    [bar]  <m:ss> so far, about <n> min left
                    > Steps (<n>)  [collapsed; m:ss and step, oldest first]
done                Done at <hh:mm>, <n> files [, <n> problems, see Log]  [Open folder]
failed              Failed at <hh:mm> while <step>, see Log
refused             Not started: <reason>                           (no valid airframe; also in the Log)
```

## Behavior

```text
Runs
    - the server owns every run (webapp/cad.py); the view only starts one and shows GET /api/cad/state
    - leaving the tab, reloading or closing the browser does not stop an export; coming back shows it where it is
    - GET /api/cad/state is polled every 400 ms while a run goes, every 2 s otherwise
    - one run at a time: every Export button and option is disabled while an export or Compute Geometry runs
      (Compute Geometry shows in the top bar)
    - Whole airframe shows whichever of its STEP or STL export ran last
    - each row shows its export's last result for the open project only
    - the format, propeller discs and part format choices are remembered per browser

Steps and time
    - a step is each progress message of the CAD worker: "building <part>" (sent once the part is built),
      "writing <part>", "assembling S<nn>", "fusing", "projecting <drawing>", "drawing <drawing>", "writing varsets"
    - the bar and "<done> of <total>" are the latest step's
    - time left: the current phase only (steps with the same verb and total), at its pace so far, once it has 3 steps;
      before that only the time so far shows
    - failures: the traceback is in the Log

Open folder
    - POST /api/cad/open: the server opens the export's folder in the file manager (xdg-open):
      Whole airframe and Drawings cad/, Printed parts cad/sections/, FreeCAD varsets cad/varsets/

CPUs for CAD
    - 1 to WORKER_CPUS (2), applied at once to the CAD worker, also during an export; kept until the server restarts
    - Priority is fixed (low; lowest when the server itself runs at the lowest priority): a process cannot raise its
      priority again once lowered

What each export writes
    - Whole airframe: cad/<project>.step (separate parts: one sub-assembly per fuselage section; one solid: fused)
      or cad/<project>.stl (one fused mesh); if the fuse fails, an unfused compound is written and the result notes it
    - Printed parts: into cad/sections/, every fuselage section (S01.., with attached surface roots and pods), coupler
      (C01.., by cut), detached fixed surface segment (W<surface>_P<panel>_S<segment>), unsplit control (CS<n>) and
      retained hatch piece (H<nn>, with copy/piece suffixes); problems (a root chord past a section end, a part larger
      than the printer bed, a part in several pieces) go to the Log; the file count is the files this export wrote
    - Drawings: cutaway, threeview and xray, each .svg and .png in cad/:
        cutaway: side cutaway (near half removed, cut faces hatched, far half in hidden-line view, parts numbered
          with a key)
        threeview: side, top and front views from outside, an opaque iso view from above, ahead and to the left in the
          fourth corner, name and a 100 mm scale bar under it
        xray: from above, ahead and to the left, the printed skin see-through (grey, far side faint), the parts, rods,
          spars and motors inside drawn over it, parts labelled
    - FreeCAD varsets: one JSON per varset in cad/varsets/ (Airframe, one per surface set, Propulsion, one per motor
      mount)
    - every solid is built once per design and shared by all exports and Compute Geometry: after Compute Geometry, or
      after any export of the same design, the exports only write files
    - access cuts affect fuselage material; installed components stay intact; seams and attachment intersections
      produce notes; every retained hatch piece has a unique filename; overlapping openings remove material once
```
