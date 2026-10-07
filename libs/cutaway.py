"""
Usage:
from libs.cutaway import export_cutaway
paths = export_cutaway(resolved, project.cad_dir)  # cad/cutaway.svg and cad/cutaway.png

Blueprint-style side cutaway of the printable parts (libs/cad.py build_printable_parts), the motors and the boxed parts:
the near half (y < 0) is cut away, every part's faces on the centreline plane are hatched, and the far half is drawn from
the side with hidden-line removal (OpenCascade HLR). Facet seams between nearly tangent faces (polygonal airfoils,
sampled nose and tail profiles) are drawn as smooth surface. Parts are numbered front to back, with a key.
"""

import math
from pathlib import Path

import numpy as np

from libs.cad import M_TO_MM, build_motor_solid, build_printable_parts
from libs.geometry import ResolvedAirframe

FAR_MM = 5000.0  # half-space and section plane size, bigger than any aircraft
SMOOTH_SEAM_DEG = 20.0  # facet seams flatter than this are drawn as smooth surface, not as lines
CURVE_STEP_MM = 0.5  # polyline step along drawn curves
SECTION_HATCHES = ("////", "\\\\\\\\")  # alternating from one section to the next
KIND_HATCHES = {"coupler": "xxxx", "motor": "------", "component": "++++"}
LINE_WIDTH = 0.7
CUT_LINE_WIDTH = 1.0
HATCH_LINE_WIDTH = 0.5
LABEL_OFFSET_MM = 25.0  # leader length beyond the drawing, above (sections) or below (the rest)
FIGURE_WIDTH_IN = 22.0
PNG_DPI = 110


def _points(curve):
    """A cq edge or wire as an (n, 3) array of points."""
    count = max(2, int(curve.Length() / CURVE_STEP_MM) + 1)
    return np.array([(p.x, p.y, p.z) for p in curve.positions(np.linspace(0.0, 1.0, count))])


def _far_half(shape):
    import cadquery as cq

    return shape.intersect(cq.Solid.makeBox(FAR_MM, FAR_MM, FAR_MM, cq.Vector(-FAR_MM / 2.0, 0.0, -FAR_MM / 2.0)))


def _section_faces(shape) -> list:
    import cadquery as cq

    return shape.intersect(cq.Face.makePlane(FAR_MM, FAR_MM, basePnt=(0, 0, 0), dir=(0, 1, 0))).Faces()


def _face_path(face):
    """A cut face as one compound path in (x, z): its outer wire anticlockwise and its holes clockwise (matplotlib fills
    by winding, and OCC wires come in either direction)."""
    from matplotlib.path import Path as MplPath

    vertices, codes = [], []
    for wire, anticlockwise in ((face.outerWire(), True), *((hole, False) for hole in face.innerWires())):
        points = _points(wire)[:, [0, 2]]
        signed_area = (points[:, 0] * np.roll(points[:, 1], -1) - np.roll(points[:, 0], -1) * points[:, 1]).sum()
        if (signed_area > 0.0) != anticlockwise:
            points = points[::-1]
        vertices += [*points, points[0]]
        codes += [MplPath.MOVETO] + [MplPath.LINETO] * (len(points) - 1) + [MplPath.CLOSEPOLY]
    return MplPath(vertices, codes)


def _visible_lines(shape) -> list:
    """Visible sharp edges and silhouettes seen from -y, as (n, 2) arrays in (x, z)."""
    import cadquery as cq
    from OCP.BRepLib import BRepLib
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape

    BRepLib.EncodeRegularity_s(shape.wrapped, math.radians(SMOOTH_SEAM_DEG))
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0))))
    algo.Update()
    algo.Hide()
    hlr = HLRBRep_HLRToShape(algo)
    lines = []
    for compound in (hlr.VCompound(), hlr.OutLineVCompound()):
        if not compound.IsNull():
            lines += [_points(edge)[:, :2] for edge in cq.Shape.cast(compound).Edges()]
    return lines


def drawing_parts(resolved: ResolvedAirframe) -> list[tuple[str, object, str]]:
    """(label, cq.Shape, kind) front to back: printed sections and couplers, motors, parts with library dimensions."""
    import cadquery as cq

    parts = [(name, shape, "coupler" if name.startswith("C") else "section") for name, shape, _notes in build_printable_parts(resolved)]
    parts += [(motor.name, build_motor_solid(motor).val(), "motor") for motor in resolved.motors]
    for part in resolved.airframe.parts:
        if part.size_m is not None:
            size = cq.Vector(*part.size_m) * M_TO_MM
            parts.append((part.name, cq.Solid.makeBox(size.x, size.y, size.z, cq.Vector(*part.position_m) * M_TO_MM - size * 0.5), "component"))
    return sorted(parts, key=lambda item: item[1].BoundingBox().xmin)


def export_cutaway(resolved: ResolvedAirframe, directory) -> list[Path]:
    """Draw the cutaway into directory/cutaway.svg and .png; returns the files written."""
    import matplotlib
    from matplotlib.figure import Figure
    from matplotlib.patches import PathPatch

    parts = drawing_parts(resolved)
    boxes = [shape.BoundingBox() for _name, shape, _kind in parts]
    x_min, x_max = min(box.xmin for box in boxes), max(box.xmax for box in boxes)
    z_min, z_max = min(box.zmin for box in boxes), max(box.zmax for box in boxes)
    height_in = FIGURE_WIDTH_IN * (z_max - z_min + 3.0 * LABEL_OFFSET_MM) / (x_max - x_min + 2.0 * LABEL_OFFSET_MM)
    radius = resolved.airframe.fuselage.section.width_m * M_TO_MM / 2.0

    with matplotlib.rc_context({"hatch.linewidth": HATCH_LINE_WIDTH}):
        figure = Figure(figsize=(FIGURE_WIDTH_IN, max(height_in, 4.0)), facecolor="white")
        ax = figure.add_axes((0.0, 0.0, 1.0, 1.0))
        labels = []
        section_count = 0
        for name, shape, kind in parts:
            for line in _visible_lines(_far_half(shape)):
                ax.plot(line[:, 0], line[:, 1], color="black", lw=LINE_WIDTH, solid_capstyle="round")
            if kind == "section":
                hatch = SECTION_HATCHES[section_count % 2]
                section_count += 1
            else:
                hatch = KIND_HATCHES[kind]
            for face in _section_faces(shape):
                ax.add_patch(PathPatch(_face_path(face), facecolor="white", edgecolor="black", lw=CUT_LINE_WIDTH, hatch=hatch))
            if kind != "motor" or not any(label_kind == "motor" for _label, label_kind, _box in labels):
                labels.append(("motor" if kind == "motor" else name, kind, shape.BoundingBox()))

        for number, (label, kind, box) in enumerate(labels, start=1):
            if kind == "section":  # above, to the top wall
                x, target, text_z = (box.xmin + box.xmax) / 2.0, radius * 0.98, z_max + LABEL_OFFSET_MM
            elif kind == "coupler":  # below, to the bottom of the coupler
                x, target, text_z = (box.xmin + box.xmax) / 2.0, -radius * 0.9, z_min - LABEL_OFFSET_MM
            else:  # below, into the part
                x, target, text_z = box.xmin + (box.xmax - box.xmin) * 0.3, (box.zmin + box.zmax) / 2.0, z_min - LABEL_OFFSET_MM
            ax.annotate(f"{number}", xy=(x, target), xytext=(x, text_z), ha="center", va="center", fontsize=13, fontstyle="italic",
                        arrowprops={"arrowstyle": "-", "lw": 0.6, "color": "black", "shrinkA": 8, "shrinkB": 0})
            ax.plot([x], [target], "o", ms=2.5, color="black")
        ax.plot([x_min - LABEL_OFFSET_MM, x_max + LABEL_OFFSET_MM], [0.0, 0.0], color="black", lw=0.5, dashes=(12, 3, 2, 3))
        key = "   ".join(f"{number} {label}" for number, (label, _kind, _box) in enumerate(labels, start=1))
        ax.text(x_min, z_min - 2.4 * LABEL_OFFSET_MM, f"{resolved.airframe.name}:   {key}", fontsize=10, va="top")
        ax.set_xlim(x_min - LABEL_OFFSET_MM, x_max + LABEL_OFFSET_MM)
        ax.set_ylim(z_min - 2.8 * LABEL_OFFSET_MM, z_max + 1.6 * LABEL_OFFSET_MM)
        ax.set_aspect("equal")
        ax.axis("off")

        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        written = [directory / "cutaway.svg", directory / "cutaway.png"]
        figure.savefig(written[0], facecolor="white")
        figure.savefig(written[1], dpi=PNG_DPI, facecolor="white")
    return written
