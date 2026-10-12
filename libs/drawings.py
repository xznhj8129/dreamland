"""
Usage:
from libs.drawings import export_drawings
paths = export_drawings(resolved, project.cad_dir)  # cad/cutaway, cad/threeview, cad/xray, each .svg and .png

Blueprint-style drawings of the printable parts (libs/cad.py build_printable_parts), the motors and the boxed parts,
all with hidden-line removal (OpenCascade polygonal HLR on a fine mesh: exact HLR takes minutes for views along the
axes); facet seams between nearly tangent faces are drawn as smooth surface.
- cutaway: from the side, the near half (y < 0) cut away, every part's faces on the centreline plane hatched; parts
  numbered front to back, with a key.
- threeview: side, top and front of the whole aircraft as it looks from outside, with a scale bar.
- xray: from above, ahead and to the left, the printed skin as if transparent (its far side faint), and what is inside
  it (parts, rods and spars, motors) drawn over it, parts labelled.
"""

import math
from pathlib import Path

import numpy as np

from libs.cad import M_TO_MM, Progress, build_motor_solid, build_printable_parts, build_rod_solids
from libs.geometry import ResolvedAirframe

FAR_MM = 5000.0  # half-space and section plane size, bigger than any aircraft
SMOOTH_SEAM_DEG = 20.0
MESH_DEFLECTION_MM = 0.05  # the drawing mesh: far below a line width at drawing scale
MESH_ANGLE_RAD = 0.15  # facet seams flatter than this are drawn as smooth surface, not as lines
CURVE_STEP_MM = 0.5  # polyline step along drawn curves
MIN_EDGE_MM = 1e-6
SECTION_HATCHES = ("////", "\\\\\\\\")  # alternating from one section to the next
KIND_HATCHES = {"coupler": "xxxx", "motor": "------", "component": "++++"}
LINE_WIDTH = 0.7
CUT_LINE_WIDTH = 1.0
HATCH_LINE_WIDTH = 0.5
VIEW_AXES = {  # (towards the viewer, drawing right) in body axes (x aft, y right, z up); up = towards x right
    "SIDE": ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),
    "TOP": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    "FRONT": ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
}
XRAY_TOWARD_VIEWER = (-1.0, -1.0, 1.0)  # above, ahead and to the left
VIEW_GAP_MM = 60.0  # between the three views
SCALE_BAR_MM = 100.0
SKIN_COLOR = "0.5"
SKIN_HIDDEN_COLOR = "0.8"
HIDDEN_LINE_WIDTH = 0.4
INSIDE_LINE_WIDTH = 0.9
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


def _projected_lines(shape, toward, right, hidden: bool = False) -> tuple[list, list]:
    """Sharp edges and silhouettes seen from `toward` with `right` to the right, as (n, 2) arrays in the drawing plane
    (right, up = toward x right): (visible, hidden), hidden empty unless asked for."""
    import cadquery as cq
    from OCP.BRepLib import BRepLib
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.HLRBRep import HLRBRep_PolyAlgo, HLRBRep_PolyHLRToShape

    shape = shape.copy()  # meshed for the drawing only; the parts are shared with the section export
    BRepLib.EncodeRegularity_s(shape.wrapped, math.radians(SMOOTH_SEAM_DEG))
    BRepMesh_IncrementalMesh(shape.wrapped, MESH_DEFLECTION_MM, False, MESH_ANGLE_RAD, True)
    algo = HLRBRep_PolyAlgo()
    algo.Load(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*toward), gp_Dir(*right))))
    algo.Update()
    hlr = HLRBRep_PolyHLRToShape()
    hlr.Update(algo)

    def lines(*compounds) -> list:
        # hidden-line removal leaves zero-length fragments where edges meet; they have no points to draw
        return [_points(edge)[:, :2] if edge.geomType() != "LINE" else np.array([edge.startPoint().toTuple()[:2], edge.endPoint().toTuple()[:2]])
                for compound in compounds if not compound.IsNull()
                for edge in cq.Shape.cast(compound).Edges() if edge.Length() > MIN_EDGE_MM]

    return lines(hlr.VCompound(), hlr.OutLineVCompound()), lines(hlr.HCompound(), hlr.OutLineHCompound()) if hidden else []


def _draw_lines(ax, lines: list, color, width: float, offset=(0.0, 0.0)) -> None:
    from matplotlib.collections import LineCollection

    ax.add_collection(LineCollection([line + offset for line in lines], colors=color, linewidths=width, capstyle="round"))


def _bounds(lines: list) -> tuple[float, float, float, float]:
    points = np.concatenate(lines)
    return points[:, 0].min(), points[:, 0].max(), points[:, 1].min(), points[:, 1].max()


def _compound(shapes):
    import cadquery as cq

    return cq.Compound.makeCompound(list(shapes))


def _save(figure, directory, stem: str) -> list[Path]:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written = [directory / f"{stem}.svg", directory / f"{stem}.png"]
    figure.savefig(written[0], facecolor="white")
    figure.savefig(written[1], dpi=PNG_DPI, facecolor="white")
    return written


def _scale_bar(ax, x: float, y: float) -> None:
    ax.plot([x, x + SCALE_BAR_MM], [y, y], color="black", lw=CUT_LINE_WIDTH)
    for tick in (x, x + SCALE_BAR_MM / 2.0, x + SCALE_BAR_MM):
        ax.plot([tick, tick], [y - 2.0, y + 2.0], color="black", lw=CUT_LINE_WIDTH)
    ax.text(x + SCALE_BAR_MM / 2.0, y - 5.0, f"{SCALE_BAR_MM:g} mm", ha="center", va="top", fontsize=9)


def drawing_parts(resolved: ResolvedAirframe, *, progress: Progress | None = None) -> list[tuple[str, object, str]]:
    """(label, cq.Shape, kind) front to back: printed sections and couplers, motors, parts with library dimensions.

    progress (optional): forwarded to build_printable_parts ("building <name>" per printable part)."""
    import cadquery as cq

    parts = [(name, shape, "coupler" if name.startswith("C") else "section") for name, shape, _notes in build_printable_parts(resolved, progress=progress)]
    parts += [(motor.name, build_motor_solid(motor).val(), "motor") for motor in resolved.motors]
    for part in resolved.airframe.parts:
        if part.size_m is not None:  # its box about the origin, turned roll, pitch then yaw about the body axes, then placed
            size = cq.Vector(*part.size_m) * M_TO_MM
            box = cq.Solid.makeBox(size.x, size.y, size.z, size * -0.5)
            yaw, pitch, roll = part.orientation_deg
            for axis, angle in (((1, 0, 0), roll), ((0, 1, 0), pitch), ((0, 0, 1), yaw)):
                box = box.rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), angle)
            parts.append((part.name, box.translate(cq.Vector(*part.position_m) * M_TO_MM), "component"))
    return sorted(parts, key=lambda item: item[1].BoundingBox().xmin)


def export_drawings(resolved: ResolvedAirframe, directory, *, progress: Progress | None = None) -> list[Path]:
    """Draw the cutaway, the three-view and the x-ray into directory (each .svg and .png); returns the files written.

    progress (optional): "building <name>" per printable part actually built (only on a cache miss), then
    "drawing cutaway"/"drawing threeview"/"drawing xray" (done/total = 3)."""
    parts = drawing_parts(resolved, progress=progress)
    written = []
    for index, (name, draw) in enumerate((("cutaway", _cutaway), ("threeview", _three_view), ("xray", _xray)), start=1):
        if progress is not None:
            progress(f"drawing {name}", index, 3)
        written += draw(resolved, parts, directory)
    return written


def _cutaway(resolved: ResolvedAirframe, parts: list, directory) -> list[Path]:
    import matplotlib
    from matplotlib.figure import Figure
    from matplotlib.patches import PathPatch

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
            _draw_lines(ax, _projected_lines(_far_half(shape), *VIEW_AXES["SIDE"])[0], "black", LINE_WIDTH)
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
        return _save(figure, directory, "cutaway")


def _three_view(resolved: ResolvedAirframe, parts: list, directory) -> list[Path]:
    """Side view, the top view under it and the front view to its right, each as seen from outside."""
    from matplotlib.figure import Figure

    whole = _compound(shape for _name, shape, _kind in parts)
    views = {name: _projected_lines(whole, toward, right)[0] for name, (toward, right) in VIEW_AXES.items()}
    side_x0, side_x1, side_y0, side_y1 = _bounds(views["SIDE"])
    top_x0, top_x1, top_y0, top_y1 = _bounds(views["TOP"])
    front_x0, front_x1, front_y0, front_y1 = _bounds(views["FRONT"])
    shifts = {"SIDE": (0.0, 0.0), "TOP": (0.0, side_y0 - VIEW_GAP_MM - top_y1), "FRONT": (side_x1 + VIEW_GAP_MM - front_x0, 0.0)}
    x_min, x_max = min(side_x0, top_x0), side_x1 + VIEW_GAP_MM + (front_x1 - front_x0)
    y_min, y_max = shifts["TOP"][1] + top_y0, max(side_y1, front_y1)
    margin = LABEL_OFFSET_MM
    height_in = FIGURE_WIDTH_IN * (y_max - y_min + 2.0 * margin) / (x_max - x_min + 2.0 * margin)

    figure = Figure(figsize=(FIGURE_WIDTH_IN, max(height_in, 4.0)), facecolor="white")
    ax = figure.add_axes((0.0, 0.0, 1.0, 1.0))
    for name, lines in views.items():
        dx, dy = shifts[name]
        _draw_lines(ax, lines, "black", LINE_WIDTH, (dx, dy))
        x0, x1, y0, _y1 = _bounds(lines)
        ax.text((x0 + x1) / 2.0 + dx, y0 + dy - margin / 2.0, name, ha="center", va="top", fontsize=11, fontstyle="italic")
    corner_x, corner_y = shifts["FRONT"][0] + front_x0, shifts["TOP"][1] + top_y1  # under the front view, beside the top view
    ax.text(corner_x, corner_y, resolved.airframe.name, fontsize=14, va="top")
    _scale_bar(ax, corner_x, corner_y - 2.0 * margin)
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.set_aspect("equal")
    ax.axis("off")
    return _save(figure, directory, "threeview")


def _xray(resolved: ResolvedAirframe, parts: list, directory) -> list[Path]:
    """From above, ahead and to the left: the printed skin see-through, what is inside it drawn over it."""
    from matplotlib.figure import Figure

    toward = np.array(XRAY_TOWARD_VIEWER) / np.linalg.norm(XRAY_TOWARD_VIEWER)
    right = np.cross((0.0, 0.0, 1.0), toward)
    right /= np.linalg.norm(right)
    up = np.cross(toward, right)
    skin = _compound(shape for _name, shape, kind in parts if kind in ("section", "coupler"))
    inside = [(name, shape, kind) for name, shape, kind in parts if kind in ("motor", "component")]
    inside += [(name, solid, "rod") for name, solid in build_rod_solids(resolved)]
    skin_visible, skin_hidden = _projected_lines(skin, tuple(toward), tuple(right), hidden=True)
    inside_visible = _projected_lines(_compound(shape for _name, shape, _kind in inside), tuple(toward), tuple(right))[0] if inside else []
    x_min, x_max, y_min, y_max = _bounds(skin_visible + inside_visible)
    margin = 2.0 * LABEL_OFFSET_MM
    height_in = FIGURE_WIDTH_IN * (y_max - y_min + 2.0 * margin) / (x_max - x_min + 2.0 * margin)

    figure = Figure(figsize=(FIGURE_WIDTH_IN, max(height_in, 4.0)), facecolor="white")
    ax = figure.add_axes((0.0, 0.0, 1.0, 1.0))
    _draw_lines(ax, skin_hidden, SKIN_HIDDEN_COLOR, HIDDEN_LINE_WIDTH)
    _draw_lines(ax, skin_visible, SKIN_COLOR, LINE_WIDTH)
    _draw_lines(ax, inside_visible, "black", INSIDE_LINE_WIDTH)
    for name, shape, kind in inside:
        if kind == "component":  # labelled from its centre, up and to the right of the drawing
            centre = np.array(shape.Center().toTuple())
            x, y = centre @ right, centre @ up
            ax.annotate(name, xy=(x, y), xytext=(x + LABEL_OFFSET_MM, y + LABEL_OFFSET_MM), fontsize=10, fontstyle="italic",
                        arrowprops={"arrowstyle": "-", "lw": 0.6, "color": "black", "shrinkA": 2, "shrinkB": 0})
            ax.plot([x], [y], "o", ms=2.5, color="black")
    ax.text(x_min, y_min - margin / 2.0, resolved.airframe.name, fontsize=14, va="top")
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.set_aspect("equal")
    ax.axis("off")
    return _save(figure, directory, "xray")
