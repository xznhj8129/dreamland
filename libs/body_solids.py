"""Low-level fuselage and volume solids of a resolved airframe (headless, below the CAD export adapters).

The primitives libs.cad has always built the airframe from, moved here en bloc so the physical consumers (access cuts,
exact mass) can use the same kernel geometry without importing an exporter: the body lofted or
revolved between two stations, its internal volumes, the section bounds, and the printed section material (the outer
body less the inner cavity or the volumes, exactly as the printable sections are hollowed). Solid geometry is built
in millimetres, as in libs.cad.
"""

import math

from libs.airframe import BOX_SHAPES
from libs.geometry import Body, ResolvedAirframe, body_half_extents, body_section_at, build_body, superellipse_radius
from libs.solid_cache import cached

M_TO_MM = 1000.0
PROFILE_SAMPLES = 48
SECTION_POLYGON_SAMPLES = 72
GEOM_EPS_MM = 1e-4
LOFT_TIP_MM = 0.05  # non-circular bodies loft to a tiny section instead of a point
CUT_OVERSHOOT_MM = 0.5  # mm: holes and bores reach past the material, so no skin is left


def _revolve_profile_solid(stations: list[tuple[float, float]]):
    """Revolve an (x_mm, radius_mm) station polyline around the X axis into a solid.

    Curved sections are represented by dense station sampling rather than
    splines: an interpolating spline through a pointed-tip profile can
    overshoot to negative radius and self-intersect the revolve, and the
    polyline volume is closer to analytic anyway at PROFILE_SAMPLES density.
    The profile is closed against the axis at both ends.
    """
    import cadquery as cq

    points: list[tuple[float, float]] = []
    for pt in stations:
        if points and math.hypot(pt[0] - points[-1][0], pt[1] - points[-1][1]) < GEOM_EPS_MM:
            continue
        # A station on the straight line through its neighbours adds only another face on the same cone or cylinder;
        # OCCT's clean then merges those faces, which crashed on a straight cone nose (faces meeting at its tip).
        if len(points) >= 2:
            (ax, ar), (bx, br) = points[-2], points[-1]
            if abs((bx - ax) * (pt[1] - br) - (br - ar) * (pt[0] - bx)) <= GEOM_EPS_MM * math.hypot(pt[0] - ax, pt[1] - ar):
                points[-1] = pt
                continue
        points.append(pt)
    if len(points) < 2:
        raise RuntimeError("Revolve profile needs at least two distinct stations")

    first = points[0]
    last = points[-1]
    workplane = cq.Workplane("XY").moveTo(*first)
    for pt in points[1:]:
        workplane = workplane.lineTo(*pt)
    if last[1] > GEOM_EPS_MM:
        workplane = workplane.lineTo(last[0], 0.0)
    if first[1] > GEOM_EPS_MM:
        workplane = workplane.lineTo(first[0], 0.0)
    return workplane.close().revolve(360.0, (0, 0, 0), (1, 0, 0))


def _clipped_stations(body: Body, x0: float, x1: float) -> list[tuple[float, float, float, float]]:
    """(x, half_width, half_height, shape) from x0 to x1, with interpolated end stations."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    return [(x, *body_half_extents(body, x), body_section_at(body, x).shape) for x in xs]


def _superellipse_wire(x_mm: float, half_width_mm: float, half_height_mm: float, shape: float):
    """A section of the body's shape at station x as one closed periodic spline through SECTION_POLYGON_SAMPLES points, so
    a loft between sections is one face per segment rather than one per sample (which made booleans crawl)."""
    import cadquery as cq

    half_width_mm = max(half_width_mm, LOFT_TIP_MM)
    half_height_mm = max(half_height_mm, LOFT_TIP_MM)
    points = []
    for index in range(SECTION_POLYGON_SAMPLES):
        angle = 2.0 * math.pi * index / SECTION_POLYGON_SAMPLES
        radius = superellipse_radius(half_width_mm, half_height_mm, shape, angle)
        points.append(cq.Vector(x_mm, radius * math.cos(angle), radius * math.sin(angle)))
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(points, periodic=True)])


def build_body_solid(resolved: ResolvedAirframe, x0: float, x1: float, inset_m: float = 0.0):
    """Fuselage between two stations, capped flat at the ends: revolved when circular, lofted otherwise. inset_m shrinks
    it radially (the bore inside a wall of inset_m), down to one point station where the body is thinner than that;
    None when nothing is left. Built once per design and span (libs.solid_cache)."""
    return cached(resolved, ("body", x0, x1, inset_m), lambda: _build_body_solid(resolved, x0, x1, inset_m))


def _build_body_solid(resolved: ResolvedAirframe, x0: float, x1: float, inset_m: float):
    import cadquery as cq

    body = build_body(resolved.airframe, PROFILE_SAMPLES)
    stations = [(x, max(half_w - inset_m, 0.0), max(half_h - inset_m, 0.0), shape) for x, half_w, half_h, shape in _clipped_stations(body, x0, x1)]
    solid = [index for index, (_x, half_w, half_h, _shape) in enumerate(stations) if min(half_w, half_h) > 0.0]
    if not solid:
        return None
    stations = stations[max(solid[0] - 1, 0): solid[-1] + 2]  # one point station either side of the part with a section
    circular = all(shape == 2.0 and abs(half_w - half_h) < 1e-12 for _x, half_w, half_h, shape in stations)
    if circular:
        return _revolve_profile_solid([(x * M_TO_MM, half_w * M_TO_MM) for x, half_w, _half_h, _shape in stations])
    wires = [_superellipse_wire(x * M_TO_MM, half_w * M_TO_MM, half_h * M_TO_MM, shape) for x, half_w, half_h, shape in stations]
    return cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=True)])


def _prism(x0_m: float, x1_m: float, half_width_m: float, half_height_m: float, shape: float):
    """A straight piece of the body's cross-section shape along x (a cylinder when circular), as a cq.Solid."""
    import cadquery as cq

    x0_mm, x1_mm = x0_m * M_TO_MM, x1_m * M_TO_MM
    if shape == 2.0 and abs(half_width_m - half_height_m) < 1e-12:
        return cq.Solid.makeCylinder(half_width_m * M_TO_MM, x1_mm - x0_mm, cq.Vector(x0_mm, 0, 0), cq.Vector(1, 0, 0))
    wires = [_superellipse_wire(x, half_width_m * M_TO_MM, half_height_m * M_TO_MM, shape) for x in (x0_mm, x1_mm)]
    return cq.Solid.makeLoft(wires, ruled=True)


def _taper(x0_m: float, x1_m: float, half0: tuple[float, float], half1: tuple[float, float], shape: float):
    """A straight taper of the body's cross-section shape along x, half extents half0 at x0_m to half1 at x1_m (a cone
    when circular), as a cq.Solid."""
    import cadquery as cq

    x0_mm, x1_mm = x0_m * M_TO_MM, x1_m * M_TO_MM
    if shape == 2.0 and abs(half0[0] - half0[1]) < 1e-12 and abs(half1[0] - half1[1]) < 1e-12:
        return cq.Solid.makeCone(half0[0] * M_TO_MM, half1[0] * M_TO_MM, x1_mm - x0_mm, cq.Vector(x0_mm, 0, 0), cq.Vector(1, 0, 0))
    wires = [_superellipse_wire(x, half[0] * M_TO_MM, half[1] * M_TO_MM, shape) for x, half in ((x0_mm, half0), (x1_mm, half1))]
    return cq.Solid.makeLoft(wires, ruled=True)


def _ring(x0_m: float, x1_m: float, outer_half: tuple[float, float], inner_half: tuple[float, float], shape: float):
    return _prism(x0_m, x1_m, *outer_half, shape).cut(_prism(x0_m, x1_m, *inner_half, shape))


def build_volume_solid(resolved: ResolvedAirframe, volume):
    """A volume's space as a cq.Solid (box, rounded box, cylinder, or the bore over its length)."""
    import cadquery as cq

    centre = cq.Vector(volume.x_m, volume.y_m, volume.z_m) * M_TO_MM
    if volume.shape in BOX_SHAPES:
        size = cq.Vector(*volume.size_m) * M_TO_MM
        box = cq.Solid.makeBox(size.x, size.y, size.z, centre - size * 0.5)
        if volume.shape == "box":
            return box
        return cq.Workplane("XY").add(box).edges(f"|{volume.axis.upper()}").fillet(volume.corner_radius_m * M_TO_MM).val()
    if volume.shape == "cylinder":
        axis = cq.Vector(*(1.0 if name == volume.axis else 0.0 for name in "xyz"))
        return cq.Solid.makeCylinder(volume.diameter_m / 2.0 * M_TO_MM, volume.length_m * M_TO_MM, centre - axis * (volume.length_m / 2.0 * M_TO_MM), axis)
    if volume.shape == "bore":
        bore = build_body_solid(resolved, volume.x_m - volume.length_m / 2.0, volume.x_m + volume.length_m / 2.0,
                                inset_m=resolved.airframe.fuselage.section.wall_radial_m)
        if bore is None:
            raise ValueError(f"volume {volume.name!r}: no fuselage interior over its length")
        return bore.val()
    raise ValueError(f"volume {volume.name!r}: a {volume.shape} has no solid")


def section_bounds(resolved: ResolvedAirframe) -> list[tuple[float, float]]:
    bounds = [0.0, *resolved.airframe.sections.cuts_m, resolved.body.length_m]
    return list(zip(bounds, bounds[1:]))


def section_cavity(resolved: ResolvedAirframe, index: int, x0_m: float, x1_m: float) -> list:
    """The cutters that hollow a section (cq.Shape, mm): its bore for a hollow section (past both ends by
    CUT_OVERSHOOT_MM), the internal volumes for a solid one; [] when nothing is cut."""
    airframe = resolved.airframe
    if not airframe.sections.per_section[index].solid:
        bore = build_body_solid(resolved, x0_m - CUT_OVERSHOOT_MM / M_TO_MM, x1_m + CUT_OVERSHOOT_MM / M_TO_MM,
                                inset_m=airframe.fuselage.section.wall_radial_m)
        return [] if bore is None else [bore.val()]
    return [build_volume_solid(resolved, volume) for volume in airframe.volumes if volume.shape != "point"]


def hollowed_section(shape, resolved: ResolvedAirframe, index: int, x0_m: float, x1_m: float):
    """A section solid hollowed as the printable section is (section_cavity)."""
    for cutter in section_cavity(resolved, index, x0_m, x1_m):
        shape = shape.cut(cutter)
    return shape


def printed_section_material(resolved: ResolvedAirframe, index: int):
    """The printed fuselage material of one section (cq.Shape, mm): its outer body less the cavity or volumes, the
    same material the printable section is made of. No attachments are fused in. Built once per design."""
    x0, x1 = section_bounds(resolved)[index]
    return cached(resolved, ("section material", index), lambda: hollowed_section(build_body_solid(resolved, x0, x1).val(), resolved, index, x0, x1))
