"""NACA submerged inlet geometry (libs.airframe.NacaDuct): the guide curves, the flow volume and the placement checks.

The duct lives in the body's own surface coordinates: s runs aft along the skin from the inlet lip, depths are along
the local surface normal, and widths are arc lengths across the section (libs.geometry.skin_normal and
skin_clock_for_arc), so circular, superellipse, lofted and tapered bodies need no special cases. The CAD junction
(libs.cad) and the live 3D preview (webapp.model3d) both sample naca_duct_profile over the duct length; nothing here
constructs kernel geometry.

Guide curves (the FCStd reference model's; these constants are the tuning points):
- naca_ramp_fraction: the floor leaves the lip flush (slope 0) and meets the throat with the mean slope (tangent
  throat_depth_m / ramp_length_m), d/D = t^2 (2 - t); the depth then stays constant through the tunnel.
- naca_plan_fraction: the planform half-width grows from the inlet to the throat as a smooth cubic with zero end
  slope, so the walls carry straight into the tunnel and keep their width there.
"""

from __future__ import annotations

import math

from libs.airframe import Airframe, BOX_SHAPES, NacaDuct
from libs.geometry import Body, DuctInstance, body_section_at, body_surface_point, cross, dot, skin_clock_for_arc, skin_normal, unit

RAMP_SAMPLES = 15  # preview stations across the ramp
TUNNEL_SAMPLES = 5  # preview stations across the straight tunnel
ACROSS_SAMPLES = 9  # preview points across the duct
SKIN_STANDOFF_M = 0.0002  # the preview's skin face sits this far proud of the body, avoiding z-fighting


def naca_ramp_fraction(t: float) -> float:
    """Ramp depth at fraction t of the ramp length, over its throat depth: 0 at the flush lip, 1 at the throat."""
    t = min(max(t, 0.0), 1.0)
    return t * t * (2.0 - t)


def naca_plan_fraction(t: float) -> float:
    """Planform half-width at fraction t of the ramp length, from the inlet (0) to the throat (1)."""
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def naca_duct_profile(duct: NacaDuct, s_m: float) -> tuple[float, float]:
    """(depth below the skin, flow half-width) at s_m aft of the lip along the surface: the ramp curves, then the
    tunnel keeps the throat section straight."""
    fraction = s_m / duct.ramp_length_m
    depth = duct.throat_depth_m * naca_ramp_fraction(fraction)
    inlet_half, throat_half = duct.inlet_width_m / 2.0, duct.throat_width_m / 2.0
    return depth, inlet_half + (throat_half - inlet_half) * naca_plan_fraction(fraction)


def duct_surface_outline(instance: DuctInstance, body: Body, *, s_end: float | None = None) -> tuple[tuple[float, float, float], ...]:
    """The closed curve where the duct's flow volume meets the skin, in the airframe frame: the lip edge, both side
    edges and the end edge. Default is the whole footprint (the recorded 2D views' surface cut shape, and the skin
    opening: ramp and tunnel are one hole); s_end ends it early, and s_end = ramp_length is the ramp's mouth alone.
    Sampled in surface coordinates so it wraps the body."""
    duct = instance.entry
    clock0 = math.radians(instance.clock_deg)
    last = duct.length_m if s_end is None else min(s_end, duct.length_m)
    stations = [duct.ramp_length_m * index / (RAMP_SAMPLES - 1) for index in range(RAMP_SAMPLES)]
    stations += [duct.ramp_length_m + duct.tunnel_length_m * index / (TUNNEL_SAMPLES - 1) for index in range(1, TUNNEL_SAMPLES)]
    stations = [s_m for s_m in stations if s_m < last - 1e-12] + [last]

    def skin(s_m: float, across: float) -> tuple[float, float, float]:
        _depth, half = naca_duct_profile(duct, s_m)
        clock = skin_clock_for_arc(body, duct.x_m + s_m, clock0, across * half)
        point, _radius = body_surface_point(body, duct.x_m + s_m, clock)
        return point

    ring = [skin(0.0, -1.0 + 2.0 * index / (ACROSS_SAMPLES - 1)) for index in range(ACROSS_SAMPLES)]  # lip edge
    ring += [skin(s_m, 1.0) for s_m in stations[1:]]  # right side edge aft
    ring += [skin(last, 1.0 - 2.0 * index / (ACROSS_SAMPLES - 1)) for index in range(ACROSS_SAMPLES)]  # end edge
    ring += [skin(s_m, -1.0) for s_m in reversed(stations[1:-1])]  # left side edge forward
    return tuple(ring)


def _mouth_frame(instance: DuctInstance, body: Body) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """The cookie cutter's frame: (origin, e1, e2, direction). direction is the one cut direction - straight in along
    the inward surface normal at the duct's centre; e1, e2 span the plane the cutter is pressed along."""
    duct = instance.entry
    direction = unit(tuple(-value for value in skin_normal(body, duct.x_m + duct.ramp_length_m / 2.0, math.radians(instance.clock_deg))))
    reference = (1.0, 0.0, 0.0)
    along = dot(reference, direction)
    e1 = unit(tuple(reference[axis] - along * direction[axis] for axis in range(3)))
    e2 = cross(direction, e1)
    ring = duct_surface_outline(instance, body, s_end=duct.ramp_length_m)
    return ring[0], e1, e2, direction


def _signed_area(polygon: list[tuple[float, float]]) -> float:
    return 0.5 * sum(polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
                     - polygon[(index + 1) % len(polygon)][0] * polygon[index][1] for index in range(len(polygon)))


def _point_in_triangle(point: tuple[float, float], a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> bool:
    def side(first, second) -> float:
        return (second[0] - first[0]) * (point[1] - first[1]) - (second[1] - first[1]) * (point[0] - first[0])
    values = (side(a, b), side(b, c), side(c, a))
    return all(value >= 0.0 for value in values) or all(value <= 0.0 for value in values)


def _triangulate(polygon: list[tuple[float, float]]) -> list[tuple[int, int, int]]:
    """Ear clipping for a simple, counter-clockwise polygon: its triangles as index triples into the polygon."""
    remaining = list(range(len(polygon)))
    triangles: list[tuple[int, int, int]] = []
    guard = 0
    while len(remaining) > 3 and guard <= len(polygon) ** 2:
        guard += 1
        clipped = False
        for position in range(len(remaining)):
            previous = remaining[position - 1]
            current = remaining[position]
            following = remaining[(position + 1) % len(remaining)]
            a, b, c = polygon[previous], polygon[current], polygon[following]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 0.0:
                continue  # a reflex corner: not an ear
            if any(_point_in_triangle(polygon[other], a, b, c) for other in remaining if other not in (previous, current, following)):
                continue
            triangles.append((previous, current, following))
            remaining.pop(position)
            clipped = True
            break
        if not clipped:  # degenerate input: fan what is left rather than spin
            break
    if len(remaining) == 3:
        triangles.append(tuple(remaining))
    elif len(remaining) > 3:
        for position in range(1, len(remaining) - 1):
            triangles.append((remaining[0], remaining[position], remaining[position + 1]))
    return triangles


def _polygon_edges(polygon: list[tuple[float, float]]) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """(point, inward normal) per convex polygon edge; the region in front of every edge is the polygon."""
    center = (sum(point[0] for point in polygon) / len(polygon), sum(point[1] for point in polygon) / len(polygon))
    edges = []
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        edge = (second[0] - first[0], second[1] - first[1])
        normal = (-edge[1], edge[0])
        if (center[0] - first[0]) * normal[0] + (center[1] - first[1]) * normal[1] < 0.0:
            normal = (-normal[0], -normal[1])
        edges.append((first, normal))
    return edges


def _clip_polygon(points: list[tuple[float, float]], edges) -> list[tuple[float, float]]:
    """Sutherland-Hodgman: the part of a convex polygon in front of every edge, without repeated vertices."""
    for origin, normal in edges:
        if len(points) < 3:
            return []
        output: list[tuple[float, float]] = []
        for index, current in enumerate(points):
            following = points[(index + 1) % len(points)]
            current_side = (current[0] - origin[0]) * normal[0] + (current[1] - origin[1]) * normal[1]
            following_side = (following[0] - origin[0]) * normal[0] + (following[1] - origin[1]) * normal[1]
            if current_side >= 0.0 and (not output or math.dist(output[-1], current) > 1e-12):
                output.append(current)
            if (current_side > 0.0) != (following_side > 0.0):
                ratio = current_side / (current_side - following_side)
                point = (current[0] + ratio * (following[0] - current[0]), current[1] + ratio * (following[1] - current[1]))
                if not output or math.dist(output[-1], point) > 1e-12:
                    output.append(point)
        if len(output) >= 2 and math.dist(output[0], output[-1]) <= 1e-12:
            output.pop()
        points = output
    return points


def _subtract_convex(polygon: list[tuple[float, float]], edges) -> list[list[tuple[float, float]]]:
    """A convex polygon minus the convex region the edges bound, as convex pieces with area.

    Piece i is the part still inside every earlier edge and outside edge i: a point outside the region violates at
    least one edge, and this assigns it to the first it violates, so the pieces cover the difference exactly. Slivers
    (zero-area pieces from boundary contacts) are dropped: they would otherwise multiply through the chain."""
    pieces = []
    remaining = list(polygon)
    for origin, normal in edges:
        if len(remaining) < 3:
            break
        outside = _clip_polygon(remaining, [(origin, (-normal[0], -normal[1]))])
        if len(outside) >= 3 and abs(_signed_area(outside)) > 1e-10:
            pieces.append(outside)
        remaining = _clip_polygon(remaining, [(origin, normal)])
    return pieces


def clip_body_mouth(instance: DuctInstance, body: Body, positions: list, indices: list[int]) -> list[int]:
    """The body's triangles with the duct's surface cut shape opened: the cookie cutter.

    The surface cut shape (the lip, the ramp's flaring side edges, the tunnel's straight side edges and the end
    edge) is projected onto the plane perpendicular to one cut direction - straight in along the inward normal at
    the duct's centre - and the body's triangles are cut by that closed profile pressed through the skin, so ramp
    and tunnel are one hole. The profile may be any simple shape (the smooth, flaring NACA plan is not convex): it
    is triangulated and subtracted piece by piece, and each new rim vertex is the exact intersection with the skin
    triangle it came from, interpolated in that triangle's plane. Only skin facing the cutter is cut (the same
    projection covers the fuselage's opposite wall, which faces away), so the opposite wall stays. New rim vertices
    are appended to positions."""
    origin, e1, e2, direction = _mouth_frame(instance, body)
    ring = duct_surface_outline(instance, body)

    def offset(point) -> tuple[float, float, float]:
        return tuple(point[axis] - origin[axis] for axis in range(3))

    profile = [(dot(offset(p), e1), dot(offset(p), e2)) for p in ring]
    if _signed_area(profile) < 0.0:
        profile.reverse()
    profile_triangles = [[profile[a], profile[b], profile[c]] for a, b, c in _triangulate(profile)]
    profile_edges = [_polygon_edges(triangle) for triangle in profile_triangles]
    profile_min = (min(point[0] for point in profile), min(point[1] for point in profile))
    profile_max = (max(point[0] for point in profile), max(point[1] for point in profile))
    kept: list[int] = []
    for offset_index in range(0, len(indices), 3):
        triangle = indices[offset_index:offset_index + 3]
        points = [positions[index] for index in triangle]
        outward = cross(tuple(points[1][axis] - points[0][axis] for axis in range(3)),
                        tuple(points[2][axis] - points[0][axis] for axis in range(3)))
        if dot(outward, direction) >= 0.0:
            kept.extend(triangle)  # facing away from the cutter (the far wall, the tunnel roof): never cut
            continue
        flat = [(dot(offset(point), e1), dot(offset(point), e2)) for point in points]
        if (max(point[0] for point in flat) < profile_min[0] or min(point[0] for point in flat) > profile_max[0]
                or max(point[1] for point in flat) < profile_min[1] or min(point[1] for point in flat) > profile_max[1]):
            kept.extend(triangle)
            continue
        pieces = [flat]
        for edges in profile_edges:
            following = []
            for piece in pieces:
                following.extend(_subtract_convex(piece, edges))
            pieces = following
            if not pieces:
                break
        if len(pieces) == 1 and pieces[0] == flat:
            kept.extend(triangle)  # wholly outside the profile: keep the exact original vertices
            continue
        area = ((flat[1][0] - flat[0][0]) * (flat[2][1] - flat[0][1])
                - (flat[1][1] - flat[0][1]) * (flat[2][0] - flat[0][0]))
        if abs(area) < 1e-18:  # a triangle parallel to the cut direction projects to a line: keep it whole
            kept.extend(triangle)
            continue
        for piece in pieces:  # a piece vertex is a corner of the triangle or a rim intersection in its plane
            for step in range(1, len(piece) - 1):
                triple = []
                for vertex in (piece[0], piece[step], piece[step + 1]):
                    corner = next((index for index, point in enumerate(flat) if math.dist(point, vertex) < 1e-12), None)
                    if corner is not None:
                        triple.append(triangle[corner])
                        continue
                    w0 = ((flat[1][0] - vertex[0]) * (flat[2][1] - vertex[1]) - (flat[1][1] - vertex[1]) * (flat[2][0] - vertex[0])) / area
                    w1 = ((flat[2][0] - vertex[0]) * (flat[0][1] - vertex[1]) - (flat[2][1] - vertex[1]) * (flat[0][0] - vertex[0])) / area
                    w2 = 1.0 - w0 - w1
                    positions.append(tuple(w0 * points[0][axis] + w1 * points[1][axis] + w2 * points[2][axis] for axis in range(3)))
                    triple.append(len(positions) - 1)
                kept.extend(triple)
    return kept


def duct_cavity_mesh(instance: DuctInstance, body: Body, *, mouth_open: bool = False) -> tuple[list, list]:
    """The duct's flow volume (ramp and tunnel) as a triangle mesh (positions, indices) in the airframe frame: the
    skin patch, the offset floor and the side walls, sampled in surface coordinates so it wraps the real body. The
    tunnel's end is open - the passage discharges into the interior - and mouth_open also drops the skin patch, so
    the live 3D view shows the scoop through the skin's opening; the printable solid is CAD work (libs.cad)."""
    duct = instance.entry
    clock0 = math.radians(instance.clock_deg)
    stations = [duct.ramp_length_m * index / (RAMP_SAMPLES - 1) for index in range(RAMP_SAMPLES)]
    stations += [duct.ramp_length_m + duct.tunnel_length_m * index / (TUNNEL_SAMPLES - 1) for index in range(1, TUNNEL_SAMPLES)]
    top: list[tuple[float, float, float]] = []
    floor: list[tuple[float, float, float]] = []
    for s_m in stations:
        depth, half = naca_duct_profile(duct, s_m)
        x_m = duct.x_m + s_m
        for index in range(ACROSS_SAMPLES):
            across = -half + 2.0 * half * index / (ACROSS_SAMPLES - 1) if ACROSS_SAMPLES > 1 else 0.0
            clock = skin_clock_for_arc(body, x_m, clock0, across)
            point, _radius = body_surface_point(body, x_m, clock)
            normal = skin_normal(body, x_m, clock)
            top.append(tuple(value + SKIN_STANDOFF_M * direction for value, direction in zip(point, normal)))
            floor.append(tuple(value - depth * direction for value, direction in zip(point, normal)))
    count = len(stations)
    positions = top + floor

    def surface(station: int, index: int) -> int:
        return station * ACROSS_SAMPLES + index

    def bottom(station: int, index: int) -> int:
        return count * ACROSS_SAMPLES + station * ACROSS_SAMPLES + index

    indices: list[int] = []
    for station in range(count - 1):
        for index in range(ACROSS_SAMPLES - 1):
            if not mouth_open:  # the skin patch; open, the rim shows the skin's cut edge instead
                indices.extend((surface(station, index), surface(station + 1, index), surface(station + 1, index + 1),
                                surface(station, index), surface(station + 1, index + 1), surface(station, index + 1)))
            indices.extend((bottom(station, index), bottom(station, index + 1), bottom(station + 1, index + 1),
                            bottom(station, index), bottom(station + 1, index + 1), bottom(station + 1, index)))
        for index in (0, ACROSS_SAMPLES - 1):  # the side walls
            indices.extend((surface(station, index), bottom(station, index), bottom(station + 1, index),
                            surface(station, index), bottom(station + 1, index), surface(station + 1, index)))
    return positions, indices


def _volume_points(volume) -> list[tuple[float, float, float]]:
    """Sample points of a volume's surface: a box's 8 corners, a cylinder's rim at both ends and the middle. Bores
    are the cavity itself and points are markers, so they are not obstacles."""
    if volume.shape in BOX_SHAPES:
        half = tuple(value / 2.0 for value in volume.size_m)
        return [(volume.x_m + sx * half[0], volume.y_m + sy * half[1], volume.z_m + sz * half[2])
                for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    if volume.shape == "cylinder":
        centre = (volume.x_m, volume.y_m, volume.z_m)
        radius, half = volume.diameter_m / 2.0, volume.length_m / 2.0
        axis = {"x": 0, "y": 1, "z": 2}.get(volume.axis, 0)
        others = [index for index in range(3) if index != axis]
        points = []
        for along in (-half, 0.0, half):
            for k in range(8):
                angle = 2.0 * math.pi * k / 8.0
                point = list(centre)
                point[axis] += along
                point[others[0]] += radius * math.cos(angle)
                point[others[1]] += radius * math.sin(angle)
                points.append(tuple(point))
        return points
    return []


def _volume_reaches_duct(instance: DuctInstance, body: Body, x_m: float, y: float, z: float) -> bool:
    """Whether a point of an interior volume lies inside the duct copy's occupied band (the flow path widened by the
    wall), at its station along the duct."""
    duct = instance.entry
    if not duct.x_m <= x_m <= duct.x_m + duct.length_m:
        return False
    depth, half = naca_duct_profile(duct, x_m - duct.x_m)
    reach = depth + duct.wall_m
    clock0 = math.radians(instance.clock_deg)
    angle = math.atan2(z, y)
    try:
        left = skin_clock_for_arc(body, x_m, clock0, -(half + duct.wall_m))
        right = skin_clock_for_arc(body, x_m, clock0, half + duct.wall_m)
    except ValueError:
        return False
    if ((angle - left) % (2.0 * math.pi)) > ((right - left) % (2.0 * math.pi)):
        return False
    _point, skin_radius = body_surface_point(body, x_m, angle)
    return skin_radius - math.hypot(y, z) < reach


def duct_checks(airframe: Airframe, body: Body, ducts: tuple[DuctInstance, ...]) -> list[str]:
    """Placement problems for the saved ducts (every radial copy): each must fit the body and clear the reinforcement
    rods and the interior volumes (the section-cut clearance is a definition error at parse, libs.airframe). These
    are user placement errors the CAD junction would build as defined, so an overlap would be a physical clash."""
    problems: list[str] = []
    for instance in ducts:
        duct = instance.entry
        label = f"duct {instance.instance_id!r}"
        x0, x1 = duct.x_m, duct.x_m + duct.length_m
        if x1 > body.length_m + 1e-9:
            problems.append(f"{label}: runs from x {x0 * 1000:.0f} to {x1 * 1000:.0f} mm, past the body length {body.length_m * 1000:.0f} mm")
            continue
        half_width = duct.throat_width_m / 2.0
        tapered = None
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = x0 + (x1 - x0) * fraction
            section = body_section_at(body, x)
            if min(section.width_m, section.height_m) <= 1e-4:
                tapered = x
                break
            try:
                for side in (-1.0, 1.0):
                    skin_clock_for_arc(body, x, math.radians(instance.clock_deg), side * half_width)
            except ValueError:
                tapered = x
                break
        if tapered is not None:
            problems.append(f"{label}: does not fit the body's cross-section at x {tapered * 1000:.0f} mm (move it onto the body's barrel)")
            continue
        rods = airframe.fuselage.reinforcement
        if rods is not None:
            low, high = max(x0, rods.from_x_m), min(x1, rods.to_x_m)
            if high > low:
                x_mid = (low + high) / 2.0
                try:
                    left = skin_clock_for_arc(body, x_mid, math.radians(instance.clock_deg), -(half_width + rods.pattern.diameter_m))
                    right = skin_clock_for_arc(body, x_mid, math.radians(instance.clock_deg), half_width + rods.pattern.diameter_m)
                except ValueError:
                    left = right = None
                if left is not None:
                    span = (right - left) % (2.0 * math.pi)
                    for rod, (y, z) in enumerate(rods.pattern.positions_m(), start=1):
                        if ((math.atan2(z, y) - left) % (2.0 * math.pi)) <= span:
                            problems.append(f"{label}: the tunnel reaches reinforcement rod {rod} between x {low * 1000:.0f} and {high * 1000:.0f} mm")
        for volume in airframe.volumes:
            points = _volume_points(volume)
            if not points:
                continue
            if volume.shape in BOX_SHAPES:
                half_x = volume.size_m[0] / 2.0
            elif volume.shape == "cylinder" and volume.axis == "x":
                half_x = volume.length_m / 2.0
            else:  # other axes: the sample points are tested where they are
                half_x = 0.0
            low, high = max(x0, volume.x_m - half_x), min(x1, volume.x_m + half_x)
            if high < low:
                continue
            stations: tuple[float | None, ...] = (low, (low + high) / 2.0, high) if half_x > 0.0 else (None,)
            reached = any(_volume_reaches_duct(instance, body, x if station is None else station, y, z)
                          for x, y, z in points for station in stations)
            if reached:
                problems.append(f"{label}: the tunnel reaches volume {volume.name!r} (move one of them)")
    return problems
