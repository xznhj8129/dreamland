"""Triangle meshes and markers for the browser's live Three.js view of the resolved airframe.

The view is built from the same resolved model as every other consumer (libs.geometry.resolve_airframe): one
lightweight triangle mesh per body, wing panel, control surface, volume, placed part, motor, prop, rod and spar,
plus the resolved CG, the CG/NP/Lift markers and the overall bounds. A control surface's mesh carries its hinge
point, hinge axis, servo output and deflection limits so the client can animate it. World frame is the airframe's
(x from the nose, y right, z up);
the client sets its camera up to +z. Plain Python, no numpy and no CAD kernel: the body lofts the resolved body
stations with superellipse_radius, the panels use the exact airfoil sections libs.cad builds its solids from
(webapp.drawing_enrich.airfoil_profile/section_points) as closed shells (the profile loop wraps and the root/tip
fan caps close it, and a panel's control is the rear part of that same skin, partitioned at its span and chord
bounds), everything else is placed from the resolved fields.
A component that cannot be built is skipped, never raised, so one bad item cannot take the endpoint down.
"""

from __future__ import annotations

import math
from typing import Any

from libs.airframe import BATTERY_PART_NAME, BOX_SHAPES
from libs.geometry import (
    PROP_SHAFT_LEN_M, BodyStation, ResolvedAirframe, body_section_at, chord_direction, control_cut_frames, cross, dot,
    superellipse_radius, thickness_direction, unit,
)
from libs.graphics import FEATURE_COLOR, LOFT_COLOR, PROP_SHAFT_DIAM_M, marker_positions, part_corners
from libs.print_features import joint_holes, motor_mount_openings
from webapp.drawing_enrich import airfoil_profile, section_points

BODY_SAMPLES = 32
CYLINDER_SEGMENTS = 16
PROP_SEGMENTS = 32
POD_SEGMENTS = 24
POINT_RADIUS_M = 0.02  # a "point" volume (no size of its own) shows as a small octahedron

BODY_COLOR = "#c0c8d0"
PANEL_COLORS = ("#4fa3ff", "#ff9f43", "#2ecc71", "#e74c3c", "#9b59b6", "#f1c40f")
CONTROL_COLOR = "#cc66ff"
VOLUME_COLOR = "#ffcc33"
PART_COLOR = "#ff9966"
MOTOR_COLOR = "#8a9099"
PROP_COLOR = "#66d9ff"
ROCKET_COLOR = "#ff8a3d"  # rocket motor casings, as libs/graphics.py ROCKET_COLOR
ROD_COLOR = "#ff66aa"  # rods and spars, as libs/graphics.py ROD_COLOR
VOLUME_OPACITY = 0.25
PART_OPACITY = 0.85
PROP_OPACITY = 0.25
SECTION_COLOR = "#33ccff"
SECTION_OPACITY = 0.12
SECTION_PLANE_MARGIN = 1.35
FEATURE_OPACITY = 0.1
MARKER_COLORS = {"cg": "#0000ff", "np": "#ff0000", "lift": "#00ff00"}
MARKER_NAMES = {"cg": "CG", "np": "NP", "lift": "Lift"}

_AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
# Part corners come from libs.graphics.part_corners, ordered (-x,+x) x (-y,+y) x (-z,+z); these quads are wound
# so every face normal points outward.
_BOX_QUADS = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
# Octahedron faces over vertices +x, -x, +y, -y, +z, -z, all outward.
_OCTAHEDRON_FACES = ((0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5))


def build_model3d(resolved: ResolvedAirframe, aero: Any | None) -> dict[str, Any]:
    """The JSON-safe 3D payload: {"meshes": [...], "markers": [...], "cg": [...], "bounds": {...}}.

    aero is the current AeroResult (or None); NP and Lift markers appear only when it is present. Every mesh is a
    flat positions/indices pair in metres and the airframe frame; a control mesh's "control" metadata carries its
    servo and deflection limits (libs.geometry.ControlPlacement) and its hinge point and axis. bounds cover every
    vertex and marker and fall back to the fuselage extents when nothing could be built. cg is the resolved CG in
    the airframe frame (libs.geometry.ResolvedAirframe.cg_m).
    """
    meshes: list[dict[str, Any]] = []
    _try_mesh(meshes, lambda: _mesh("body", "body", *_body_mesh(resolved.body.stations), BODY_COLOR))
    for surface_index, surface_set in enumerate(resolved.surfaces):
        color = PANEL_COLORS[surface_index % len(PANEL_COLORS)]
        definition = surface_set.definition.control
        placements = {control.panel_index: control for control in resolved.controls if control.surface == surface_set.name}
        for panel in surface_set.panels:
            placement = placements.get(panel.index)
            _try_meshes(meshes, lambda panel=panel, color=color, definition=definition, placement=placement: _panel_meshes(panel, color, definition, placement))
    for volume in resolved.airframe.volumes:
        _try_mesh(meshes, lambda volume=volume: _mesh(volume.name, "volume", *_volume_mesh(resolved, volume), VOLUME_COLOR, VOLUME_OPACITY))
    part_names = set()
    for part in resolved.airframe.parts:
        part_names.add(part.name)
        _try_mesh(meshes, lambda part=part: _mesh(part.name, "part", *_part_mesh(part), PART_COLOR, PART_OPACITY))
    propulsion = resolved.airframe.propulsion
    if propulsion is not None and propulsion.electric is not None and BATTERY_PART_NAME not in part_names:  # the parser keeps it in parts; be explicit anyway
        battery = propulsion.electric.battery.placed
        _try_mesh(meshes, lambda battery=battery: _mesh(BATTERY_PART_NAME, "part", *_part_mesh(battery), PART_COLOR, PART_OPACITY))
    for motor in resolved.motors:
        _try_mesh(meshes, lambda motor=motor: _mesh(motor.name, "motor", *_motor_mesh(motor), MOTOR_COLOR))
        if motor.pod_stations:
            _try_mesh(meshes, lambda motor=motor: _mesh(f"pod {motor.name}", "pod", *_pod_mesh(motor), BODY_COLOR))
        _try_mesh(meshes, lambda motor=motor: _mesh(f"prop {motor.name}", "prop", *_prop_mesh(motor), PROP_COLOR, PROP_OPACITY))
    for index, rocket in enumerate(resolved.rockets):  # the casing from the nozzle end forward along the thrust axis
        front = tuple(n + rocket.motor.length_m * t for n, t in zip(rocket.nozzle, rocket.thrust_dir))
        _try_mesh(meshes, lambda index=index, rocket=rocket, front=front: _rocket_mesh(index, rocket, front))
    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        pattern = reinforcement.pattern
        for index, (y, z) in enumerate(pattern.positions_m(), start=1):
            start, end = (reinforcement.from_x_m, y, z), (reinforcement.to_x_m, y, z)
            _try_mesh(meshes, lambda index=index, start=start, end=end: _mesh(
                f"rod {index}", "rod", *_cylinder(start, end, pattern.diameter_m / 2.0), ROD_COLOR,
            ))
    for spar in resolved.spars:
        _try_mesh(meshes, lambda spar=spar: _mesh(
            f"spar {spar.surface}[{spar.panel_index}]", "spar", *_cylinder(spar.start, spar.end, spar.diameter_m / 2.0), ROD_COLOR,
        ))
    for control in resolved.controls:
        if control.rod_diameter_m is not None:
            meshes.append(_mesh(f"pivot rod C{control.number}", "spar", *_cylinder(*control.hinge, control.rod_diameter_m / 2.0), ROD_COLOR))
    markers = _markers(resolved, aero)
    bounds = _bounds(meshes, markers, resolved.body)
    for hole in joint_holes(resolved):
        meshes.append(_mesh(hole.name, "feature", *_opening_mesh(hole), FEATURE_COLOR, FEATURE_OPACITY))
    for opening in motor_mount_openings(resolved):
        meshes.append(_mesh(opening.name, "feature", *_opening_mesh(opening), FEATURE_COLOR, FEATURE_OPACITY))
    for index, cut in enumerate(resolved.airframe.sections.cuts_m):
        meshes.append(_mesh(f"cut {index + 1}", "section", *_section_plane(resolved, cut), SECTION_COLOR, SECTION_OPACITY))
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            for index, cut in enumerate(surface_set.definition.sections.cuts_m, start=1):
                meshes.append(_mesh(f"{surface_set.name} P{panel.index + 1} cut {index}", "section",
                                    *_surface_section_plane(panel, cut / surface_set.definition.span_m), SECTION_COLOR, SECTION_OPACITY))
    for index, guide in enumerate(resolved.airframe.fuselage.loft_stations):
        meshes.append(_mesh(f"guide {index + 1}", "loft", *_section_plane(resolved, guide.x_m), LOFT_COLOR, SECTION_OPACITY))
    cg = [float(value) for value in resolved.cg_m]
    return {"meshes": meshes, "markers": markers, "cg": cg, "bounds": bounds}


def _opening_mesh(opening) -> tuple[list, list]:
    front, back = opening.rings()
    count = len(front)
    positions = [*front, *back]
    indices = []
    for index in range(count):
        following = (index + 1) % count
        indices.extend((index, following, count + index, following, count + following, count + index))
    for index in range(1, count - 1):
        indices.extend((0, index + 1, index, count, count + index, count + index + 1))
    return positions, indices


def _section_plane(resolved: ResolvedAirframe, x_m: float) -> tuple[list, list]:
    """The body's superellipse cross-section at a section cut or loft guide, as a fan of BODY_SAMPLES triangles."""
    station = body_section_at(resolved.body, x_m)
    half_width, half_height = station.width_m / 2.0, station.height_m / 2.0
    positions = [(x_m, 0.0, 0.0)]
    for index in range(BODY_SAMPLES):
        angle = 2.0 * math.pi * index / BODY_SAMPLES
        radius = superellipse_radius(half_width, half_height, station.shape, angle) if half_width > 0.0 and half_height > 0.0 else 0.0
        positions.append((x_m, radius * math.cos(angle), radius * math.sin(angle)))
    indices = [index for step in range(BODY_SAMPLES) for index in (0, 1 + step, 1 + (step + 1) % BODY_SAMPLES)]
    return positions, indices


def _surface_section_plane(panel, fraction: float) -> tuple[list, list]:
    coordinates = airfoil_profile(panel.airfoil)
    low = min(y for _x, y in coordinates)
    high = max(y for _x, y in coordinates)
    margin = (high - low) * (SECTION_PLANE_MARGIN - 1.0) / 2.0
    loop = ((-0.05, low - margin), (1.05, low - margin), (1.05, high + margin), (-0.05, high + margin))
    leading_edge = tuple(a + fraction * (b - a) for a, b in zip(panel.attach_le, panel.tip_le))
    chord = panel.chord_root_m + fraction * (panel.chord_tip_m - panel.chord_root_m)
    return section_points(leading_edge, chord, chord_direction(panel), thickness_direction(panel), loop), [0, 1, 2, 0, 2, 3]


def _mesh(name: str, kind: str, positions, indices, color: str, opacity: float = 1.0) -> dict[str, Any]:
    return {
        "name": name, "kind": kind, "color": color, "opacity": float(opacity),
        "positions": [float(value) for point in positions for value in point], "indices": [int(index) for index in indices],
    }


def _try_mesh(meshes: list[dict[str, Any]], builder) -> None:
    """Build one mesh; a failing or non-finite component is skipped (the endpoint must never go down with it)."""
    try:
        mesh = builder()
    except Exception:
        return
    if all(math.isfinite(value) for value in mesh["positions"]):
        meshes.append(mesh)


def _try_meshes(meshes: list[dict[str, Any]], builder) -> None:
    """Build one panel's mesh list (skin, and control when it has one); a failing or non-finite mesh is skipped."""
    try:
        built = builder()
    except Exception:
        return
    meshes.extend(mesh for mesh in built if all(math.isfinite(value) for value in mesh["positions"]))


def _merge(parts: list[tuple[list, list]]) -> tuple[list, list]:
    positions: list[tuple[float, float, float]] = []
    indices: list[int] = []
    for part_positions, part_indices in parts:
        offset = len(positions)
        positions.extend(part_positions)
        indices.extend(index + offset for index in part_indices)
    return positions, indices


def _basis(direction: tuple[float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Two unit vectors u, v with u x v = direction (a right-handed frame about it)."""
    reference = (1.0, 0.0, 0.0) if abs(direction[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = unit(cross(direction, reference))
    return u, cross(direction, u)


def _cylinder(start, end, radius: float, segments: int = CYLINDER_SEGMENTS) -> tuple[list, list]:
    """A capped cylinder from start to end, wound with outward normals."""
    axis = unit(tuple(b - a for a, b in zip(start, end)))
    u, v = _basis(axis)
    positions: list[tuple[float, float, float]] = []
    for center in (start, end):
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            positions.append(tuple(c + radius * (math.cos(angle) * u_i + math.sin(angle) * v_i) for c, u_i, v_i in zip(center, u, v)))
    start_cap, end_cap = len(positions), len(positions) + 1
    positions.append(tuple(start))
    positions.append(tuple(end))
    indices: list[int] = []
    for index in range(segments):
        a0, a1 = index, (index + 1) % segments
        b0, b1 = segments + index, segments + (index + 1) % segments
        indices.extend((a0, a1, b0, a1, b1, b0))
        indices.extend((start_cap, a1, a0))
        indices.extend((end_cap, b0, b1))
    return positions, indices


def _disc(center, normal, radius: float, segments: int = PROP_SEGMENTS) -> tuple[list, list]:
    """A flat fan disc at center, facing normal."""
    u, v = _basis(normal)
    positions: list[tuple[float, float, float]] = [tuple(center)]
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        positions.append(tuple(c + radius * (math.cos(angle) * u_i + math.sin(angle) * v_i) for c, u_i, v_i in zip(center, u, v)))
    indices = [index for step in range(segments) for index in (0, 1 + step, 1 + (step + 1) % segments)]
    return positions, indices


def _box(corners) -> tuple[list, list]:
    """A closed box over corners ordered as libs.graphics.part_corners returns them."""
    indices = [index for a, b, c, d in _BOX_QUADS for index in (a, b, c, a, c, d)]
    return [tuple(corner) for corner in corners], indices


def _axis_box(center, half) -> tuple[list, list]:
    corners = [tuple(c + s * h for c, s, h in zip(center, (sx, sy, sz), half))
               for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    return _box(corners)


def _octahedron(center, radius: float) -> tuple[list, list]:
    positions = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            offset = [0.0, 0.0, 0.0]
            offset[axis] = sign * radius
            positions.append(tuple(c + o for c, o in zip(center, offset)))
    indices = [index for face in _OCTAHEDRON_FACES for index in face]
    return positions, indices


def _body_mesh(stations: tuple[BodyStation, ...]) -> tuple[list, list]:
    """Loft the body stations into a ring tube with nose/tail cap fans (superellipse sections, 32 samples)."""
    rings = []
    for station in stations:
        half_width, half_height = station.width_m / 2.0, station.height_m / 2.0
        ring = []
        for index in range(BODY_SAMPLES):
            angle = 2.0 * math.pi * index / BODY_SAMPLES
            radius = superellipse_radius(half_width, half_height, station.shape, angle) if half_width > 0.0 and half_height > 0.0 else 0.0
            ring.append((station.x_m, radius * math.cos(angle), radius * math.sin(angle)))
        rings.append(ring)
    positions = [point for ring in rings for point in ring]
    nose_cap, tail_cap = len(positions), len(positions) + 1
    positions.append((stations[0].x_m, 0.0, 0.0))
    positions.append((stations[-1].x_m, 0.0, 0.0))
    indices: list[int] = []
    for first, second in zip(range(len(rings) - 1), range(1, len(rings))):
        for index in range(BODY_SAMPLES):
            a0, a1 = first * BODY_SAMPLES + index, first * BODY_SAMPLES + (index + 1) % BODY_SAMPLES
            b0, b1 = second * BODY_SAMPLES + index, second * BODY_SAMPLES + (index + 1) % BODY_SAMPLES
            indices.extend((a0, a1, b0, a1, b1, b0))
    last = (len(rings) - 1) * BODY_SAMPLES
    for index in range(BODY_SAMPLES):
        a0, a1 = index, (index + 1) % BODY_SAMPLES
        b0, b1 = last + index, last + (index + 1) % BODY_SAMPLES
        indices.extend((nose_cap, a1, a0))
        indices.extend((tail_cap, b0, b1))
    return positions, indices


def _closed_profile(coordinates) -> list[tuple[float, float]]:
    """The section profile as an open polyline with a distinct last point, so the tube's wrap closes it: an explicit
    closing point equal to the first is dropped."""
    points = [(float(x), float(y)) for x, y in coordinates]
    if len(points) > 3 and math.dist(points[0], points[-1]) < 1e-12:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("airfoil profile has fewer than three distinct points")
    return points


def _panel_section(panel, coordinates, direction, thickness, span_fraction: float) -> list[tuple[float, float, float]]:
    """One section of the panel's linear loft: the root/tip leading edge and chord interpolated, then the shared
    webapp.drawing_enrich.section_points construction."""
    leading_edge = tuple(a + (b - a) * span_fraction for a, b in zip(panel.attach_le, panel.tip_le))
    chord_m = panel.chord_root_m + (panel.chord_tip_m - panel.chord_root_m) * span_fraction
    return section_points(leading_edge, chord_m, direction, thickness, coordinates)


def _centroid(points) -> tuple[float, float, float]:
    return tuple(sum(point[axis] for point in points) / len(points) for axis in range(3))


def _split_spans(definition) -> tuple[float, ...]:
    """The panel span fractions to insert sections at for a split control: its stored span_start/span_end."""
    spans = {0.0, 1.0}
    for bound in (definition.span_start, definition.span_end):
        if bound is not None and 0.0 < bound < 1.0:
            spans.add(float(bound))
    return tuple(sorted(spans))


def _control_name(placement) -> str:
    return f"control {placement.number} ({placement.surface}[{placement.panel_index}])"


def _fan_triangles(loop: list[int], indices: list[int]) -> None:
    """Append a convex vertex loop (indices into the shared positions) as a triangle fan; loops under three emit nothing."""
    for step in range(1, len(loop) - 1):
        indices.extend((loop[0], loop[step], loop[step + 1]))


def _clip_triangle(positions: list, triangle: tuple[int, int, int], plane_point, plane_normal,
                   intersections: dict[tuple[int, int], int]) -> tuple[list[int], list[int]]:
    """Split one triangle of `positions` by a plane into the vertex loops on its moving (signed distance >= 0) and
    fixed (<= 0) sides. A crossing edge's intersection vertex is appended once and memoized by its vertex pair, so
    triangles sharing the edge reuse the same vertex (and a vertex on the plane joins both loops): the two skins
    stay closed together with the straight plane line between them, and the caller fan-triangulates the loops."""
    loop = [(index, positions[index]) for index in triangle]
    distances = [dot(tuple(value - origin for value, origin in zip(vertex, plane_point)), plane_normal) for _index, vertex in loop]
    moving: list[int] = []
    fixed: list[int] = []
    for step in range(3):
        index, vertex = loop[step]
        following_index, following = loop[(step + 1) % 3]
        distance, following_distance = distances[step], distances[(step + 1) % 3]
        if distance >= 0.0:
            moving.append(index)
        if distance <= 0.0:
            fixed.append(index)
        if distance * following_distance < 0.0:
            edge = (index, following_index) if index < following_index else (following_index, index)
            if edge not in intersections:
                ratio = distance / (distance - following_distance)
                positions.append(tuple(value + ratio * (other - value) for value, other in zip(vertex, following)))
                intersections[edge] = len(positions) - 1
            moving.append(intersections[edge])
            fixed.append(intersections[edge])
    return moving, fixed


def _panel_meshes(panel, color: str, definition, placement) -> list[dict[str, Any]]:
    """The panel skin as one closed shell, split into a panel mesh and, when the panel carries one, its control mesh.

    The profile loop wraps and the root/tip ends are closed by triangle fans from the section centroids, so the
    shell has no opening. A split panel inserts sections at the control's span bounds and gives each skin triangle
    to the control when its midpoint is aft of chord_start (the airfoil coordinates' x); an align_hinge_vertically
    control clips every in-band triangle at the straight hinge plane (control_cut_frames hinge_point and hinge_dir),
    so the boundary is the straight plane line instead of a triangle-diagonal zigzag. The two meshes then tile the
    same surface with no gap or overlap and the root/tip caps stay with the panel. An all-moving panel's whole
    shell, caps included, is the control and no panel-skin mesh is emitted."""
    coordinates = _closed_profile(airfoil_profile(panel.airfoil))
    direction = chord_direction(panel)
    thickness = thickness_direction(panel)
    kind = definition.kind if definition is not None and placement is not None else None
    spans = _split_spans(definition) if kind == "split" else (0.0, 1.0)
    rings = [_panel_section(panel, coordinates, direction, thickness, span) for span in spans]
    count = len(coordinates)
    positions = [point for ring in rings for point in ring]
    root_cap, tip_cap = len(positions), len(positions) + 1
    positions.append(_centroid(rings[0]))
    positions.append(_centroid(rings[-1]))
    panel_indices: list[int] = []
    control_indices: list[int] = []
    low = definition.span_start if getattr(definition, "span_start", None) is not None else 0.0
    high = definition.span_end if getattr(definition, "span_end", None) is not None else 1.0
    hinge_plane = None
    intersections: dict[tuple[int, int], int] = {}
    if kind == "split" and definition.align_hinge_vertically:
        frames = control_cut_frames(panel, definition)
        hinge_plane = (frames["hinge_point"], frames["hinge_dir"])
    for section in range(len(rings) - 1):
        root_base, tip_base = section * count, (section + 1) * count
        # A split control only owns the span band inside [span_start, span_end]; elsewhere the skin is panel.
        in_band = kind == "split" and low - 1e-9 <= 0.5 * (spans[section] + spans[section + 1]) <= high + 1e-9
        for step in range(count):
            a0, a1 = root_base + step, root_base + (step + 1) % count
            b0, b1 = tip_base + step, tip_base + (step + 1) % count
            if kind == "split" and in_band:
                if hinge_plane is not None:  # the aligned straight hinge: clip every triangle at the shared plane
                    for triangle in ((a0, a1, b0), (a1, b1, b0)):
                        moving, fixed = _clip_triangle(positions, triangle, *hinge_plane, intersections)
                        _fan_triangles(moving, control_indices)
                        _fan_triangles(fixed, panel_indices)
                else:  # the quad's two triangles by their midpoint chord fractions
                    x0, x1 = coordinates[step][0], coordinates[(step + 1) % count][0]
                    (control_indices if (2.0 * x0 + x1) / 3.0 >= definition.chord_start else panel_indices).extend((a0, a1, b0))
                    (control_indices if (2.0 * x1 + x0) / 3.0 >= definition.chord_start else panel_indices).extend((a1, b1, b0))
            elif kind == "all_moving":
                control_indices.extend((a0, a1, b0, a1, b1, b0))
            else:
                panel_indices.extend((a0, a1, b0, a1, b1, b0))
    caps = control_indices if kind == "all_moving" else panel_indices
    tip_base = (len(rings) - 1) * count
    for step in range(count):
        caps.extend((root_cap, (step + 1) % count, step))
        caps.extend((tip_cap, tip_base + step, tip_base + (step + 1) % count))
    meshes = []
    if panel_indices:
        meshes.append(_mesh(f"{panel.surface}[{panel.index}]", "panel", positions, panel_indices, color))
    if control_indices:
        mesh = _mesh(_control_name(placement), "control", positions, control_indices, CONTROL_COLOR)
        mesh["control"] = {
            "number": int(placement.number),
            "servo": int(placement.servo[1:]) - 1,  # "S4" -> the servo<N> log column index
            "min_deg": float(definition.deflection_min_deg),
            "max_deg": float(definition.deflection_max_deg),
            "hinge": [float(value) for value in placement.hinge[0]],
            "axis": [float(value) for value in placement.axis],
        }
        meshes.append(mesh)
    return meshes


def _rocket_mesh(index: int, rocket, front) -> dict[str, Any]:
    """The casing, plus what the flight scene needs for its flame: the rocket<N>_thrust_n log column (resolved rocket
    order, libs.simrun.rocket_columns), the nozzle, the thrust direction and the motor's peak thrust."""
    mesh = _mesh(rocket.name, "motor", *_cylinder(rocket.nozzle, front, rocket.motor.diameter_m / 2.0), ROCKET_COLOR)
    mesh["rocket"] = {
        "number": index + 1,
        "nozzle": [float(value) for value in rocket.nozzle],
        "thrust_dir": [float(value) for value in rocket.thrust_dir],
        "radius_m": float(rocket.motor.diameter_m / 2.0),
        "peak_thrust_n": float(rocket.motor.peak_thrust_n),
    }
    return mesh


def _volume_mesh(resolved: ResolvedAirframe, volume) -> tuple[list, list]:
    center = (volume.x_m, volume.y_m, volume.z_m)
    if volume.shape in BOX_SHAPES:
        return _axis_box(center, tuple(value / 2.0 for value in volume.size_m))
    if volume.shape == "cylinder":
        axis = _AXES[volume.axis]
        half = volume.length_m / 2.0
        return _cylinder(tuple(c - half * a for c, a in zip(center, axis)), tuple(c + half * a for c, a in zip(center, axis)), volume.diameter_m / 2.0)
    if volume.shape == "bore":
        x0, x1 = volume.x_m - volume.length_m / 2.0, volume.x_m + volume.length_m / 2.0
        xs = [x0, *(station.x_m for station in resolved.body.stations if x0 < station.x_m < x1), x1]
        wall = resolved.airframe.fuselage.section.wall_radial_m
        stations = tuple(BodyStation(x, max(section.width_m - 2.0 * wall, 0.0), max(section.height_m - 2.0 * wall, 0.0), section.shape)
                         for x in xs for section in (body_section_at(resolved.body, x),))
        return _body_mesh(stations)
    if volume.shape == "point":
        return _octahedron(center, POINT_RADIUS_M)
    raise ValueError(f"unknown volume shape {volume.shape!r}")


def _part_mesh(part) -> tuple[list, list]:
    if part.size_m is None:
        raise ValueError(f"part {part.name!r} has no size in the library")
    return _box(part_corners(part))


def _motor_mesh(motor) -> tuple[list, list]:
    """The bell as a cylinder along thrust_dir, plus the shaft stub out to the prop when there is room for it."""
    axis = unit(motor.thrust_dir)
    half = motor.bell_length_m / 2.0
    bell_start = tuple(c - half * a for c, a in zip(motor.position, axis))
    bell_end = tuple(c + half * a for c, a in zip(motor.position, axis))
    parts = [_cylinder(bell_start, bell_end, motor.bell_diameter_m / 2.0)]
    if PROP_SHAFT_LEN_M > 0.0:
        face = bell_end if dot(tuple(p - c for p, c in zip(motor.prop_center, motor.position)), axis) >= 0.0 else bell_start
        parts.append(_cylinder(face, motor.prop_center, PROP_SHAFT_DIAM_M / 2.0))
    return _merge(parts)


def _prop_mesh(motor) -> tuple[list, list]:
    """The prop as a flat disc through prop_center, perpendicular to the thrust direction."""
    return _disc(motor.prop_center, unit(motor.thrust_dir), motor.prop_radius_m)


def _pod_mesh(motor) -> tuple[list, list]:
    """The motor's pod as a surface of revolution about the X axis line through the motor centre.

    motor.pod_stations are (x_world_m, radius_m) along that line (libs.geometry.build_motors), one ring of
    POD_SEGMENTS per non-collapsed station with consecutive rings connected; a radius at or below zero is the
    pointed tip and stays a single axis vertex, so the cone closure has no degenerate ring. The first end gets a
    fan cap and the last end gets one whenever it does not collapse, so the shell is closed either way.
    """
    stations = [(float(x), float(radius)) for x, radius in motor.pod_stations]
    if len(stations) < 2:
        raise ValueError("pod needs at least two stations")
    center_y, center_z = motor.position[1], motor.position[2]
    positions: list[tuple[float, float, float]] = []
    rings: list[tuple[int, int]] = []  # (first vertex, vertex count) per station
    for x, radius in stations:
        if radius <= 0.0:
            rings.append((len(positions), 1))
            positions.append((x, center_y, center_z))
            continue
        rings.append((len(positions), POD_SEGMENTS))
        for index in range(POD_SEGMENTS):
            angle = 2.0 * math.pi * index / POD_SEGMENTS
            positions.append((x, center_y + radius * math.cos(angle), center_z + radius * math.sin(angle)))

    def vertex(ring: tuple[int, int], index: int) -> int:
        start, count = ring
        return start + index % count

    indices: list[int] = []
    for first, second in zip(rings, rings[1:]):
        first_count, second_count = first[1], second[1]
        if first_count == 1 and second_count == 1:
            continue  # both stations on the axis: no surface between them
        for index in range(POD_SEGMENTS):
            if first_count == 1:  # cone from the axis point to the second ring
                indices.extend((first[0], vertex(second, index + 1), vertex(second, index)))
            elif second_count == 1:  # cone from the first ring to the axis point
                indices.extend((vertex(first, index), vertex(first, index + 1), second[0]))
            else:
                a0, a1 = vertex(first, index), vertex(first, index + 1)
                b0, b1 = vertex(second, index), vertex(second, index + 1)
                indices.extend((a0, a1, b0, a1, b1, b0))
    if rings[0][1] > 1:  # fan cap on the first (flat) end, facing back along the axis
        cap = len(positions)
        positions.append((stations[0][0], center_y, center_z))
        for index in range(POD_SEGMENTS):
            indices.extend((cap, vertex(rings[0], index + 1), vertex(rings[0], index)))
    if rings[-1][1] > 1:  # fan cap on a last station that ends with a small radius rather than a point
        cap = len(positions)
        positions.append((stations[-1][0], center_y, center_z))
        for index in range(POD_SEGMENTS):
            indices.extend((cap, vertex(rings[-1], index), vertex(rings[-1], index + 1)))
    return positions, indices


def _markers(resolved: ResolvedAirframe, aero: Any | None) -> list[dict[str, Any]]:
    """CG always, NP and Lift only when an aero result is present; all on the centreline (y = z = 0)."""
    positions = marker_positions(resolved, aero)
    markers = []
    for key in ("cg", "np", "lift"):
        if key != "cg" and aero is None:
            continue
        x = positions.get(key)
        if x is None or not math.isfinite(x):
            continue
        markers.append({"name": MARKER_NAMES[key], "position": [float(x), 0.0, 0.0], "color": MARKER_COLORS[key]})
    return markers


def _bounds(meshes: list[dict[str, Any]], markers: list[dict[str, Any]], body) -> dict[str, list[float]]:
    """min/max over every vertex and marker; the fuselage extents when nothing could be built."""
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for positions in (mesh["positions"] for mesh in meshes):
        for index in range(0, len(positions), 3):
            for axis in range(3):
                value = positions[index + axis]
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
    for marker in markers:
        for axis in range(3):
            minimum[axis] = min(minimum[axis], marker["position"][axis])
            maximum[axis] = max(maximum[axis], marker["position"][axis])
    if not (math.isfinite(minimum[0]) and math.isfinite(maximum[0])):
        stations = body.stations
        half_width = max(station.width_m for station in stations) / 2.0
        half_height = max(station.height_m for station in stations) / 2.0
        minimum = [min(station.x_m for station in stations), -half_width, -half_height]
        maximum = [max(station.x_m for station in stations), half_width, half_height]
    return {"min": [float(value) for value in minimum], "max": [float(value) for value in maximum]}
