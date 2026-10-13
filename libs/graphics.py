"""
Usage:
from libs.graphics import draw_canvas_view
metrics = draw_canvas_view(canvas, resolved, aero, "Longitudinal", roll_deg, ui_draw, ui_bg)

Orthographic drawing of a resolved airframe (drawn into a duck-typed canvas: a Tk canvas in the old UI, a recording canvas for the web). Everything drawn comes from the resolved model.
Volumes carry the canvas tags ("drag", "volume:<index>"), wing sets ("drag", "surface:<index>"), a wing set's spars
("drag", "spars:<index>") and section cuts ("drag", "cut:<index>"), so the view can move them along x; metrics["scale"]
converts screen pixels to metres.
"""

import math
from typing import Any

from libs.airframe import BOX_SHAPES
from libs.geometry import PROP_SHAFT_LEN_M, ResolvedAirframe, body_half_extents, body_section_at, cross, deflected_outline, superellipse_radius, unit
from libs.print_features import joint_holes, motor_mount_openings

PROP_SHAFT_DIAM_M = 0.005
MOTOR_BASE_DIAM_FACTOR = 0.5
MOTOR_BASE_LEN_M = 0.005
SECTION_OUTLINE_SAMPLES = 72
ROUNDED_CORNER_SAMPLES = 8
POINT_MASS_MARK_PX = 5
VOLUME_COLOR = "#ffcc33"
PART_COLOR = "#ff9966"
WIRE_COLOR = "#ffff00"
WIRE_DASH = (2, 3)  # dotted
CUT_COLOR = "#33ccff"
LOFT_COLOR = "#ffaa55"
FEATURE_COLOR = "#ff6688"
SPAR_WIDTH_PX = 3
SPAR_STOP_DASH = (6, 3)  # a spar that stops where it meets another; a through spar is solid
CARRY_THROUGH_DASH = (3, 2)
ROD_COLOR = "#ff66aa"
ROD_SECTION_SAMPLES = 20
ROD_SILHOUETTE_DASH = (2, 3)
ROD_CENTRE_DOT_RADIUS_PX = 1.5  # fixed-pixel centre mark at each rod end: a thin rod stays locatable at any zoom
ROD_DASH = (8, 4)
CUT_OVERHANG_PX = 14
CONTROL_COLOR = "#cc66ff"
ROCKET_COLOR = "#ff8a3d"
ROCKET_DASH = (5, 3)  # the casing sits inside the body
CONTROL_LABEL_OFFSET_PX = 10
MARKER_RADIUS_PX = 5
MARKER_COLORS = {"cg": "blue", "np": "red", "lift": "green"}
MARKER_LABELS = {"cg": "CG", "np": "Neutral point", "lift": "Lift @ AoA"}


def longitudinal_transverse(y_value: float, z_value: float, roll_rad: float) -> float:
    return z_value * math.cos(roll_rad) + y_value * math.sin(roll_rad)


def section_projected_half(half_width: float, half_height: float, shape: float, roll_rad: float) -> float:
    """Half extent of a superellipse section seen along the side-view direction at this roll."""
    if half_width <= 0.0 or half_height <= 0.0:
        return 0.0
    extent = 0.0
    for index in range(SECTION_OUTLINE_SAMPLES):
        angle = 2.0 * math.pi * index / SECTION_OUTLINE_SAMPLES
        radius = superellipse_radius(half_width, half_height, shape, angle)
        extent = max(extent, abs(longitudinal_transverse(radius * math.cos(angle), radius * math.sin(angle), roll_rad)))
    return extent


def front_profile_outlines(resolved: ResolvedAirframe, positions: list[float], metrics: dict) -> list[list[float]]:
    """Actual profiles at supplied body positions, in front-view pixels."""
    center_x = metrics["origin_x"] + metrics["pan_x"]
    center_y = metrics["origin_y"] + metrics["pan_y"]
    scale = metrics["scale"]
    outlines = []
    for x in positions:
        station = body_section_at(resolved.body, x)
        outline = []
        for index in range(SECTION_OUTLINE_SAMPLES):
            angle = 2.0 * math.pi * index / SECTION_OUTLINE_SAMPLES
            radius = superellipse_radius(station.width_m / 2.0, station.height_m / 2.0, station.shape, angle)
            outline.extend((center_x + radius * math.cos(angle) * scale, center_y - radius * math.sin(angle) * scale))
        outlines.append(outline)
    return outlines


def panel_outline(panel) -> list[tuple[float, float, float]]:
    """True outline: root edge along the body surface, then tip trailing edge and tip leading edge."""
    return [panel.attach_le, panel.attach_te, panel.tip_te, panel.tip_le]


def arm_point(arm, first_axis, along_first: float, along_arm: float) -> tuple[float, float, float]:
    """A point of a carry-through arm profile: along_first on first_axis (x for the side profile, across for the front
    one) and along_arm along the arm, from its start."""
    return tuple(s + along_first * f + along_arm * d for s, f, d in zip(arm.start, first_axis, arm.direction))


def volume_half_extents(resolved: ResolvedAirframe, volume) -> tuple[float, float, float]:
    if volume.shape in BOX_SHAPES:
        return volume.size_m[0] / 2.0, volume.size_m[1] / 2.0, volume.size_m[2] / 2.0
    if volume.shape == "cylinder":
        radius = volume.diameter_m / 2.0
        return {"x": (volume.length_m / 2.0, radius, radius), "y": (radius, volume.length_m / 2.0, radius), "z": (radius, radius, volume.length_m / 2.0)}[volume.axis]
    if volume.shape == "bore":
        half_w, half_h = body_half_extents(resolved.body, volume.x_m)
        wall = resolved.airframe.fuselage.section.wall_radial_m
        return volume.length_m / 2.0, max(half_w - wall, 0.0), max(half_h - wall, 0.0)
    return 0.0, 0.0, 0.0


def part_corners(part) -> list[tuple[float, float, float]]:
    """The 8 corners of a placed part's turned box, in the body frame."""
    rotation = part.rotation()
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = (sx * part.size_m[0] / 2.0, sy * part.size_m[1] / 2.0, sz * part.size_m[2] / 2.0)
                corners.append(tuple(p + sum(row[k] * local[k] for k in range(3)) for p, row in zip(part.position_m, rotation)))
    return corners


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The convex outline of 2D points, anticlockwise (monotone chain)."""
    points = sorted(set(points))
    if len(points) < 3:
        return points

    def turn(o, a, b) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def rounded_rectangle(x_centre: float, y_centre: float, half_width: float, half_height: float, radius: float) -> list[float]:
    """Screen points (flat x, y list) round a rounded rectangle, closed."""
    points = []
    for corner, (sx, sy) in enumerate(((1, -1), (-1, -1), (-1, 1), (1, 1))):
        cx, cy = x_centre + sx * (half_width - radius), y_centre + sy * (half_height - radius)
        for step in range(ROUNDED_CORNER_SAMPLES + 1):
            angle = math.pi / 2.0 * (corner + step / ROUNDED_CORNER_SAMPLES)
            points.extend((cx + radius * math.cos(angle), cy - radius * math.sin(angle)))
    return points + points[:2]


def draw_projected_rod(canvas, start, end, diameter_m: float, project, color: str, tags=()) -> None:
    """Actual cylinder end circles, dotted silhouette, and a fixed-pixel centre dot at each end, in any orthographic projection."""
    axis = unit(tuple(b - a for a, b in zip(start, end)))
    reference = (1.0, 0.0, 0.0) if abs(axis[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = unit(cross(reference, axis))
    w = cross(axis, u)
    projected = []
    for center in (start, end):
        screen = [project(tuple(c + diameter_m / 2.0 * (math.cos(angle) * uc + math.sin(angle) * wc)
                                for c, uc, wc in zip(center, u, w)))
                  for angle in (2.0 * math.pi * sample / ROD_SECTION_SAMPLES for sample in range(ROD_SECTION_SAMPLES))]
        projected.extend(screen)
        canvas.create_polygon([value for point in screen for value in point], outline=color, fill="", width=2, tags=tags)
    hull = convex_hull(projected)
    canvas.create_polygon([value for point in hull for value in point], outline=color, fill="", width=1, dash=ROD_SILHOUETTE_DASH, tags=tags)
    for center in (start, end):  # a true-scale thin rod can vanish between pixels; mark its ends
        x, y = project(center)
        canvas.create_oval(x - ROD_CENTRE_DOT_RADIUS_PX, y - ROD_CENTRE_DOT_RADIUS_PX,
                           x + ROD_CENTRE_DOT_RADIUS_PX, y + ROD_CENTRE_DOT_RADIUS_PX,
                           fill=color, outline=color, tags=tags)


def draw_surface_cut(canvas, points, project, tags=()) -> None:
    canvas.create_polygon([value for point in points for value in project(point)], outline=CUT_COLOR, fill="", width=2, dash=(6, 3), tags=tags)


def volume_projected_half(resolved: ResolvedAirframe, volume, roll_rad: float) -> float:
    """Half extent of a volume across the side view at this roll: round sections stay round (a bore is the internal
    section, a cylinder's support is half-length x |cos| + radius x sin of its axis to the view's transverse axis)."""
    _half_x, half_y, half_z = volume_half_extents(resolved, volume)
    if volume.shape == "bore":
        return section_projected_half(half_y, half_z, body_section_at(resolved.body, volume.x_m).shape, roll_rad)
    if volume.shape == "cylinder":
        along = {"x": 0.0, "y": math.sin(roll_rad), "z": math.cos(roll_rad)}[volume.axis]
        return volume.length_m / 2.0 * abs(along) + volume.diameter_m / 2.0 * math.sqrt(1.0 - along * along)
    return half_y * abs(math.sin(roll_rad)) + half_z * abs(math.cos(roll_rad))


def marker_positions(resolved: ResolvedAirframe, aero) -> dict:
    positions = {"cg": resolved.cg_m[0], "np": float("nan"), "lift": float("nan")}
    if aero is not None:
        positions["np"] = aero.x_np_m
        positions["lift"] = aero.lift_center_x_m
    return positions


def draw_print_features(canvas: Any, resolved: ResolvedAirframe, project) -> None:
    """Projected negative volumes, visible over the skin so their placement and depth can be edited."""
    for opening in (*joint_holes(resolved), *motor_mount_openings(resolved)):
        rings = opening.rings()
        points = convex_hull([project(point) for ring in rings for point in ring])
        coords = [value for point in points for value in point]
        canvas.create_polygon(coords, outline=FEATURE_COLOR, fill="", width=1.5, tags=("feature",))


def draw_marker_guide(canvas: Any, positions: dict) -> None:
    """Legend; the CG, neutral point and lift centre with their x from the nose (marker_positions)."""
    guide = [(f"{MARKER_LABELS[key]}  x {positions[key] * 1000.0:.1f} mm" if not math.isnan(positions[key]) else f"{MARKER_LABELS[key]}  (aero pending)", color)
             for key, color in MARKER_COLORS.items()]
    for index, (text, color) in enumerate(guide):
        canvas.create_text(12, 12 + index * 16, anchor="nw", text=text, fill=color, font=("TkDefaultFont", 9))


def measure_longitudinal_view(resolved: ResolvedAirframe, roll_deg: float, width: float, height: float) -> dict:
    pad_x = 30.0
    pad_y = 30.0
    roll_rad = math.radians(roll_deg)
    min_x = 0.0
    max_x = resolved.body.length_m
    half_span = max(station.width_m for station in resolved.body.stations) / 2.0
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            for x, y, z in panel_outline(panel):
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                half_span = max(half_span, abs(longitudinal_transverse(y, z, roll_rad)))
    for motor in resolved.motors:
        half_span = max(half_span, abs(longitudinal_transverse(motor.position[1], motor.position[2], roll_rad)) + motor.prop_radius_m)
        xs = [motor.position[0] - motor.bell_length_m, motor.prop_center[0]] + [x for x, _ in motor.pod_stations]
        min_x = min(min_x, *xs)
        max_x = max(max_x, *xs)
    for rocket in resolved.rockets:
        half_span = max(half_span, abs(longitudinal_transverse(rocket.nozzle[1], rocket.nozzle[2], roll_rad)) + rocket.motor.diameter_m / 2.0)
        xs = [rocket.nozzle[0], rocket.nozzle[0] + rocket.motor.length_m * rocket.thrust_dir[0]]
        min_x = min(min_x, *xs)
        max_x = max(max_x, *xs)
    half_span = max(half_span, 0.1)
    content_x_span = max_x - min_x
    scale = min((width - 2.0 * pad_x) / content_x_span, (height - 2.0 * pad_y) / (2.2 * half_span))
    return {
        "mode": "Longitudinal",
        "width": width,
        "height": height,
        "pad_x": pad_x,
        "pad_y": pad_y,
        "origin_x": pad_x - min_x * scale,
        "origin_y": height / 2.0,
        "base_scale": scale,
        "content_width": content_x_span * scale,
        "content_half_height": half_span * scale,
        "roll_rad": roll_rad,
    }


def measure_front_view(resolved: ResolvedAirframe, width: float, height: float) -> dict:
    pad_x = 30.0
    pad_y = 30.0
    extent_y = max(station.width_m for station in resolved.body.stations) / 2.0
    extent_z = max(station.height_m for station in resolved.body.stations) / 2.0
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            extent_y = max(extent_y, abs(panel.root_le[1]), abs(panel.tip_le[1]))
            extent_z = max(extent_z, abs(panel.root_le[2]), abs(panel.tip_le[2]))
    for motor in resolved.motors:
        extent_y = max(extent_y, abs(motor.position[1]) + motor.prop_radius_m)
        extent_z = max(extent_z, abs(motor.position[2]) + motor.prop_radius_m)
    for rocket in resolved.rockets:
        extent_y = max(extent_y, abs(rocket.nozzle[1]) + rocket.motor.diameter_m / 2.0)
        extent_z = max(extent_z, abs(rocket.nozzle[2]) + rocket.motor.diameter_m / 2.0)
    scale = min((width - 2.0 * pad_x) / (2.2 * extent_y), (height - 2.0 * pad_y) / (2.2 * extent_z))
    return {
        "mode": "Front",
        "width": width,
        "height": height,
        "pad_x": pad_x,
        "pad_y": pad_y,
        "origin_x": width / 2.0,
        "origin_y": height / 2.0,
        "base_scale": scale,
        "content_half_width": extent_y * scale,
        "content_half_height": extent_z * scale,
    }


def draw_longitudinal_view(canvas: Any, resolved: ResolvedAirframe, aero, roll_deg: float, ui_draw: str, zoom_scale: float, pan_x: float, pan_y: float,
                           deflections: dict[int, float]) -> dict:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    metrics = measure_longitudinal_view(resolved, roll_deg, width, height)
    scale = metrics["base_scale"] * zoom_scale
    roll_rad = metrics["roll_rad"]
    center_y = metrics["origin_y"] + pan_y

    def x_to_screen(x_value: float) -> float:
        return metrics["origin_x"] + pan_x + x_value * scale

    def transverse_to_screen(transverse_value: float) -> float:
        return center_y - transverse_value * scale

    for surface_index, surface_set in enumerate(resolved.surfaces):
        for panel in surface_set.panels:
            points = []
            for x, y, z in panel_outline(panel):
                points.extend((x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad))))
            canvas.create_polygon(points, outline=ui_draw, fill="", width=2, tags=("drag", f"surface:{surface_index}"))
    surface_indices = {surface_set.name: index for index, surface_set in enumerate(resolved.surfaces)}
    for control in resolved.controls:
        tags = ("drag", f"surface:{surface_indices[control.surface]}")
        outline = deflected_outline(control, deflections.get(control.number, 0.0))
        screen = [(x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad))) for x, y, z in outline]
        canvas.create_polygon([value for point in screen for value in point], outline=CONTROL_COLOR, fill=CONTROL_COLOR, stipple="gray25", width=1, tags=tags)
        hinge = [value for x, y, z in control.hinge for value in (x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad)))]
        canvas.create_line(hinge, fill=CONTROL_COLOR, width=2, dash=(4, 2), tags=tags)
        if control.rod_diameter_m is not None:
            draw_projected_rod(canvas, *control.hinge, control.rod_diameter_m,
                               lambda point: (x_to_screen(point[0]), transverse_to_screen(longitudinal_transverse(point[1], point[2], roll_rad))),
                               ROD_COLOR, (*tags, "hinge-spar"))
        canvas.create_text(sum(x for x, _t in screen) / 4.0, sum(t for _x, t in screen) / 4.0, text=f"C{control.number} {control.servo}",
                           fill=CONTROL_COLOR, font=("TkDefaultFont", 8, "bold"), tags=tags)

    body = resolved.body
    top_points = []
    bottom_points = []
    for station in body.stations:
        half = section_projected_half(station.width_m / 2.0, station.height_m / 2.0, station.shape, roll_rad)
        top_points.extend((x_to_screen(station.x_m), transverse_to_screen(half)))
        bottom_points.extend((x_to_screen(station.x_m), transverse_to_screen(-half)))
    canvas.create_line(top_points, fill=ui_draw, width=2)
    canvas.create_line(bottom_points, fill=ui_draw, width=2)

    for volume_index, volume in enumerate(resolved.airframe.volumes):
        tags = ("drag", f"volume:{volume_index}")
        x_screen = x_to_screen(volume.x_m)
        t_screen = transverse_to_screen(longitudinal_transverse(volume.y_m, volume.z_m, roll_rad))
        if volume.shape == "point":
            mark = POINT_MASS_MARK_PX
            canvas.create_line(x_screen - mark, t_screen - mark, x_screen + mark, t_screen + mark, fill=VOLUME_COLOR, width=2, tags=tags)
            canvas.create_line(x_screen - mark, t_screen + mark, x_screen + mark, t_screen - mark, fill=VOLUME_COLOR, width=2, tags=tags)
            canvas.create_text(x_screen, t_screen - mark - 2, anchor="s", text=volume.name, fill=VOLUME_COLOR, font=("TkDefaultFont", 8), tags=tags)
            continue
        half_x = volume_half_extents(resolved, volume)[0]
        half_t = volume_projected_half(resolved, volume, roll_rad)
        canvas.create_rectangle(x_screen - half_x * scale, t_screen - half_t * scale, x_screen + half_x * scale, t_screen + half_t * scale,
                                outline=VOLUME_COLOR, fill="", width=1, dash=(4, 2), tags=tags)
        canvas.create_text(x_screen - half_x * scale, t_screen - half_t * scale - 2, anchor="sw", text=volume.name, fill=VOLUME_COLOR, font=("TkDefaultFont", 8), tags=tags)

    for part in resolved.airframe.parts:
        x, y, z = part.position_m
        x_screen = x_to_screen(x)
        t_screen = transverse_to_screen(longitudinal_transverse(y, z, roll_rad))
        if part.size_m is None:
            mark = POINT_MASS_MARK_PX
            canvas.create_line(x_screen - mark, t_screen, x_screen + mark, t_screen, fill=PART_COLOR, width=2)
            canvas.create_line(x_screen, t_screen - mark, x_screen, t_screen + mark, fill=PART_COLOR, width=2)
        else:  # the outline of its turned box
            outline = convex_hull([(x_to_screen(cx), transverse_to_screen(longitudinal_transverse(cy, cz, roll_rad))) for cx, cy, cz in part_corners(part)])
            canvas.create_polygon([value for point in outline for value in point], outline=PART_COLOR, fill="", width=1)
        canvas.create_text(x_screen, t_screen + POINT_MASS_MARK_PX + 2, anchor="n", text=part.name, fill=PART_COLOR, font=("TkDefaultFont", 8))

    if resolved.wires is not None:
        for route in (resolved.wires.battery, *resolved.wires.motors):
            points = [value for x, y, z in route for value in (x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad)))]
            canvas.create_line(points, fill=WIRE_COLOR, width=2, dash=WIRE_DASH)

    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        pattern = reinforcement.pattern
        for y, z in pattern.positions_m():
            t_screen = transverse_to_screen(longitudinal_transverse(y, z, roll_rad))
            canvas.create_line(x_to_screen(reinforcement.from_x_m), t_screen, x_to_screen(reinforcement.to_x_m), t_screen, fill=ROD_COLOR, width=1, dash=ROD_DASH)

    for spar in resolved.spars:
        tags = ("drag", f"spars:{surface_indices[spar.surface]}")
        ends = [value for x, y, z in (spar.start, spar.end) for value in (x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad)))]
        canvas.create_line(ends, fill=ROD_COLOR, width=SPAR_WIDTH_PX, dash=() if spar.through else SPAR_STOP_DASH, tags=tags)
    for arm in resolved.carry_through:
        points = [arm_point(arm, (1.0, 0.0, 0.0), u, r) for u, r in arm.side_profile]
        canvas.create_polygon([value for x, y, z in points for value in (x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad)))],
                              outline=ROD_COLOR, fill="", width=1, dash=CARRY_THROUGH_DASH)

    sections = resolved.airframe.sections
    for index, cut in enumerate(sections.cuts_m):
        half_w, half_h = body_half_extents(body, cut)
        half = section_projected_half(half_w, half_h, body_section_at(body, cut).shape, roll_rad) * scale + CUT_OVERHANG_PX
        canvas.create_line(x_to_screen(cut), center_y - half, x_to_screen(cut), center_y + half, fill=CUT_COLOR, width=3, dash=(6, 3), tags=("drag", f"cut:{index}"))
    for index, guide in enumerate(resolved.airframe.fuselage.loft_stations):
        local = body_section_at(body, guide.x_m)
        half = section_projected_half(local.width_m / 2.0, local.height_m / 2.0, local.shape, roll_rad) * scale + CUT_OVERHANG_PX
        canvas.create_line(x_to_screen(guide.x_m), transverse_to_screen(0.0) - half,
                           x_to_screen(guide.x_m), transverse_to_screen(0.0) + half,
                           fill=LOFT_COLOR, dash=(2, 4), tags=("drag", f"loft:{index}"))
    for index, section in enumerate(resolved.sections):
        x0, x1 = section.x0_m, section.x1_m
        computed = "*" if sections.per_section[index].empty_mass_kg is None else ""
        label = f"S{index + 1} {section.empty_kg * 1000.0:.0f} g{computed}\n{(x1 - x0) * 1000.0:.0f} mm{' solid' if section.solid else ''}"
        if section.complete_kg is not None:
            label += f"\n{section.complete_kg * 1000.0:.0f} g all"
        canvas.create_text(x_to_screen((x0 + x1) / 2.0), transverse_to_screen(0.0) + section_projected_half(*body_half_extents(body, (x0 + x1) / 2.0), body_section_at(body, (x0 + x1) / 2.0).shape, roll_rad) * scale + 4,
                           anchor="n", text=label, fill=CUT_COLOR, font=("TkDefaultFont", 8), justify="center")

    for motor in resolved.motors:
        x_screen = x_to_screen(motor.position[0])
        t_screen = transverse_to_screen(longitudinal_transverse(motor.position[1], motor.position[2], roll_rad))
        if motor.pod_stations:
            pod_top = []
            pod_bottom = []
            for pod_x, pod_radius in motor.pod_stations:
                pod_top.extend((x_to_screen(pod_x), t_screen - pod_radius * scale))
                pod_bottom.extend((x_to_screen(pod_x), t_screen + pod_radius * scale))
            canvas.create_line(pod_top, fill=ui_draw, width=2)
            canvas.create_line(pod_bottom, fill=ui_draw, width=2)
        bell_half_len = motor.bell_length_m * scale / 2.0
        bell_half_diam = motor.bell_diameter_m * scale / 2.0
        canvas.create_rectangle(x_screen - bell_half_len, t_screen - bell_half_diam, x_screen + bell_half_len, t_screen + bell_half_diam, outline=ui_draw, width=2)
        prop_side = 1.0 if motor.prop_center[0] > motor.position[0] else -1.0
        shaft_half = PROP_SHAFT_DIAM_M * scale / 2.0
        shaft_inner = x_screen + prop_side * bell_half_len
        shaft_outer = x_screen + prop_side * (bell_half_len + PROP_SHAFT_LEN_M * scale)
        canvas.create_rectangle(min(shaft_inner, shaft_outer), t_screen - shaft_half, max(shaft_inner, shaft_outer), t_screen + shaft_half, outline=ui_draw, width=2)
        base_half = motor.bell_diameter_m * MOTOR_BASE_DIAM_FACTOR * scale / 2.0
        base_inner = x_screen - prop_side * bell_half_len
        base_outer = x_screen - prop_side * (bell_half_len + MOTOR_BASE_LEN_M * scale)
        canvas.create_rectangle(min(base_inner, base_outer), t_screen - base_half, max(base_inner, base_outer), t_screen + base_half, outline=ui_draw, width=2)
        prop_radius = motor.prop_radius_m * scale
        canvas.create_line(shaft_outer, t_screen - prop_radius, shaft_outer, t_screen + prop_radius, fill="red", width=2)
        if abs(motor.thrust_dir[2]) > 1e-9:  # tilted thrust line, across the body length, to check it against the CG
            reach = [(motor.prop_center[0] - x) / -motor.thrust_dir[0] for x in (0.0, body.length_m)]
            ends = [tuple(c + t * d for c, d in zip(motor.prop_center, motor.thrust_dir)) for t in reach]
            canvas.create_line([value for x, y, z in ends for value in (x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad)))],
                               fill="red", width=1, dash=(6, 4))

    for rocket in resolved.rockets:  # the casing outline along its thrust axis, the nozzle end marked
        ends = [rocket.nozzle, tuple(n + rocket.motor.length_m * t for n, t in zip(rocket.nozzle, rocket.thrust_dir))]
        (x0, t0), (x1, t1) = ((x_to_screen(x), transverse_to_screen(longitudinal_transverse(y, z, roll_rad))) for x, y, z in ends)
        length_px = math.hypot(x1 - x0, t1 - t0)
        across = (-(t1 - t0) / length_px * rocket.motor.diameter_m * scale / 2.0, (x1 - x0) / length_px * rocket.motor.diameter_m * scale / 2.0)
        corners = [(x0 + across[0], t0 + across[1]), (x1 + across[0], t1 + across[1]), (x1 - across[0], t1 - across[1]), (x0 - across[0], t0 - across[1])]
        canvas.create_polygon([value for point in corners for value in point], outline=ROCKET_COLOR, fill="", width=2, dash=ROCKET_DASH)
        canvas.create_line(*corners[0], *corners[3], fill=ROCKET_COLOR, width=3)
        canvas.create_text((x0 + x1) / 2.0, (t0 + t1) / 2.0, text=rocket.motor.designation, fill=ROCKET_COLOR, font=("TkDefaultFont", 8))
    draw_print_features(canvas, resolved, lambda point: (x_to_screen(point[0]),
                        transverse_to_screen(longitudinal_transverse(point[1], point[2], roll_rad))))
    for key, x_value in marker_positions(resolved, aero).items():
        if math.isfinite(x_value):
            x_screen = x_to_screen(x_value)
            canvas.create_oval(x_screen - MARKER_RADIUS_PX, center_y - MARKER_RADIUS_PX, x_screen + MARKER_RADIUS_PX, center_y + MARKER_RADIUS_PX, fill=MARKER_COLORS[key], outline="")
    cg_low, cg_high = resolved.cg_x_range_m
    if cg_low != cg_high:
        canvas.create_line(x_to_screen(cg_low), center_y + 2 * MARKER_RADIUS_PX, x_to_screen(cg_high), center_y + 2 * MARKER_RADIUS_PX, fill=MARKER_COLORS["cg"], width=3)
    draw_marker_guide(canvas, marker_positions(resolved, aero))
    metrics["scale"] = scale
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_front_view(canvas: Any, resolved: ResolvedAirframe, aero, ui_draw: str, ui_bg: str, zoom_scale: float, pan_x: float, pan_y: float,
                    deflections: dict[int, float]) -> dict:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    metrics = measure_front_view(resolved, width, height)
    center_x = metrics["origin_x"] + pan_x
    center_y = metrics["origin_y"] + pan_y
    scale = metrics["base_scale"] * zoom_scale
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            canvas.create_line(center_x + panel.root_le[1] * scale, center_y - panel.root_le[2] * scale,
                               center_x + panel.tip_le[1] * scale, center_y - panel.tip_le[2] * scale, fill=ui_draw, width=2)
    body = resolved.body
    outline = []
    for index in range(SECTION_OUTLINE_SAMPLES + 1):
        angle = 2.0 * math.pi * index / SECTION_OUTLINE_SAMPLES
        radius = max(superellipse_radius(station.width_m / 2.0, station.height_m / 2.0, station.shape, angle)
                     for station in body.stations if min(station.width_m, station.height_m) > 0.0)
        outline.extend((center_x + radius * math.cos(angle) * scale, center_y - radius * math.sin(angle) * scale))
    canvas.create_line(outline, fill=ui_draw, width=2)
    for volume in resolved.airframe.volumes:
        if volume.shape == "point":
            continue
        _half_x, half_y, half_z = volume_half_extents(resolved, volume)
        x_screen = center_x + volume.y_m * scale
        y_screen = center_y - volume.z_m * scale
        if volume.shape == "bore":  # the internal section at its centre
            outline = []
            for index in range(SECTION_OUTLINE_SAMPLES + 1):
                angle = 2.0 * math.pi * index / SECTION_OUTLINE_SAMPLES
                radius = superellipse_radius(half_y, half_z, body_section_at(body, volume.x_m).shape, angle)
                outline.extend((x_screen + radius * math.cos(angle) * scale, y_screen - radius * math.sin(angle) * scale))
            canvas.create_line(outline, fill=VOLUME_COLOR, width=1, dash=(4, 2))
        elif volume.shape == "cylinder" and volume.axis == "x":
            canvas.create_oval(x_screen - half_y * scale, y_screen - half_z * scale, x_screen + half_y * scale, y_screen + half_z * scale,
                               outline=VOLUME_COLOR, width=1, dash=(4, 2))
        elif volume.shape == "rounded_box" and volume.axis == "x":  # the rounded section
            canvas.create_line(rounded_rectangle(x_screen, y_screen, half_y * scale, half_z * scale, volume.corner_radius_m * scale), fill=VOLUME_COLOR, width=1, dash=(4, 2))
        else:
            canvas.create_rectangle(x_screen - half_y * scale, y_screen - half_z * scale, x_screen + half_y * scale, y_screen + half_z * scale,
                                    outline=VOLUME_COLOR, fill="", width=1, dash=(4, 2))
    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        pattern = reinforcement.pattern
        rod_radius = pattern.diameter_m / 2.0 * scale
        for y, z in pattern.positions_m():
            x_screen = center_x + y * scale
            y_screen = center_y - z * scale
            canvas.create_oval(x_screen - rod_radius, y_screen - rod_radius, x_screen + rod_radius, y_screen + rod_radius, outline=ROD_COLOR, width=1)
    for part in resolved.airframe.parts:
        _x, y, z = part.position_m
        x_screen, y_screen = center_x + y * scale, center_y - z * scale
        if part.size_m is None:
            mark = POINT_MASS_MARK_PX
            canvas.create_line(x_screen - mark, y_screen, x_screen + mark, y_screen, fill=PART_COLOR, width=2)
            canvas.create_line(x_screen, y_screen - mark, x_screen, y_screen + mark, fill=PART_COLOR, width=2)
        else:  # the outline of its turned box
            outline = convex_hull([(center_x + cy * scale, center_y - cz * scale) for _cx, cy, cz in part_corners(part)])
            canvas.create_polygon([value for point in outline for value in point], outline=PART_COLOR, fill="", width=1)
    if resolved.wires is not None:
        for route in (resolved.wires.battery, *resolved.wires.motors):
            canvas.create_line([value for _x, y, z in route for value in (center_x + y * scale, center_y - z * scale)], fill=WIRE_COLOR, width=2, dash=WIRE_DASH)
    for motor in resolved.motors:
        x_screen = center_x + motor.position[1] * scale
        y_screen = center_y - motor.position[2] * scale
        pod_radius = max((radius for _x, radius in motor.pod_stations), default=0.0)
        for radius, color, fill in ((pod_radius, ui_draw, ui_bg), (motor.bell_diameter_m / 2.0, ui_draw, ui_bg), (PROP_SHAFT_DIAM_M / 2.0, ui_draw, ui_bg), (motor.prop_radius_m, "red", "")):
            if radius > 0.0:
                canvas.create_oval(x_screen - radius * scale, y_screen - radius * scale, x_screen + radius * scale, y_screen + radius * scale, outline=color, fill=fill, width=2)
    for rocket in resolved.rockets:
        x_screen, y_screen, radius = center_x + rocket.nozzle[1] * scale, center_y - rocket.nozzle[2] * scale, rocket.motor.diameter_m / 2.0 * scale
        canvas.create_oval(x_screen - radius, y_screen - radius, x_screen + radius, y_screen + radius, outline=ROCKET_COLOR, fill="", width=2, dash=ROCKET_DASH)
    for spar in resolved.spars:
        canvas.create_line(center_x + spar.start[1] * scale, center_y - spar.start[2] * scale, center_x + spar.end[1] * scale, center_y - spar.end[2] * scale,
                           fill=ROD_COLOR, width=SPAR_WIDTH_PX, dash=() if spar.through else SPAR_STOP_DASH)
    for arm in resolved.carry_through:
        points = [arm_point(arm, arm.across, v, r) for v, r in arm.front_profile]
        canvas.create_polygon([value for _x, y, z in points for value in (center_x + y * scale, center_y - z * scale)],
                              outline=ROD_COLOR, fill="", width=1, dash=CARRY_THROUGH_DASH)
    for control in resolved.controls:
        outline = deflected_outline(control, deflections.get(control.number, 0.0))
        canvas.create_polygon([value for _x, y, z in outline for value in (center_x + y * scale, center_y - z * scale)],
                              outline=CONTROL_COLOR, fill=CONTROL_COLOR, stipple="gray25", width=1)
        (_x0, y0, z0), (_x1, y1, z1) = control.hinge
        canvas.create_line(center_x + y0 * scale, center_y - z0 * scale, center_x + y1 * scale, center_y - z1 * scale, fill=CONTROL_COLOR, width=2, dash=(4, 2))
        if control.rod_diameter_m is not None:
            draw_projected_rod(canvas, *control.hinge, control.rod_diameter_m,
                               lambda point: (center_x + point[1] * scale, center_y - point[2] * scale), ROD_COLOR, ("hinge-spar",))
        length = math.hypot(y1 - y0, z1 - z0)
        canvas.create_text(center_x + y1 * scale + CONTROL_LABEL_OFFSET_PX * (y1 - y0) / length, center_y - z1 * scale - CONTROL_LABEL_OFFSET_PX * (z1 - z0) / length,
                           text=f"C{control.number}", fill=CONTROL_COLOR, font=("TkDefaultFont", 8, "bold"))
    draw_print_features(canvas, resolved, lambda point: (center_x + point[1] * scale, center_y - point[2] * scale))
    draw_marker_guide(canvas, marker_positions(resolved, aero))
    metrics["scale"] = scale
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_half_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_canvas_view(
    canvas: Any,
    resolved: ResolvedAirframe,
    aero,
    view_mode: str,
    roll_deg: float,
    ui_draw: str,
    ui_bg: str,
    zoom_scale: float = 1.0,
    pan_x: float = 0.0,
    pan_y: float = 0.0,
    deflections: dict[int, float] | None = None,
) -> dict:
    """aero is the aero result for this geometry, or None when not (yet) computed. deflections: control number -> deg,
    positive = TE away from the panel normal; controls not listed are drawn neutral."""
    deflections = deflections or {}
    if view_mode == "Longitudinal":
        return draw_longitudinal_view(canvas, resolved, aero, roll_deg, ui_draw, zoom_scale, pan_x, pan_y, deflections)
    return draw_front_view(canvas, resolved, aero, ui_draw, ui_bg, zoom_scale, pan_x, pan_y, deflections)
