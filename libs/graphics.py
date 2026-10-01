"""
Usage:
from libs.graphics import draw_canvas_view
metrics = draw_canvas_view(canvas, resolved, aero, detailed, "Longitudinal", roll_deg, ui_draw, ui_bg)

Orthographic Tk drawing of a resolved airframe (libs/geometry.py). Everything drawn comes from the resolved model.
Volumes carry the canvas tags ("drag", "volume:<index>"), wing sets ("drag", "surface:<index>") and section cuts
("drag", "cut:<index>"), so the view can move them along x; metrics["scale"] converts screen pixels to metres.
"""

import math
import tkinter as tk

from libs.geometry import PROP_SHAFT_LEN_M, ResolvedAirframe, body_half_extents, superellipse_radius

PROP_SHAFT_DIAM_M = 0.005
MOTOR_BASE_DIAM_FACTOR = 0.5
MOTOR_BASE_LEN_M = 0.005
SECTION_OUTLINE_SAMPLES = 72
POINT_MASS_MARK_PX = 5
VOLUME_COLOR = "#ffcc33"
CUT_COLOR = "#33ccff"
CUT_OVERHANG_PX = 14
MARKER_RADIUS_PX = 5
MARKER_COLORS = {"cg": "blue", "np": "red", "lift": "green", "np_detailed": "orange"}
MARKER_LABELS = {"cg": "CG", "np": "Neutral point (fast)", "lift": "Lift @ AoA", "np_detailed": "Neutral point (detailed)"}


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


def chord_vector(panel, chord: float) -> tuple[float, float, float]:
    incidence = math.radians(panel.incidence_deg)
    return tuple(chord * (math.cos(incidence) * (1.0 if axis == 0 else 0.0) - math.sin(incidence) * panel.normal[axis]) for axis in range(3))


def panel_outline(panel) -> list[tuple[float, float, float]]:
    """True outline: root edge along the body surface, then tip trailing edge and tip leading edge."""
    tip_te = tuple(a + b for a, b in zip(panel.tip_le, chord_vector(panel, panel.chord_tip_m)))
    return [panel.attach_le, panel.attach_te, tip_te, panel.tip_le]


def volume_half_extents(resolved: ResolvedAirframe, volume) -> tuple[float, float, float]:
    if volume.shape == "box":
        return volume.size_m[0] / 2.0, volume.size_m[1] / 2.0, volume.size_m[2] / 2.0
    if volume.shape == "cylinder":
        radius = volume.diameter_m / 2.0
        return {"x": (volume.length_m / 2.0, radius, radius), "y": (radius, volume.length_m / 2.0, radius), "z": (radius, radius, volume.length_m / 2.0)}[volume.axis]
    if volume.shape == "bore":
        half_w, half_h = body_half_extents(resolved.body, volume.x_m)
        wall = resolved.airframe.fuselage.section.wall_radial_m
        return volume.length_m / 2.0, max(half_w - wall, 0.0), max(half_h - wall, 0.0)
    return 0.0, 0.0, 0.0


def marker_positions(resolved: ResolvedAirframe, aero, detailed) -> dict:
    positions = {"cg": resolved.cg_m[0], "np": float("nan"), "lift": float("nan"), "np_detailed": float("nan")}
    if aero is not None:
        positions["np"] = aero.x_np_m
        positions["lift"] = aero.lift_center_x_m
    if detailed is not None:
        positions["np_detailed"] = detailed.x_np_m
    return positions


def draw_marker_guide(canvas: tk.Canvas) -> None:
    for index, key in enumerate(("cg", "np", "np_detailed", "lift")):
        canvas.create_text(12, 12 + index * 16, anchor="nw", text=MARKER_LABELS[key], fill=MARKER_COLORS[key], font=("TkDefaultFont", 9))
    canvas.create_text(12, 12 + 4 * 16, anchor="nw", text="Volumes (drag along x)", fill=VOLUME_COLOR, font=("TkDefaultFont", 9))
    canvas.create_text(12, 12 + 5 * 16, anchor="nw", text="Section cuts (drag along x); * = computed mass", fill=CUT_COLOR, font=("TkDefaultFont", 9))


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


def draw_longitudinal_view(canvas: tk.Canvas, resolved: ResolvedAirframe, aero, detailed, roll_deg: float, ui_draw: str, zoom_scale: float, pan_x: float, pan_y: float) -> dict:
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

    body = resolved.body
    top_points = []
    bottom_points = []
    for station in body.stations:
        half = section_projected_half(station.width_m / 2.0, station.height_m / 2.0, body.shape, roll_rad)
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
        half_x, half_y, half_z = volume_half_extents(resolved, volume)
        half_t = half_y * abs(math.sin(roll_rad)) + half_z * abs(math.cos(roll_rad))
        canvas.create_rectangle(x_screen - half_x * scale, t_screen - half_t * scale, x_screen + half_x * scale, t_screen + half_t * scale,
                                outline=VOLUME_COLOR, fill="", width=1, dash=(4, 2), tags=tags)
        canvas.create_text(x_screen, t_screen, text=volume.name, fill=VOLUME_COLOR, font=("TkDefaultFont", 8), tags=tags)

    sections = resolved.airframe.sections
    bounds = [0.0, *sections.cuts_m, body.length_m]
    section_points = [point for point in resolved.mass_points if point.name.startswith("section ")]
    for index, cut in enumerate(sections.cuts_m):
        half_w, half_h = body_half_extents(body, cut)
        half = section_projected_half(half_w, half_h, body.shape, roll_rad) * scale + CUT_OVERHANG_PX
        canvas.create_line(x_to_screen(cut), center_y - half, x_to_screen(cut), center_y + half, fill=CUT_COLOR, width=3, dash=(6, 3), tags=("drag", f"cut:{index}"))
    for index, (x0, x1) in enumerate(zip(bounds, bounds[1:])):
        mass_g = section_points[index].mass_kg[0] * 1000.0
        label = f"S{index + 1} {mass_g:.0f} g{'*' if section_points[index].computed else ''}"
        canvas.create_text(x_to_screen((x0 + x1) / 2.0), transverse_to_screen(0.0) + section_projected_half(*body_half_extents(body, (x0 + x1) / 2.0), body.shape, roll_rad) * scale + 4,
                           anchor="n", text=label, fill=CUT_COLOR, font=("TkDefaultFont", 8))

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

    for key, x_value in marker_positions(resolved, aero, detailed).items():
        if math.isfinite(x_value):
            x_screen = x_to_screen(x_value)
            canvas.create_oval(x_screen - MARKER_RADIUS_PX, center_y - MARKER_RADIUS_PX, x_screen + MARKER_RADIUS_PX, center_y + MARKER_RADIUS_PX, fill=MARKER_COLORS[key], outline="")
    cg_low, cg_high = resolved.cg_x_range_m
    if cg_low != cg_high:
        canvas.create_line(x_to_screen(cg_low), center_y + 2 * MARKER_RADIUS_PX, x_to_screen(cg_high), center_y + 2 * MARKER_RADIUS_PX, fill=MARKER_COLORS["cg"], width=3)
    draw_marker_guide(canvas)
    metrics["scale"] = scale
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_front_view(canvas: tk.Canvas, resolved: ResolvedAirframe, ui_draw: str, ui_bg: str, zoom_scale: float, pan_x: float, pan_y: float) -> dict:
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
    widest = max(body.stations, key=lambda station: station.width_m)
    outline = []
    for index in range(SECTION_OUTLINE_SAMPLES + 1):
        angle = 2.0 * math.pi * index / SECTION_OUTLINE_SAMPLES
        radius = superellipse_radius(widest.width_m / 2.0, widest.height_m / 2.0, body.shape, angle)
        outline.extend((center_x + radius * math.cos(angle) * scale, center_y - radius * math.sin(angle) * scale))
    canvas.create_line(outline, fill=ui_draw, width=2)
    for volume in resolved.airframe.volumes:
        if volume.shape == "point":
            continue
        _half_x, half_y, half_z = volume_half_extents(resolved, volume)
        x_screen = center_x + volume.y_m * scale
        y_screen = center_y - volume.z_m * scale
        canvas.create_rectangle(x_screen - half_y * scale, y_screen - half_z * scale, x_screen + half_y * scale, y_screen + half_z * scale,
                                outline=VOLUME_COLOR, fill="", width=1, dash=(4, 2))
    for motor in resolved.motors:
        x_screen = center_x + motor.position[1] * scale
        y_screen = center_y - motor.position[2] * scale
        pod_radius = max((radius for _x, radius in motor.pod_stations), default=0.0)
        for radius, color, fill in ((pod_radius, ui_draw, ui_bg), (motor.bell_diameter_m / 2.0, ui_draw, ui_bg), (PROP_SHAFT_DIAM_M / 2.0, ui_draw, ui_bg), (motor.prop_radius_m, "red", "")):
            if radius > 0.0:
                canvas.create_oval(x_screen - radius * scale, y_screen - radius * scale, x_screen + radius * scale, y_screen + radius * scale, outline=color, fill=fill, width=2)
    draw_marker_guide(canvas)
    metrics["scale"] = scale
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_half_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_canvas_view(
    canvas: tk.Canvas,
    resolved: ResolvedAirframe,
    aero,
    detailed,
    view_mode: str,
    roll_deg: float,
    ui_draw: str,
    ui_bg: str,
    zoom_scale: float = 1.0,
    pan_x: float = 0.0,
    pan_y: float = 0.0,
) -> dict:
    """aero / detailed are the fast and detailed aero results for this geometry, or None when not (yet) computed."""
    if view_mode == "Longitudinal":
        return draw_longitudinal_view(canvas, resolved, aero, detailed, roll_deg, ui_draw, zoom_scale, pan_x, pan_y)
    return draw_front_view(canvas, resolved, ui_draw, ui_bg, zoom_scale, pan_x, pan_y)
