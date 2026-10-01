"""
Usage:
from libs.graphics import draw_canvas_view
"""

import math
import tkinter as tk

import numpy as np
from libs.pointed_shapes import pointed_shape_param_values, sample_pointed_profile

PROP_SHAFT_DIAM_M = 0.005
PROP_SHAFT_LEN_M = 0.015
MOTOR_BASE_DIAM_FACTOR = 0.5
MOTOR_BASE_LEN_M = 0.005


def structural_rows(module_rows: list[dict]) -> list[dict]:
    return [row for row in module_rows if row["module_type"] in {"body", "end", "coupler", "wingmount"}]


def draw_markers(canvas: tk.Canvas, analysis: dict, x_to_screen, y_screen: float) -> None:
    marker_radius = 5
    for color, x_value in (
        ("blue", analysis["cg_x_m"]),
        ("red", analysis["x_np"]),
        ("green", analysis["force_center_x"]),
    ):
        if not math.isfinite(x_value):
            continue
        x_screen = x_to_screen(x_value)
        canvas.create_oval(
            x_screen - marker_radius,
            y_screen - marker_radius,
            x_screen + marker_radius,
            y_screen + marker_radius,
            fill=color,
            outline="",
        )


def draw_marker_guide(canvas: tk.Canvas) -> None:
    guide_rows = [
        ("blue", "CG"),
        ("red", "Neutral point"),
        ("green", "Lift @ AoA"),
    ]
    for index, (color, text) in enumerate(guide_rows):
        canvas.create_text(12, 12 + index * 16, anchor="nw", text=text, fill=color, font=("TkDefaultFont", 9))


def measure_longitudinal_view(analysis: dict, roll_deg: float, width: float, height: float) -> dict:
    pad_x = 30.0
    pad_y = 30.0
    fuselage_length = analysis["fuselage_length"]
    min_x = 0.0
    max_x = fuselage_length
    roll_rad = math.radians(roll_deg)
    half_span = analysis["tube_diam"] / 2.0
    for surface in analysis["surface_sets"]:
        aoa_rad = math.radians(surface["aoa"])
        aoa_transverse_scale = math.cos(roll_rad)
        x_root_le = surface["x_root_le"]
        x_tip_le = x_root_le + surface["sweep_len"]
        root_dt = surface["chord_root"] * math.sin(aoa_rad) * aoa_transverse_scale
        tip_dt = surface["chord_tip"] * math.sin(aoa_rad) * aoa_transverse_scale
        root_dx = surface["chord_root"] * math.cos(aoa_rad)
        tip_dx = surface["chord_tip"] * math.cos(aoa_rad)
        min_x = min(min_x, x_root_le, x_root_le + root_dx, x_tip_le, x_tip_le + tip_dx)
        max_x = max(max_x, x_root_le, x_root_le + root_dx, x_tip_le, x_tip_le + tip_dx)
        for panel in surface["panels"]:
            root_transverse = longitudinal_transverse(panel["root_y"], panel["root_z"], roll_rad)
            tip_transverse = longitudinal_transverse(panel["tip_y"], panel["tip_z"], roll_rad)
            half_span = max(half_span, abs(root_transverse), abs(root_transverse - root_dt), abs(tip_transverse), abs(tip_transverse - tip_dt))
    for motor in analysis["motor_draw_rows"]:
        center_transverse = longitudinal_transverse(motor["y_m"], motor["z_m"], roll_rad)
        half_span = max(half_span, abs(center_transverse) + max(motor["pod_diam_m"], motor["prop_diam_m"], motor["motor_bell_diam_m"]) / 2.0)
        x_half_extent = motor["motor_bell_len_m"] * 0.5 + max(PROP_SHAFT_LEN_M, MOTOR_BASE_LEN_M, motor["pod_len_flat_m"] + motor["pod_len_curve_m"])
        min_x = min(min_x, motor["x_m"] - x_half_extent)
        max_x = max(max_x, motor["x_m"] + x_half_extent)
    for item in analysis["internal_draw_rows"]:
        center_transverse = longitudinal_transverse(item["y_m"], item["z_m"], roll_rad)
        half_span = max(half_span, abs(center_transverse) + item["height_m"] / 2.0)
        min_x = min(min_x, item["x_m"] - item["len_m"] * 0.5)
        max_x = max(max_x, item["x_m"] + item["len_m"] * 0.5)
    for x_value in (analysis["cg_x_m"], analysis["x_np"], analysis["force_center_x"]):
        if math.isfinite(x_value):
            min_x = min(min_x, x_value)
            max_x = max(max_x, x_value)
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


def measure_front_view(analysis: dict, width: float, height: float) -> dict:
    pad_x = 30.0
    pad_y = 30.0
    extent_y = analysis["tube_diam"] / 2.0
    extent_z = analysis["tube_diam"] / 2.0
    for surface in analysis["surface_sets"]:
        for panel in surface["panels"]:
            extent_y = max(extent_y, abs(panel["root_y"]), abs(panel["tip_y"]))
            extent_z = max(extent_z, abs(panel["root_z"]), abs(panel["tip_z"]))
    for motor in analysis["motor_draw_rows"]:
        outer_radius = max(motor["prop_diam_m"], motor["pod_diam_m"], motor["motor_bell_diam_m"]) / 2.0
        extent_y = max(extent_y, abs(motor["y_m"]) + outer_radius)
        extent_z = max(extent_z, abs(motor["z_m"]) + outer_radius)
    for item in analysis["internal_draw_rows"]:
        extent_y = max(extent_y, abs(item["y_m"]) + item["width_m"] / 2.0)
        extent_z = max(extent_z, abs(item["z_m"]) + item["height_m"] / 2.0)
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


def draw_longitudinal_pod(canvas: tk.Canvas, motor: dict, x_screen: float, transverse_screen: float, scale: float, ui_draw: str) -> None:
    pod_half = motor["pod_diam_m"] * scale / 2.0
    pod_flat_len = motor["pod_len_flat_m"] * scale
    pod_curve_len = motor["pod_len_curve_m"] * scale
    if pod_flat_len <= 0.0 and pod_curve_len <= 0.0:
        return
    front_sign = -1.0 if motor["direction"] == "front" else 1.0
    pod_sign = -front_sign
    motor_edge_x = x_screen + pod_sign * (motor["motor_bell_len_m"] * scale / 2.0)
    flat_end_x = motor_edge_x + pod_sign * pod_flat_len
    if pod_flat_len > 0.0:
        canvas.create_rectangle(
            min(motor_edge_x, flat_end_x),
            transverse_screen - pod_half,
            max(motor_edge_x, flat_end_x),
            transverse_screen + pod_half,
            outline=ui_draw,
            fill="",
            width=2,
        )
    arc_center_x = flat_end_x if pod_flat_len > 0.0 else motor_edge_x
    if pod_curve_len > 0.0:
        top_points = []
        bottom_points = []
        for x_local, radius_value in sample_pointed_profile(motor["pod_shape"], motor["pod_len_curve_m"], motor["pod_diam_m"] / 2.0, 16):
            x_profile = arc_center_x + pod_sign * (pod_curve_len - x_local * scale)
            radius_screen = radius_value * scale
            top_points.extend((x_profile, transverse_screen - radius_screen))
            bottom_points.extend((x_profile, transverse_screen + radius_screen))
        canvas.create_line(top_points, fill=ui_draw, width=2)
        canvas.create_line(bottom_points, fill=ui_draw, width=2)


def longitudinal_transverse(y_value: float, z_value: float, roll_rad: float) -> float:
    return z_value * math.cos(roll_rad) + y_value * math.sin(roll_rad)


def draw_longitudinal_motors(canvas: tk.Canvas, analysis: dict, x_to_screen, transverse_to_screen, scale: float, ui_draw: str, roll_rad: float) -> None:
    for motor in analysis["motor_draw_rows"]:
        x_screen = x_to_screen(motor["x_m"])
        transverse_screen = transverse_to_screen(longitudinal_transverse(motor["y_m"], motor["z_m"], roll_rad))
        bell_half_diam = motor["motor_bell_diam_m"] * scale / 2.0
        bell_half_len = motor["motor_bell_len_m"] * scale / 2.0
        shaft_half = PROP_SHAFT_DIAM_M * scale / 2.0
        shaft_len = PROP_SHAFT_LEN_M * scale
        base_half = motor["motor_bell_diam_m"] * MOTOR_BASE_DIAM_FACTOR * scale / 2.0
        base_len = MOTOR_BASE_LEN_M * scale
        prop_radius = motor["prop_diam_m"] * scale / 2.0
        front_sign = -1.0 if motor["direction"] == "front" else 1.0
        draw_longitudinal_pod(canvas, motor, x_screen, transverse_screen, scale, ui_draw)
        canvas.create_rectangle(
            x_screen - bell_half_len,
            transverse_screen - bell_half_diam,
            x_screen + bell_half_len,
            transverse_screen + bell_half_diam,
            outline=ui_draw,
            fill="",
            width=2,
        )
        shaft_outer_x = x_screen + front_sign * (bell_half_len + shaft_len)
        shaft_inner_x = x_screen + front_sign * bell_half_len
        canvas.create_rectangle(
            min(shaft_inner_x, shaft_outer_x),
            transverse_screen - shaft_half,
            max(shaft_inner_x, shaft_outer_x),
            transverse_screen + shaft_half,
            outline=ui_draw,
            fill="",
            width=2,
        )
        base_outer_x = x_screen - front_sign * (bell_half_len + base_len)
        base_inner_x = x_screen - front_sign * bell_half_len
        canvas.create_rectangle(
            min(base_inner_x, base_outer_x),
            transverse_screen - base_half,
            max(base_inner_x, base_outer_x),
            transverse_screen + base_half,
            outline=ui_draw,
            fill="",
            width=2,
        )
        prop_x_screen = shaft_outer_x
        canvas.create_line(prop_x_screen, transverse_screen - prop_radius, prop_x_screen, transverse_screen + prop_radius, fill="red", width=2)


def draw_longitudinal_internal_objects(canvas: tk.Canvas, analysis: dict, x_to_screen, transverse_to_screen, scale: float, ui_draw: str, roll_rad: float) -> None:
    for item in analysis["internal_draw_rows"]:
        transverse_screen = transverse_to_screen(longitudinal_transverse(item["y_m"], item["z_m"], roll_rad))
        half_len = item["len_m"] * scale / 2.0
        half_height = item["height_m"] * scale / 2.0
        canvas.create_rectangle(
            x_to_screen(item["x_m"]) - half_len,
            transverse_screen - half_height,
            x_to_screen(item["x_m"]) + half_len,
            transverse_screen + half_height,
            outline=ui_draw,
            fill="",
            width=1,
            dash=(4, 2),
        )


def draw_front_internal_objects(canvas: tk.Canvas, analysis: dict, center_x: float, center_y: float, scale: float, ui_draw: str) -> None:
    for item in analysis["internal_draw_rows"]:
        half_width = item["width_m"] * scale / 2.0
        half_height = item["height_m"] * scale / 2.0
        x_screen = center_x + item["y_m"] * scale
        y_screen = center_y - item["z_m"] * scale
        canvas.create_rectangle(
            x_screen - half_width,
            y_screen - half_height,
            x_screen + half_width,
            y_screen + half_height,
            outline=ui_draw,
            fill="",
            width=1,
            dash=(4, 2),
        )


def draw_front_motors(canvas: tk.Canvas, analysis: dict, center_x: float, center_y: float, scale: float, ui_draw: str, ui_bg: str) -> None:
    for motor in analysis["motor_draw_rows"]:
        x_screen = center_x + motor["y_m"] * scale
        y_screen = center_y - motor["z_m"] * scale
        pod_radius = motor["pod_diam_m"] * scale / 2.0
        motor_radius = motor["motor_bell_diam_m"] * scale / 2.0
        shaft_radius = PROP_SHAFT_DIAM_M * scale / 2.0
        prop_radius = motor["prop_diam_m"] * scale / 2.0
        canvas.create_oval(
            x_screen - pod_radius,
            y_screen - pod_radius,
            x_screen + pod_radius,
            y_screen + pod_radius,
            outline=ui_draw,
            fill=ui_bg,
            width=2,
        )
        canvas.create_oval(
            x_screen - motor_radius,
            y_screen - motor_radius,
            x_screen + motor_radius,
            y_screen + motor_radius,
            outline=ui_draw,
            fill=ui_bg,
            width=2,
        )
        canvas.create_oval(
            x_screen - shaft_radius,
            y_screen - shaft_radius,
            x_screen + shaft_radius,
            y_screen + shaft_radius,
            outline=ui_draw,
            fill=ui_bg,
            width=2,
        )
        canvas.create_oval(
            x_screen - prop_radius,
            y_screen - prop_radius,
            x_screen + prop_radius,
            y_screen + prop_radius,
            outline="red",
            width=2,
        )


def draw_longitudinal_fuselage(canvas: tk.Canvas, analysis: dict, x_to_screen, transverse_to_screen, center_line: float, ui_draw: str) -> None:
    for row in structural_rows(analysis["module_rows"]):
        radius = row["diam"] / 2.0
        if row["module_type"] == "end":
            module_len = row["x_end"] - row["x_start"]
            if row["x_start"] == 0.0:
                top_points = []
                bottom_points = []
                for x_local, radius_value in sample_pointed_profile(
                    row["pointed_shape"]["shape"],
                    module_len,
                    radius,
                    16,
                    pointed_shape_param_values(row["pointed_shape"]),
                ):
                    x_screen = x_to_screen(row["x_start"] + x_local)
                    top_points.extend((x_screen, transverse_to_screen(radius_value)))
                    bottom_points.extend((x_screen, transverse_to_screen(-radius_value)))
                canvas.create_line(top_points, fill=ui_draw, width=2)
                canvas.create_line(bottom_points, fill=ui_draw, width=2)
            else:
                top_points = []
                bottom_points = []
                for x_local, radius_value in sample_pointed_profile(
                    row["pointed_shape"]["shape"],
                    module_len,
                    radius,
                    16,
                    pointed_shape_param_values(row["pointed_shape"]),
                ):
                    x_screen = x_to_screen(row["x_end"] - x_local)
                    top_points.extend((x_screen, transverse_to_screen(radius_value)))
                    bottom_points.extend((x_screen, transverse_to_screen(-radius_value)))
                canvas.create_line(top_points, fill=ui_draw, width=2)
                canvas.create_line(bottom_points, fill=ui_draw, width=2)
            continue
        canvas.create_rectangle(
            x_to_screen(row["x_start"]),
            transverse_to_screen(radius),
            x_to_screen(row["x_end"]),
            transverse_to_screen(-radius),
            outline=ui_draw,
            width=2,
        )
    draw_markers(canvas, analysis, x_to_screen, center_line)


def draw_longitudinal_view(canvas: tk.Canvas, analysis: dict, roll_deg: float, ui_draw: str, zoom_scale: float = 1.0, pan_x: float = 0.0, pan_y: float = 0.0) -> dict:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    metrics = measure_longitudinal_view(analysis, roll_deg, width, height)
    scale = metrics["base_scale"] * zoom_scale
    roll_rad = metrics["roll_rad"]
    center_y = metrics["origin_y"] + pan_y

    def x_to_screen(x_value: float) -> float:
        return metrics["origin_x"] + pan_x + x_value * scale

    def transverse_to_screen(transverse_value: float) -> float:
        return center_y - transverse_value * scale

    for surface in analysis["surface_sets"]:
        x_root_le = surface["x_root_le"]
        x_tip_le = x_root_le + surface["sweep_len"]
        aoa_rad = math.radians(surface["aoa"])
        aoa_transverse_scale = math.cos(roll_rad)
        root_dx = surface["chord_root"] * math.cos(aoa_rad)
        root_dt = surface["chord_root"] * math.sin(aoa_rad) * aoa_transverse_scale
        tip_dx = surface["chord_tip"] * math.cos(aoa_rad)
        tip_dt = surface["chord_tip"] * math.sin(aoa_rad) * aoa_transverse_scale
        for panel in surface["panels"]:
            root_transverse = longitudinal_transverse(panel["root_y"], panel["root_z"], roll_rad)
            tip_transverse = longitudinal_transverse(panel["tip_y"], panel["tip_z"], roll_rad)
            points = [
                x_to_screen(x_root_le),
                transverse_to_screen(root_transverse),
                x_to_screen(x_root_le + root_dx),
                transverse_to_screen(root_transverse - root_dt),
                x_to_screen(x_tip_le + tip_dx),
                transverse_to_screen(tip_transverse - tip_dt),
                x_to_screen(x_tip_le),
                transverse_to_screen(tip_transverse),
            ]
            canvas.create_polygon(points, outline=ui_draw, fill="", width=2)
    draw_longitudinal_internal_objects(canvas, analysis, x_to_screen, transverse_to_screen, scale, ui_draw, roll_rad)
    draw_longitudinal_fuselage(canvas, analysis, x_to_screen, transverse_to_screen, center_y, ui_draw)
    draw_longitudinal_motors(canvas, analysis, x_to_screen, transverse_to_screen, scale, ui_draw, roll_rad)
    draw_marker_guide(canvas)
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_front_view(canvas: tk.Canvas, analysis: dict, ui_draw: str, ui_bg: str, zoom_scale: float = 1.0, pan_x: float = 0.0, pan_y: float = 0.0) -> dict:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    metrics = measure_front_view(analysis, width, height)
    center_x = metrics["origin_x"] + pan_x
    center_y = metrics["origin_y"] + pan_y
    scale = metrics["base_scale"] * zoom_scale
    for surface in analysis["surface_sets"]:
        for panel in surface["panels"]:
            canvas.create_line(
                center_x + panel["root_y"] * scale,
                center_y - panel["root_z"] * scale,
                center_x + panel["tip_y"] * scale,
                center_y - panel["tip_z"] * scale,
                fill=ui_draw,
                width=2,
            )
    draw_front_internal_objects(canvas, analysis, center_x, center_y, scale, ui_draw)
    body_half = analysis["tube_diam"] / 2.0
    canvas.create_oval(
        center_x - body_half * scale,
        center_y - body_half * scale,
        center_x + body_half * scale,
        center_y + body_half * scale,
        outline=ui_draw,
        width=2,
    )
    draw_front_motors(canvas, analysis, center_x, center_y, scale, ui_draw, ui_bg)
    draw_marker_guide(canvas)
    legend_y = 64
    for index, color in enumerate(("blue", "red", "green")):
        canvas.create_oval(20 + index * 22, legend_y, 30 + index * 22, legend_y + 10, fill=color, outline="")
    metrics["zoom_scale"] = zoom_scale
    metrics["pan_x"] = pan_x
    metrics["pan_y"] = pan_y
    metrics["content_half_width"] *= zoom_scale
    metrics["content_half_height"] *= zoom_scale
    return metrics


def draw_canvas_view(
    canvas: tk.Canvas,
    analysis: dict,
    view_mode: str,
    roll_deg: float,
    ui_draw: str,
    ui_bg: str,
    zoom_scale: float = 1.0,
    pan_x: float = 0.0,
    pan_y: float = 0.0,
) -> dict:
    if view_mode == "Longitudinal":
        return draw_longitudinal_view(canvas, analysis, roll_deg, ui_draw, zoom_scale, pan_x, pan_y)
    return draw_front_view(canvas, analysis, ui_draw, ui_bg, zoom_scale, pan_x, pan_y)
