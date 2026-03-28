"""
Usage:
from graphics import draw_canvas_view
"""

import math
import tkinter as tk

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
        ("red", analysis["x_cp_total"]),
        ("green", analysis["lift_center_x"]),
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
        if motor["pod_shape"] == "ogive":
            tip_x = arc_center_x + pod_sign * pod_curve_len
            canvas.create_line(arc_center_x, transverse_screen - pod_half, tip_x, transverse_screen, fill=ui_draw, width=2)
            canvas.create_line(arc_center_x, transverse_screen + pod_half, tip_x, transverse_screen, fill=ui_draw, width=2)
        elif pod_sign > 0.0:
            canvas.create_arc(
                arc_center_x - pod_curve_len,
                transverse_screen - pod_half,
                arc_center_x + pod_curve_len,
                transverse_screen + pod_half,
                start=270,
                extent=180,
                style=tk.ARC,
                outline=ui_draw,
                width=2,
            )
        else:
            canvas.create_arc(
                arc_center_x - pod_curve_len,
                transverse_screen - pod_half,
                arc_center_x + pod_curve_len,
                transverse_screen + pod_half,
                start=90,
                extent=180,
                style=tk.ARC,
                outline=ui_draw,
                width=2,
            )


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
                canvas.create_arc(
                    x_to_screen(row["x_start"]),
                    transverse_to_screen(radius),
                    x_to_screen(row["x_start"] + 2.0 * module_len),
                    transverse_to_screen(-radius),
                    start=90,
                    extent=180,
                    style=tk.ARC,
                    outline=ui_draw,
                    width=2,
                )
            else:
                canvas.create_arc(
                    x_to_screen(row["x_end"] - 2.0 * module_len),
                    transverse_to_screen(radius),
                    x_to_screen(row["x_end"]),
                    transverse_to_screen(-radius),
                    start=270,
                    extent=180,
                    style=tk.ARC,
                    outline=ui_draw,
                    width=2,
                )
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


def draw_longitudinal_view(canvas: tk.Canvas, analysis: dict, roll_deg: float, ui_draw: str) -> None:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    pad_x = 30
    pad_y = 30
    fuselage_length = analysis["fuselage_length"]
    roll_rad = math.radians(roll_deg)
    half_span = analysis["tube_diam"] / 2.0
    for surface in analysis["surface_sets"]:
        for panel in surface["panels"]:
            root_transverse = longitudinal_transverse(panel["root_y"], panel["root_z"], roll_rad)
            tip_transverse = longitudinal_transverse(panel["tip_y"], panel["tip_z"], roll_rad)
            half_span = max(half_span, abs(root_transverse), abs(tip_transverse))
    for motor in analysis["motor_draw_rows"]:
        center_transverse = longitudinal_transverse(motor["y_m"], motor["z_m"], roll_rad)
        half_span = max(half_span, abs(center_transverse) + max(motor["pod_diam_m"], motor["prop_diam_m"], motor["motor_bell_diam_m"]) / 2.0)
    half_span = max(half_span, 0.1)
    scale = min((width - 2 * pad_x) / fuselage_length, (height - 2 * pad_y) / (2.2 * half_span))
    center_y = height / 2.0

    def x_to_screen(x_value: float) -> float:
        return pad_x + x_value * scale

    def transverse_to_screen(transverse_value: float) -> float:
        return center_y - transverse_value * scale

    draw_longitudinal_fuselage(canvas, analysis, x_to_screen, transverse_to_screen, center_y, ui_draw)
    for surface in analysis["surface_sets"]:
        x_root_le = surface["x_root_le"]
        x_root_te = x_root_le + surface["chord_root"]
        x_tip_le = x_root_le + surface["sweep_len"]
        x_tip_te = x_tip_le + surface["chord_tip"]
        for panel in surface["panels"]:
            root_transverse = longitudinal_transverse(panel["root_y"], panel["root_z"], roll_rad)
            tip_transverse = longitudinal_transverse(panel["tip_y"], panel["tip_z"], roll_rad)
            points = [
                x_to_screen(x_root_le),
                transverse_to_screen(root_transverse),
                x_to_screen(x_root_te),
                transverse_to_screen(root_transverse),
                x_to_screen(x_tip_te),
                transverse_to_screen(tip_transverse),
                x_to_screen(x_tip_le),
                transverse_to_screen(tip_transverse),
            ]
            canvas.create_polygon(points, outline=ui_draw, fill="", width=2)
    draw_longitudinal_motors(canvas, analysis, x_to_screen, transverse_to_screen, scale, ui_draw, roll_rad)


def draw_front_view(canvas: tk.Canvas, analysis: dict, ui_draw: str, ui_bg: str) -> None:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 10)
    height = max(canvas.winfo_height(), 10)
    center_x = width / 2.0
    center_y = height / 2.0
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
    scale = min((width - 60) / (2.2 * extent_y), (height - 60) / (2.2 * extent_z))
    body_half = analysis["tube_diam"] / 2.0
    canvas.create_oval(
        center_x - body_half * scale,
        center_y - body_half * scale,
        center_x + body_half * scale,
        center_y + body_half * scale,
        outline=ui_draw,
        width=2,
    )
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
    draw_front_motors(canvas, analysis, center_x, center_y, scale, ui_draw, ui_bg)
    legend_y = 20
    for index, color in enumerate(("blue", "red", "green")):
        canvas.create_oval(20 + index * 22, legend_y, 30 + index * 22, legend_y + 10, fill=color, outline="")


def draw_canvas_view(canvas: tk.Canvas, analysis: dict, view_mode: str, roll_deg: float, ui_draw: str, ui_bg: str) -> None:
    if view_mode == "Longitudinal":
        draw_longitudinal_view(canvas, analysis, roll_deg, ui_draw)
        return
    draw_front_view(canvas, analysis, ui_draw, ui_bg)
