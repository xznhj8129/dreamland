"""
Usage:
from libs.views.editor_view import EditorMainView

Editor view: summary, drawing and the plane-first editors (Airframe, Surfaces, Propulsion, Volumes).
Editors write straight into app.definition (the airframe/2 dict) and call app.recompute(). Lengths are shown in
mm and masses in g; the definition stays in SI. In the side view, volumes and wing sets can be dragged along x.
"""

from __future__ import annotations

import copy
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from libs.airframe import (
    AXES, BATTERY_PART_NAME, ORIENTATION_KEYS, CONTROL_KINDS, CONTROL_ROLES, CUT_JOINTS, FIN_POSITIONS, INCIDENCE_MODES, JOINT_TYPES, LAYOUT_PANEL_COUNTS, LAYOUTS, MOUNT_TARGETS, REINFORCEMENT_TYPES,
    SERVO_OUTPUTS, PART_CATEGORIES, SINGLE_PROP_SPINS, SURFACE_PROP_SPINS, SURFACE_ROOTS, THRUST_DIRECTIONS, VOLUME_SHAPES, AirframeError,
    cell_format, motor_count, motor_repo,
)
from libs.geometry import route_length_m
from libs.graphics import draw_canvas_view
from libs.presets import (
    ROD_PATTERN_KEYS, apply_preset, default_rod_extent, preset_carry_through, preset_names, preset_spars, preset_status, read_presets, save_preset,
)
from libs.propulsion import summary_lines
from libs.pointed_shapes import POINTED_SHAPE_PARAM_KEYS

# AeroSandbox's matplotlib draw needs a pyplot figure. With pyplot on Agg those figures get no Tk window of their
# own (a TkAgg pyplot window keeps Tk's mainloop alive after the app closes); the view embeds them itself.
plt.switch_backend("Agg")

MM_KEYS = {
    "length_m", "width_m", "height_m", "wall_radial_m", "insertion_depth_m", "diameter_m", "radial_position_m",
    "from_x_m", "to_x_m", "x_root_le_m", "span_m", "chord_root_m", "chord_tip_m", "span_position_m",
    "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m", "lead_length_m", "flat_length_m", "curve_length_m", "nose_radius", "size_m", "printer_bed_m", "cuts_m",
    "corner_radius_m", "stem_width_m", "flare_radius_m", "root_length_m", "wall_fillet_m", "coupler_wall_m", "fit_clearance_m", "socket_wall_m", "hole_edge_dist_m", "hole_diameter_m", "spacing_m", "forward_offset_m", "leading_cut_m", "trailing_cut_m",
}
GRAM_KEYS = {"mass_kg", "coupler_mass_kg", "empty_mass_kg", "complete_mass_kg"}
UNIT_SUFFIXES = ("_kg_m3", "_m_s", "_deg_s", "_deg", "_kg", "_m")
CHOICES = {
    "layout": list(LAYOUTS),
    "fin_position": list(FIN_POSITIONS),
    "root": list(SURFACE_ROOTS),
    "incidence_mode": list(INCIDENCE_MODES),
    "axis": list(AXES),
    "on": list(MOUNT_TARGETS),
    "direction": list(THRUST_DIRECTIONS),
    "type": list(JOINT_TYPES),
    "profile": sorted(POINTED_SHAPE_PARAM_KEYS),
}
# Explicit defaults for fields that appear when the user switches a profile, shape or adds an item.
PROFILE_PARAM_DEFAULTS = {"k_prime": 1.0, "nose_radius": 0.00465, "break_fraction": 0.5, "break_radius_fraction": 0.5, "rho_factor": 1.25, "exponent": 0.66, "c": 0.0}
VOLUME_SHAPE_FIELDS = {
    "box": {"size_m": [0.02, 0.02, 0.02]},
    "rounded_box": {"size_m": [0.02, 0.02, 0.02], "axis": "x", "corner_radius_m": 0.004},
    "cylinder": {"diameter_m": 0.02, "length_m": 0.03, "axis": "x"},
    "bore": {"length_m": 0.05},
    "point": {},
}
NEW_SURFACE = {
    "name": "wing", "x_root_le_m": 0.2, "layout": "H", "fin_position": "top", "dihedral_deg": 0.0, "root_offset": 0.0,
    "root": "body", "span_m": 0.12, "chord_root_m": 0.1, "chord_tip_m": 0.07, "sweep_le_deg": 10.0, "incidence_deg": 0.0, "incidence_mode": "symmetric", "airfoil": "naca2412",
}
CONTROL_KIND_FIELDS = {"split": {"chord_start": 0.75, "span_start": 0.0, "span_end": 1.0}, "all_moving": {"pivot": 0.25}}
NEW_CONTROL = {"kind": "split", "roles": [], "deflection_min_deg": -25.0, "deflection_max_deg": 25.0, "rate_deg_s": 300.0}  # + kind fields, servos
NEW_POD = {"diameter_m": 0.035, "flat_length_m": 0.02, "curve_length_m": 0.05, "profile": "elliptical"}
# Spars and carry-through for an aircraft without a preset to take them from (the M62 preset's values).
NEW_SPARS = {"material": "carbon_rod", "diameter_m": 0.0064, "count": 2, "spacing_m": 0.0075, "x_offset_m": 0.02, "y_offset_m": 0.0}
NEW_CARRY_THROUGH = {"stem_width_m": 0.01, "length_m": 0.02, "flare_radius_m": 0.02, "root_length_m": 0.04, "wall_fillet_m": 0.004}
NEW_ROOT_BASE = {"length_m": 0.096, "width_m": 0.018, "height_m": 0.0, "forward_offset_m": 0.0, "leading_cut_m": 0.01, "trailing_cut_m": 0.06}
NEW_MOTOR = {"manufacturer": "DYS", "name": "X2807-1300"}  # motorlib
NEW_PROP = "APC E 7x5"  # motorlib
# Rods for an aircraft without a preset to take them from: 4 x 2.5 mm carbon (the built M62 aircraft), touching the inner wall.
NEW_ROD_PATTERN = {"type": "cf_rod", "material": "carbon_rod", "diameter_m": 0.0025, "polar_n": 4, "polar_offset_angle": 45.0, "polar_split": 0, "polar_split_angle": 0.0}
NO_PRESET = "(none)"
NO_VOLUME = "(none)"  # a part placed at its own x, y, z
EDITOR_TABS = {"Design": ("Airframe", "Fuselage", "Sections", "Surfaces", "Volumes", "Motor mounts"), "Build": ("Propulsion", "Parts", "Wiring")}
WIRING_COLUMNS = ("run", "length mm", "AWG", "wires", "plugs, along the run")
# Wire runs for a newly powered aircraft and for added motors without one to copy: the values data/specs.json held before
# wiring became per aircraft.
NEW_BATTERY_RUN = {"gauge_awg": 14, "conductors": 2, "plugs": ["XT60", "XT60"]}  # after the battery's own connector
NEW_MOTOR_RUN = {"gauge_awg": 18, "conductors": 3, "plugs": ["MR60"]}
BATTERY_MODEL_LABELS = {"commercial": "Commercial", "custom_liion": "Custom Li-Ion"}  # libs/airframe.py BATTERY_MODELS
# A custom pack as first picked (then edited): 4S1P of the first stackable motorlib Li-ion cell, lying along x.
NEW_CUSTOM_PACK = {"model": "custom_liion", "s": 4, "p": 1, "cell": "Molicel INR21700-P45B 7/10C", "connector": "XT60", "cell_axis": "x",
                   "stacking": [4, 1, 1], "lead_awg": 14, "lead_length_m": 0.1}
PLACEMENT_KEYS = ("volume", "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m")
REMOVE_PLUG = "(remove)"
ADD_PLUG = "+"
SECTION_COLUMNS = ("section mm", "cut mm", "joint", "solid", "empty g", "calc", "complete g", "items", "")
NEW_CUT_JOINT = "coupler"
SECTION_ENTRY_WIDTH = 5
TYPE_FLAGS = ("tailsitter", "vtol", "powered")  # each implies the ones after it
DRAG_KEYS = {"volume": ("volumes", "x_m"), "surface": ("surfaces", "x_root_le_m")}  # cuts drag the sections.cuts_m entry, spars their x_offset_m
MIN_SECTION_LENGTH_M = 0.005
WHEEL_STEP_FRACTION = 0.01  # mouse wheel over a value field changes it by this fraction per notch


def field_label(key: str) -> str:
    base = key
    for suffix in UNIT_SUFFIXES:
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    unit = "mm" if key in MM_KEYS else "g" if key in GRAM_KEYS else "deg/s" if key.endswith("_deg_s") else "deg" if key.endswith("_deg") else "m/s" if key.endswith("_m_s") else "kg/m^3" if key.endswith("_kg_m3") else "m" if key.endswith("_m") else ""
    return f"{base.replace('_', ' ')} ({unit})" if unit else base.replace("_", " ")


def display_factor(key: str) -> float:
    return 1000.0 if key in MM_KEYS or key in GRAM_KEYS else 1.0


def format_value(key: str, value) -> str:
    if value is None:
        return ""
    if type(value) is list:
        return "-".join(f"{item * display_factor(key):g}" for item in value)
    if type(value) in (int, float):
        return f"{value * display_factor(key):g}" if type(value) is float else str(value)
    return str(value)


def parse_value(key: str, text: str, existing):
    """Text from an entry back to the stored value; raises ValueError on bad input. 'a-b' gives a mass range."""
    text = text.strip()
    if type(existing) is str:
        return text
    if type(existing) is int:
        return int(text)
    if key == "mass_kg" and "-" in text[1:]:
        low, high = text.split("-", 1)
        return [float(low) / display_factor(key), float(high) / display_factor(key)]
    return float(text) / display_factor(key)


class EditorMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.view_zoom_scale = 1.0
        self.view_pan_x = 0.0
        self.view_pan_y = 0.0
        self.view_drag_last_xy = None
        self.drag_target = None
        self.view_metrics = None
        self.control_deflections: dict[int, float] = {}  # control number -> deg, drawn in the views; no UI yet
        self.stashed_propulsion: dict | None = None  # propulsion block removed by unticking powered, restored on ticking it
        self.stashed_batteries: dict[str, dict] = {}  # per battery model, its fields when another model was picked
        self.stashed_reinforcement: dict | None = None  # rods removed by unticking them, restored on ticking them
        self.preset_status_var = tk.StringVar(value="")
        self.section_vars: list[tuple[tk.StringVar, tk.StringVar, tk.StringVar]] = []  # per section: x range, computed empty, itemised complete
        self.motor_info: ttk.LabelFrame | None = None  # rebuilt after every recompute: it reads positions (wiring, hover)
        self.view_scroll_x = None
        self.view_scroll_y = None
        self.editor_frames: dict[str, ttk.Frame] = {}
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.editor_view = self.frame

        pane_window = ttk.Panedwindow(self.frame, orient=tk.HORIZONTAL)
        pane_window.grid(row=0, column=0, sticky="nsew")

        left_frame = ttk.Frame(pane_window, padding=8)
        center_frame = ttk.Frame(pane_window, padding=8)
        right_frame = ttk.Frame(pane_window, padding=8)
        pane_window.add(left_frame, weight=1)
        pane_window.add(center_frame, weight=2)
        pane_window.add(right_frame, weight=1)

        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=1)
        center_frame.columnconfigure(0, weight=1)
        center_frame.rowconfigure(0, weight=1)
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(1, weight=1)

        left_header = ttk.LabelFrame(left_frame, text="Recompute", padding=6)
        left_header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        left_header.columnconfigure(1, weight=1)
        ttk.Button(left_header, text="Recompute (F5)", command=app.recompute).grid(row=0, column=0, sticky="w")
        ttk.Label(left_header, textvariable=app.recompute_status_var).grid(row=0, column=1, sticky="e", padx=(8, 0))

        left_notebook = ttk.Notebook(left_frame)
        left_notebook.grid(row=1, column=0, sticky="nsew")
        app.summary_text = self.text_tab(left_notebook, "Summary")
        app.log_text = self.text_tab(left_notebook, "Log")
        app.left_notebook, app.log_tab = left_notebook, left_notebook.tabs()[-1]

        center_notebook = ttk.Notebook(center_frame)
        center_notebook.grid(row=0, column=0, sticky="nsew")
        simple_tab = ttk.Frame(center_notebook, padding=4)
        asb_tab = ttk.Frame(center_notebook, padding=4)
        simple_tab.columnconfigure(0, weight=1)
        simple_tab.rowconfigure(1, weight=1)
        asb_tab.columnconfigure(0, weight=1)
        asb_tab.rowconfigure(0, weight=1)
        center_notebook.add(simple_tab, text="Simple")
        center_notebook.add(asb_tab, text="ASB")

        graphics_controls = ttk.LabelFrame(simple_tab, text="Graphics", padding=6)
        graphics_controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        graphics_controls.columnconfigure(0, weight=1)
        ttk.Label(graphics_controls, text="X = Front   Y = Right   Z = Up").grid(row=0, column=0, sticky="w")
        button_bar = ttk.Frame(graphics_controls)
        button_bar.grid(row=1, column=0, sticky="w", pady=(6, 0))
        for view_name in app.view_options:
            ttk.Radiobutton(
                button_bar,
                text=view_name,
                value=view_name,
                variable=app.view_mode_var,
                command=self.draw_active_view,
            ).pack(side="left", padx=(0, 8))
        ttk.Button(button_bar, text="Reset View", command=self.reset_view_transform).pack(side="left", padx=(8, 0))
        simple_view_frame = ttk.Frame(simple_tab)
        simple_view_frame.grid(row=1, column=0, sticky="nsew")
        simple_view_frame.columnconfigure(0, weight=1)
        simple_view_frame.rowconfigure(0, weight=1)

        app.view_canvas = self.make_view_canvas(simple_view_frame, "Orthographic View", 0)

        app.roll_frame = ttk.LabelFrame(simple_view_frame, text="Roll", padding=4)
        app.roll_frame.grid(row=0, column=1, sticky="ns", padx=(8, 0))
        app.roll_frame.rowconfigure(1, weight=1)
        ttk.Label(app.roll_frame, text="90 top").grid(row=0, column=0, sticky="n")
        roll_slider = tk.Scale(
            app.roll_frame,
            from_=90.0,
            to=0.0,
            orient="vertical",
            resolution=1.0,
            variable=app.roll_view_var,
            showvalue=False,
            command=self.on_roll_slider_changed,
            background=app.ui_bg,
            foreground=app.ui_fg,
            troughcolor=app.ui_panel,
            activebackground=app.ui_border,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            bd=0,
            relief="flat",
            length=420,
        )
        roll_slider.grid(row=1, column=0, sticky="ns", pady=(6, 6))
        roll_slider.configure(command=self.on_roll_slider_changed)
        ttk.Label(app.roll_frame, text="0 side").grid(row=2, column=0, sticky="s")
        ttk.Label(app.roll_frame, textvariable=app.roll_view_display_var).grid(row=3, column=0, sticky="s", pady=(6, 0))

        app.asb_frame = ttk.LabelFrame(asb_tab, text="ASB View", padding=6)
        app.asb_frame.grid(row=0, column=0, sticky="nsew")
        app.asb_frame.columnconfigure(0, weight=1)
        app.asb_frame.rowconfigure(1, weight=1)
        asb_controls = ttk.Frame(app.asb_frame)
        asb_controls.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(asb_controls, text="Update View", command=app.update_asb_view).pack(side="left")

        top = ttk.Notebook(right_frame)
        top.grid(row=0, column=0, rowspan=2, sticky="nsew")
        for group_name, tab_names in EDITOR_TABS.items():
            group = ttk.Frame(top, padding=(0, 4, 0, 0))
            group.columnconfigure(0, weight=1)
            group.rowconfigure(0, weight=1)
            top.add(group, text=group_name)
            notebook = ttk.Notebook(group)
            notebook.grid(row=0, column=0, sticky="nsew")
            for tab_name in tab_names:
                tab = ttk.Frame(notebook, padding=4)
                tab.columnconfigure(0, weight=1)
                tab.rowconfigure(0, weight=1)
                notebook.add(tab, text=tab_name)
                self.editor_frames[tab_name] = self.make_scroll_area(tab)


    def make_scroll_area(self, tab: ttk.Frame) -> ttk.Frame:
        canvas = tk.Canvas(tab, background=self.app.ui_bg, highlightthickness=1, highlightbackground=self.app.ui_border, highlightcolor=self.app.ui_accent, relief="flat")
        scroll_y = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.columnconfigure(0, weight=1)
        inner.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        window_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll_y.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window_id, width=event.width))
        return inner

    def text_tab(self, notebook: ttk.Notebook, title: str) -> tk.Text:
        """A notebook tab holding one scrollable, word-wrapped text box."""
        app = self.app
        tab = ttk.Frame(notebook, padding=4)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        notebook.add(tab, text=title)
        frame = ttk.LabelFrame(tab, text=title, padding=4)
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        text = tk.Text(
            frame,
            wrap="word",
            font=("TkFixedFont", 10),
            background=app.ui_panel,
            foreground=app.ui_fg,
            insertbackground=app.ui_fg,
            selectbackground=app.ui_border,
            selectforeground=app.ui_accent,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            relief="flat",
        )
        scroll_y = ttk.Scrollbar(frame, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scroll_y.set)
        for level, color in app.ui_level_colors.items():
            text.tag_configure(level, foreground=color)
        text.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        return text

    def draw_active_view(self) -> None:
        app = self.app
        if app.resolved is None or app.view_canvas is None:
            return
        if app.view_mode_var.get() == "Front":
            app.roll_frame.grid_remove()
        else:
            app.roll_frame.grid()
        draw_args = (app.view_canvas, app.resolved, app.aero, app.view_mode_var.get(), app.roll_view_var.get(), app.ui_draw, app.ui_bg)
        self.view_metrics = draw_canvas_view(*draw_args, self.view_zoom_scale, self.view_pan_x, self.view_pan_y, self.control_deflections)
        if self.clamp_view_pan():
            self.view_metrics = draw_canvas_view(*draw_args, self.view_zoom_scale, self.view_pan_x, self.view_pan_y, self.control_deflections)
        self.update_view_scrollbars()

    def on_view_drag_start(self, event) -> None:
        self.view_drag_last_xy = (event.x, event.y)
        self.drag_target = None
        if self.app.view_mode_var.get() != "Longitudinal":
            return
        for tag in self.app.view_canvas.gettags("current"):
            kind, _, index = tag.partition(":")
            if kind == "cut" and index:
                cuts = self.app.definition["sections"]["cuts_m"]
                self.drag_target = (cuts, int(index), event.x, cuts[int(index)])
                return
            if kind == "spars" and index:  # along the root chord from its leading edge
                spars = self.app.definition["surfaces"][int(index)]["spars"]
                self.drag_target = (spars, "x_offset_m", event.x, spars["x_offset_m"])
                return
            if kind in DRAG_KEYS and index:
                collection, key = DRAG_KEYS[kind]
                item = self.app.definition[collection][int(index)]
                self.drag_target = (item, key, event.x, item[key])
                return

    def on_view_drag_motion(self, event) -> None:
        if self.drag_target is not None:
            container, key, start_x, start_value = self.drag_target
            value = max(0.0, start_value + (event.x - start_x) / self.view_metrics["scale"])
            if container is self.app.definition["sections"]["cuts_m"]:
                low = container[key - 1] if key > 0 else 0.0
                high = container[key + 1] if key + 1 < len(container) else self.app.definition["fuselage"]["length_m"]
                value = min(max(value, low + MIN_SECTION_LENGTH_M), high - MIN_SECTION_LENGTH_M)
            container[key] = round(value, 4)
            self.app.recompute(run_aero=False)
            return
        if self.view_drag_last_xy is None:
            return
        last_x, last_y = self.view_drag_last_xy
        self.view_pan_x += event.x - last_x
        self.view_pan_y += event.y - last_y
        self.view_drag_last_xy = (event.x, event.y)
        self.draw_active_view()

    def on_view_drag_end(self, _event) -> None:
        self.view_drag_last_xy = None
        if self.drag_target is not None:
            self.drag_target = None
            self.app.recompute()
            self.rebuild_editors()

    def changed(self, rebuild: bool = False) -> None:
        self.sync_motor_runs()
        self.app.recompute()
        if rebuild:
            self.rebuild_editors()  # after recompute, so editors can show resolved values (control numbers)
            return
        if self.motor_info is not None and self.motor_info.winfo_exists():
            self.build_motor_info(self.motor_info.master, int(self.motor_info.grid_info()["row"]))
        self.refresh_fuselage_status()

    def refresh_fuselage_status(self) -> None:
        """Preset label and the computed section masses, after any edit."""
        self.preset_status_var.set(preset_status(self.app.definition))
        resolved = self.app.resolved
        if resolved is None or len(resolved.sections) != len(self.section_vars):
            return
        for index, (section, (range_var, empty_var, itemised_var)) in enumerate(zip(resolved.sections, self.section_vars)):
            range_var.set(f"S{index + 1} {section.x0_m * 1000:.0f}-{section.x1_m * 1000:.0f}")
            empty_var.set("-" if section.empty_estimate_kg is None else f"{section.empty_estimate_kg * 1000:.1f}")
            low, high = section.complete_itemised_kg
            itemised_var.set(f"{low * 1000:.1f}" if low == high else f"{low * 1000:.1f}-{high * 1000:.1f}")

    def add_entry(self, frame, row: int, data, key, optional: bool = False, label: str | None = None, unit_key: str | None = None, blank_hint: str | None = None,
                  on_commit=None) -> None:
        """Labelled entry row for one value of a dict (by key) or list (by index); see make_entry. unit_key names the
        field for units and label when key is a list index."""
        unit_key = unit_key or key
        ttk.Label(frame, text=label or field_label(unit_key)).grid(row=row, column=0, sticky="w")
        self.make_entry(frame, data, key, unit_key, optional, on_commit).grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=1)
        if optional:
            hint = blank_hint or ("blank = computed" if unit_key in GRAM_KEYS else "blank = none")
            ttk.Label(frame, text=hint).grid(row=row, column=2, sticky="w", padx=(6, 0))

    def make_entry(self, frame, data, key, unit_key: str, optional: bool = False, on_commit=None, width: int | None = None) -> ttk.Entry:
        """Entry (not gridded) for one value of a dict or list; commits on Return/focus-out only when the text changed.
        Optional: blank removes the key (a list entry becomes None). on_commit(old, new) runs after a change (e.g. to carry
        a rename to references), and the editors are then rebuilt."""
        present = type(data) is list or key in data
        initial = format_value(unit_key, data[key] if present else None)
        variable = tk.StringVar(value=initial)
        entry = ttk.Entry(frame, textvariable=variable) if width is None else ttk.Entry(frame, textvariable=variable, width=width)
        state = {"text": initial}

        def commit(_event=None) -> str:
            text = variable.get()
            if text.strip() == state["text"].strip():
                return "break"
            previous = data[key] if type(data) is list or key in data else None
            if optional and not text.strip():
                if type(data) is list:
                    data[key] = None
                else:
                    data.pop(key, None)
            else:
                existing = data[key] if (type(data) is list or key in data) and data[key] is not None else 0.0
                try:
                    data[key] = parse_value(unit_key, text, existing)
                except ValueError:
                    variable.set(state["text"])
                    self.app.log_error("Not a valid value", f"{field_label(unit_key)}: {text!r}")
                    return "break"
            state["text"] = text
            if on_commit is None:
                self.changed()
            else:
                on_commit(previous, data[key] if type(data) is list or key in data else None)
                self.changed(rebuild=True)
            return "break"

        def wheel(step: int) -> str:
            """Mouse wheel over a number: WHEEL_STEP_FRACTION of its value per notch, up multiplying and down dividing so a notch
            back undoes it (an integer by 1), committed at once."""
            value = data[key] if type(data) is list or key in data else None
            if type(value) not in (int, float) or type(value) is bool:
                return "break"
            variable.set(format_value(unit_key, value + step if type(value) is int else value * (1.0 + WHEEL_STEP_FRACTION) ** step))
            return commit()

        for sequence in ("<Return>", "<KP_Enter>", "<FocusOut>"):
            entry.bind(sequence, commit)
        entry.bind("<MouseWheel>", lambda event: wheel(1 if event.delta > 0 else -1))
        entry.bind("<Button-4>", lambda _event: wheel(1))
        entry.bind("<Button-5>", lambda _event: wheel(-1))
        return entry

    def add_vector_entries(self, frame, row: int, data: dict, key: str, axis_labels: tuple[str, ...]) -> int:
        for index, axis_label in enumerate(axis_labels):
            self.add_entry(frame, row + index, data[key], index, label=f"{field_label(key)} {axis_label}", unit_key=key)
        return row + len(axis_labels)

    def add_choice(self, frame, row: int, data, key, options: list[str], on_change=None, label: str | None = None, convert=str) -> None:
        """Labelled combobox row for one value of a dict (by key) or list (by index, with a label); see make_choice."""
        ttk.Label(frame, text=label or field_label(key)).grid(row=row, column=0, sticky="w")
        self.make_choice(frame, data, key, options, on_change, convert=convert).grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=1)

    def make_choice(self, frame, data, key, options: list[str], on_change=None, width: int | None = None, convert=str) -> ttk.Combobox:
        """Combobox (not gridded) for one value of a dict or list; a pick stores it (through convert) and rebuilds the editors."""
        variable = tk.StringVar(value=str(data[key] if type(data) is list else data.get(key, "")))
        box = ttk.Combobox(frame, values=options, textvariable=variable, state="readonly")
        if width is not None:
            box.configure(width=width)

        def select(_event=None) -> None:
            data[key] = convert(variable.get())
            if on_change is not None:
                on_change()
            self.changed(rebuild=True)

        box.bind("<<ComboboxSelected>>", select)
        return box

    def add_fields(self, frame, data: dict, start_row: int = 0, skip: set[str] = frozenset(), on_change: dict | None = None) -> int:
        """Generic rows for every field of a definition dict, in stored order; on_change maps a choice key to a callback."""
        row = start_row
        for key, value in list(data.items()):
            if key in skip:
                continue
            if type(value) is dict:
                group = ttk.LabelFrame(frame, text=field_label(key), padding=4)
                group.grid(row=row, column=0, columnspan=3, sticky="ew", pady=2)
                group.columnconfigure(1, weight=1)
                self.add_fields(group, value)
            elif key in CHOICES and type(value) is str:
                self.add_choice(frame, row, data, key, CHOICES[key], on_change=(on_change or {}).get(key))
            elif type(value) is list and key == "printer_bed_m":
                row = self.add_vector_entries(frame, row, data, key, ("x", "y", "z")) - 1
            elif type(value) is list and key == "size_m":
                row = self.add_vector_entries(frame, row, data, key, ("x", "y", "z")) - 1
            elif type(value) is list:
                ttk.Label(frame, text=f"{field_label(key)}: {len(value)} entries (edit in file)").grid(row=row, column=0, columnspan=3, sticky="w")
            else:
                self.add_entry(frame, row, data, key)
            row += 1
        return row

    def card(self, parent, row: int, title: str, on_delete) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=row, column=0, sticky="ew", padx=4, pady=4)
        frame.columnconfigure(1, weight=1)
        ttk.Button(frame, text="Delete", command=on_delete).grid(row=0, column=2, sticky="e")
        return frame

    def reset_definition_state(self) -> None:
        """A newly created or loaded definition: drop the blocks stashed from the previous one."""
        self.stashed_propulsion = None
        self.stashed_batteries = {}
        self.stashed_reinforcement = None

    def rebuild_editors(self) -> None:
        for frame in self.editor_frames.values():
            for child in frame.winfo_children():
                child.destroy()
        self.build_airframe_tab(self.editor_frames["Airframe"])
        self.build_fuselage_tab(self.editor_frames["Fuselage"])
        self.build_sections_group(self.editor_frames["Sections"], 0)
        self.refresh_fuselage_status()
        self.build_surfaces_tab(self.editor_frames["Surfaces"])
        self.build_volumes_tab(self.editor_frames["Volumes"])
        self.build_mounts_tab(self.editor_frames["Motor mounts"])
        self.build_propulsion_tab(self.editor_frames["Propulsion"])
        self.build_parts_tab(self.editor_frames["Parts"])
        self.build_wiring_tab(self.editor_frames["Wiring"])

    def build_airframe_tab(self, parent) -> None:
        definition = self.app.definition
        general = ttk.LabelFrame(parent, text="Airframe", padding=6)
        general.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        general.columnconfigure(1, weight=1)
        self.add_entry(general, 0, definition, "name")
        self.add_entry(general, 1, definition, "notes", optional=True)
        ttk.Label(general, text="type").grid(row=2, column=0, sticky="w")
        flags = ttk.Frame(general)
        flags.grid(row=2, column=1, columnspan=2, sticky="w", padx=(8, 0))
        for column, key in enumerate(TYPE_FLAGS):
            variable = tk.BooleanVar(value=definition[key])
            ttk.Checkbutton(flags, text=key, variable=variable, command=lambda k=key, v=variable: self.set_type_flag(k, v)).grid(row=0, column=column, padx=(0, 8))
        for row, (key, title) in enumerate((("operating_point", "Operating point (aero and stability)"), ("print", "Print")), start=1):
            group = ttk.LabelFrame(parent, text=title, padding=6)
            group.grid(row=row, column=0, sticky="ew", padx=4, pady=4)
            group.columnconfigure(1, weight=1)
            if key == "print":
                print_settings = definition["print"]
                self.add_choice(group, 0, print_settings, "material", sorted(self.app.parts_data["material"]))
                next_row = self.add_vector_entries(group, 1, print_settings, "printer_bed_m", ("x", "y", "z"))
                self.add_entry(group, next_row, print_settings, "bulk_density_kg_m3", optional=True)
            else:
                self.add_fields(group, definition[key])

    def set_type_flag(self, key: str, variable: tk.BooleanVar) -> None:
        """Keeps the flags consistent (tailsitter -> vtol -> powered); powered adds or removes the propulsion block,
        keeping the removed one to restore."""
        definition = self.app.definition
        chosen = {flag: definition[flag] for flag in TYPE_FLAGS} | {key: variable.get()}
        if chosen[key]:
            for implied in TYPE_FLAGS[TYPE_FLAGS.index(key) + 1:]:
                chosen[implied] = True
        else:
            for dependent in TYPE_FLAGS[:TYPE_FLAGS.index(key)]:
                chosen[dependent] = False
        if chosen["powered"] and "propulsion" not in definition:
            definition["propulsion"] = self.stashed_propulsion or self.new_propulsion()
        elif not chosen["powered"] and "propulsion" in definition:
            self.stashed_propulsion = definition.pop("propulsion")
        definition.update(chosen)
        self.changed(rebuild=True)

    def parts_of(self, category: str) -> list[str]:
        """Names of the placed parts of one library category."""
        return [part["name"] for part in self.app.definition["parts"] if part["part"].partition(":")[0] == category]

    def new_propulsion(self) -> dict:
        """NEW_MOTOR, NEW_PROP, the first library pack (in the first volume, else at mid-fuselage), the first ESC part
        (empty until placed and picked), one nose tractor."""
        definition = self.app.definition
        place = {"volume": definition["volumes"][0]["name"]} if definition["volumes"] else {"x_m": round(definition["fuselage"]["length_m"] / 2.0, 4)}
        battery = {"model": "commercial", "part": sorted(self.app.parts_data["battery"])[0], **place}
        return {
            "motor": dict(NEW_MOTOR), "prop": NEW_PROP, "battery": battery, "esc": next(iter(self.parts_of("esc")), ""),
            "mounts": [{"on": "nose", "direction": "tractor", "spin": SINGLE_PROP_SPINS[0], "thrust_axis_tilt_deg": 0.0}],
            "wiring": {"battery": copy.deepcopy(NEW_BATTERY_RUN), "motors": [copy.deepcopy(NEW_MOTOR_RUN)]},
        }

    def sync_motor_runs(self) -> None:
        """propulsion.wiring.motors follows the motor count: an added motor gets a copy of the last run, a removed one
        loses its own. Left alone while a mount names a surface that does not exist (a definition error says so)."""
        propulsion = self.app.definition.get("propulsion")
        if propulsion is None:
            return
        layouts = {surface["name"]: surface["layout"] for surface in self.app.definition["surfaces"]}
        mounts = [(mount["on"], mount.get("surface")) for mount in propulsion["mounts"]]
        if any(on == "surface" and surface not in layouts for on, surface in mounts):
            return
        count = motor_count(mounts, layouts)
        runs = propulsion["wiring"]["motors"]
        while len(runs) < count:
            runs.append(copy.deepcopy(runs[-1] if runs else NEW_MOTOR_RUN))
        del runs[count:]

    def build_wiring_tab(self, parent) -> None:
        definition = self.app.definition
        if "propulsion" not in definition:
            ttk.Label(parent, text="Not powered (type on the Airframe tab).").grid(row=0, column=0, sticky="w", padx=4, pady=4)
            return
        wiring = definition["propulsion"]["wiring"]
        resolved = self.app.resolved
        current = resolved is not None and resolved.wires is not None and len(resolved.wires.motors) == len(wiring["motors"])
        names = [motor.name for motor in resolved.motors] if current else [f"motor {index + 1}" for index in range(len(wiring["motors"]))]
        routes = [resolved.wires.battery, *resolved.wires.motors] if current else [None] * (len(wiring["motors"]) + 1)
        runs = [("battery -> ESC", wiring["battery"]), *((f"ESC -> {name}", run) for name, run in zip(names, wiring["motors"]))]
        group = ttk.LabelFrame(parent, text="Wire runs (auto-routed: along the axes, wing motors inside their panel; drawn in the side view)", padding=6)
        group.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        for column, title in enumerate(WIRING_COLUMNS):
            ttk.Label(group, text=title).grid(row=0, column=column, sticky="w", padx=(0, 6))
        gauges = [str(gauge) for gauge in sorted(wire.gauge for wire in motor_repo().wires)]
        connectors = sorted(name for name, connector in self.app.parts_data["connector"].items() if "contact_resistance_ohm" in connector)
        for row, ((label, run), route) in enumerate(zip(runs, routes), start=1):
            ttk.Label(group, text=label).grid(row=row, column=0, sticky="w", padx=(0, 6))
            ttk.Label(group, text="-" if route is None else f"{route_length_m(route) * 1000:.0f}").grid(row=row, column=1, sticky="w", padx=(0, 6))
            self.make_choice(group, run, "gauge_awg", gauges, width=4, convert=int).grid(row=row, column=2, sticky="w", padx=(0, 6), pady=1)
            self.make_entry(group, run, "conductors", "conductors", width=3).grid(row=row, column=3, sticky="w", padx=(0, 6), pady=1)
            plugs = ttk.Frame(group)
            plugs.grid(row=row, column=4, sticky="w")
            first = 0
            if run is wiring["battery"]:  # the battery's own connector comes first, set in its part library entry
                ttk.Label(plugs, text=self.battery_connector()).grid(row=0, column=0, padx=(0, 6))
                first = 1
            for index in range(len(run["plugs"])):
                self.plug_box(plugs, run["plugs"], index, [*connectors, REMOVE_PLUG]).grid(row=0, column=first + index, padx=(0, 2))
            self.plug_box(plugs, run["plugs"], None, connectors).grid(row=0, column=first + len(run["plugs"]))

    def battery_connector(self) -> str:
        """The propulsion battery's own connector: its library entry's, or the custom pack's ("?" while it has none)."""
        battery = self.app.definition["propulsion"]["battery"]
        if battery["model"] == "commercial":
            return self.app.parts_data["battery"].get(battery.get("part"), {}).get("connector", "?")
        return battery.get("connector", "?")

    def plug_box(self, frame, plugs: list[str], index: int | None, options: list[str]) -> ttk.Combobox:
        """A plug on a run (REMOVE_PLUG takes it off), or with index None the ADD_PLUG box that appends one."""
        variable = tk.StringVar(value=ADD_PLUG if index is None else plugs[index])
        box = ttk.Combobox(frame, values=options, textvariable=variable, state="readonly", width=max(len(option) for option in [ADD_PLUG, *options]))

        def select(_event=None) -> None:
            choice = variable.get()
            if index is None:
                plugs.append(choice)
            elif choice == REMOVE_PLUG:
                del plugs[index]
            else:
                plugs[index] = choice
            self.changed(rebuild=True)

        box.bind("<<ComboboxSelected>>", select)
        return box

    def build_fuselage_tab(self, parent) -> None:
        fuselage = self.app.definition["fuselage"]
        self.build_preset_group(parent, 0)
        group = ttk.LabelFrame(parent, text="Fuselage", padding=6)
        group.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        self.add_entry(group, 0, fuselage, "length_m")
        row = 1
        for key in ("section", "nose", "tail", "joints"):
            sub = ttk.LabelFrame(group, text=key, padding=4)
            sub.grid(row=row, column=0, columnspan=3, sticky="ew", pady=2)
            sub.columnconfigure(1, weight=1)
            if key in ("nose", "tail"):
                end = fuselage[key]
                self.add_choice(sub, 0, end, "profile", CHOICES["profile"], on_change=lambda e=end: self.reset_profile_params(e))
                self.add_entry(sub, 1, end, "length_m")
                self.add_fields(sub, end["params"], start_row=2)
            else:
                self.add_fields(sub, fuselage[key])
            row += 1
        self.build_reinforcement_group(parent, 2)

    def build_preset_group(self, parent, grid_row: int) -> None:
        group = ttk.LabelFrame(parent, text="Size preset (data/presets/)", padding=6)
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        ttk.Label(group, text="aircraft").grid(row=0, column=0, sticky="w")
        ttk.Label(group, textvariable=self.preset_status_var).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(group, text="preset").grid(row=1, column=0, sticky="w")
        chosen = tk.StringVar(value=self.app.definition.get("preset") or NO_PRESET)
        ttk.Combobox(group, values=[NO_PRESET, *preset_names()], textvariable=chosen, state="readonly").grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=1)
        buttons = ttk.Frame(group)
        buttons.grid(row=2, column=1, sticky="w", padx=(8, 0), pady=(4, 0))
        ttk.Button(buttons, text="Load", command=lambda: self.load_preset(chosen.get())).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(buttons, text="Save", command=lambda: self.save_preset_as(chosen.get(), overwrite=True)).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(buttons, text="New", command=self.new_preset).grid(row=0, column=2)

    def load_preset(self, name: str) -> None:
        """Fill the section, joints and rods from the preset and label the aircraft with it; none clears the label."""
        if name == NO_PRESET:
            self.app.definition.pop("preset", None)
        else:
            apply_preset(self.app.definition, name)
        self.changed(rebuild=True)

    def save_preset_as(self, name: str, overwrite: bool) -> None:
        if name == NO_PRESET:
            self.app.recompute_status_var.set("Pick a preset to save over, or New")
            return
        if overwrite and not messagebox.askyesno(
                "Save preset", f"Overwrite preset {name} (data/presets/{name}.json) with this aircraft's section, joints and rods?\n"
                f"Every aircraft labelled {name} then shows as modified where it differs. Its spars, limits and CAD data stay.", parent=self.frame):
            return
        try:
            save_preset(name, self.app.definition, self.app.parts_data)
        except AirframeError as error:
            self.app.log_error("Save preset", f"Not saved, data/presets/ unchanged:\n{error}")
            return
        self.app.definition["preset"] = name
        self.changed(rebuild=True)

    def new_preset(self) -> None:
        name = simpledialog.askstring("New preset", "Preset name (e.g. M62):", parent=self.frame)
        if not name:
            return
        if name in preset_names() or name == NO_PRESET:
            self.app.log_error("New preset", f"A preset named {name} already exists.")
            return
        self.save_preset_as(name, overwrite=False)

    def build_reinforcement_group(self, parent, grid_row: int) -> None:
        fuselage = self.app.definition["fuselage"]
        group = ttk.LabelFrame(parent, text="Reinforcement (longitudinal rods)", padding=6)
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        enabled = tk.BooleanVar(value="reinforcement" in fuselage)
        ttk.Checkbutton(group, text="rods", variable=enabled, command=lambda: self.toggle_reinforcement(enabled)).grid(row=0, column=0, sticky="w")
        if "reinforcement" not in fuselage:
            return
        rods = fuselage["reinforcement"]
        self.add_choice(group, 1, rods, "type", list(REINFORCEMENT_TYPES))
        self.add_choice(group, 2, rods, "material", sorted(self.app.parts_data["material"]))
        for row, key in enumerate(("diameter_m", "radial_position_m", "polar_n", "polar_offset_angle", "polar_split", "polar_split_angle", "from_x_m", "to_x_m"), start=3):
            self.add_entry(group, row, rods, key)
        ttk.Label(group, text="one rod position per quadrant, polar_offset_angle (deg) up from the horizontal, mirrored left/right and top/bottom; "
                                   "polar_split 2 = a pair at +/- polar_split_angle (deg), 0 = single; polar_n = all rods", wraplength=360, justify="left").grid(
            row=11, column=0, columnspan=3, sticky="w")

    def toggle_reinforcement(self, variable: tk.BooleanVar) -> None:
        """Ticking restores the unticked rods, or takes the preset's pattern, or NEW_ROD_PATTERN against the inner wall."""
        definition = self.app.definition
        fuselage = definition["fuselage"]
        if not variable.get():
            self.stashed_reinforcement = fuselage.pop("reinforcement", None)
        elif self.stashed_reinforcement is not None:
            fuselage["reinforcement"] = self.stashed_reinforcement
        else:
            preset = read_presets().get(definition.get("preset"), {})
            if "reinforcement" in preset:
                pattern = {key: preset["reinforcement"][key] for key in ROD_PATTERN_KEYS}
            else:
                section = fuselage["section"]
                inner_radius = min(section["width_m"], section["height_m"]) / 2.0 - section["wall_radial_m"]
                pattern = NEW_ROD_PATTERN | {"radial_position_m": round(inner_radius - NEW_ROD_PATTERN["diameter_m"] / 2.0, 6)}
            fuselage["reinforcement"] = copy.deepcopy(pattern) | default_rod_extent(definition)
        self.changed(rebuild=True)

    def build_sections_group(self, parent, grid_row: int) -> None:
        sections = self.app.definition["sections"]
        cuts, rows, joints = sections["cuts_m"], sections["per_section"], sections["cut_joints"]
        group = ttk.LabelFrame(parent, text="Sections (cut lines; drag them in the side view)", padding=6)
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(group, text="Add cut", command=self.add_cut).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        for column, title in enumerate(SECTION_COLUMNS):
            ttk.Label(group, text=title).grid(row=1, column=column, sticky="w", padx=(0, 4))
        self.section_vars = []
        for index, row_data in enumerate(rows):
            grid_row = index + 2
            range_var, empty_var, itemised_var = tk.StringVar(), tk.StringVar(), tk.StringVar()
            self.section_vars.append((range_var, empty_var, itemised_var))
            ttk.Label(group, textvariable=range_var).grid(row=grid_row, column=0, sticky="w", padx=(0, 4))
            if index < len(cuts):
                self.make_entry(group, cuts, index, "cuts_m", width=SECTION_ENTRY_WIDTH).grid(row=grid_row, column=1, sticky="w", padx=(0, 4), pady=1)
                self.make_choice(group, joints, index, list(CUT_JOINTS), width=10).grid(row=grid_row, column=2, sticky="w", padx=(0, 4), pady=1)
            solid = tk.BooleanVar(value=row_data["solid"])
            ttk.Checkbutton(group, variable=solid, command=lambda r=row_data, v=solid: self.set_solid(r, v)).grid(row=grid_row, column=3)
            for column, key in ((4, "empty_mass_kg"), (6, "complete_mass_kg")):
                self.make_entry(group, row_data, key, key, optional=True, width=SECTION_ENTRY_WIDTH).grid(row=grid_row, column=column, sticky="w", padx=(0, 4), pady=1)
            ttk.Label(group, textvariable=empty_var).grid(row=grid_row, column=5, sticky="w", padx=(0, 4))
            ttk.Label(group, textvariable=itemised_var).grid(row=grid_row, column=7, sticky="w", padx=(0, 4))
            if index < len(cuts):
                ttk.Button(group, text="Del", width=4, command=lambda i=index: self.delete_cut(i)).grid(row=grid_row, column=8, sticky="e")
        ttk.Label(group, wraplength=360, justify="left",
                  text="cut mm: the section's end. Blank empty = calc (hollow: shell wall; solid: less its volumes). Blank complete = items: "
                       "everything attached in the section, which a weighed complete mass replaces.").grid(
            row=len(rows) + 2, column=0, columnspan=len(SECTION_COLUMNS), sticky="w", pady=(4, 0))

    def set_solid(self, row: dict, variable: tk.BooleanVar) -> None:
        row["solid"] = variable.get()
        self.changed()

    def add_cut(self) -> None:
        """Split the longest section in two; both halves keep its solid flag and lose its weighed masses."""
        sections = self.app.definition["sections"]
        cuts, rows = sections["cuts_m"], sections["per_section"]
        bounds = [0.0, *cuts, self.app.definition["fuselage"]["length_m"]]
        longest = max(range(len(bounds) - 1), key=lambda i: bounds[i + 1] - bounds[i])
        cuts.insert(longest, round((bounds[longest] + bounds[longest + 1]) / 2.0, 4))
        sections["cut_joints"].insert(longest, NEW_CUT_JOINT)
        rows[longest:longest + 1] = [{"solid": rows[longest]["solid"]}, {"solid": rows[longest]["solid"]}]
        self.changed(rebuild=True)

    def delete_cut(self, index: int) -> None:
        """Merge the sections either side of a cut: the left one's solid flag, each weighed mass summed when both have one."""
        sections = self.app.definition["sections"]
        sections["cuts_m"].pop(index)
        sections["cut_joints"].pop(index)
        left, right = sections["per_section"][index:index + 2]
        merged = {"solid": left["solid"]}
        for key in ("empty_mass_kg", "complete_mass_kg"):
            if key in left and key in right:
                merged[key] = left[key] + right[key]
        sections["per_section"][index:index + 2] = [merged]
        self.changed(rebuild=True)

    def reset_profile_params(self, end: dict) -> None:
        end["params"] = {key: end["params"].get(key, PROFILE_PARAM_DEFAULTS[key]) for key in POINTED_SHAPE_PARAM_KEYS[end["profile"]]}

    def build_surfaces_tab(self, parent) -> None:
        surfaces = self.app.definition["surfaces"]
        ttk.Button(parent, text="Add surface set", command=self.add_surface).grid(row=0, column=0, sticky="w", padx=4, pady=4)
        for index, surface in enumerate(surfaces):
            frame = self.card(parent, index + 1, surface["name"], lambda i=index: self.delete_item("surfaces", i))
            skip = set() if surface["layout"] in ("1", "3") else {"fin_position"}
            row = self.add_fields(frame, surface, start_row=1, skip=skip | {"mass_kg", "control", "spars", "root_base"}, on_change={"layout": lambda s=surface: self.fit_servos(s)})
            self.add_entry(frame, row, surface, "mass_kg", optional=True)
            self.build_control(frame, row + 1, surface)
            row += 3
            definition = self.app.definition
            row = self.optional_block(frame, row, surface, "spars", "spars", preset_spars(definition) or NEW_SPARS,
                                      nested=("carry_through", "carry-through", preset_carry_through(definition) or NEW_CARRY_THROUGH))
            row = self.optional_block(frame, row, surface, "root_base", "root base (not built yet)", NEW_ROOT_BASE)

    def build_control(self, frame, row: int, surface: dict) -> None:
        enabled = tk.BooleanVar(value="control" in surface)
        ttk.Checkbutton(frame, text="control surface", variable=enabled, command=lambda: self.toggle_control(surface, enabled)).grid(row=row, column=0, sticky="w")
        if "control" not in surface:
            return
        control = surface["control"]
        group = ttk.LabelFrame(frame, text="control surface", padding=4)
        group.grid(row=row + 1, column=0, columnspan=3, sticky="ew", pady=2)
        group.columnconfigure(1, weight=1)
        self.add_choice(group, 0, control, "kind", list(CONTROL_KINDS), on_change=lambda: self.reset_control_kind(control))
        inner = 1
        for key in CONTROL_KIND_FIELDS[control["kind"]]:
            self.add_entry(group, inner, control, key)
            inner += 1
        ttk.Label(group, text="roles").grid(row=inner, column=0, sticky="w")
        roles = ttk.Frame(group)
        roles.grid(row=inner, column=1, columnspan=2, sticky="w", padx=(8, 0))
        for column, role in enumerate(CONTROL_ROLES):
            variable = tk.BooleanVar(value=role in control["roles"])
            ttk.Checkbutton(roles, text=role, variable=variable, command=lambda r=role, v=variable: self.toggle_role(control, r, v)).grid(row=0, column=column, padx=(0, 8))
        inner += 1
        for key in ("deflection_min_deg", "deflection_max_deg", "rate_deg_s"):
            self.add_entry(group, inner, control, key)
            inner += 1
        resolved = self.app.resolved
        numbers = {(placement.surface, placement.panel_index): placement.number for placement in resolved.controls} if resolved is not None else {}
        for index in range(len(control["servos"])):
            number = numbers.get((surface["name"], index))
            label = f"panel {index + 1} servo" + (f" (C{number})" if number is not None else "")
            self.add_choice(group, inner, control["servos"], index, list(SERVO_OUTPUTS), label=label)
            inner += 1

    def free_servos(self, count: int) -> list[str]:
        """Lowest servo outputs no control surface uses yet, then S1 (outputs may be shared)."""
        used = {servo for surface in self.app.definition["surfaces"] if "control" in surface for servo in surface["control"]["servos"]}
        return ([servo for servo in SERVO_OUTPUTS if servo not in used] + [SERVO_OUTPUTS[0]] * count)[:count]

    def toggle_control(self, surface: dict, variable: tk.BooleanVar) -> None:
        if variable.get():
            control = copy.deepcopy(NEW_CONTROL)
            control.update(copy.deepcopy(CONTROL_KIND_FIELDS[control["kind"]]))
            control["servos"] = self.free_servos(LAYOUT_PANEL_COUNTS[surface["layout"]])
            surface["control"] = control
        else:
            surface.pop("control", None)
        self.changed(rebuild=True)

    def reset_control_kind(self, control: dict) -> None:
        for fields in CONTROL_KIND_FIELDS.values():
            for key in fields:
                control.pop(key, None)
        control.update(copy.deepcopy(CONTROL_KIND_FIELDS[control["kind"]]))

    def fit_servos(self, surface: dict) -> None:
        """Layout change: keep one servo per panel."""
        if "control" not in surface:
            return
        servos = surface["control"]["servos"]
        count = LAYOUT_PANEL_COUNTS[surface["layout"]]
        surface["control"]["servos"] = servos[:count] + self.free_servos(count - len(servos)) if count > len(servos) else servos[:count]

    def toggle_role(self, control: dict, role: str, variable: tk.BooleanVar) -> None:
        chosen = (set(control["roles"]) - {role}) | ({role} if variable.get() else set())
        control["roles"] = [item for item in CONTROL_ROLES if item in chosen]
        self.changed()

    def add_surface(self) -> None:
        surface = copy.deepcopy(NEW_SURFACE)
        names = {item["name"] for item in self.app.definition["surfaces"]}
        surface["name"] = next(f"wing {n}" for n in range(1, len(names) + 2) if f"wing {n}" not in names)
        self.app.definition["surfaces"].append(surface)
        self.changed(rebuild=True)

    def delete_item(self, collection: str, index: int) -> None:
        self.app.definition[collection].pop(index)
        self.changed(rebuild=True)

    def build_propulsion_tab(self, parent) -> None:
        definition = self.app.definition
        if "propulsion" not in definition:
            ttk.Label(parent, text="Not powered (type on the Airframe tab).").grid(row=0, column=0, sticky="w", padx=4, pady=4)
            return
        propulsion = definition["propulsion"]
        motor = propulsion["motor"]
        self.build_battery_group(parent, 0, propulsion)
        group = ttk.LabelFrame(parent, text="Motor, prop and ESC (motorlib)", padding=6)
        group.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        repo = motor_repo()
        motors_by_manufacturer = {}  # motors with a stator size (the drawing and mass model need it)
        for code, name in repo.manufacturer_by_idx.items():
            names = sorted(m.name for m in repo.motors_for(code) if m.stator_diameter_mm is not None and m.stator_height_mm is not None)
            if names:
                motors_by_manufacturer[name] = names
        self.add_choice(group, 1, motor, "manufacturer", sorted(motors_by_manufacturer, key=str.lower),
                        on_change=lambda: motor.update(name=motors_by_manufacturer[motor["manufacturer"]][0]))
        self.add_choice(group, 2, motor, "name", motors_by_manufacturer.get(motor["manufacturer"], []), label="motor")
        self.add_choice(group, 3, propulsion, "prop", sorted(repo.prop_by_name, key=str.lower))
        self.add_choice(group, 4, propulsion, "esc", self.parts_of("esc"), label="ESC")
        if self.app.propulsion is not None:
            self.build_motor_info(parent, 2)

    def build_battery_group(self, parent, grid_row: int, propulsion: dict) -> None:
        """The pack: a commercial one from data/parts/battery.json, or a custom stack of motorlib cells; where it sits."""
        battery = propulsion["battery"]
        group = ttk.LabelFrame(parent, text="Battery", padding=6)
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        ttk.Label(group, text="Battery model").grid(row=0, column=0, sticky="w")
        model_var = tk.StringVar(value=BATTERY_MODEL_LABELS[battery["model"]])
        model_box = ttk.Combobox(group, values=list(BATTERY_MODEL_LABELS.values()), textvariable=model_var, state="readonly")
        model_box.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=1)
        model_box.bind("<<ComboboxSelected>>", lambda _event: self.set_battery_model(
            propulsion, next(model for model, label in BATTERY_MODEL_LABELS.items() if label == model_var.get())))
        if battery["model"] == "commercial":
            self.add_choice(group, 1, battery, "part", sorted(self.app.parts_data["battery"]), label="pack")
            self.add_entry(group, 2, battery, "mass_kg", optional=True, blank_hint="blank = library")
            row = 3
        else:
            cells = sorted(name for name in motor_repo().cell_by_name if cell_format(name) is not None)
            connectors = sorted(name for name, connector in self.app.parts_data["connector"].items() if "contact_resistance_ohm" in connector)
            gauges = [str(gauge) for gauge in sorted(wire.gauge for wire in motor_repo().wires)]
            self.add_entry(group, 1, battery, "s", label="S (cells in series)")
            self.add_entry(group, 2, battery, "p", label="P (in parallel)")
            self.add_choice(group, 3, battery, "cell", cells, label="cell (motorlib)")
            self.add_choice(group, 4, battery, "connector", connectors)
            self.add_choice(group, 5, battery, "cell_axis", list(AXES), label="cell length along")
            for index, axis in enumerate(AXES):
                self.add_entry(group, 6 + index, battery["stacking"], index, label=f"cells along {axis}", unit_key="stacking")
            self.add_choice(group, 9, battery, "lead_awg", gauges, label="lead AWG", convert=int)
            self.add_entry(group, 10, battery, "lead_length_m", label="lead length (mm)")
            row = 11
        row = self.add_placement(group, row, battery, BATTERY_PART_NAME)
        resolved = self.app.resolved
        if resolved is not None and resolved.airframe.propulsion is not None:
            pack = resolved.airframe.propulsion.battery
            size = "no size" if pack.placed.size_m is None else " x ".join(f"{value * 1000:.0f}" for value in pack.placed.size_m) + " mm"
            ttk.Label(group, text=f"{pack.label}: {size}, {pack.placed.mass_kg[0] * 1000:.0f} g, {pack.capacity_mah:.0f} mAh", wraplength=400).grid(
                row=row, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def set_battery_model(self, propulsion: dict, model: str) -> None:
        """Another battery model keeps the placement; its other fields come back as they were when it was last left
        (NEW_CUSTOM_PACK, or the first library pack, the first time)."""
        battery = propulsion["battery"]
        if model == battery["model"]:
            return
        kept = (*PLACEMENT_KEYS, *ORIENTATION_KEYS)  # where it is and how it is turned
        self.stashed_batteries[battery["model"]] = {key: value for key, value in battery.items() if key not in kept}
        placement = {key: value for key, value in battery.items() if key in kept}
        fields = self.stashed_batteries.get(model) or (copy.deepcopy(NEW_CUSTOM_PACK) if model == "custom_liion" else
                                                         {"model": "commercial", "part": sorted(self.app.parts_data["battery"])[0]})
        propulsion["battery"] = {**fields, **placement}
        self.changed(rebuild=True)

    def build_mounts_tab(self, parent) -> None:
        definition = self.app.definition
        if "propulsion" not in definition:
            ttk.Label(parent, text="Not powered (type on the Airframe tab).").grid(row=0, column=0, sticky="w", padx=4, pady=4)
            return
        propulsion = definition["propulsion"]
        ttk.Button(parent, text="Add motor mount", command=self.add_mount).grid(row=0, column=0, sticky="w", padx=4, pady=4)
        surface_names = [surface["name"] for surface in definition["surfaces"]]
        for index, mount in enumerate(propulsion["mounts"]):
            frame = self.card(parent, index + 1, f"mount {index + 1}", lambda i=index: self.delete_mount(i))
            self.add_choice(frame, 1, mount, "on", CHOICES["on"], on_change=lambda m=mount: self.normalize_mount(m))
            row = 2
            if mount["on"] == "surface":
                self.add_choice(frame, row, mount, "surface", surface_names)
                self.add_entry(frame, row + 1, mount, "span_position_m")
                self.add_entry(frame, row + 2, mount, "x_offset_m")
                row += 3
            self.add_choice(frame, row, mount, "direction", CHOICES["direction"])
            self.add_choice(frame, row + 1, mount, "spin", list(SURFACE_PROP_SPINS if mount["on"] == "surface" else SINGLE_PROP_SPINS))
            self.add_entry(frame, row + 2, mount, "thrust_axis_tilt_deg")
            row += 2
            self.optional_block(frame, row + 1, mount, "pod", "pod", NEW_POD)

    def optional_block(self, frame, row: int, data: dict, key: str, label: str, template: dict, nested: tuple[str, str, dict] | None = None) -> int:
        """A tick box adding (from template) or removing data[key], and its fields when present, with nested (key, label,
        template) as an optional block of its own inside it; returns the next free row."""
        variable = tk.BooleanVar(value=key in data)
        ttk.Checkbutton(frame, text=label, variable=variable, command=lambda: self.toggle_block(data, key, template, variable)).grid(row=row, column=0, sticky="w")
        if key not in data:
            return row + 1
        group = ttk.LabelFrame(frame, text=label, padding=4)
        group.grid(row=row + 1, column=0, columnspan=3, sticky="ew")
        group.columnconfigure(1, weight=1)
        block = data[key]
        if "material" in block:
            self.add_choice(group, 0, block, "material", sorted(self.app.parts_data["material"]))
        inner = self.add_fields(group, block, start_row=1, skip={"material"} | ({nested[0]} if nested else set()))
        if nested is not None:
            self.optional_block(group, inner, block, *nested)
        return row + 2

    def toggle_block(self, data: dict, key: str, template: dict, variable: tk.BooleanVar) -> None:
        if variable.get():
            data[key] = copy.deepcopy(template)
        else:
            data.pop(key, None)
        self.changed(rebuild=True)

    def build_motor_info(self, parent, grid_row: int) -> None:
        """Static propulsion of the last valid airframe (app.propulsion): motorlib's model against INAV motor output (duty)."""
        app = self.app
        if self.motor_info is not None and self.motor_info.winfo_exists():
            self.motor_info.destroy()
        group = ttk.LabelFrame(parent, text="Motor info (motorlib, static)", padding=6)
        self.motor_info = group
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(0, weight=1)
        summary = app.propulsion
        figure = Figure(figsize=(4.6, 3.4), dpi=100, facecolor=app.ui_bg)
        axis = figure.add_subplot()
        curve = summary.curve
        for label, values, color in (("Thrust per motor (N)", curve.thrust_n, "#19ff19"), ("RPM / 1000", curve.rpm / 1000.0, "#33ccff"),
                                     ("kW per motor", curve.power_w / 1000.0, "#ffcc33")):
            axis.plot(curve.duty, values, color=color, linewidth=2.0, label=label)
        if summary.hover is not None:
            axis.axvline(float(summary.hover.duty), color=app.ui_fg, linestyle="--", linewidth=1.0, label="hover")
        axis.set_xlabel("INAV motor output (ESC duty)", color=app.ui_fg)
        axis.tick_params(colors=app.ui_fg)
        axis.set_facecolor(app.ui_bg)
        axis.grid(color=app.ui_border, alpha=0.4)
        legend = axis.legend(loc="upper left", facecolor=app.ui_panel, edgecolor=app.ui_border, framealpha=1.0, fontsize=8)
        for text in legend.get_texts():
            text.set_color(app.ui_fg)
        figure.tight_layout()
        canvas = FigureCanvasTkAgg(figure, master=group)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky="ew")
        ttk.Label(group, text="\n".join(line.text for line in summary_lines(summary)), font=("TkFixedFont", 9), justify="left", wraplength=440).grid(row=1, column=0, sticky="w", pady=(6, 0))

    def normalize_mount(self, mount: dict) -> None:
        surfaces = self.app.definition["surfaces"]
        if mount["on"] == "surface":
            mount.setdefault("surface", surfaces[0]["name"] if surfaces else "")
            mount.setdefault("span_position_m", 0.0)
            mount.setdefault("x_offset_m", 0.0)
            if mount["spin"] not in SURFACE_PROP_SPINS:
                mount["spin"] = SURFACE_PROP_SPINS[0]
        else:
            for key in ("surface", "span_position_m", "x_offset_m"):
                mount.pop(key, None)
            mount["direction"] = "tractor" if mount["on"] == "nose" else "pusher"
            if mount["spin"] not in SINGLE_PROP_SPINS:
                mount["spin"] = SINGLE_PROP_SPINS[0]

    def add_mount(self) -> None:
        mount = {"on": "nose", "direction": "tractor", "spin": SINGLE_PROP_SPINS[0], "thrust_axis_tilt_deg": 0.0}
        self.app.definition["propulsion"]["mounts"].append(mount)
        self.changed(rebuild=True)

    def delete_mount(self, index: int) -> None:
        self.app.definition["propulsion"]["mounts"].pop(index)
        self.changed(rebuild=True)

    def build_volumes_tab(self, parent) -> None:
        volumes = self.app.definition["volumes"]
        ttk.Label(parent, text="Internal space; parts sit in volumes (Parts tab). Mass: grams, or a range like 10-30.\n"
                               "Drag volumes along the body in the side view (their parts move with them).").grid(row=0, column=0, sticky="w", padx=4)
        ttk.Button(parent, text="Add volume", command=self.add_volume).grid(row=1, column=0, sticky="w", padx=4, pady=4)
        for index, volume in enumerate(volumes):
            frame = self.card(parent, index + 2, volume["name"], lambda i=index: self.delete_volume(i))
            self.add_entry(frame, 1, volume, "name", on_commit=self.rename_volume)
            self.add_choice(frame, 2, volume, "shape", list(VOLUME_SHAPES), on_change=lambda v=volume: self.reset_volume_shape(v))
            row = 3
            for key in ("x_m", "y_m", "z_m"):
                self.add_entry(frame, row, volume, key, optional=key != "x_m")
                row += 1
            if "size_m" in VOLUME_SHAPE_FIELDS[volume["shape"]]:
                row = self.add_vector_entries(frame, row, volume, "size_m", ("x", "y", "z"))
            for key in VOLUME_SHAPE_FIELDS[volume["shape"]]:
                if key != "size_m":
                    if key == "axis":
                        self.add_choice(frame, row, volume, key, CHOICES["axis"])
                    else:
                        self.add_entry(frame, row, volume, key)
                    row += 1
            self.add_entry(frame, row, volume, "mass_kg", optional=True, blank_hint="blank = space only")
            row += 1
            contents = [part["name"] for part in self.app.definition["parts"] if part.get("volume") == volume["name"]]
            ttk.Label(frame, text=f"parts: {', '.join(contents) if contents else 'none'}").grid(row=row, column=0, columnspan=3, sticky="w")

    def delete_volume(self, index: int) -> None:
        """Delete a volume; the parts (and the battery) in it stay where they are, at their own x, y, z."""
        definition = self.app.definition
        volume = definition["volumes"][index]
        centre = (volume["x_m"], volume.get("y_m", 0.0), volume.get("z_m", 0.0))
        held = list(definition["parts"])
        if "propulsion" in definition:
            held.append(definition["propulsion"]["battery"])
        for item in held:
            if item.get("volume") != volume["name"]:
                continue
            position = [c + item.get(f"{axis}_offset_m", 0.0) for c, axis in zip(centre, AXES)]
            for key in PLACEMENT_KEYS:
                item.pop(key, None)
            item.update({f"{axis}_m": round(value, 6) for axis, value in zip(AXES, position)})
        definition["volumes"].pop(index)
        self.changed(rebuild=True)

    def rename_volume(self, old: str, new: str) -> None:
        for part in self.app.definition["parts"]:
            if part.get("volume") == old:
                part["volume"] = new

    def build_parts_tab(self, parent) -> None:
        definition = self.app.definition
        ttk.Label(parent, text="Library components (data/parts/), in a volume (offset from its centre) or at their own x, y, z.\n"
                               "Mass: grams, or a range like 10-30; 0 = counted elsewhere (e.g. a weighed section).").grid(row=0, column=0, sticky="w", padx=4)
        ttk.Button(parent, text="Add part", command=self.add_part).grid(row=1, column=0, sticky="w", padx=4, pady=4)
        library = [f"{category}:{name}" for category in PART_CATEGORIES for name in sorted(self.app.parts_data[category])]
        for index, part in enumerate(definition["parts"]):
            frame = self.card(parent, index + 2, part["name"], lambda i=index: self.delete_item("parts", i))
            self.add_entry(frame, 1, part, "name", on_commit=self.rename_part)
            self.add_choice(frame, 2, part, "part", library)
            mass_row = self.add_placement(frame, 3, part, part["name"])
            category, _, name = part["part"].partition(":")
            library_mass = self.app.parts_data[category][name].get("mass") if name in self.app.parts_data.get(category, {}) else None
            hint = "blank = library" if library_mass else "required (none in library)"
            self.add_entry(frame, mass_row, part, "mass_kg", optional=True, blank_hint=hint)

    def add_placement(self, frame, row: int, item: dict, placed_name: str) -> int:
        """Volume dropdown (NO_VOLUME first) and the offsets in it, or the item's own x, y, z, then its yaw, pitch, roll;
        returns the next row. placed_name: the item among the resolved parts."""
        ttk.Label(frame, text="volume").grid(row=row, column=0, sticky="w")
        volume_var = tk.StringVar(value=item.get("volume", NO_VOLUME))
        volume_box = ttk.Combobox(frame, values=[NO_VOLUME, *(volume["name"] for volume in self.app.definition["volumes"])], textvariable=volume_var, state="readonly")
        volume_box.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=1)
        volume_box.bind("<<ComboboxSelected>>", lambda _event: self.set_part_volume(item, volume_var.get(), placed_name))
        if "volume" in item:
            for offset, key in enumerate(("x_offset_m", "y_offset_m", "z_offset_m"), start=1):
                self.add_entry(frame, row + offset, item, key, optional=True, blank_hint="blank = 0")
        else:
            self.add_entry(frame, row + 1, item, "x_m")
            for offset, key in enumerate(("y_m", "z_m"), start=2):
                self.add_entry(frame, row + offset, item, key, optional=True, blank_hint="blank = 0")
        for offset, key in enumerate(ORIENTATION_KEYS, start=4):
            self.add_entry(frame, row + offset, item, key, optional=True, blank_hint="blank = 0")
        return row + 7

    def set_part_volume(self, part: dict, volume: str, placed_name: str) -> None:
        """Into a volume: at its centre (offsets cleared). Out of one (NO_VOLUME): stays where it is, at its own x, y, z."""
        resolved = self.app.resolved
        placed = None if resolved is None else next((item for item in resolved.airframe.parts if item.name == placed_name), None)
        for key in ("volume", "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m"):
            part.pop(key, None)
        if volume != NO_VOLUME:
            part["volume"] = volume
        else:
            position = placed.position_m if placed is not None else (round(self.app.definition["fuselage"]["length_m"] / 2.0, 4), 0.0, 0.0)
            part.update(x_m=round(position[0], 4), y_m=round(position[1], 4), z_m=round(position[2], 4))
        self.changed(rebuild=True)

    def rename_part(self, old: str, new: str) -> None:
        propulsion = self.app.definition.get("propulsion")
        if propulsion is not None and propulsion["esc"] == old:
            propulsion["esc"] = new

    def add_part(self) -> None:
        definition = self.app.definition
        names = {part["name"] for part in definition["parts"]}
        name = next(f"part {n}" for n in range(1, len(names) + 2) if f"part {n}" not in names)
        library = [f"{category}:{entry}" for category in PART_CATEGORIES for entry, data in sorted(self.app.parts_data[category].items()) if data.get("mass")]
        place = {"volume": definition["volumes"][0]["name"]} if definition["volumes"] else {"x_m": round(definition["fuselage"]["length_m"] / 2.0, 4)}
        definition["parts"].append({"name": name, "part": library[0], **place})
        self.changed(rebuild=True)

    def reset_volume_shape(self, volume: dict) -> None:
        for fields in VOLUME_SHAPE_FIELDS.values():
            for key in fields:
                volume.pop(key, None)
        for key, value in VOLUME_SHAPE_FIELDS[volume["shape"]].items():
            volume[key] = copy.deepcopy(value)

    def add_volume(self) -> None:
        volumes = self.app.definition["volumes"]
        names = {volume["name"] for volume in volumes}
        name = next(f"volume {n}" for n in range(1, len(names) + 2) if f"volume {n}" not in names)
        volume = {"name": name, "shape": "box", "x_m": round(self.app.definition["fuselage"]["length_m"] / 2.0, 4)}
        volume.update(copy.deepcopy(VOLUME_SHAPE_FIELDS["box"]))
        volumes.append(volume)
        self.changed(rebuild=True)


    def on_roll_slider_changed(self, _value: str) -> None:
        self.app.roll_view_display_var.set(f"{self.app.roll_view_var.get():.1f} deg")
        self.draw_active_view()

    def reset_view_transform(self) -> None:
        self.view_zoom_scale = 1.0
        self.view_pan_x = 0.0
        self.view_pan_y = 0.0
        self.draw_active_view()

    def clamp_view_pan(self) -> bool:
        if self.view_metrics is None:
            return False
        changed = False
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            min_pan_x = min(0.0, available_x - metrics["content_width"])
            max_pan_x = 0.0
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                min_pan_y = 0.0
                max_pan_y = 0.0
            else:
                min_pan_y = -(content_height - available_y) / 2.0
                max_pan_y = (content_height - available_y) / 2.0
        else:
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            content_width = 2.0 * metrics["content_half_width"]
            if content_width <= available_x:
                min_pan_x = 0.0
                max_pan_x = 0.0
            else:
                min_pan_x = -(content_width - available_x) / 2.0
                max_pan_x = (content_width - available_x) / 2.0
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                min_pan_y = 0.0
                max_pan_y = 0.0
            else:
                min_pan_y = -(content_height - available_y) / 2.0
                max_pan_y = (content_height - available_y) / 2.0
        clamped_pan_x = min(max(self.view_pan_x, min_pan_x), max_pan_x)
        clamped_pan_y = min(max(self.view_pan_y, min_pan_y), max_pan_y)
        if clamped_pan_x != self.view_pan_x:
            self.view_pan_x = clamped_pan_x
            changed = True
        if clamped_pan_y != self.view_pan_y:
            self.view_pan_y = clamped_pan_y
            changed = True
        return changed

    def update_view_scrollbars(self) -> None:
        if self.view_metrics is None or self.view_scroll_x is None or self.view_scroll_y is None:
            return
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if metrics["content_width"] <= available_x:
                self.view_scroll_x.set(0.0, 1.0)
            else:
                first_x = -self.view_pan_x / metrics["content_width"]
                last_x = first_x + available_x / metrics["content_width"]
                self.view_scroll_x.set(first_x, last_x)
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                self.view_scroll_y.set(0.0, 1.0)
            else:
                first_y = ((content_height - available_y) / 2.0 - self.view_pan_y) / content_height
                last_y = first_y + available_y / content_height
                self.view_scroll_y.set(first_y, last_y)
            return
        available_x = metrics["width"] - 2.0 * metrics["pad_x"]
        content_width = 2.0 * metrics["content_half_width"]
        if content_width <= available_x:
            self.view_scroll_x.set(0.0, 1.0)
        else:
            first_x = ((content_width - available_x) / 2.0 - self.view_pan_x) / content_width
            last_x = first_x + available_x / content_width
            self.view_scroll_x.set(first_x, last_x)
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            self.view_scroll_y.set(0.0, 1.0)
        else:
            first_y = ((content_height - available_y) / 2.0 - self.view_pan_y) / content_height
            last_y = first_y + available_y / content_height
            self.view_scroll_y.set(first_y, last_y)

    def on_view_xscroll(self, *args) -> None:
        if self.view_metrics is None:
            return
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            content_width = metrics["content_width"]
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if content_width <= available_x:
                return
            if args[0] == "moveto":
                self.view_pan_x = -float(args[1]) * content_width
            else:
                step = 40.0 if args[2] == "units" else available_x * 0.9
                self.view_pan_x -= int(args[1]) * step
        else:
            content_width = 2.0 * metrics["content_half_width"]
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if content_width <= available_x:
                return
            if args[0] == "moveto":
                self.view_pan_x = (content_width - available_x) / 2.0 - float(args[1]) * content_width
            else:
                step = 40.0 if args[2] == "units" else available_x * 0.9
                self.view_pan_x -= int(args[1]) * step
        self.draw_active_view()

    def on_view_yscroll(self, *args) -> None:
        if self.view_metrics is None:
            return
        metrics = self.view_metrics
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            return
        if args[0] == "moveto":
            self.view_pan_y = (content_height - available_y) / 2.0 - float(args[1]) * content_height
        else:
            step = 40.0 if args[2] == "units" else available_y * 0.9
            self.view_pan_y -= int(args[1]) * step
        self.draw_active_view()

    def zoom_view_at(self, event_x: float, event_y: float, zoom_factor: float) -> None:
        if self.view_metrics is None:
            return
        new_zoom_scale = min(max(self.view_zoom_scale * zoom_factor, 1.0), 25.0)
        if new_zoom_scale == self.view_zoom_scale:
            return
        zoom_ratio = new_zoom_scale / self.view_zoom_scale
        self.view_pan_x = event_x - self.view_metrics["origin_x"] - zoom_ratio * (event_x - self.view_metrics["origin_x"] - self.view_pan_x)
        self.view_pan_y = event_y - self.view_metrics["origin_y"] - zoom_ratio * (event_y - self.view_metrics["origin_y"] - self.view_pan_y)
        self.view_zoom_scale = new_zoom_scale
        self.draw_active_view()

    def on_view_mousewheel(self, event) -> None:
        zoom_factor = 1.1 if event.delta > 0 else 1.0 / 1.1
        self.zoom_view_at(event.x, event.y, zoom_factor)

    def on_view_wheel_up(self, event) -> None:
        self.zoom_view_at(event.x, event.y, 1.1)

    def on_view_wheel_down(self, event) -> None:
        self.zoom_view_at(event.x, event.y, 1.0 / 1.1)

    def make_view_canvas(self, parent: ttk.Frame, title: str, row: int) -> tk.Canvas:
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.grid(row=row, column=0, sticky="nsew", padx=4, pady=4)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        canvas = tk.Canvas(
            frame,
            bg=self.app.ui_canvas,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            relief="flat",
        )
        scroll_x = ttk.Scrollbar(frame, orient="horizontal", command=self.on_view_xscroll)
        scroll_y = ttk.Scrollbar(frame, orient="vertical", command=self.on_view_yscroll)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")
        self.view_scroll_x = scroll_x
        self.view_scroll_y = scroll_y
        canvas.bind("<Configure>", lambda event: self.draw_active_view())
        canvas.bind("<ButtonPress-1>", self.on_view_drag_start)
        canvas.bind("<B1-Motion>", self.on_view_drag_motion)
        canvas.bind("<ButtonRelease-1>", self.on_view_drag_end)
        canvas.bind("<MouseWheel>", self.on_view_mousewheel)
        canvas.bind("<Button-4>", self.on_view_wheel_up)
        canvas.bind("<Button-5>", self.on_view_wheel_down)
        return canvas

    def draw_asb_view(self, force: bool = False) -> None:
        app = self.app
        if app.asb_airplane is None or app.asb_frame is None:
            return
        if app.asb_draw_busy:
            return
        frame_width = max(app.asb_frame.winfo_width(), 200)
        frame_height = max(app.asb_frame.winfo_height(), 200)
        if not force and app.asb_canvas is not None and app.asb_last_size == (frame_width, frame_height):
            return
        app.asb_last_size = (frame_width, frame_height)
        fig_width = frame_width / 100.0
        fig_height = frame_height / 100.0
        app.asb_draw_busy = True
        try:
            if app.asb_canvas is not None:
                app.asb_canvas.get_tk_widget().destroy()
            if app.asb_figure is not None:
                plt.close(app.asb_figure)
            app.asb_figure, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=100, subplot_kw={"projection": "3d"})
            ax.set_facecolor(app.ui_bg)
            ax.tick_params(colors=app.ui_fg)
            ax.xaxis.label.set_color(app.ui_fg)
            ax.yaxis.label.set_color(app.ui_fg)
            ax.zaxis.label.set_color(app.ui_fg)
            ax.xaxis.pane.set_facecolor(app.ui_bg)
            ax.yaxis.pane.set_facecolor(app.ui_bg)
            ax.zaxis.pane.set_facecolor(app.ui_bg)
            ax.xaxis.pane.set_alpha(0.0)
            ax.yaxis.pane.set_alpha(0.0)
            ax.zaxis.pane.set_alpha(0.0)
            app.asb_airplane.draw(
                backend="matplotlib",
                ax=ax,
                use_preset_view_angle="left_isometric",
                set_background_pane_color=app.ui_bg,
                set_background_pane_alpha=0.0,
                set_axis_visibility=False,
                show=False,
            )
            app.asb_figure.patch.set_facecolor(app.ui_bg)
            app.asb_figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
            ax.set_position([0.0, 0.0, 1.0, 1.0])
            app.asb_canvas = FigureCanvasTkAgg(app.asb_figure, master=app.asb_frame)
            app.asb_canvas.draw()
            app.asb_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        finally:
            app.asb_draw_busy = False
