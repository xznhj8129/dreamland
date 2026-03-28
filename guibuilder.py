"""
Usage:
python guibuilder.py
"""

from __future__ import annotations

import argparse
import copy
import json
import queue
import sys
import threading
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

SCRIPT_DIR = Path(__file__).resolve().parent
TEMPLATES_PATH = SCRIPT_DIR / "libs" / "templates.json"
sys.path.insert(0, str(SCRIPT_DIR))
from analysis import build_asb_airplane, json_safe_data, load_builder_data as analysis_load_builder_data, parse_motor_name, recompute_analysis, run_vlm_sweep
from graphics import draw_canvas_view

MODULE_TYPE_OPTIONS = ["body", "end", "coupler", "wingmount"]
SURFACE_TYPE_OPTIONS = ["wing"]
MOTOR_DIRECTION_OPTIONS = ["front", "back"]
STANDARD_NONE_OPTION = "None"
MODULE_FIELD_ORDER = [
    "name",
    "type",
    "len",
    "mass",
    "cg",
    "wings_n",
    "wing_phase",
    "wings_ang",
    "wings_x",
    "eo",
    "ballast",
    "battery",
    "payload",
    "esc",
    "motor",
    "motor_x_pos",
    "motor_y_pos",
    "motor_dir",
]
SURFACE_FIELD_ORDER = [
    "name",
    "type",
    "semispan",
    "mass",
    "cg",
    "chord_root",
    "chord_tip",
    "aoa_ang",
    "sweep_ang",
    "airfoil",
    "motor",
    "motor_x_pos",
    "motor_y_pos",
    "motor_dir",
    "pod_diam",
    "pod_len_flat",
    "pod_len_curve",
    "pod_shape",
    "ctl_surface",
    "all_moving",
]
CONFIG_FIELD_ORDER = [
    "name",
    "vtol",
    "tailless",
    "canards",
    "powered",
    "thrust_axis_tilt_deg",
    "standard_profile",
    "apply_standard",
]
VIEW_OPTIONS = ["Longitudinal", "Front"]
MAIN_DRIVER_KEYS = ["fuselage_od_mm", "fuselage_id_mm", "material_density_g_cm3"]
POD_SHAPE_OPTIONS = ["elliptical", "ogive"]
MAIN_DRIVER_LABELS = {
    "assembly_name": "Assembly Name",
    "fuselage_od_mm": "Fuselage OD (mm)",
    "fuselage_id_mm": "Fuselage ID (mm)",
    "material_density_g_cm3": "Material Density (g/cm^3)",
}
LENGTH_FIELD_KEYS_M = {
    "len",
    "diam",
    "semispan",
    "chord_root",
    "chord_tip",
    "motor_x_pos",
    "motor_y_pos",
    "pod_diam",
    "pod_len_flat",
    "pod_len_curve",
}
MASS_FIELD_KEYS_KG = {"mass", "payload_mass", "ballast_mass"}
MOTOR_EDIT_KEYS = {"motor_x_pos", "motor_y_pos", "motor_dir", "pod_diam", "pod_len_flat", "pod_len_curve", "pod_shape"}
UI_BG = "#000000"
UI_PANEL = "#031a03"
UI_BORDER = "#0a5f0a"
UI_FG = "#67ff67"
UI_ACCENT = "#19ff19"
UI_CANVAS = "#001000"
UI_DRAW = "#2aff2a"
AERO_QUEUE_POLL_MS = 100
AERO_VLM_DEFAULTS = {
    "aoa_start": 0.0,
    "aoa_end": 10.0,
    "aoa_increment": 1.0,
    "airspeed_start": 10.0,
    "airspeed_end": 25.0,
    "airspeed_increment": 5.0,
    "altitude_start": 0.0,
    "altitude_end": 0.0,
    "altitude_increment": 10.0,
    "vlm_threads": 1,
    "vlm_span_res": 25,
    "vlm_chord_res": 8,
}
AERO_VLM_LABELS = {
    "aoa_start": "AoA start (deg)",
    "aoa_end": "AoA end (deg)",
    "aoa_increment": "AoA step (deg)",
    "airspeed_start": "Airspeed start (m/s)",
    "airspeed_end": "Airspeed end (m/s)",
    "airspeed_increment": "Airspeed step (m/s)",
    "altitude_start": "Altitude start (m)",
    "altitude_end": "Altitude end (m)",
    "altitude_increment": "Altitude step (m)",
    "vlm_threads": "Worker threads",
    "vlm_span_res": "Spanwise resolution",
    "vlm_chord_res": "Chordwise resolution",
}


def ordered_keys(data: dict, preferred_keys: list[str]) -> list[str]:
    keys = list(data.keys())
    ordered = [key for key in preferred_keys if key in data]
    ordered.extend(sorted(key for key in keys if key not in ordered))
    return ordered


def format_field_value(key: str, value):
    if key in LENGTH_FIELD_KEYS_M:
        return f"{value * 1000.0:g}"
    if key in MASS_FIELD_KEYS_KG:
        return f"{value * 1000.0:g}"
    return str(value)


def field_label(key: str) -> str:
    if key == "wing_phase":
        return "wing_phase / around tube (deg)"
    if key == "wings_ang":
        return "dihedral (deg)"
    if key in LENGTH_FIELD_KEYS_M:
        return f"{key} (mm)"
    if key in MASS_FIELD_KEYS_KG:
        return f"{key} (g)"
    return key


def load_templates_data() -> dict:
    with open(TEMPLATES_PATH, "r") as file:
        return json.loads(file.read())


def parse_value_for_schema(field_types: dict, schema_name: str, key: str, text: str):
    schema_field_types = field_types[schema_name]
    if key in schema_field_types:
        value_type = schema_field_types[key]
    else:
        value_type = None
    if value_type == "int":
        value = int(text)
    elif value_type == "float":
        value = float(text)
    elif value_type == "bool":
        value = text.lower() == "true"
    else:
        value = text
    if key in LENGTH_FIELD_KEYS_M:
        return value / 1000.0
    if key in MASS_FIELD_KEYS_KG:
        return value / 1000.0
    return value


def parse_value_from_existing(field_types: dict, schema_name: str, key: str, text: str, existing_value):
    schema_field_types = field_types[schema_name]
    if key in schema_field_types:
        return parse_value_for_schema(field_types, schema_name, key, text)
    if type(existing_value) is int:
        value = int(text)
    elif type(existing_value) is float:
        value = float(text)
    elif type(existing_value) is bool:
        value = text.lower() == "true"
    else:
        value = text
    if key in LENGTH_FIELD_KEYS_M:
        return value / 1000.0
    if key in MASS_FIELD_KEYS_KG:
        return value / 1000.0
    return value


def value_type_from_existing(field_types: dict, schema_name: str, key: str, existing_value) -> str:
    schema_field_types = field_types[schema_name]
    if key in schema_field_types:
        return schema_field_types[key]
    if type(existing_value) is int:
        return "int"
    if type(existing_value) is float:
        return "float"
    if type(existing_value) is bool:
        return "bool"
    return "str"


def apply_template_values(existing: dict, template: dict) -> dict:
    updated = copy.deepcopy(template)
    for key in updated:
        if key in existing:
            updated[key] = existing[key]
    return updated


class BuilderV3TkApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Builder V3")
        self.root.geometry("1800x980")
        self.root.configure(background=UI_BG)
        self.root.report_callback_exception = self.handle_callback_exception
        self.modules_data, self.parts_data, self.default_config, self.standards_data, self.motor_data_df, self.motor_names, self.propeller_names = analysis_load_builder_data()
        self.templates_data = load_templates_data()
        self.field_types = self.templates_data["field_types"]
        self.part_defaults = self.templates_data["part_defaults"]
        self.current_assembly_name = "default"
        self.current_config = self.normalize_config_from_templates(self.default_config)
        self.apply_standard_dimensions_to_config(self.current_config)
        self.standard_names = sorted(self.standards_data["specs"].keys())
        self.last_aoa_eval_deg = 0.0
        self.aoa_var = tk.DoubleVar(value=0.0)
        self.aoa_display_var = tk.StringVar(value="0.0 deg")
        self.recompute_status_var = tk.StringVar(value="Up to date")
        self.assembly_name_var = tk.StringVar(value=self.current_assembly_name)
        self.standard_profile_var = tk.StringVar(value=STANDARD_NONE_OPTION if self.current_config["standard_profile"] == "" else self.current_config["standard_profile"])
        self.apply_standard_var = tk.BooleanVar(value=self.current_config["apply_standard"])
        self.view_mode_var = tk.StringVar(value="Longitudinal")
        self.roll_view_var = tk.DoubleVar(value=0.0)
        self.roll_view_display_var = tk.StringVar(value="0.0 deg")
        self.view_canvas: tk.Canvas | None = None
        self.asb_frame: ttk.Frame | None = None
        self.asb_canvas = None
        self.asb_figure = None
        self.asb_draw_busy = False
        self.asb_last_size = (0, 0)
        self.configuration_view = None
        self.aero_view = None
        self.active_main_view = "Configuration"
        self.aero_run_button = None
        self.aero_progressbar = None
        self.aero_results_text: tk.Text | None = None
        self.aero_display_label = None
        self.aero_status_var = tk.StringVar(value="Idle")
        self.aero_progress_text_var = tk.StringVar(value="0 / 0")
        self.aero_vlm_settings = copy.deepcopy(AERO_VLM_DEFAULTS)
        self.aero_vlm_vars = {}
        self.aero_vlm_thread: threading.Thread | None = None
        self.aero_vlm_queue: queue.Queue = queue.Queue()
        self.aero_vlm_result = None
        self.summary_text: tk.Text | None = None
        self.module_canvas: tk.Canvas | None = None
        self.module_inner: ttk.Frame | None = None
        self.module_window_id = None
        self.assembly_inner: ttk.Frame | None = None
        self.analysis = None
        self.asb_airplane = None
        self.heavy_dirty = False
        self.error_dialog: tk.Toplevel | None = None
        self.setup_theme()
        self.build_menu()
        self.build_layout()
        self.root.bind("<F5>", self.on_manual_recompute)
        self.rebuild_module_editor()
        self.heavy_recompute()

    def normalize_config_from_templates(self, config: dict) -> dict:
        normalized = copy.deepcopy(config)
        for key, value in self.templates_data["config_defaults"].items():
            if key not in normalized:
                normalized[key] = copy.deepcopy(value)
        normalized_parts = []
        for part in normalized["parts"]:
            normalized_part = apply_template_values(part, self.part_defaults)
            module_block = part["module"]
            if type(module_block) is str:
                module_data = copy.deepcopy(self.modules_data[module_block])
                module_data["name"] = module_block
            else:
                module_data = copy.deepcopy(module_block)
            module_type = module_data["type"]
            normalized_part["module"] = apply_template_values(module_data, self.templates_data["module_templates"][module_type])
            if "surfaces" in part:
                surface_block = part["surfaces"]
                if type(surface_block) is str:
                    surface_data = copy.deepcopy(self.modules_data[surface_block])
                    surface_data["name"] = surface_block
                else:
                    surface_data = copy.deepcopy(surface_block)
                surface_type = surface_data["type"]
                normalized_part["surfaces"] = apply_template_values(surface_data, self.templates_data["surface_templates"][surface_type])
            normalized_parts.append(normalized_part)
        normalized["parts"] = normalized_parts
        return normalized

    def apply_standard_dimensions_to_config(self, config: dict) -> None:
        if config["apply_standard"] and config["standard_profile"]:
            standard_spec = self.standards_data["specs"][config["standard_profile"]]
            config["fuselage_od_mm"] = standard_spec["module_size"]["nominal_od"]
            config["fuselage_id_mm"] = standard_spec["module_size"]["nominal_id"]

    def handle_callback_exception(self, exc_type, exc_value, exc_traceback) -> None:
        traceback_text = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        self.show_error_dialog(traceback_text)

    def show_error_dialog(self, traceback_text: str) -> None:
        if self.error_dialog is not None and self.error_dialog.winfo_exists():
            self.error_dialog.destroy()
        dialog = tk.Toplevel(self.root, background=UI_BG)
        dialog.title("Error")
        dialog.geometry("980x560")
        dialog.transient(self.root)
        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(1, weight=1)
        dialog.bind("<Escape>", lambda event: dialog.destroy())
        dialog.protocol("WM_DELETE_WINDOW", dialog.destroy)
        ttk.Label(dialog, text="Traceback").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        text_frame = ttk.Frame(dialog, padding=(8, 0, 8, 8))
        text_frame.grid(row=1, column=0, sticky="nsew")
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        text = tk.Text(
            text_frame,
            wrap="none",
            font=("TkFixedFont", 10),
            background=UI_PANEL,
            foreground=UI_FG,
            insertbackground=UI_FG,
            selectbackground=UI_BORDER,
            selectforeground=UI_ACCENT,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        scroll_y = ttk.Scrollbar(text_frame, orient="vertical", command=text.yview)
        scroll_x = ttk.Scrollbar(text_frame, orient="horizontal", command=text.xview)
        text.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        text.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")
        text.insert("1.0", traceback_text)
        text.configure(state="disabled")
        button_frame = ttk.Frame(dialog, padding=(8, 0, 8, 8))
        button_frame.grid(row=2, column=0, sticky="e")
        ttk.Button(button_frame, text="Close", command=dialog.destroy).pack(side="right")
        dialog.focus_set()
        self.error_dialog = dialog

    def setup_theme(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=UI_BG, foreground=UI_FG, fieldbackground=UI_PANEL)
        style.configure("TFrame", background=UI_BG)
        style.configure(
            "TLabelframe",
            background=UI_BG,
            foreground=UI_ACCENT,
            bordercolor=UI_BORDER,
            lightcolor=UI_BORDER,
            darkcolor=UI_BORDER,
        )
        style.configure("TLabelframe.Label", background=UI_BG, foreground=UI_ACCENT)
        style.configure("TLabel", background=UI_BG, foreground=UI_FG)
        style.configure(
            "TEntry",
            fieldbackground=UI_PANEL,
            foreground=UI_FG,
            insertcolor=UI_FG,
            bordercolor=UI_BORDER,
            lightcolor=UI_BORDER,
            darkcolor=UI_BORDER,
        )
        style.configure(
            "TCombobox",
            fieldbackground=UI_PANEL,
            foreground=UI_FG,
            background=UI_PANEL,
            bordercolor=UI_BORDER,
            arrowcolor=UI_FG,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", UI_PANEL)],
            foreground=[("readonly", UI_FG)],
            selectbackground=[("readonly", UI_PANEL)],
            selectforeground=[("readonly", UI_FG)],
        )
        style.configure("TButton", background=UI_PANEL, foreground=UI_FG, bordercolor=UI_BORDER, lightcolor=UI_BORDER, darkcolor=UI_BORDER)
        style.map("TButton", background=[("active", UI_BORDER)], foreground=[("active", UI_ACCENT)])
        style.configure("TCheckbutton", background=UI_BG, foreground=UI_FG, indicatorcolor=UI_PANEL)
        style.map("TCheckbutton", indicatorcolor=[("selected", UI_ACCENT)], foreground=[("active", UI_ACCENT)])
        style.configure("TRadiobutton", background=UI_BG, foreground=UI_FG, indicatorcolor=UI_PANEL)
        style.map("TRadiobutton", indicatorcolor=[("selected", UI_ACCENT)], foreground=[("active", UI_ACCENT)])
        style.configure("TNotebook", background=UI_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=UI_PANEL, foreground=UI_FG, borderwidth=1, lightcolor=UI_BORDER, darkcolor=UI_BORDER)
        style.map("TNotebook.Tab", background=[("selected", UI_BORDER)], foreground=[("selected", UI_ACCENT)])

    def build_menu(self) -> None:
        menu_bar = tk.Menu(
            self.root,
            background=UI_PANEL,
            foreground=UI_FG,
            activebackground=UI_BORDER,
            activeforeground=UI_ACCENT,
            borderwidth=1,
        )
        file_menu = tk.Menu(
            menu_bar,
            tearoff=0,
            background=UI_PANEL,
            foreground=UI_FG,
            activebackground=UI_BORDER,
            activeforeground=UI_ACCENT,
            borderwidth=1,
        )
        file_menu.add_command(label="Load", command=self.load_json)
        file_menu.add_command(label="Save", command=self.save_json)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.destroy)
        menu_bar.add_cascade(label="File", menu=file_menu)
        self.root.config(menu=menu_bar)

    def build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)

        main_view_host = ttk.Frame(self.root)
        main_view_host.grid(row=0, column=0, sticky="nsew")
        main_view_host.columnconfigure(0, weight=1)
        main_view_host.rowconfigure(0, weight=1)

        self.configuration_view = ttk.Frame(main_view_host)
        self.aero_view = ttk.Frame(main_view_host)
        for frame in (self.configuration_view, self.aero_view):
            frame.grid(row=0, column=0, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)

        pane_window = ttk.Panedwindow(self.configuration_view, orient=tk.HORIZONTAL)
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
        ttk.Button(left_header, text="Recompute (F5)", command=self.heavy_recompute).grid(row=0, column=0, sticky="w")
        ttk.Label(left_header, textvariable=self.recompute_status_var).grid(row=0, column=1, sticky="e", padx=(8, 0))

        left_notebook = ttk.Notebook(left_frame)
        left_notebook.grid(row=1, column=0, sticky="nsew")
        summary_tab = ttk.Frame(left_notebook, padding=4)
        controls_tab = ttk.Frame(left_notebook, padding=4)
        summary_tab.columnconfigure(0, weight=1)
        summary_tab.rowconfigure(0, weight=1)
        controls_tab.columnconfigure(0, weight=1)
        left_notebook.add(summary_tab, text="Summary")
        left_notebook.add(controls_tab, text="Controls")

        summary_frame = ttk.LabelFrame(summary_tab, text="Summary", padding=4)
        summary_frame.grid(row=0, column=0, sticky="nsew")
        summary_frame.columnconfigure(0, weight=1)
        summary_frame.rowconfigure(0, weight=1)
        self.summary_text = tk.Text(
            summary_frame,
            wrap="none",
            font=("TkFixedFont", 10),
            background=UI_PANEL,
            foreground=UI_FG,
            insertbackground=UI_FG,
            selectbackground=UI_BORDER,
            selectforeground=UI_ACCENT,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        summary_scroll_y = ttk.Scrollbar(summary_frame, orient="vertical", command=self.summary_text.yview)
        summary_scroll_x = ttk.Scrollbar(summary_frame, orient="horizontal", command=self.summary_text.xview)
        self.summary_text.configure(yscrollcommand=summary_scroll_y.set, xscrollcommand=summary_scroll_x.set)
        self.summary_text.grid(row=0, column=0, sticky="nsew")
        summary_scroll_y.grid(row=0, column=1, sticky="ns")
        summary_scroll_x.grid(row=1, column=0, sticky="ew")

        controls_frame = ttk.LabelFrame(controls_tab, text="Controls", padding=6)
        controls_frame.grid(row=0, column=0, sticky="new")
        controls_frame.columnconfigure(0, weight=1)
        ttk.Label(controls_frame, text="AoA eval").grid(row=0, column=0, sticky="w")
        slider_row = ttk.Frame(controls_frame)
        slider_row.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        slider_row.columnconfigure(0, weight=1)
        aoa_slider = tk.Scale(
            slider_row,
            from_=-45.0,
            to=45.0,
            orient="horizontal",
            resolution=1.0,
            variable=self.aoa_var,
            showvalue=False,
            command=self.on_aoa_slider_changed,
            background=UI_BG,
            foreground=UI_FG,
            troughcolor=UI_PANEL,
            activebackground=UI_BORDER,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            bd=0,
            relief="flat",
        )
        aoa_slider.grid(row=0, column=0, sticky="ew")
        ttk.Label(slider_row, textvariable=self.aoa_display_var).grid(row=0, column=1, sticky="e", padx=(8, 0))

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
        ttk.Label(graphics_controls, text="Blue: CG   Red: CP   Green: CL@AoA").grid(row=0, column=0, sticky="w")
        button_bar = ttk.Frame(graphics_controls)
        button_bar.grid(row=1, column=0, sticky="w", pady=(6, 0))
        for view_name in VIEW_OPTIONS:
            ttk.Radiobutton(
                button_bar,
                text=view_name,
                value=view_name,
                variable=self.view_mode_var,
                command=self.draw_active_view,
            ).pack(side="left", padx=(0, 8))
        simple_view_frame = ttk.Frame(simple_tab)
        simple_view_frame.grid(row=1, column=0, sticky="nsew")
        simple_view_frame.columnconfigure(0, weight=1)
        simple_view_frame.rowconfigure(0, weight=1)

        self.view_canvas = self.make_view_canvas(simple_view_frame, "Orthographic View", 0)

        roll_frame = ttk.LabelFrame(simple_view_frame, text="Roll", padding=4)
        roll_frame.grid(row=0, column=1, sticky="ns", padx=(8, 0))
        roll_frame.rowconfigure(1, weight=1)
        ttk.Label(roll_frame, text="90 top").grid(row=0, column=0, sticky="n")
        roll_slider = tk.Scale(
            roll_frame,
            from_=90.0,
            to=0.0,
            orient="vertical",
            resolution=1.0,
            variable=self.roll_view_var,
            showvalue=False,
            command=self.on_roll_slider_changed,
            background=UI_BG,
            foreground=UI_FG,
            troughcolor=UI_PANEL,
            activebackground=UI_BORDER,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            bd=0,
            relief="flat",
            length=420,
        )
        roll_slider.grid(row=1, column=0, sticky="ns", pady=(6, 6))
        ttk.Label(roll_frame, text="0 side").grid(row=2, column=0, sticky="s")
        ttk.Label(roll_frame, textvariable=self.roll_view_display_var).grid(row=3, column=0, sticky="s", pady=(6, 0))

        self.asb_frame = ttk.LabelFrame(asb_tab, text="ASB View", padding=6)
        self.asb_frame.grid(row=0, column=0, sticky="nsew")
        self.asb_frame.columnconfigure(0, weight=1)
        self.asb_frame.rowconfigure(1, weight=1)
        asb_controls = ttk.Frame(self.asb_frame)
        asb_controls.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(asb_controls, text="Update View", command=self.heavy_recompute).pack(side="left")

        notebook = ttk.Notebook(right_frame)
        notebook.grid(row=0, column=0, rowspan=2, sticky="nsew")
        parameters_tab = ttk.Frame(notebook, padding=4)
        assembly_tab = ttk.Frame(notebook, padding=4)
        parameters_tab.columnconfigure(0, weight=1)
        assembly_tab.columnconfigure(0, weight=1)
        assembly_tab.rowconfigure(1, weight=1)
        notebook.add(parameters_tab, text="Parameters")
        notebook.add(assembly_tab, text="Assembly")

        modules_header = ttk.LabelFrame(parameters_tab, text="Parameters", padding=6)
        modules_header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        modules_header.columnconfigure(1, weight=1)
        ttk.Label(modules_header, text=MAIN_DRIVER_LABELS["assembly_name"]).grid(row=0, column=0, sticky="w")
        assembly_entry = ttk.Entry(modules_header, textvariable=self.assembly_name_var)
        assembly_entry.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
        assembly_entry.bind("<Return>", lambda event: self.on_assembly_name_commit())
        assembly_entry.bind("<KP_Enter>", lambda event: self.on_assembly_name_commit())
        assembly_entry.bind("<FocusOut>", lambda event: self.on_assembly_name_commit())
        ttk.Label(modules_header, text="Standard Profile").grid(row=1, column=0, sticky="w")
        standard_box = ttk.Combobox(modules_header, values=[STANDARD_NONE_OPTION, *self.standard_names], textvariable=self.standard_profile_var, state="readonly")
        standard_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
        standard_box.bind("<<ComboboxSelected>>", lambda event: self.on_standard_profile_selected())
        apply_standard_box = ttk.Checkbutton(modules_header, text="Apply", variable=self.apply_standard_var, command=self.on_apply_standard_changed)
        apply_standard_box.grid(row=1, column=2, sticky="w", padx=(8, 0), pady=2)
        for row_index, key in enumerate(MAIN_DRIVER_KEYS, start=2):
            ttk.Label(modules_header, text=MAIN_DRIVER_LABELS[key]).grid(row=row_index, column=0, sticky="w")
            variable = tk.StringVar(value=format_field_value(key, self.current_config[key]))
            entry = ttk.Entry(modules_header, textvariable=variable)
            entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.bind_value_entry(entry, self.current_config, key, variable, "config")

        self.assembly_inner = ttk.Frame(parameters_tab)
        self.assembly_inner.grid(row=1, column=0, sticky="nsew")

        assembly_controls = ttk.Frame(assembly_tab)
        assembly_controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(assembly_controls, text="Add New", command=self.add_new_part).pack(side="left")

        module_scroll_frame = ttk.Frame(assembly_tab)
        module_scroll_frame.grid(row=1, column=0, sticky="nsew")
        module_scroll_frame.columnconfigure(0, weight=1)
        module_scroll_frame.rowconfigure(0, weight=1)
        self.module_canvas = tk.Canvas(
            module_scroll_frame,
            background=UI_BG,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        module_scroll_y = ttk.Scrollbar(module_scroll_frame, orient="vertical", command=self.module_canvas.yview)
        self.module_inner = ttk.Frame(self.module_canvas)
        self.module_inner.bind("<Configure>", lambda event: self.module_canvas.configure(scrollregion=self.module_canvas.bbox("all")))
        self.module_window_id = self.module_canvas.create_window((0, 0), window=self.module_inner, anchor="nw")
        self.module_canvas.configure(yscrollcommand=module_scroll_y.set)
        self.module_canvas.grid(row=0, column=0, sticky="nsew")
        module_scroll_y.grid(row=0, column=1, sticky="ns")
        self.module_canvas.bind("<Configure>", lambda event: self.module_canvas.itemconfigure(self.module_window_id, width=event.width))

        aero_panes = ttk.Panedwindow(self.aero_view, orient=tk.HORIZONTAL)
        aero_panes.grid(row=0, column=0, sticky="nsew")

        aero_left_frame = ttk.Frame(aero_panes, padding=8)
        aero_right_frame = ttk.Frame(aero_panes, padding=8)
        aero_left_frame.columnconfigure(0, weight=1)
        aero_left_frame.rowconfigure(0, weight=1)
        aero_right_frame.columnconfigure(0, weight=1)
        aero_right_frame.rowconfigure(0, weight=1)
        aero_panes.add(aero_left_frame, weight=1)
        aero_panes.add(aero_right_frame, weight=2)

        aero_left_panes = ttk.Panedwindow(aero_left_frame, orient=tk.VERTICAL)
        aero_left_panes.grid(row=0, column=0, sticky="nsew")

        aero_top_frame = ttk.Frame(aero_left_panes)
        aero_bottom_frame = ttk.Frame(aero_left_panes)
        aero_top_frame.columnconfigure(0, weight=1)
        aero_top_frame.rowconfigure(0, weight=1)
        aero_bottom_frame.columnconfigure(0, weight=1)
        aero_bottom_frame.rowconfigure(0, weight=1)
        aero_left_panes.add(aero_top_frame, weight=1)
        aero_left_panes.add(aero_bottom_frame, weight=1)

        aero_controls_frame = ttk.LabelFrame(aero_top_frame, text="VLM Controls", padding=6)
        aero_controls_frame.grid(row=0, column=0, sticky="nsew")
        aero_controls_frame.columnconfigure(0, weight=1)

        header_frame = ttk.Frame(aero_controls_frame)
        header_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        header_frame.columnconfigure(1, weight=1)
        self.aero_run_button = ttk.Button(header_frame, text="Run", command=self.start_aero_vlm_run)
        self.aero_run_button.grid(row=0, column=0, sticky="w")
        self.aero_progressbar = ttk.Progressbar(header_frame, orient="horizontal", mode="determinate", maximum=100.0)
        self.aero_progressbar.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(header_frame, textvariable=self.aero_progress_text_var).grid(row=0, column=2, sticky="e")
        ttk.Label(aero_controls_frame, textvariable=self.aero_status_var).grid(row=1, column=0, sticky="w", pady=(0, 8))

        params_grid = ttk.Frame(aero_controls_frame)
        params_grid.grid(row=2, column=0, sticky="nsew")
        for column_index in range(2):
            params_grid.columnconfigure(column_index, weight=1)

        grouped_keys = [
            ("AoA", ["aoa_start", "aoa_end", "aoa_increment"]),
            ("Airspeed", ["airspeed_start", "airspeed_end", "airspeed_increment"]),
            ("Altitude", ["altitude_start", "altitude_end", "altitude_increment"]),
            ("Solver", ["vlm_threads", "vlm_span_res", "vlm_chord_res"]),
        ]
        self.aero_vlm_vars = {}
        for group_index, (title, keys) in enumerate(grouped_keys):
            group_frame = ttk.LabelFrame(params_grid, text=title, padding=6)
            group_frame.grid(row=group_index // 2, column=group_index % 2, sticky="nsew", padx=4, pady=4)
            group_frame.columnconfigure(1, weight=1)
            for row_index, key in enumerate(keys):
                ttk.Label(group_frame, text=AERO_VLM_LABELS[key]).grid(row=row_index, column=0, sticky="w")
                variable = tk.StringVar(value=f"{self.aero_vlm_settings[key]:g}")
                entry = ttk.Entry(group_frame, textvariable=variable)
                entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                self.bind_aero_entry(entry, key, variable)
                self.aero_vlm_vars[key] = variable

        aero_results_frame = ttk.LabelFrame(aero_bottom_frame, text="Results", padding=4)
        aero_results_frame.grid(row=0, column=0, sticky="nsew")
        aero_results_frame.columnconfigure(0, weight=1)
        aero_results_frame.rowconfigure(0, weight=1)
        self.aero_results_text = tk.Text(
            aero_results_frame,
            wrap="none",
            font=("TkFixedFont", 10),
            background=UI_PANEL,
            foreground=UI_FG,
            insertbackground=UI_FG,
            selectbackground=UI_BORDER,
            selectforeground=UI_ACCENT,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        aero_scroll_y = ttk.Scrollbar(aero_results_frame, orient="vertical", command=self.aero_results_text.yview)
        aero_scroll_x = ttk.Scrollbar(aero_results_frame, orient="horizontal", command=self.aero_results_text.xview)
        self.aero_results_text.configure(yscrollcommand=aero_scroll_y.set, xscrollcommand=aero_scroll_x.set)
        self.aero_results_text.grid(row=0, column=0, sticky="nsew")
        aero_scroll_y.grid(row=0, column=1, sticky="ns")
        aero_scroll_x.grid(row=1, column=0, sticky="ew")

        aero_display_frame = ttk.LabelFrame(aero_right_frame, text="Display", padding=6)
        aero_display_frame.grid(row=0, column=0, sticky="nsew")
        aero_display_frame.columnconfigure(0, weight=1)
        aero_display_frame.rowconfigure(0, weight=1)
        self.aero_display_label = ttk.Label(
            aero_display_frame,
            text="VLM display pending.",
            anchor="center",
            justify="center",
        )
        self.aero_display_label.grid(row=0, column=0, sticky="nsew")

        bottom_bar = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        bottom_bar.grid(row=1, column=0, sticky="ew")
        ttk.Button(bottom_bar, text="Configuration", command=lambda: self.show_main_view("Configuration")).pack(side="left")
        ttk.Button(bottom_bar, text="Aero Analysis", command=lambda: self.show_main_view("Aero Analysis")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="CAD").pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Module Editor").pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Part Editor").pack(side="left", padx=(8, 0))

        self.root.update_idletasks()
        aero_panes.sashpos(0, max(self.root.winfo_width() // 3, 280))
        aero_left_panes.sashpos(0, max(self.root.winfo_height() // 2, 240))
        self.show_main_view("Configuration")

    def make_view_canvas(self, parent: ttk.Frame, title: str, row: int) -> tk.Canvas:
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.grid(row=row, column=0, sticky="nsew", padx=4, pady=4)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        canvas = tk.Canvas(
            frame,
            bg=UI_CANVAS,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        canvas.grid(row=0, column=0, sticky="nsew")
        canvas.bind("<Configure>", lambda event: self.analysis is not None and self.draw_active_view())
        return canvas

    def on_aoa_slider_changed(self, _value: str) -> None:
        self.aoa_display_var.set(f"{self.aoa_var.get():.1f} deg")
        self.light_recompute()

    def on_roll_slider_changed(self, _value: str) -> None:
        self.roll_view_display_var.set(f"{self.roll_view_var.get():.1f} deg")
        self.draw_active_view()

    def on_manual_recompute(self, _event=None) -> str:
        if self.active_main_view == "Aero Analysis":
            self.start_aero_vlm_run()
            return "break"
        self.heavy_recompute()
        return "break"

    def on_assembly_name_commit(self) -> str:
        self.current_assembly_name = self.assembly_name_var.get()
        self.light_recompute()
        return "break"

    def on_standard_profile_selected(self) -> None:
        selected = self.standard_profile_var.get()
        self.current_config["standard_profile"] = "" if selected == STANDARD_NONE_OPTION else selected
        if self.current_config["apply_standard"]:
            self.apply_standard_dimensions_to_config(self.current_config)
            self.rebuild_module_editor()
        self.light_recompute()

    def on_apply_standard_changed(self) -> None:
        self.current_config["apply_standard"] = bool(self.apply_standard_var.get())
        if self.current_config["apply_standard"]:
            self.apply_standard_dimensions_to_config(self.current_config)
            self.rebuild_module_editor()
        self.light_recompute()

    def on_select_value(self, target: dict, key: str, variable: tk.StringVar, rebuild: bool = False) -> None:
        target[key] = variable.get()
        if rebuild:
            self.rebuild_module_editor()
        self.light_recompute()

    def current_battery_series(self) -> int | None:
        for part in self.current_config["parts"]:
            if part["module"].get("battery", False) and part["battery"]:
                return int(self.parts_data["battery"][part["battery"]]["s"])
        return None

    def available_props_for_motor(self, motor_name: str) -> list[str]:
        if not motor_name:
            return []
        _motor_name, brand, model, kv = parse_motor_name(motor_name)
        motor_rows = self.motor_data_df[
            (self.motor_data_df["Brand"].astype(str).str.lower() == str(brand).lower())
            & (self.motor_data_df["Motor"].astype(str).str.lower() == str(model).lower())
            & (self.motor_data_df["KV"].astype(int) == int(kv))
        ]
        batt_s = self.current_battery_series()
        if batt_s is not None:
            motor_rows = motor_rows[motor_rows["Data Batt S"].astype(int) == batt_s]
        return sorted(motor_rows["Prop"].dropna().astype(str).unique().tolist())

    def on_motor_selected(self, part: dict, motor_var: tk.StringVar, prop_var: tk.StringVar, prop_box: ttk.Combobox) -> None:
        part["motors"] = motor_var.get()
        available_props = self.available_props_for_motor(part["motors"])
        prop_box.configure(values=available_props)
        if part["props"] not in available_props:
            part["props"] = available_props[0] if available_props else ""
            prop_var.set(part["props"])
        self.light_recompute()

    def on_bool_changed(self, target: dict, key: str, variable: tk.BooleanVar, rebuild: bool = True) -> None:
        target[key] = bool(variable.get())
        if rebuild:
            self.rebuild_module_editor()
        self.light_recompute()

    def on_entry_commit(self, target: dict, key: str, variable: tk.StringVar, schema_name: str, rebuild: bool = False) -> str:
        try:
            target[key] = parse_value_from_existing(self.field_types, schema_name, key, variable.get(), target[key])
        except ValueError:
            variable.set(format_field_value(key, target[key]))
            return "break"
        if rebuild:
            self.rebuild_module_editor()
        self.light_recompute()
        return "break"

    def on_length_scroll(self, target: dict, key: str, variable: tk.StringVar, schema_name: str, event) -> str:
        try:
            current_value = float(variable.get())
        except ValueError:
            variable.set(format_field_value(key, target[key]))
            return "break"
        delta = 0
        if hasattr(event, "delta") and event.delta != 0:
            delta = 1 if event.delta > 0 else -1
        elif getattr(event, "num", None) == 4:
            delta = 1
        elif getattr(event, "num", None) == 5:
            delta = -1
        if delta == 0:
            return "break"
        variable.set(f"{current_value + delta:g}")
        return self.on_entry_commit(target, key, variable, schema_name)

    def on_numeric_key_step(self, target: dict, key: str, variable: tk.StringVar, schema_name: str, delta: int) -> str:
        try:
            if value_type_from_existing(self.field_types, schema_name, key, target[key]) == "int":
                current_value = int(float(variable.get()))
                variable.set(str(current_value + delta))
            else:
                current_value = float(variable.get())
                variable.set(f"{current_value + delta:g}")
        except ValueError:
            variable.set(format_field_value(key, target[key]))
            return "break"
        return self.on_entry_commit(target, key, variable, schema_name)

    def bind_length_scroll(self, widget, target: dict, key: str, variable: tk.StringVar, schema_name: str) -> None:
        if key not in LENGTH_FIELD_KEYS_M and key not in {"fuselage_od_mm", "fuselage_id_mm"}:
            return
        widget.bind("<MouseWheel>", lambda event, d=target, k=key, v=variable, s=schema_name: self.on_length_scroll(d, k, v, s, event))
        widget.bind("<Button-4>", lambda event, d=target, k=key, v=variable, s=schema_name: self.on_length_scroll(d, k, v, s, event))
        widget.bind("<Button-5>", lambda event, d=target, k=key, v=variable, s=schema_name: self.on_length_scroll(d, k, v, s, event))

    def bind_numeric_step(self, widget, target: dict, key: str, variable: tk.StringVar, schema_name: str) -> None:
        if value_type_from_existing(self.field_types, schema_name, key, target[key]) not in {"int", "float"}:
            return
        widget.bind("<Up>", lambda event, d=target, k=key, v=variable, s=schema_name: self.on_numeric_key_step(d, k, v, s, 1))
        widget.bind("<Down>", lambda event, d=target, k=key, v=variable, s=schema_name: self.on_numeric_key_step(d, k, v, s, -1))

    def bind_value_entry(self, widget, target: dict, key: str, variable: tk.StringVar, schema_name: str, rebuild: bool = False) -> None:
        self.bind_length_scroll(widget, target, key, variable, schema_name)
        self.bind_numeric_step(widget, target, key, variable, schema_name)
        widget.bind("<Return>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))
        widget.bind("<KP_Enter>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))
        widget.bind("<FocusOut>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))

    def on_aero_entry_commit(self, key: str, variable: tk.StringVar) -> str:
        try:
            if type(self.aero_vlm_settings[key]) is int:
                value = int(float(variable.get()))
            else:
                value = float(variable.get())
        except ValueError:
            variable.set(f"{self.aero_vlm_settings[key]:g}")
            return "break"
        self.aero_vlm_settings[key] = value
        variable.set(f"{value:g}")
        return "break"

    def on_aero_numeric_key_step(self, key: str, variable: tk.StringVar, delta: int) -> str:
        try:
            if type(self.aero_vlm_settings[key]) is int:
                current_value = int(float(variable.get()))
                variable.set(str(current_value + delta))
            else:
                current_value = float(variable.get())
                variable.set(f"{current_value + delta:g}")
        except ValueError:
            variable.set(f"{self.aero_vlm_settings[key]:g}")
            return "break"
        return self.on_aero_entry_commit(key, variable)

    def bind_aero_entry(self, widget, key: str, variable: tk.StringVar) -> None:
        widget.bind("<Return>", lambda event, k=key, v=variable: self.on_aero_entry_commit(k, v))
        widget.bind("<KP_Enter>", lambda event, k=key, v=variable: self.on_aero_entry_commit(k, v))
        widget.bind("<FocusOut>", lambda event, k=key, v=variable: self.on_aero_entry_commit(k, v))
        widget.bind("<Up>", lambda event, k=key, v=variable: self.on_aero_numeric_key_step(k, v, 1))
        widget.bind("<Down>", lambda event, k=key, v=variable: self.on_aero_numeric_key_step(k, v, -1))

    def show_main_view(self, view_name: str) -> None:
        self.active_main_view = view_name
        if view_name == "Aero Analysis":
            self.aero_view.tkraise()
            return
        self.configuration_view.tkraise()

    def start_aero_vlm_run(self) -> None:
        if self.aero_vlm_thread is not None and self.aero_vlm_thread.is_alive():
            return
        self.light_recompute()
        self.aero_vlm_queue = queue.Queue()
        self.aero_status_var.set("Running")
        self.aero_progress_text_var.set("0 / 0")
        if self.aero_progressbar is not None:
            self.aero_progressbar.configure(value=0.0)
        if self.aero_run_button is not None:
            self.aero_run_button.configure(state="disabled")
        analysis_snapshot = copy.deepcopy(self.analysis)
        settings_snapshot = copy.deepcopy(self.aero_vlm_settings)
        self.aero_vlm_thread = threading.Thread(
            target=self.run_aero_vlm_worker,
            args=(analysis_snapshot, settings_snapshot),
            daemon=True,
        )
        self.aero_vlm_thread.start()
        self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_vlm_queue)

    def run_aero_vlm_worker(self, analysis_snapshot: dict, settings_snapshot: dict) -> None:
        try:
            result = run_vlm_sweep(
                analysis_snapshot,
                settings_snapshot,
                progress_callback=lambda completed, total, status: self.aero_vlm_queue.put(("progress", completed, total, status)),
            )
            self.aero_vlm_queue.put(("done", result))
        except Exception:
            self.aero_vlm_queue.put(("error", "".join(traceback.format_exc())))

    def poll_aero_vlm_queue(self) -> None:
        while True:
            try:
                message = self.aero_vlm_queue.get_nowait()
            except queue.Empty:
                break
            if message[0] == "progress":
                _, completed, total, status = message
                self.aero_status_var.set(status)
                self.aero_progress_text_var.set(f"{completed} / {total}")
                if self.aero_progressbar is not None and total > 0:
                    self.aero_progressbar.configure(value=(completed / total) * 100.0)
            elif message[0] == "done":
                self.aero_vlm_result = message[1]
                self.aero_status_var.set("Complete")
                self.aero_progress_text_var.set(f"{len(self.aero_vlm_result['rows'])} / {len(self.aero_vlm_result['rows'])}")
                if self.aero_progressbar is not None:
                    self.aero_progressbar.configure(value=100.0)
                if self.aero_results_text is not None:
                    self.aero_results_text.delete("1.0", tk.END)
                    self.aero_results_text.insert("1.0", self.aero_vlm_result["report_text"])
                if self.aero_run_button is not None:
                    self.aero_run_button.configure(state="normal")
            elif message[0] == "error":
                self.aero_status_var.set("Error")
                if self.aero_run_button is not None:
                    self.aero_run_button.configure(state="normal")
                self.show_error_dialog(message[1])
        if self.aero_vlm_thread is not None and self.aero_vlm_thread.is_alive():
            self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_vlm_queue)

    def apply_module_template(self, part: dict, module_type: str) -> None:
        part["module"] = apply_template_values(part["module"], self.templates_data["module_templates"][module_type])
        part["module"]["type"] = module_type
        if module_type == "wingmount":
            if "surfaces" not in part:
                part["surfaces"] = copy.deepcopy(self.templates_data["surface_templates"]["wing"])
        else:
            part.pop("surfaces", None)

    def apply_surface_template(self, part: dict, surface_type: str) -> None:
        part["surfaces"] = apply_template_values(part["surfaces"], self.templates_data["surface_templates"][surface_type])
        part["surfaces"]["type"] = surface_type

    def on_module_type_changed(self, part: dict, variable: tk.StringVar) -> None:
        self.apply_module_template(part, variable.get())
        self.rebuild_module_editor()
        self.light_recompute()

    def on_surface_type_changed(self, part: dict, variable: tk.StringVar) -> None:
        self.apply_surface_template(part, variable.get())
        self.rebuild_module_editor()
        self.light_recompute()

    def add_new_part(self) -> None:
        module_type = self.templates_data["new_part_module_type"]
        new_part = copy.deepcopy(self.part_defaults)
        new_part["module"] = copy.deepcopy(self.templates_data["module_templates"][module_type])
        if module_type == "wingmount":
            new_part["surfaces"] = copy.deepcopy(self.templates_data["surface_templates"]["wing"])
        self.current_config["parts"].append(new_part)
        self.rebuild_module_editor()
        self.light_recompute()

    def move_part(self, index: int, direction: int) -> None:
        new_index = index + direction
        if new_index < 0 or new_index >= len(self.current_config["parts"]):
            return
        parts_list = self.current_config["parts"]
        parts_list[index], parts_list[new_index] = parts_list[new_index], parts_list[index]
        self.rebuild_module_editor()
        self.light_recompute()

    def delete_part(self, index: int) -> None:
        self.current_config["parts"].pop(index)
        self.rebuild_module_editor()
        self.light_recompute()

    def save_json(self) -> None:
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save Aircraft JSON",
            initialdir=str(SCRIPT_DIR),
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        analysis_payload = None
        if self.analysis is not None:
            analysis_payload = {
                key: value
                for key, value in self.analysis.items()
                if key != "assembly"
            }
        payload = {
            "assembly_name": self.assembly_name_var.get(),
            "config": self.current_config,
            "analysis": analysis_payload,
        }
        with open(path, "w") as file:
            json.dump(json_safe_data(payload), file, indent=4, allow_nan=False)

    def load_json(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="Load Aircraft JSON",
            initialdir=str(SCRIPT_DIR),
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        with open(path, "r") as file:
            data = json.load(file)
        if "parts" in data:
            assembly_name = Path(path).stem
            config = data
        elif "assembly_name" in data and "config" in data:
            assembly_name = data["assembly_name"]
            config = data["config"]
        else:
            assembly_name = next(iter(data.keys()))
            config = data[assembly_name]
        self.current_assembly_name = assembly_name
        self.current_config = self.normalize_config_from_templates(config)
        self.apply_standard_dimensions_to_config(self.current_config)
        self.assembly_name_var.set(assembly_name)
        self.standard_profile_var.set(STANDARD_NONE_OPTION if self.current_config["standard_profile"] == "" else self.current_config["standard_profile"])
        self.apply_standard_var.set(self.current_config["apply_standard"])
        self.rebuild_module_editor()
        self.heavy_recompute()

    def rebuild_module_editor(self) -> None:
        for child in self.module_inner.winfo_children():
            child.destroy()
        for child in self.assembly_inner.winfo_children():
            child.destroy()
        self.module_inner.columnconfigure(0, weight=1)

        config_frame = ttk.LabelFrame(self.assembly_inner, text="Assembly", padding=6)
        config_frame.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        config_frame.columnconfigure(1, weight=1)
        row_index = 0
        for key in ordered_keys(self.current_config, CONFIG_FIELD_ORDER):
            if key in {"parts", *MAIN_DRIVER_KEYS, "standard_profile", "apply_standard"}:
                continue
            value = self.current_config[key]
            ttk.Label(config_frame, text=field_label(key)).grid(row=row_index, column=0, sticky="w")
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                widget = ttk.Checkbutton(config_frame, variable=variable, command=lambda d=self.current_config, k=key, v=variable: self.on_bool_changed(d, k, v, False))
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            else:
                variable = tk.StringVar(value=format_field_value(key, value))
                widget = ttk.Entry(config_frame, textvariable=variable)
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                self.bind_value_entry(widget, self.current_config, key, variable, "config")
            row_index += 1

        for index, part in enumerate(self.current_config["parts"]):
            module_name = part["module"]["name"]
            frame = ttk.LabelFrame(self.module_inner, text=f"{index + 1}. {module_name}", padding=6)
            frame.grid(row=index, column=0, sticky="ew", padx=4, pady=4)
            frame.columnconfigure(0, weight=1)

            button_bar = ttk.Frame(frame)
            button_bar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
            ttk.Button(button_bar, text="Up", command=lambda idx=index: self.move_part(idx, -1)).pack(side="left")
            ttk.Button(button_bar, text="Down", command=lambda idx=index: self.move_part(idx, 1)).pack(side="left", padx=4)
            ttk.Button(button_bar, text="Delete", command=lambda idx=index: self.delete_part(idx)).pack(side="left")

            self.build_dict_editor(frame, "Parameters", part["module"], MODULE_FIELD_ORDER, MODULE_TYPE_OPTIONS, 1, {"diam", "model"}, "module", part)
            next_row = 2
            if "surfaces" in part:
                self.build_dict_editor(frame, "Surface", part["surfaces"], SURFACE_FIELD_ORDER, SURFACE_TYPE_OPTIONS, next_row, {"model"}, "surface", part)
                next_row += 1

            show_motor_controls = part["module"].get("motor", False) or ("surfaces" in part and part["surfaces"]["motor"])
            if show_motor_controls:
                motor_frame = ttk.LabelFrame(frame, text="Propulsion", padding=6)
                motor_frame.grid(row=next_row, column=0, sticky="ew", pady=(6, 0))
                motor_frame.columnconfigure(1, weight=1)
                available_props = self.available_props_for_motor(part["motors"])
                if part["props"] not in available_props:
                    part["props"] = available_props[0] if available_props else ""
                motor_var = tk.StringVar(value=str(part["motors"]))
                prop_var = tk.StringVar(value=str(part["props"]))
                ttk.Label(motor_frame, text="motor").grid(row=0, column=0, sticky="w")
                motor_box = ttk.Combobox(motor_frame, values=self.motor_names, textvariable=motor_var, state="readonly")
                motor_box.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
                ttk.Label(motor_frame, text="prop").grid(row=1, column=0, sticky="w")
                prop_box = ttk.Combobox(motor_frame, values=available_props, textvariable=prop_var, state="readonly")
                prop_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
                motor_box.bind("<<ComboboxSelected>>", lambda event, d=part, mv=motor_var, pv=prop_var, pb=prop_box: self.on_motor_selected(d, mv, pv, pb))
                prop_box.bind("<<ComboboxSelected>>", lambda event, d=part, v=prop_var: self.on_select_value(d, "props", v))
                next_row += 1

            part_keys = []
            if part["module"].get("battery", False):
                part_keys.append("battery")
            if part["module"].get("payload", False):
                part_keys.append("payload_mass")
            if part["module"].get("ballast", False):
                part_keys.append("ballast_mass")

            if part_keys:
                aux_frame = ttk.LabelFrame(frame, text="Part Fields", padding=6)
                aux_frame.grid(row=next_row, column=0, sticky="ew", pady=(6, 0))
                aux_frame.columnconfigure(1, weight=1)
                for aux_row, key in enumerate(part_keys):
                    value = part[key]
                    ttk.Label(aux_frame, text=field_label(key)).grid(row=aux_row, column=0, sticky="w")
                    variable = tk.StringVar(value=format_field_value(key, value))
                    if key == "battery":
                        entry = ttk.Combobox(aux_frame, values=sorted(self.parts_data["battery"].keys()), textvariable=variable, state="readonly")
                        entry.grid(row=aux_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                        entry.bind("<<ComboboxSelected>>", lambda event, d=part, v=variable: self.on_select_value(d, "battery", v, True))
                    else:
                        entry = ttk.Entry(aux_frame, textvariable=variable)
                        entry.grid(row=aux_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                        self.bind_value_entry(entry, part, key, variable, "part")

    def build_dict_editor(
        self,
        parent,
        title: str,
        data: dict,
        preferred_order: list[str],
        type_options: list[str] | None,
        row: int,
        exclude_keys: set[str],
        schema_name: str,
        owner_part: dict,
    ) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        frame.columnconfigure(1, weight=1)
        visible_keys = [key for key in ordered_keys(data, preferred_order) if key not in exclude_keys]
        if "motor" in data and not data["motor"]:
            visible_keys = [key for key in visible_keys if key not in MOTOR_EDIT_KEYS]
        for row_index, key in enumerate(visible_keys):
            value = data[key]
            ttk.Label(frame, text=field_label(key)).grid(row=row_index, column=0, sticky="w")
            if key == "type" and type_options is not None:
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=type_options, textvariable=variable, state="readonly")
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                if schema_name == "module":
                    widget.bind("<<ComboboxSelected>>", lambda event, p=owner_part, v=variable: self.on_module_type_changed(p, v))
                else:
                    widget.bind("<<ComboboxSelected>>", lambda event, p=owner_part, v=variable: self.on_surface_type_changed(p, v))
                continue
            if key == "motor_dir":
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=MOTOR_DIRECTION_OPTIONS, textvariable=variable, state="readonly")
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                widget.bind("<<ComboboxSelected>>", lambda event, d=data, v=variable: self.on_select_value(d, "motor_dir", v, True))
                continue
            if key == "pod_shape":
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=POD_SHAPE_OPTIONS, textvariable=variable, state="readonly")
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                widget.bind("<<ComboboxSelected>>", lambda event, d=data, v=variable: self.on_select_value(d, "pod_shape", v, True))
                continue
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                widget = ttk.Checkbutton(frame, variable=variable, command=lambda d=data, k=key, v=variable: self.on_bool_changed(d, k, v))
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                continue
            variable = tk.StringVar(value=format_field_value(key, value))
            widget = ttk.Entry(frame, textvariable=variable)
            widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.bind_value_entry(widget, data, key, variable, schema_name, key == "name")

    def light_recompute(self) -> None:
        self.current_assembly_name = self.assembly_name_var.get()
        self.last_aoa_eval_deg = float(self.aoa_var.get())
        self.aoa_display_var.set(f"{self.last_aoa_eval_deg:.1f} deg")
        self.analysis = recompute_analysis(
            assembly_name=self.current_assembly_name,
            config=self.current_config,
            modules_data=self.modules_data,
            parts_data=self.parts_data,
            standards_data=self.standards_data,
            motor_data_df=self.motor_data_df,
            aoa_eval_deg=self.last_aoa_eval_deg,
        )
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", self.analysis["report_text"])
        self.draw_active_view()
        self.heavy_dirty = True
        self.recompute_status_var.set("ASB stale (F5)")

    def heavy_recompute(self) -> None:
        self.light_recompute()
        self.asb_airplane = build_asb_airplane(self.analysis)
        self.heavy_dirty = False
        self.recompute_status_var.set("Up to date")
        self.draw_asb_view(force=True)

    def draw_active_view(self) -> None:
        if self.analysis is None:
            return
        draw_canvas_view(self.view_canvas, self.analysis, self.view_mode_var.get(), self.roll_view_var.get(), UI_DRAW, UI_BG)

    def draw_asb_view(self, force: bool = False) -> None:
        if self.asb_airplane is None or self.asb_frame is None:
            return
        if self.asb_draw_busy:
            return
        frame_width = max(self.asb_frame.winfo_width(), 200)
        frame_height = max(self.asb_frame.winfo_height(), 200)
        if not force and self.asb_canvas is not None and self.asb_last_size == (frame_width, frame_height):
            return
        self.asb_last_size = (frame_width, frame_height)
        fig_width = frame_width / 100.0
        fig_height = frame_height / 100.0
        self.asb_draw_busy = True
        try:
            if self.asb_canvas is not None:
                self.asb_canvas.get_tk_widget().destroy()
            if self.asb_figure is not None:
                plt.close(self.asb_figure)
            self.asb_figure, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=100, subplot_kw={"projection": "3d"})
            ax.set_facecolor(UI_BG)
            ax.tick_params(colors=UI_FG)
            ax.xaxis.label.set_color(UI_FG)
            ax.yaxis.label.set_color(UI_FG)
            ax.zaxis.label.set_color(UI_FG)
            ax.xaxis.pane.set_facecolor(UI_BG)
            ax.yaxis.pane.set_facecolor(UI_BG)
            ax.zaxis.pane.set_facecolor(UI_BG)
            ax.xaxis.pane.set_alpha(0.0)
            ax.yaxis.pane.set_alpha(0.0)
            ax.zaxis.pane.set_alpha(0.0)
            self.asb_airplane.draw(
                backend="matplotlib",
                ax=ax,
                use_preset_view_angle="left_isometric",
                set_background_pane_color=UI_BG,
                set_background_pane_alpha=0.0,
                set_axis_visibility=False,
                show=False,
            )
            self.asb_figure.patch.set_facecolor(UI_BG)
            self.asb_figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
            ax.set_position([0.0, 0.0, 1.0, 1.0])
            self.asb_canvas = FigureCanvasTkAgg(self.asb_figure, master=self.asb_frame)
            self.asb_canvas.draw()
            self.asb_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        finally:
            self.asb_draw_busy = False


def main() -> None:
    parser = argparse.ArgumentParser(description="Builder V3 Tk frontend")
    parser.parse_args()
    root = tk.Tk()
    BuilderV3TkApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
