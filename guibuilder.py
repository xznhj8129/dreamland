"""
Usage:
python guibuilder.py
"""

from __future__ import annotations

import argparse
import copy
import json
import queue
import re
import sys
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

SCRIPT_DIR = Path(__file__).resolve().parent
TEMPLATES_PATH = SCRIPT_DIR / "data" / "templates.json"
sys.path.insert(0, str(SCRIPT_DIR))
from libs.analysis import build_asb_airplane, build_vlm_report_text, json_safe_data, load_builder_data as analysis_load_builder_data, parse_motor_name, recompute_analysis, vlm_point_key
from libs.views.aero_analysis_view import AeroAnalysisMainView
from libs.views.cad_view import CADMainView
from libs.views.configuration_view import ConfigurationMainView
from libs.views.module_editor_view import ModuleEditorMainView
from libs.views.part_editor_view import PartEditorMainView
from libs.views.vlm_view import VLMMainView

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
    "pointed_shape",
    "fins",
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
    "powered",
    "thrust_axis_tilt_deg",
    "standard_profile",
    "apply_standard",
]
VIEW_OPTIONS = ["Longitudinal", "Front"]
MAIN_DRIVER_KEYS = ["fuselage_od_mm", "fuselage_id_mm", "material_density_g_cm3"]
POD_SHAPE_OPTIONS = ["elliptical", "ogive"]
POINTED_SHAPE_OPTIONS = [
    "conic",
    "elliptical",
    "parabolic",
    "spherically_blunted_conic",
    "bi_conic",
    "tangent_ogive",
    "spherically_blunted_tangent_ogive",
    "secant_ogive",
    "power_series",
    "haack_series",
]
POINTED_SHAPE_PARAM_KEYS = {
    "conic": [],
    "elliptical": [],
    "parabolic": ["k_prime"],
    "spherically_blunted_conic": ["nose_radius"],
    "bi_conic": ["break_fraction", "break_radius_fraction"],
    "tangent_ogive": [],
    "spherically_blunted_tangent_ogive": ["nose_radius"],
    "secant_ogive": ["rho_factor"],
    "power_series": ["exponent"],
    "haack_series": ["c"],
}
POINTED_SHAPE_FIELD_ORDER = ["shape", "k_prime", "rho_factor", "exponent", "c", "break_fraction", "break_radius_fraction", "nose_radius"]
MAIN_DRIVER_LABELS = {
    "assembly_name": "Assembly Name",
    "fuselage_od_mm": "Fuselage OD (mm)",
    "fuselage_id_mm": "Fuselage ID (mm)",
    "material_density_g_cm3": "Material Density (g/cm^3)",
}
PROPULSION_LABELS = {
    "battery": "Battery",
    "motors": "Motor",
    "props": "Propeller",
}
LENGTH_FIELD_KEYS_M = {
    "len",
    "diam",
    "x",
    "y",
    "z",
    "nose_radius",
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
CONFIG_FORCE_DEFAULTS = {
    "aoa_deg": 0.0,
    "airspeed_m_s": 10.0,
    "altitude_m": 0.0,
}
CONFIG_FORCE_LABELS = {
    "aoa_deg": "AoA (deg)",
    "airspeed_m_s": "Airspeed (m/s)",
    "altitude_m": "Altitude (m)",
}
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
    if key == "wings_x":
        return "fore/aft mount (0=front, 1=back)"
    if key == "pointed_shape":
        return "pointed shape"
    if key == "shape":
        return "shape"
    if key == "k_prime":
        return "parabolic K'"
    if key == "rho_factor":
        return "secant rho / tangent rho"
    if key == "exponent":
        return "power n"
    if key == "c":
        return "haack C"
    if key == "break_fraction":
        return "bi-conic L1 / L"
    if key == "break_radius_fraction":
        return "bi-conic R1 / R"
    if key == "nose_radius":
        return "nose radius r_n (mm)"
    if key == "x":
        return "position x / front (mm)"
    if key == "y":
        return "position y / right (mm)"
    if key == "z":
        return "position z / up (mm)"
    if key == "pitch":
        return "pitch (deg)"
    if key == "yaw":
        return "yaw (deg)"
    if key == "roll":
        return "roll (deg)"
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


def text_is_valid_for_value_type(value_type: str, proposed_text: str) -> bool:
    if value_type == "int":
        return re.fullmatch(r"[+-]?\d*", proposed_text) is not None
    if value_type == "float":
        return re.fullmatch(r"[+-]?((\d+(\.\d*)?)|(\.\d*))?([eE][+-]?\d*)?", proposed_text) is not None
    return True


def apply_template_values(existing: dict, template: dict) -> dict:
    updated = copy.deepcopy(template)
    for key in updated:
        if key in existing:
            updated[key] = existing[key]
    return updated


def module_supports_surfaces(module_data: dict) -> bool:
    if module_data["type"] == "wingmount":
        return True
    return module_data["type"] == "end" and module_data.get("fins", False)


class BuilderV3TkApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Builder V3")
        self.root.geometry("1800x980")
        self.root.configure(background=UI_BG)
        self.root.report_callback_exception = self.handle_callback_exception
        self.ui_bg = UI_BG
        self.ui_panel = UI_PANEL
        self.ui_border = UI_BORDER
        self.ui_fg = UI_FG
        self.ui_accent = UI_ACCENT
        self.ui_canvas = UI_CANVAS
        self.ui_draw = UI_DRAW
        self.view_options = VIEW_OPTIONS
        self.main_driver_keys = MAIN_DRIVER_KEYS
        self.main_driver_labels = MAIN_DRIVER_LABELS
        self.propulsion_labels = PROPULSION_LABELS
        self.module_type_options = MODULE_TYPE_OPTIONS
        self.surface_type_options = SURFACE_TYPE_OPTIONS
        self.motor_direction_options = MOTOR_DIRECTION_OPTIONS
        self.pod_shape_options = POD_SHAPE_OPTIONS
        self.pointed_shape_options = POINTED_SHAPE_OPTIONS
        self.pointed_shape_param_keys = POINTED_SHAPE_PARAM_KEYS
        self.pointed_shape_field_order = POINTED_SHAPE_FIELD_ORDER
        self.config_field_order = CONFIG_FIELD_ORDER
        self.module_field_order = MODULE_FIELD_ORDER
        self.surface_field_order = SURFACE_FIELD_ORDER
        self.motor_edit_keys = MOTOR_EDIT_KEYS
        self.standard_none_option = STANDARD_NONE_OPTION
        self.aero_vlm_labels = AERO_VLM_LABELS
        self.aero_queue_poll_ms = AERO_QUEUE_POLL_MS
        self.config_force_labels = CONFIG_FORCE_LABELS
        self.ordered_keys = ordered_keys
        self.format_field_value = format_field_value
        self.field_label = field_label
        self.modules_data, self.parts_data, self.default_config, self.standards_data, self.motor_data_df, self.motor_names, self.propeller_names = analysis_load_builder_data()
        self.templates_data = load_templates_data()
        self.field_types = self.templates_data["field_types"]
        self.part_defaults = self.templates_data["part_defaults"]
        self.module_library_names = self.list_module_library_names()
        self.current_assembly_name = "default"
        self.current_config = self.normalize_config_from_templates(self.default_config)
        self.apply_standard_dimensions_to_config(self.current_config)
        self.standard_names = sorted(self.standards_data["specs"].keys())
        self.recompute_status_var = tk.StringVar(value="Up to date")
        self.assembly_name_var = tk.StringVar(value=self.current_assembly_name)
        self.standard_profile_var = tk.StringVar(value=STANDARD_NONE_OPTION if self.current_config["standard_profile"] == "" else self.current_config["standard_profile"])
        self.apply_standard_var = tk.BooleanVar(value=self.current_config["apply_standard"])
        self.propulsion_battery_var = tk.StringVar(value=self.current_config["battery"])
        self.propulsion_motor_var = tk.StringVar(value=self.current_config["motors"])
        self.propulsion_prop_var = tk.StringVar(value=self.current_config["props"])
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
        self.airfoil_view = None
        self.aero_view = None
        self.cad_view = None
        self.module_editor_view = None
        self.part_editor_view = None
        self.module_editor_panel = None
        self.part_editor_panel = None
        self.configuration_main_view = None
        self.airfoil_main_view = None
        self.vlm_main_view = None
        self.cad_main_view = None
        self.module_editor_main_view = None
        self.part_editor_main_view = None
        self.active_main_view = "Configuration"
        self.aero_run_button = None
        self.aero_progressbar = None
        self.aero_results_text: tk.Text | None = None
        self.aero_display_label = None
        self.aero_display_frame = None
        self.aero_status_var = tk.StringVar(value="Idle")
        self.aero_progress_text_var = tk.StringVar(value="0 / 0")
        self.config_force_settings = copy.deepcopy(CONFIG_FORCE_DEFAULTS)
        self.config_force_vars = {key: tk.StringVar(value=f"{value:g}") for key, value in self.config_force_settings.items()}
        self.aero_vlm_settings = copy.deepcopy(AERO_VLM_DEFAULTS)
        self.aero_vlm_vars = {}
        self.aero_vlm_thread: threading.Thread | None = None
        self.aero_vlm_queue: queue.Queue = queue.Queue()
        self.aero_vlm_generation = 0
        self.aero_vlm_result = None
        self.aero_vlm_cache = {}
        self.aero_vlm_rows_by_key = {}
        self.aero_altitude_values = []
        self.aero_aoa_values = []
        self.aero_airspeed_values = []
        self.aero_altitude_value_var = tk.DoubleVar(value=0.0)
        self.aero_aoa_value_var = tk.DoubleVar(value=0.0)
        self.aero_airspeed_value_var = tk.DoubleVar(value=0.0)
        self.aero_altitude_display_var = tk.StringVar(value="0.0 m")
        self.aero_aoa_display_var = tk.StringVar(value="0.0 deg")
        self.aero_airspeed_display_var = tk.StringVar(value="0.0 m/s")
        self.aero_selected_point_var = tk.StringVar(value="No VLM point selected.")
        self.summary_text: tk.Text | None = None
        self.propulsion_text: tk.Text | None = None
        self.propulsion_plot_canvas = None
        self.propulsion_plot_figure = None
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
            if "x" not in part:
                normalized_part["x"] = module_data["cg"] * module_data["len"] - module_data["len"] * 0.5
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
            self.sync_part_surfaces(normalized_part)
            normalized_parts.append(normalized_part)
        normalized["parts"] = normalized_parts
        return normalized

    def sync_part_surfaces(self, part: dict) -> None:
        if module_supports_surfaces(part["module"]):
            if "surfaces" not in part:
                part["surfaces"] = copy.deepcopy(self.templates_data["surface_templates"]["wing"])
            return
        part.pop("surfaces", None)

    def apply_standard_dimensions_to_config(self, config: dict) -> None:
        if config["apply_standard"] and config["standard_profile"]:
            standard_spec = self.standards_data["specs"][config["standard_profile"]]
            config["fuselage_od_mm"] = standard_spec["module_size"]["nominal_od"]
            config["fuselage_id_mm"] = standard_spec["module_size"]["nominal_id"]

    def list_module_library_names(self) -> list[str]:
        return sorted(name for name, data in self.modules_data.items() if data["type"] in MODULE_TYPE_OPTIONS)

    def reload_library_data(self) -> None:
        (
            self.modules_data,
            self.parts_data,
            self.default_config,
            self.standards_data,
            self.motor_data_df,
            self.motor_names,
            self.propeller_names,
        ) = analysis_load_builder_data()
        self.module_library_names = self.list_module_library_names()
        self.standard_names = sorted(self.standards_data["specs"].keys())
        self.rebuild_module_editor()
        self.light_recompute()

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
        style.configure("AssemblyCard.TLabelframe.Label", background=UI_BG, foreground=UI_ACCENT, font=("TkDefaultFont", 12, "bold"))
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

        self.configuration_main_view = ConfigurationMainView(self, main_view_host)
        self.airfoil_main_view = AeroAnalysisMainView(self, main_view_host)
        self.vlm_main_view = VLMMainView(self, main_view_host)
        self.cad_main_view = CADMainView(self, main_view_host)
        self.module_editor_main_view = ModuleEditorMainView(self, main_view_host)
        self.part_editor_main_view = PartEditorMainView(self, main_view_host)

        bottom_bar = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        bottom_bar.grid(row=1, column=0, sticky="ew")
        ttk.Button(bottom_bar, text="Configuration", command=lambda: self.show_main_view("Configuration")).pack(side="left")
        ttk.Button(bottom_bar, text="Aero Analysis", command=lambda: self.show_main_view("Aero Analysis")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="VLM", command=lambda: self.show_main_view("VLM")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="CAD", command=lambda: self.show_main_view("CAD")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Module Editor", command=lambda: self.show_main_view("Module Editor")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Part Editor", command=lambda: self.show_main_view("Part Editor")).pack(side="left", padx=(8, 0))
        self.show_main_view("Configuration")

    def on_manual_recompute(self, _event=None) -> str:
        if self.active_main_view == "Aero Analysis":
            self.airfoil_main_view.recompute_airfoils()
            return "break"
        if self.active_main_view == "VLM":
            self.vlm_main_view.start_run()
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
        battery_name = self.current_config["battery"]
        if not battery_name:
            return None
        return int(self.parts_data["battery"][battery_name]["s"])

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

    def refresh_propulsion_controls(self) -> None:
        available_props = self.available_props_for_motor(self.current_config["motors"])
        self.propulsion_prop_box.configure(values=available_props)
        if self.current_config["props"] not in available_props:
            self.current_config["props"] = available_props[0] if available_props else ""
        self.propulsion_battery_var.set(self.current_config["battery"])
        self.propulsion_motor_var.set(self.current_config["motors"])
        self.propulsion_prop_var.set(self.current_config["props"])

    def on_propulsion_battery_selected(self) -> None:
        self.current_config["battery"] = self.propulsion_battery_var.get()
        self.refresh_propulsion_controls()
        self.light_recompute()

    def on_propulsion_motor_selected(self) -> None:
        self.current_config["motors"] = self.propulsion_motor_var.get()
        self.refresh_propulsion_controls()
        self.light_recompute()

    def on_propulsion_prop_selected(self) -> None:
        self.current_config["props"] = self.propulsion_prop_var.get()
        self.light_recompute()

    def on_bool_changed(self, target: dict, key: str, variable: tk.BooleanVar, rebuild: bool = True) -> None:
        target[key] = bool(variable.get())
        if key == "fins":
            for part in self.current_config["parts"]:
                if part.get("module") is target:
                    self.sync_part_surfaces(part)
                    break
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
        value_type = value_type_from_existing(self.field_types, schema_name, key, target[key])
        if value_type in {"int", "float"}:
            widget.configure(
                validate="key",
                validatecommand=(self.root.register(text_is_valid_for_value_type), value_type, "%P"),
            )
        self.bind_length_scroll(widget, target, key, variable, schema_name)
        self.bind_numeric_step(widget, target, key, variable, schema_name)
        widget.bind("<Return>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))
        widget.bind("<KP_Enter>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))
        widget.bind("<FocusOut>", lambda event, d=target, k=key, v=variable, s=schema_name, r=rebuild: self.on_entry_commit(d, k, v, s, r))

    def show_main_view(self, view_name: str) -> None:
        self.active_main_view = view_name
        if view_name == "Aero Analysis":
            self.airfoil_view.tkraise()
            return
        if view_name == "VLM":
            self.aero_view.tkraise()
            return
        if view_name == "CAD":
            self.cad_view.tkraise()
            return
        if view_name == "Module Editor":
            self.module_editor_panel.refresh_from_disk()
            self.module_editor_view.tkraise()
            return
        if view_name == "Part Editor":
            self.part_editor_panel.refresh_from_disk()
            self.part_editor_view.tkraise()
            return
        self.configuration_view.tkraise()

    def apply_module_template(self, part: dict, module_type: str) -> None:
        part["module"] = apply_template_values(part["module"], self.templates_data["module_templates"][module_type])
        part["module"]["type"] = module_type
        self.sync_part_surfaces(part)

    def apply_surface_template(self, part: dict, surface_type: str) -> None:
        part["surfaces"] = apply_template_values(part["surfaces"], self.templates_data["surface_templates"][surface_type])
        part["surfaces"]["type"] = surface_type

    def load_part_module_from_library(self, part: dict, module_name: str) -> None:
        module_data = copy.deepcopy(self.modules_data[module_name])
        module_type = module_data["type"]
        part["module"] = apply_template_values(module_data, self.templates_data["module_templates"][module_type])
        part["module"]["name"] = module_name
        part["module"]["type"] = module_type
        self.sync_part_surfaces(part)

    def on_load_module_from_library(self, part: dict, variable: tk.StringVar) -> None:
        self.load_part_module_from_library(part, variable.get())
        self.rebuild_module_editor()
        self.light_recompute()

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
        self.sync_part_surfaces(new_part)
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
                if key not in {"assembly", "report_text"}
            }
        vlm_results_payload = None
        if self.aero_vlm_result is not None:
            vlm_results_payload = {
                key: value
                for key, value in self.aero_vlm_result.items()
                if key not in {"vlm_cache", "report_text"}
            }
        payload = {
            "assembly_name": self.assembly_name_var.get(),
            "config": self.current_config,
            "analysis": analysis_payload,
            "vlm_results": vlm_results_payload,
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
            vlm_results = None
        elif "assembly_name" in data and "config" in data:
            assembly_name = data["assembly_name"]
            config = data["config"]
            vlm_results = data.get("vlm_results")
        else:
            assembly_name = next(iter(data.keys()))
            config = data[assembly_name]
            vlm_results = None
        self.current_assembly_name = assembly_name
        self.current_config = self.normalize_config_from_templates(config)
        self.apply_standard_dimensions_to_config(self.current_config)
        self.assembly_name_var.set(assembly_name)
        self.standard_profile_var.set(STANDARD_NONE_OPTION if self.current_config["standard_profile"] == "" else self.current_config["standard_profile"])
        self.apply_standard_var.set(self.current_config["apply_standard"])
        self.rebuild_module_editor()
        self.heavy_recompute()
        if vlm_results is not None:
            vlm_results["report_text"] = build_vlm_report_text(vlm_results)
            self.aero_vlm_result = vlm_results
            self.aero_vlm_rows_by_key = {
                vlm_point_key(row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"]): row
                for row in self.aero_vlm_result["rows"]
            }
            self.aero_status_var.set("Loaded")
            self.aero_progress_text_var.set(f"{len(self.aero_vlm_result['rows'])} / {len(self.aero_vlm_result['rows'])}")
            if self.aero_progressbar is not None:
                self.aero_progressbar.configure(value=100.0)
            if self.aero_results_text is not None:
                self.aero_results_text.delete("1.0", tk.END)
                self.aero_results_text.insert("1.0", self.aero_vlm_result["report_text"])
            if self.aero_display_label is not None:
                self.aero_display_label.configure(text="Loaded VLM rows only. Re-run VLM to Show geometry.")
            self.vlm_main_view.configure_selectors()
            self.vlm_main_view.on_aero_selector_changed("0")

    def rebuild_module_editor(self) -> None:
        self.configuration_main_view.rebuild_module_editor()

    def light_recompute(self) -> None:
        self.aero_vlm_generation += 1
        self.current_assembly_name = self.assembly_name_var.get()
        self.analysis = recompute_analysis(
            assembly_name=self.current_assembly_name,
            config=self.current_config,
            modules_data=self.modules_data,
            parts_data=self.parts_data,
            standards_data=self.standards_data,
            motor_data_df=self.motor_data_df,
            aoa_eval_deg=self.config_force_settings["aoa_deg"],
            force_eval_airspeed_m_s=self.config_force_settings["airspeed_m_s"],
            force_eval_altitude_m=self.config_force_settings["altitude_m"],
        )
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", self.analysis["report_text"])
        self.configuration_main_view.draw_active_view()
        self.configuration_main_view.draw_propulsion_info()
        self.vlm_main_view.clear_display("VLM cache stale. Press F5 in VLM.")
        self.aero_status_var.set("Stale (F5)")
        self.aero_progress_text_var.set("0 / 0")
        if self.aero_progressbar is not None:
            self.aero_progressbar.configure(value=0.0)
        self.heavy_dirty = True
        self.recompute_status_var.set("ASB stale (F5)")

    def heavy_recompute(self) -> None:
        self.light_recompute()
        self.asb_airplane = build_asb_airplane(self.analysis)
        self.heavy_dirty = False
        self.recompute_status_var.set("Up to date")
        self.configuration_main_view.draw_asb_view(force=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Builder V3 Tk frontend")
    parser.parse_args()
    root = tk.Tk()
    BuilderV3TkApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
