"""
Usage:
from libs.library_editors import ModuleEditorFrame, PartEditorFrame
"""

from __future__ import annotations

import copy
import json
import re
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent / "data"
MODULES_PATH = DATA_DIR / "modules.json"
PARTS_PATH = DATA_DIR / "parts.json"
TEMPLATES_PATH = DATA_DIR / "templates.json"

UI_BG = "#000000"
UI_PANEL = "#031a03"
UI_BORDER = "#0a5f0a"
UI_FG = "#67ff67"
UI_ACCENT = "#19ff19"
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
MODULE_LIBRARY_FIELD_ORDER = [
    "type",
    "model",
    "len",
    "diam",
    "mass",
    "cg",
    "wings_layout",
    "wings_fin_pos",
    "wings_ang",
    "wings_x",
    "wings_y_offset",
    "pointed_shape",
    "fins",
    "semispan",
    "chord_root",
    "chord_tip",
    "aoa_ang",
    "sweep_ang",
    "airfoil",
    "ballast",
    "battery",
    "payload",
    "esc",
    "avionics",
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
PART_LIBRARY_FIELD_ORDER = [
    "type",
    "mass",
    "s",
    "p",
    "mah",
    "c",
    "curr",
    "min_s",
    "max_s",
    "cell",
    "dimensions",
]
NEW_PART_LIBRARY_DEFAULTS = {
    "battery": {
        "type": "lipo",
        "mass": 0.0,
        "s": 4,
        "p": 1,
        "mah": 0,
        "c": 0,
        "dimensions": {"l": 0, "h": 0, "w": 0},
    },
    "esc": {
        "type": "esc",
        "mass": 0.0,
        "curr": 0,
        "min_s": 0,
        "max_s": 0,
    },
}


def load_json_file(path: Path) -> dict:
    with open(path, "r") as file:
        return json.load(file)


def save_json_file(path: Path, data: dict) -> None:
    with open(path, "w") as file:
        json.dump(data, file, indent=4)


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
    if type(value) is list:
        return json.dumps(value)
    return str(value)


def field_label(key: str) -> str:
    if key == "wings_layout":
        return "layout (H V 1 3 + X)"
    if key == "wings_fin_pos":
        return "fin position (top / bottom)"
    if key == "wings_ang":
        return "dihedral (deg)"
    if key == "wings_x":
        return "fore/aft mount (0=front, 1=back)"
    if key == "wings_y_offset":
        return "y-offset on tube (-1=bottom, 0=mid, 1=top)"
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
    if key in LENGTH_FIELD_KEYS_M:
        return f"{key} (mm)"
    if key in MASS_FIELD_KEYS_KG:
        return f"{key} (g)"
    return key


def parse_field_value(key: str, text: str, existing_value):
    if type(existing_value) is int:
        value = int(text)
    elif type(existing_value) is float:
        value = float(text)
    elif type(existing_value) is bool:
        value = text.lower() == "true"
    elif type(existing_value) is list:
        value = json.loads(text)
    else:
        value = text
    if key in LENGTH_FIELD_KEYS_M:
        return value / 1000.0
    if key in MASS_FIELD_KEYS_KG:
        return value / 1000.0
    return value


def text_is_valid_for_existing_value(value_type: str, proposed_text: str) -> bool:
    if value_type == "int":
        return re.fullmatch(r"[+-]?\d*", proposed_text) is not None
    if value_type == "float":
        return re.fullmatch(r"[+-]?((\d+(\.\d*)?)|(\.\d*))?([eE][+-]?\d*)?", proposed_text) is not None
    return True


def set_nested_value(target: dict, path: tuple[str, ...], value) -> None:
    cursor = target
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = value


def listbox_index_from_selection(listbox: tk.Listbox) -> int | None:
    selection = listbox.curselection()
    if not selection:
        return None
    return int(selection[0])


class ModuleEditorFrame(ttk.Frame):
    def __init__(self, master, on_save=None) -> None:
        super().__init__(master, padding=8)
        self.on_save = on_save
        self.templates_data = load_json_file(TEMPLATES_PATH)
        self.module_templates = self.templates_data["module_templates"]
        self.surface_templates = self.templates_data["surface_templates"]
        self.data = {}
        self.current_key = None
        self.field_bindings = []
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.build_ui()
        self.refresh_from_disk()

    def build_ui(self) -> None:
        header = ttk.Frame(self, padding=0)
        header.grid(row=0, column=0, sticky="ew")
        ttk.Button(header, text="Save", command=self.save_current).pack(side="left")
        ttk.Button(header, text="New", command=self.new_module).pack(side="left", padx=(8, 0))
        ttk.Button(header, text="Delete", command=self.delete_current).pack(side="left", padx=(8, 0))

        panes = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        panes.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

        sidebar = ttk.Frame(panes, padding=8)
        editor = ttk.Frame(panes, padding=8)
        sidebar.columnconfigure(0, weight=1)
        sidebar.rowconfigure(0, weight=1)
        editor.columnconfigure(0, weight=1)
        editor.rowconfigure(0, weight=1)
        panes.add(sidebar, weight=1)
        panes.add(editor, weight=3)

        self.listbox = tk.Listbox(
            sidebar,
            background=UI_PANEL,
            foreground=UI_FG,
            selectbackground=UI_BORDER,
            selectforeground=UI_ACCENT,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        self.listbox.grid(row=0, column=0, sticky="nsew")
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        self.editor_container = ttk.Frame(editor)
        self.editor_container.grid(row=0, column=0, sticky="nsew")
        self.editor_container.columnconfigure(0, weight=1)

    def refresh_from_disk(self) -> None:
        selected_key = self.current_key
        self.data = load_json_file(MODULES_PATH)
        self.refresh_list()
        if selected_key in self.data:
            self.current_key = selected_key
            self.listbox.selection_clear(0, tk.END)
            self.listbox.selection_set(sorted(self.data.keys()).index(selected_key))
            self.rebuild_editor()
            return
        self.current_key = None
        self.clear_editor()

    def refresh_list(self) -> None:
        self.listbox.delete(0, tk.END)
        for name in sorted(self.data.keys()):
            self.listbox.insert(tk.END, name)

    def on_select(self, _event=None) -> None:
        index = listbox_index_from_selection(self.listbox)
        if index is None:
            return
        self.current_key = self.listbox.get(index)
        self.rebuild_editor()

    def clear_editor(self) -> None:
        for child in self.editor_container.winfo_children():
            child.destroy()
        self.field_bindings = []

    def rebuild_editor(self) -> None:
        self.clear_editor()
        if self.current_key is None:
            return
        item_data = copy.deepcopy(self.data[self.current_key])
        header = ttk.LabelFrame(self.editor_container, text="Module", padding=8)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="name").grid(row=0, column=0, sticky="w")
        self.name_var = tk.StringVar(value=self.current_key)
        ttk.Entry(header, textvariable=self.name_var).grid(row=0, column=1, sticky="ew", padx=(8, 0))

        fields_frame = ttk.LabelFrame(self.editor_container, text="Properties", padding=8)
        fields_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        fields_frame.columnconfigure(1, weight=1)
        self.build_field_rows(fields_frame, item_data, MODULE_LIBRARY_FIELD_ORDER, ())

    def build_field_rows(self, parent, data: dict, preferred_order: list[str], prefix: tuple[str, ...]) -> int:
        row_index = 0
        for key in ordered_keys(data, preferred_order):
            value = data[key]
            if type(value) is dict:
                group = ttk.LabelFrame(parent, text=key, padding=6)
                group.grid(row=row_index, column=0, columnspan=2, sticky="ew", pady=(4, 0))
                group.columnconfigure(1, weight=1)
                self.build_field_rows(group, value, [], prefix + (key,))
                row_index += 1
                continue
            ttk.Label(parent, text=field_label(key)).grid(row=row_index, column=0, sticky="w")
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                ttk.Checkbutton(parent, variable=variable).grid(row=row_index, column=1, sticky="w", padx=(8, 0), pady=2)
            else:
                variable = tk.StringVar(value=format_field_value(key, value))
                entry = ttk.Entry(parent, textvariable=variable)
                if type(value) is int or type(value) is float:
                    value_type = "int" if type(value) is int else "float"
                    entry.configure(
                        validate="key",
                        validatecommand=(self.register(text_is_valid_for_existing_value), value_type, "%P"),
                    )
                entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.field_bindings.append((prefix + (key,), variable, value))
            row_index += 1
        return row_index

    def save_current(self) -> None:
        if self.current_key is None:
            return
        updated = copy.deepcopy(self.data[self.current_key])
        for path, variable, existing_value in self.field_bindings:
            set_nested_value(updated, path, parse_field_value(path[-1], str(variable.get()), existing_value))
        new_name = self.name_var.get()
        if new_name != self.current_key:
            del self.data[self.current_key]
        self.data[new_name] = updated
        self.current_key = new_name
        save_json_file(MODULES_PATH, self.data)
        self.refresh_from_disk()
        if self.on_save is not None:
            self.on_save()

    def new_module(self) -> None:
        new_name = "new-module-1"
        while new_name in self.data:
            suffix = int(new_name.rsplit("-", 1)[1]) + 1
            new_name = f"new-module-{suffix}"
        self.data[new_name] = copy.deepcopy(self.module_templates["body"])
        self.current_key = new_name
        self.refresh_list()
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(sorted(self.data.keys()).index(new_name))
        self.rebuild_editor()

    def delete_current(self) -> None:
        if self.current_key is None:
            return
        if not messagebox.askyesno("Delete Module", f"Delete module '{self.current_key}'?", parent=self.winfo_toplevel()):
            return
        del self.data[self.current_key]
        self.current_key = None
        save_json_file(MODULES_PATH, self.data)
        self.refresh_from_disk()
        if self.on_save is not None:
            self.on_save()


class PartEditorFrame(ttk.Frame):
    def __init__(self, master, on_save=None) -> None:
        super().__init__(master, padding=8)
        self.on_save = on_save
        self.data = {}
        self.current_category = None
        self.current_name = None
        self.field_bindings = []
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.build_ui()
        self.refresh_from_disk()

    def build_ui(self) -> None:
        header = ttk.Frame(self, padding=0)
        header.grid(row=0, column=0, sticky="ew")
        ttk.Button(header, text="Save", command=self.save_current).pack(side="left")
        ttk.Button(header, text="New", command=self.new_part).pack(side="left", padx=(8, 0))
        ttk.Button(header, text="Delete", command=self.delete_current).pack(side="left", padx=(8, 0))

        panes = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        panes.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

        sidebar = ttk.Frame(panes, padding=8)
        editor = ttk.Frame(panes, padding=8)
        sidebar.columnconfigure(0, weight=1)
        sidebar.rowconfigure(0, weight=1)
        editor.columnconfigure(0, weight=1)
        editor.rowconfigure(0, weight=1)
        panes.add(sidebar, weight=1)
        panes.add(editor, weight=3)

        self.listbox = tk.Listbox(
            sidebar,
            background=UI_PANEL,
            foreground=UI_FG,
            selectbackground=UI_BORDER,
            selectforeground=UI_ACCENT,
            highlightthickness=1,
            highlightbackground=UI_BORDER,
            highlightcolor=UI_ACCENT,
            relief="flat",
        )
        self.listbox.grid(row=0, column=0, sticky="nsew")
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        self.editor_container = ttk.Frame(editor)
        self.editor_container.grid(row=0, column=0, sticky="nsew")
        self.editor_container.columnconfigure(0, weight=1)

    def refresh_from_disk(self) -> None:
        selected = (self.current_category, self.current_name)
        self.data = load_json_file(PARTS_PATH)
        self.refresh_list()
        if selected in self.list_items():
            self.current_category, self.current_name = selected
            self.listbox.selection_clear(0, tk.END)
            self.listbox.selection_set(self.list_items().index(selected))
            self.rebuild_editor()
            return
        self.current_category = None
        self.current_name = None
        self.clear_editor()

    def list_items(self) -> list[tuple[str, str]]:
        items = []
        for category in sorted(self.data.keys()):
            for name in sorted(self.data[category].keys()):
                items.append((category, name))
        return items

    def refresh_list(self) -> None:
        self.listbox.delete(0, tk.END)
        for category, name in self.list_items():
            self.listbox.insert(tk.END, f"{category} / {name}")

    def on_select(self, _event=None) -> None:
        index = listbox_index_from_selection(self.listbox)
        if index is None:
            return
        self.current_category, self.current_name = self.list_items()[index]
        self.rebuild_editor()

    def clear_editor(self) -> None:
        for child in self.editor_container.winfo_children():
            child.destroy()
        self.field_bindings = []

    def rebuild_editor(self) -> None:
        self.clear_editor()
        if self.current_category is None or self.current_name is None:
            return
        item_data = copy.deepcopy(self.data[self.current_category][self.current_name])
        header = ttk.LabelFrame(self.editor_container, text="Part", padding=8)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="category").grid(row=0, column=0, sticky="w")
        self.category_var = tk.StringVar(value=self.current_category)
        ttk.Combobox(header, values=sorted(self.data.keys()), textvariable=self.category_var, state="readonly").grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
        ttk.Label(header, text="name").grid(row=1, column=0, sticky="w")
        self.name_var = tk.StringVar(value=self.current_name)
        ttk.Entry(header, textvariable=self.name_var).grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)

        fields_frame = ttk.LabelFrame(self.editor_container, text="Properties", padding=8)
        fields_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        fields_frame.columnconfigure(1, weight=1)
        self.build_field_rows(fields_frame, item_data, PART_LIBRARY_FIELD_ORDER, ())

    def build_field_rows(self, parent, data: dict, preferred_order: list[str], prefix: tuple[str, ...]) -> int:
        row_index = 0
        for key in ordered_keys(data, preferred_order):
            value = data[key]
            if type(value) is dict:
                group = ttk.LabelFrame(parent, text=key, padding=6)
                group.grid(row=row_index, column=0, columnspan=2, sticky="ew", pady=(4, 0))
                group.columnconfigure(1, weight=1)
                self.build_field_rows(group, value, [], prefix + (key,))
                row_index += 1
                continue
            ttk.Label(parent, text=field_label(key)).grid(row=row_index, column=0, sticky="w")
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                ttk.Checkbutton(parent, variable=variable).grid(row=row_index, column=1, sticky="w", padx=(8, 0), pady=2)
            else:
                variable = tk.StringVar(value=format_field_value(key, value))
                entry = ttk.Entry(parent, textvariable=variable)
                if type(value) is int or type(value) is float:
                    value_type = "int" if type(value) is int else "float"
                    entry.configure(
                        validate="key",
                        validatecommand=(self.register(text_is_valid_for_existing_value), value_type, "%P"),
                    )
                entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.field_bindings.append((prefix + (key,), variable, value))
            row_index += 1
        return row_index

    def save_current(self) -> None:
        if self.current_category is None or self.current_name is None:
            return
        updated = copy.deepcopy(self.data[self.current_category][self.current_name])
        for path, variable, existing_value in self.field_bindings:
            set_nested_value(updated, path, parse_field_value(path[-1], str(variable.get()), existing_value))
        new_category = self.category_var.get()
        new_name = self.name_var.get()
        del self.data[self.current_category][self.current_name]
        self.data.setdefault(new_category, {})
        self.data[new_category][new_name] = updated
        self.current_category = new_category
        self.current_name = new_name
        save_json_file(PARTS_PATH, self.data)
        self.refresh_from_disk()
        if self.on_save is not None:
            self.on_save()

    def new_part(self) -> None:
        category = self.current_category if self.current_category is not None else sorted(self.data.keys())[0]
        new_name = "new-part-1"
        while new_name in self.data[category]:
            suffix = int(new_name.rsplit("-", 1)[1]) + 1
            new_name = f"new-part-{suffix}"
        self.data[category][new_name] = copy.deepcopy(NEW_PART_LIBRARY_DEFAULTS[category])
        self.current_category = category
        self.current_name = new_name
        self.refresh_list()
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(self.list_items().index((category, new_name)))
        self.rebuild_editor()

    def delete_current(self) -> None:
        if self.current_category is None or self.current_name is None:
            return
        if not messagebox.askyesno("Delete Part", f"Delete part '{self.current_name}'?", parent=self.winfo_toplevel()):
            return
        del self.data[self.current_category][self.current_name]
        self.current_category = None
        self.current_name = None
        save_json_file(PARTS_PATH, self.data)
        self.refresh_from_disk()
        if self.on_save is not None:
            self.on_save()


def open_module_editor(master, on_save=None):
    window = tk.Toplevel(master)
    window.title("Module Editor")
    window.geometry("1200x760")
    window.configure(background=UI_BG)
    window.columnconfigure(0, weight=1)
    window.rowconfigure(0, weight=1)
    frame = ModuleEditorFrame(window, on_save=on_save)
    frame.grid(row=0, column=0, sticky="nsew")
    return frame


def open_part_editor(master, on_save=None):
    window = tk.Toplevel(master)
    window.title("Part Editor")
    window.geometry("1200x760")
    window.configure(background=UI_BG)
    window.columnconfigure(0, weight=1)
    window.rowconfigure(0, weight=1)
    frame = PartEditorFrame(window, on_save=on_save)
    frame.grid(row=0, column=0, sticky="nsew")
    return frame
