"""
Usage:
from libs.views.configuration_view import ConfigurationMainView

Configuration view: summary, drawing and the plane-first editors (Airframe, Surfaces, Propulsion, Volumes).
Editors write straight into app.definition (the airframe/2 dict) and call app.recompute(). Lengths are shown in
mm and masses in g; the definition stays in SI. In the side view, volumes and wing sets can be dragged along x.
"""

from __future__ import annotations

import copy
import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from libs.airframe import AXES, FIN_POSITIONS, JOINT_TYPES, LAYOUTS, MOUNT_TARGETS, PROP_SPIN_PATTERNS, SURFACE_ROOTS, THRUST_DIRECTIONS, VOLUME_SHAPES
from libs.graphics import draw_canvas_view
from libs.pointed_shapes import POINTED_SHAPE_PARAM_KEYS

# AeroSandbox's matplotlib draw needs a pyplot figure. With pyplot on Agg those figures get no Tk window of their
# own (a TkAgg pyplot window keeps Tk's mainloop alive after the app closes); the view embeds them itself.
plt.switch_backend("Agg")

MM_KEYS = {
    "length_m", "width_m", "height_m", "wall_radial_m", "insertion_depth_m", "diameter_m", "radial_position_m",
    "from_x_m", "to_x_m", "x_root_le_m", "span_m", "chord_root_m", "chord_tip_m", "span_position_m",
    "x_offset_m", "x_m", "y_m", "z_m", "flat_length_m", "curve_length_m", "nose_radius", "size_m", "printer_bed_m", "cuts_m",
}
GRAM_KEYS = {"mass_kg", "coupler_mass_kg"}
UNIT_SUFFIXES = ("_kg_m3", "_m_s", "_deg", "_kg", "_m")
CHOICES = {
    "layout": list(LAYOUTS),
    "fin_position": list(FIN_POSITIONS),
    "root": list(SURFACE_ROOTS),
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
    "cylinder": {"diameter_m": 0.02, "length_m": 0.03, "axis": "x"},
    "bore": {"length_m": 0.05},
    "point": {},
}
NEW_SURFACE = {
    "name": "wing", "x_root_le_m": 0.2, "layout": "H", "fin_position": "top", "dihedral_deg": 0.0, "root_offset": 0.0,
    "root": "body", "span_m": 0.12, "chord_root_m": 0.1, "chord_tip_m": 0.07, "sweep_le_deg": 10.0, "incidence_deg": 0.0, "airfoil": "naca2412",
}
NEW_POD = {"diameter_m": 0.035, "flat_length_m": 0.02, "curve_length_m": 0.05, "profile": "elliptical"}
DRAG_KEYS = {"volume": ("volumes", "x_m"), "surface": ("surfaces", "x_root_le_m")}  # cuts drag the sections.cuts_m entry itself
MIN_SECTION_LENGTH_M = 0.005


def field_label(key: str) -> str:
    base = key
    for suffix in UNIT_SUFFIXES:
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    unit = "mm" if key in MM_KEYS else "g" if key in GRAM_KEYS else "deg" if key.endswith("_deg") else "m/s" if key.endswith("_m_s") else "kg/m^3" if key.endswith("_kg_m3") else "m" if key.endswith("_m") else ""
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


class ConfigurationMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.view_zoom_scale = 1.0
        self.view_pan_x = 0.0
        self.view_pan_y = 0.0
        self.view_drag_last_xy = None
        self.drag_target = None
        self.view_metrics = None
        self.view_scroll_x = None
        self.view_scroll_y = None
        self.editor_frames: dict[str, ttk.Frame] = {}
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.configuration_view = self.frame

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
        summary_tab = ttk.Frame(left_notebook, padding=4)
        summary_tab.columnconfigure(0, weight=1)
        summary_tab.rowconfigure(0, weight=1)
        left_notebook.add(summary_tab, text="Summary")

        summary_frame = ttk.LabelFrame(summary_tab, text="Summary", padding=4)
        summary_frame.grid(row=0, column=0, sticky="nsew")
        summary_frame.columnconfigure(0, weight=1)
        summary_frame.rowconfigure(0, weight=1)
        app.summary_text = tk.Text(
            summary_frame,
            wrap="none",
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
        summary_scroll_y = ttk.Scrollbar(summary_frame, orient="vertical", command=app.summary_text.yview)
        summary_scroll_x = ttk.Scrollbar(summary_frame, orient="horizontal", command=app.summary_text.xview)
        app.summary_text.configure(yscrollcommand=summary_scroll_y.set, xscrollcommand=summary_scroll_x.set)
        app.summary_text.grid(row=0, column=0, sticky="nsew")
        summary_scroll_y.grid(row=0, column=1, sticky="ns")
        summary_scroll_x.grid(row=1, column=0, sticky="ew")

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

        notebook = ttk.Notebook(right_frame)
        notebook.grid(row=0, column=0, rowspan=2, sticky="nsew")
        for tab_name in ("Airframe", "Surfaces", "Propulsion", "Volumes"):
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

    def draw_active_view(self) -> None:
        app = self.app
        if app.resolved is None or app.view_canvas is None:
            return
        if app.view_mode_var.get() == "Front":
            app.roll_frame.grid_remove()
        else:
            app.roll_frame.grid()
        draw_args = (app.view_canvas, app.resolved, app.aero, app.view_mode_var.get(), app.roll_view_var.get(), app.ui_draw, app.ui_bg)
        self.view_metrics = draw_canvas_view(*draw_args, self.view_zoom_scale, self.view_pan_x, self.view_pan_y)
        if self.clamp_view_pan():
            self.view_metrics = draw_canvas_view(*draw_args, self.view_zoom_scale, self.view_pan_x, self.view_pan_y)
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
            self.rebuild_editors()
            self.app.recompute()

    def changed(self, rebuild: bool = False) -> None:
        if rebuild:
            self.rebuild_editors()
        self.app.recompute()

    def add_entry(self, frame, row: int, data, key, optional: bool = False, label: str | None = None, unit_key: str | None = None, blank_hint: str | None = None) -> None:
        """Entry for one value of a dict (by key) or list (by index); commits on Return/focus-out only when the text changed.
        unit_key names the field for units and label when key is a list index."""
        unit_key = unit_key or key
        ttk.Label(frame, text=label or field_label(unit_key)).grid(row=row, column=0, sticky="w")
        present = type(data) is list or key in data
        initial = format_value(unit_key, data[key] if present else None)
        variable = tk.StringVar(value=initial)
        entry = ttk.Entry(frame, textvariable=variable)
        entry.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=1)
        if optional:
            hint = blank_hint or ("blank = computed" if unit_key in GRAM_KEYS else "blank = none")
            ttk.Label(frame, text=hint).grid(row=row, column=2, sticky="w", padx=(6, 0))
        state = {"text": initial}

        def commit(_event=None) -> str:
            text = variable.get()
            if text.strip() == state["text"].strip():
                return "break"
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
                    self.app.recompute_status_var.set(f"Not a valid value for {field_label(unit_key)}: {text!r}")
                    return "break"
            state["text"] = text
            self.changed()
            return "break"

        for sequence in ("<Return>", "<KP_Enter>", "<FocusOut>"):
            entry.bind(sequence, commit)

    def add_vector_entries(self, frame, row: int, data: dict, key: str, axis_labels: tuple[str, ...]) -> int:
        for index, axis_label in enumerate(axis_labels):
            self.add_entry(frame, row + index, data[key], index, label=f"{field_label(key)} {axis_label}", unit_key=key)
        return row + len(axis_labels)

    def add_choice(self, frame, row: int, data: dict, key: str, options: list[str], on_change=None) -> None:
        ttk.Label(frame, text=field_label(key)).grid(row=row, column=0, sticky="w")
        variable = tk.StringVar(value=str(data.get(key, "")))
        box = ttk.Combobox(frame, values=options, textvariable=variable, state="readonly")
        box.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=1)

        def select(_event=None) -> None:
            data[key] = variable.get()
            if on_change is not None:
                on_change()
            self.changed(rebuild=True)

        box.bind("<<ComboboxSelected>>", select)

    def add_fields(self, frame, data: dict, start_row: int = 0, skip: set[str] = frozenset()) -> int:
        """Generic rows for every field of a definition dict, in stored order."""
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
                self.add_choice(frame, row, data, key, CHOICES[key])
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

    def rebuild_editors(self) -> None:
        for frame in self.editor_frames.values():
            for child in frame.winfo_children():
                child.destroy()
        self.build_airframe_tab(self.editor_frames["Airframe"])
        self.build_surfaces_tab(self.editor_frames["Surfaces"])
        self.build_propulsion_tab(self.editor_frames["Propulsion"])
        self.build_volumes_tab(self.editor_frames["Volumes"])

    def build_airframe_tab(self, parent) -> None:
        definition = self.app.definition
        general = ttk.LabelFrame(parent, text="Airframe", padding=6)
        general.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        general.columnconfigure(1, weight=1)
        self.add_entry(general, 0, definition, "name")
        self.add_entry(general, 1, definition, "notes", optional=True)
        for row, (key, title) in enumerate((("operating_point", "Operating point (aero and stability)"), ("print", "Print")), start=1):
            group = ttk.LabelFrame(parent, text=title, padding=6)
            group.grid(row=row, column=0, sticky="ew", padx=4, pady=4)
            group.columnconfigure(1, weight=1)
            if key == "print":
                print_settings = definition["print"]
                self.add_choice(group, 0, print_settings, "material", sorted(self.app.parts_data["material"]))
                next_row = self.add_vector_entries(group, 1, print_settings, "printer_bed_m", ("x", "y", "z"))
                self.add_entry(group, next_row, print_settings, "surface_bulk_density_kg_m3", optional=True)
            else:
                self.add_fields(group, definition[key])
        fuselage = definition["fuselage"]
        group = ttk.LabelFrame(parent, text="Fuselage", padding=6)
        group.grid(row=3, column=0, sticky="ew", padx=4, pady=4)
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
        self.build_sections_group(parent, 4)

    def build_sections_group(self, parent, grid_row: int) -> None:
        definition = self.app.definition
        sections = definition["sections"]
        cuts = sections["cuts_m"]
        masses = sections.setdefault("mass_kg", [None] * (len(cuts) + 1))
        group = ttk.LabelFrame(parent, text="Sections (cut lines; drag them in the side view)", padding=6)
        group.grid(row=grid_row, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        ttk.Button(group, text="Add cut", command=self.add_cut).grid(row=0, column=0, sticky="w", pady=(0, 4))
        bounds = [0.0, *cuts, definition["fuselage"]["length_m"]]
        row = 1
        for index in range(len(masses)):
            self.add_entry(group, row, masses, index, optional=True, unit_key="mass_kg",
                           label=f"S{index + 1} {bounds[index] * 1000:.0f}-{bounds[index + 1] * 1000:.0f} mm, weighed (g)")
            row += 1
            if index < len(cuts):
                self.add_entry(group, row, cuts, index, unit_key="cuts_m", label=f"  cut {index + 1} (mm)")
                ttk.Button(group, text="Delete cut", command=lambda i=index: self.delete_cut(i)).grid(row=row, column=2, sticky="e", padx=(6, 0))
                row += 1

    def add_cut(self) -> None:
        sections = self.app.definition["sections"]
        cuts = sections["cuts_m"]
        bounds = [0.0, *cuts, self.app.definition["fuselage"]["length_m"]]
        longest = max(range(len(bounds) - 1), key=lambda i: bounds[i + 1] - bounds[i])
        cuts.insert(longest, round((bounds[longest] + bounds[longest + 1]) / 2.0, 4))
        masses = sections["mass_kg"]
        masses[longest:longest + 1] = [None, None]  # a split section's weighed mass no longer applies
        self.changed(rebuild=True)

    def delete_cut(self, index: int) -> None:
        sections = self.app.definition["sections"]
        sections["cuts_m"].pop(index)
        masses = sections["mass_kg"]
        merged = None if masses[index] is None or masses[index + 1] is None else masses[index] + masses[index + 1]
        masses[index:index + 2] = [merged]
        self.changed(rebuild=True)

    def reset_profile_params(self, end: dict) -> None:
        end["params"] = {key: end["params"].get(key, PROFILE_PARAM_DEFAULTS[key]) for key in POINTED_SHAPE_PARAM_KEYS[end["profile"]]}

    def build_surfaces_tab(self, parent) -> None:
        surfaces = self.app.definition["surfaces"]
        ttk.Button(parent, text="Add surface set", command=self.add_surface).grid(row=0, column=0, sticky="w", padx=4, pady=4)
        for index, surface in enumerate(surfaces):
            frame = self.card(parent, index + 1, surface["name"], lambda i=index: self.delete_item("surfaces", i))
            skip = set() if surface["layout"] in ("1", "3") else {"fin_position"}
            row = self.add_fields(frame, surface, start_row=1, skip=skip | {"mass_kg"})
            self.add_entry(frame, row, surface, "mass_kg", optional=True)

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
        propulsion = definition["propulsion"]
        motor = propulsion["motor"]
        group = ttk.LabelFrame(parent, text="Motor and prop (bench data)", padding=6)
        group.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        group.columnconfigure(1, weight=1)
        battery_volumes = [volume["name"] for volume in definition["volumes"] if str(volume.get("part", "")).startswith("battery:")]
        self.add_choice(group, 0, propulsion, "battery_volume", battery_volumes)
        cells = None
        battery = next((volume for volume in definition["volumes"] if volume["name"] == propulsion["battery_volume"]), None)
        if battery is not None and str(battery.get("part", "")).startswith("battery:"):
            cells = int(self.app.parts_data["battery"][battery["part"].partition(":")[2]]["s"])
        combos = [combo for combo in self.app.motor_options if cells is None or combo["batt_s"] == cells]
        motor_labels = sorted({f"{combo['brand']} {combo['motor']} {combo['kv']}KV" for combo in combos})
        ttk.Label(group, text="motor").grid(row=1, column=0, sticky="w")
        motor_var = tk.StringVar(value=f"{motor['brand']} {motor['model']} {motor['kv']}KV")
        motor_box = ttk.Combobox(group, values=motor_labels, textvariable=motor_var, state="readonly")
        motor_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=1)
        props = sorted({combo["prop"] for combo in combos if (combo["brand"], combo["motor"], combo["kv"]) == (motor["brand"], motor["model"], motor["kv"])})
        self.add_choice(group, 2, motor, "prop", props)
        self.add_choice(group, 3, propulsion, "prop_spin", list(PROP_SPIN_PATTERNS))  # per-motor lists are edited in the file

        def select_motor(_event=None) -> None:
            chosen = next(combo for combo in combos if f"{combo['brand']} {combo['motor']} {combo['kv']}KV" == motor_var.get())
            motor.update(brand=chosen["brand"], model=chosen["motor"], kv=chosen["kv"])
            motor_props = sorted(combo["prop"] for combo in combos if (combo["brand"], combo["motor"], combo["kv"]) == (chosen["brand"], chosen["motor"], chosen["kv"]))
            if motor["prop"] not in motor_props:
                motor["prop"] = motor_props[0]
            self.changed(rebuild=True)

        motor_box.bind("<<ComboboxSelected>>", select_motor)
        ttk.Button(parent, text="Add motor mount", command=self.add_mount).grid(row=1, column=0, sticky="w", padx=4, pady=4)
        surface_names = [surface["name"] for surface in definition["surfaces"]]
        for index, mount in enumerate(propulsion["mounts"]):
            frame = self.card(parent, index + 2, f"mount {index + 1}", lambda i=index: self.delete_mount(i))
            self.add_choice(frame, 1, mount, "on", CHOICES["on"], on_change=lambda m=mount: self.normalize_mount(m))
            row = 2
            if mount["on"] == "surface":
                self.add_choice(frame, row, mount, "surface", surface_names)
                self.add_entry(frame, row + 1, mount, "span_position_m")
                self.add_entry(frame, row + 2, mount, "x_offset_m")
                row += 3
            self.add_choice(frame, row, mount, "direction", CHOICES["direction"])
            pod_var = tk.BooleanVar(value="pod" in mount)
            ttk.Checkbutton(frame, text="pod", variable=pod_var, command=lambda m=mount, v=pod_var: self.toggle_pod(m, v)).grid(row=row + 1, column=0, sticky="w")
            if "pod" in mount:
                pod = ttk.LabelFrame(frame, text="pod", padding=4)
                pod.grid(row=row + 2, column=0, columnspan=3, sticky="ew")
                pod.columnconfigure(1, weight=1)
                self.add_fields(pod, mount["pod"])

    def normalize_mount(self, mount: dict) -> None:
        surfaces = self.app.definition["surfaces"]
        if mount["on"] == "surface":
            mount.setdefault("surface", surfaces[0]["name"] if surfaces else "")
            mount.setdefault("span_position_m", 0.0)
            mount.setdefault("x_offset_m", 0.0)
        else:
            for key in ("surface", "span_position_m", "x_offset_m"):
                mount.pop(key, None)
            mount["direction"] = "tractor" if mount["on"] == "nose" else "pusher"

    def toggle_pod(self, mount: dict, variable: tk.BooleanVar) -> None:
        if variable.get():
            mount["pod"] = copy.deepcopy(NEW_POD)
        else:
            mount.pop("pod", None)
        self.changed(rebuild=True)

    def add_mount(self) -> None:
        mount = {"on": "nose", "direction": "tractor"}
        self.app.definition["propulsion"]["mounts"].append(mount)
        self.changed(rebuild=True)

    def delete_mount(self, index: int) -> None:
        self.app.definition["propulsion"]["mounts"].pop(index)
        self.changed(rebuild=True)

    def build_volumes_tab(self, parent) -> None:
        volumes = self.app.definition["volumes"]
        ttk.Label(parent, text="Mass: grams, or a range like 10-30. Drag volumes along the body in the side view.").grid(row=0, column=0, sticky="w", padx=4)
        ttk.Button(parent, text="Add volume", command=self.add_volume).grid(row=1, column=0, sticky="w", padx=4, pady=4)
        part_options = [""] + [f"{category}:{name}" for category in ("battery", "esc") for name in sorted(self.app.parts_data[category])]
        for index, volume in enumerate(volumes):
            frame = self.card(parent, index + 2, volume["name"], lambda i=index: self.delete_item("volumes", i))
            self.add_entry(frame, 1, volume, "name")
            self.add_choice(frame, 2, volume, "shape", list(VOLUME_SHAPES), on_change=lambda v=volume: self.reset_volume_shape(v))
            self.add_choice(frame, 3, volume, "part", part_options, on_change=lambda v=volume: self.normalize_volume_part(v))
            row = 4
            for key in ("x_m", "y_m", "z_m"):
                self.add_entry(frame, row, volume, key, optional=key != "x_m")
                row += 1
            if volume["shape"] == "box":
                if "size_m" in volume:
                    row = self.add_vector_entries(frame, row, volume, "size_m", ("x", "y", "z"))
                else:
                    ttk.Label(frame, text="size from part").grid(row=row, column=0, sticky="w")
                    row += 1
            for key in VOLUME_SHAPE_FIELDS[volume["shape"]]:
                if key != "size_m":
                    if key == "axis":
                        self.add_choice(frame, row, volume, key, CHOICES["axis"])
                    else:
                        self.add_entry(frame, row, volume, key)
                    row += 1
            self.add_entry(frame, row, volume, "mass_kg", optional="part" in volume, blank_hint="blank = from part")

    def normalize_volume_part(self, volume: dict) -> None:
        if volume["part"] != "":
            return
        volume.pop("part")
        if volume["shape"] == "box" and "size_m" not in volume:
            volume["size_m"] = copy.deepcopy(VOLUME_SHAPE_FIELDS["box"]["size_m"])
        volume.setdefault("mass_kg", 0.02)

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
        volume = {"name": name, "shape": "box", "x_m": round(self.app.definition["fuselage"]["length_m"] / 2.0, 4), "mass_kg": 0.02}
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
