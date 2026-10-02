"""
Usage:
from libs.views.aero_analysis_view import AeroAnalysisMainView

Aero Analysis view: NeuralFoil 2D comparison of airfoils at a chosen Reynolds/Mach operating point. Starts with the
current airframe's airfoils selected; DAT files (Selig or Lednicer) can be imported.
"""

from __future__ import annotations

import re
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from libs.airfoil_analysis import (
    DEFAULT_AIRFOIL_OPERATING_POINT,
    DEFAULT_NEURALFOIL_SETTINGS,
    DEFAULT_SELECTED_AIRFOIL_NAMES,
    SUMMARY_CONFIDENCE_MIN,
    DEFAULT_SWEEP_ALPHA_MAX_DEG,
    DEFAULT_SWEEP_ALPHA_MIN_DEG,
    NEURALFOIL_MODEL_SIZES,
    airfoil_from_record,
    airfoil_record_from_path,
    analyze_airfoils,
    asb_airfoil_record,
    compute_flow_conditions,
    load_airfoil_library,
)


class AeroAnalysisMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.airfoil_library = load_airfoil_library()
        self.selected_airfoil_names = []
        self.current_results = None
        self.plot_figure = None
        self.plot_canvas = None
        self.alpha_scale_var = tk.DoubleVar(value=DEFAULT_AIRFOIL_OPERATING_POINT["alpha_deg"])
        self.alpha_entry_var = tk.StringVar(value=f"{DEFAULT_AIRFOIL_OPERATING_POINT['alpha_deg']:g}")
        self.airspeed_var = tk.StringVar(value=f"{DEFAULT_AIRFOIL_OPERATING_POINT['airspeed_m_s']:g}")
        self.altitude_var = tk.StringVar(value=f"{DEFAULT_AIRFOIL_OPERATING_POINT['altitude_m']:g}")
        self.temperature_var = tk.StringVar(value=f"{DEFAULT_AIRFOIL_OPERATING_POINT['temperature_c']:g}")
        self.chord_var = tk.StringVar(value=f"{DEFAULT_AIRFOIL_OPERATING_POINT['chord_mm']:g}")
        self.n_crit_var = tk.StringVar(value=f"{DEFAULT_NEURALFOIL_SETTINGS['n_crit']:g}")
        self.xtr_upper_var = tk.StringVar(value=f"{DEFAULT_NEURALFOIL_SETTINGS['xtr_upper']:g}")
        self.xtr_lower_var = tk.StringVar(value=f"{DEFAULT_NEURALFOIL_SETTINGS['xtr_lower']:g}")
        self.model_size_var = tk.StringVar(value=DEFAULT_NEURALFOIL_SETTINGS["model_size"])
        self.include_360_var = tk.BooleanVar(value=DEFAULT_NEURALFOIL_SETTINGS["include_360_deg_effects"])
        self.flow_text_var = tk.StringVar(value="")
        self.last_valid_operating_point = dict(DEFAULT_AIRFOIL_OPERATING_POINT)
        self.last_valid_neuralfoil_settings = dict(DEFAULT_NEURALFOIL_SETTINGS)
        self.summary_text = None
        self.available_listbox = None
        self.selected_listbox = None

        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.airfoil_view = self.frame

        pane_window = ttk.Panedwindow(self.frame, orient=tk.HORIZONTAL)
        pane_window.grid(row=0, column=0, sticky="nsew")
        left_frame = ttk.Frame(pane_window, padding=8)
        right_frame = ttk.Frame(pane_window, padding=8)
        pane_window.add(left_frame, weight=1)
        pane_window.add(right_frame, weight=3)

        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=1)
        left_frame.rowconfigure(2, weight=1)
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(0, weight=1)

        self.build_parameter_panel(left_frame)
        self.build_selection_panel(left_frame)
        self.build_summary_panel(left_frame)
        self.build_plot_panel(right_frame)
        self.seed_selected_airfoils()
        self.refresh_airfoil_listboxes()
        self.recompute_airfoils()

    def build_parameter_panel(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Parameters", padding=6)
        frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)
        ttk.Button(frame, text="Recompute (F5)", command=self.recompute_airfoils).grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.flow_text_var).grid(row=0, column=1, columnspan=2, sticky="e", padx=(8, 0))

        ttk.Label(frame, text="AoA operating point (deg)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        alpha_scale = tk.Scale(
            frame,
            from_=DEFAULT_SWEEP_ALPHA_MIN_DEG,
            to=DEFAULT_SWEEP_ALPHA_MAX_DEG,
            orient="horizontal",
            resolution=0.5,
            variable=self.alpha_scale_var,
            showvalue=False,
            command=self.on_alpha_scale_changed,
            background=self.app.ui_bg,
            foreground=self.app.ui_fg,
            troughcolor=self.app.ui_panel,
            activebackground=self.app.ui_border,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            bd=0,
            relief="flat",
        )
        alpha_scale.grid(row=1, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))
        alpha_entry = ttk.Entry(frame, textvariable=self.alpha_entry_var, width=10)
        alpha_entry.grid(row=1, column=2, sticky="e", pady=(8, 0))
        self.bind_numeric_entry(alpha_entry, self.on_alpha_entry_commit)

        self.add_parameter_entry(frame, 2, "Airspeed (m/s)", self.airspeed_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 3, "Altitude (m)", self.altitude_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 4, "Temperature (C)", self.temperature_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 5, "Reference chord (mm)", self.chord_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 6, "N-Crit", self.n_crit_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 7, "Xtr Upper", self.xtr_upper_var, self.recompute_airfoils)
        self.add_parameter_entry(frame, 8, "Xtr Lower", self.xtr_lower_var, self.recompute_airfoils)

        ttk.Label(frame, text="Model size").grid(row=9, column=0, sticky="w")
        model_box = ttk.Combobox(frame, values=NEURALFOIL_MODEL_SIZES, textvariable=self.model_size_var, state="readonly")
        model_box.grid(row=9, column=1, columnspan=2, sticky="ew", padx=(8, 0), pady=2)
        model_box.bind("<<ComboboxSelected>>", lambda _event: self.recompute_airfoils())

        include_360_box = ttk.Checkbutton(
            frame,
            text="Include 360 deg effects",
            variable=self.include_360_var,
            command=self.recompute_airfoils,
        )
        include_360_box.grid(row=10, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def add_parameter_entry(self, frame, row: int, label: str, variable: tk.StringVar, callback) -> None:
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w")
        entry = ttk.Entry(frame, textvariable=variable)
        entry.grid(row=row, column=1, columnspan=2, sticky="ew", padx=(8, 0), pady=2)
        self.bind_numeric_entry(entry, callback)

    def build_selection_panel(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Airfoil Selection", padding=6)
        frame.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.rowconfigure(1, weight=1)

        ttk.Label(frame, text="Available").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, text="Selected").grid(row=0, column=2, sticky="w")

        self.available_listbox = tk.Listbox(
            frame,
            selectmode="extended",
            exportselection=False,
            background=self.app.ui_panel,
            foreground=self.app.ui_fg,
            selectbackground=self.app.ui_border,
            selectforeground=self.app.ui_accent,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            relief="flat",
        )
        self.available_listbox.grid(row=1, column=0, sticky="nsew")
        self.available_listbox.bind("<Double-Button-1>", lambda _event: self.add_selected_airfoils())

        button_bar = ttk.Frame(frame)
        button_bar.grid(row=1, column=1, sticky="ns", padx=8)
        ttk.Button(button_bar, text="Add ->", command=self.add_selected_airfoils).pack(fill="x")
        ttk.Button(button_bar, text="<- Remove", command=self.remove_selected_airfoils).pack(fill="x", pady=(6, 0))
        ttk.Button(button_bar, text="Import DAT", command=self.import_airfoil_dat).pack(fill="x", pady=(18, 0))
        ttk.Button(button_bar, text="Clear", command=self.clear_selected_airfoils).pack(fill="x", pady=(6, 0))

        self.selected_listbox = tk.Listbox(
            frame,
            selectmode="extended",
            exportselection=False,
            background=self.app.ui_panel,
            foreground=self.app.ui_fg,
            selectbackground=self.app.ui_border,
            selectforeground=self.app.ui_accent,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            relief="flat",
        )
        self.selected_listbox.grid(row=1, column=2, sticky="nsew")
        self.selected_listbox.bind("<Double-Button-1>", lambda _event: self.remove_selected_airfoils())

    def build_summary_panel(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Summary", padding=6)
        frame.grid(row=2, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        self.summary_text = tk.Text(
            frame,
            wrap="none",
            font=("TkFixedFont", 10),
            background=self.app.ui_panel,
            foreground=self.app.ui_fg,
            insertbackground=self.app.ui_fg,
            selectbackground=self.app.ui_border,
            selectforeground=self.app.ui_accent,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            relief="flat",
        )
        scroll_y = ttk.Scrollbar(frame, orient="vertical", command=self.summary_text.yview)
        scroll_x = ttk.Scrollbar(frame, orient="horizontal", command=self.summary_text.xview)
        self.summary_text.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        self.summary_text.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")

    def build_plot_panel(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="NeuralFoil Plots", padding=6)
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        self.plot_figure = Figure(figsize=(10.5, 8.0), dpi=100)
        axes = self.plot_figure.subplots(2, 2)
        self.plot_axes = {
            "drag_polar": axes[0][0],
            "lift_curve": axes[0][1],
            "efficiency_curve": axes[1][0],
            "moment_curve": axes[1][1],
        }
        self.plot_canvas = FigureCanvasTkAgg(self.plot_figure, master=frame)
        self.plot_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def bind_numeric_entry(self, entry, callback) -> None:
        entry.configure(
            validate="key",
            validatecommand=(self.frame.register(self.validate_numeric_text), "%P"),
        )
        entry.bind("<Return>", lambda _event: callback())
        entry.bind("<KP_Enter>", lambda _event: callback())
        entry.bind("<FocusOut>", lambda _event: callback())

    def validate_numeric_text(self, proposed_text: str) -> bool:
        return re.fullmatch(r"[+-]?((\d+(\.\d*)?)|(\.\d*))?([eE][+-]?\d*)?", proposed_text) is not None

    def seed_selected_airfoils(self) -> None:
        """Select the airframe's own airfoils, adding AeroSandbox-named ones the library file does not hold."""
        selected_names = []
        if self.app.resolved is not None:
            for surface_set in self.app.resolved.surfaces:
                airfoil_name = surface_set.definition.airfoil
                if airfoil_name not in self.airfoil_library:
                    self.airfoil_library[airfoil_name] = asb_airfoil_record(airfoil_name)
                if airfoil_name not in selected_names:
                    selected_names.append(airfoil_name)
        if len(selected_names) == 0:
            for airfoil_name in DEFAULT_SELECTED_AIRFOIL_NAMES:
                if airfoil_name in self.airfoil_library:
                    selected_names.append(airfoil_name)
        self.selected_airfoil_names = selected_names

    def refresh_airfoil_listboxes(self) -> None:
        available_names = sorted(self.airfoil_library.keys())
        self.available_listbox.delete(0, tk.END)
        self.selected_listbox.delete(0, tk.END)
        for airfoil_name in available_names:
            self.available_listbox.insert(tk.END, airfoil_name)
        for airfoil_name in self.selected_airfoil_names:
            self.selected_listbox.insert(tk.END, airfoil_name)

    def add_selected_airfoils(self) -> None:
        for selection_index in self.available_listbox.curselection():
            airfoil_name = self.available_listbox.get(selection_index)
            if airfoil_name not in self.selected_airfoil_names:
                self.selected_airfoil_names.append(airfoil_name)
        self.refresh_airfoil_listboxes()
        self.recompute_airfoils()

    def remove_selected_airfoils(self) -> None:
        selected_names = {self.selected_listbox.get(index) for index in self.selected_listbox.curselection()}
        self.selected_airfoil_names = [name for name in self.selected_airfoil_names if name not in selected_names]
        self.refresh_airfoil_listboxes()
        self.recompute_airfoils()

    def clear_selected_airfoils(self) -> None:
        self.selected_airfoil_names = []
        self.refresh_airfoil_listboxes()
        self.recompute_airfoils()

    def import_airfoil_dat(self) -> None:
        dat_path = filedialog.askopenfilename(
            parent=self.frame,
            title="Import Airfoil DAT",
            filetypes=[("DAT files", "*.dat"), ("All files", "*.*")],
        )
        if dat_path == "":
            return
        record = airfoil_record_from_path(dat_path)
        try:
            airfoil_from_record(record)
        except ValueError as error:
            self.summary_text.delete("1.0", tk.END)
            self.summary_text.insert("1.0", f"Could not import {Path(dat_path).name}: {error}")
            return
        import_name = record["name"]
        suffix_index = 2
        while import_name in self.airfoil_library:
            import_name = f"{record['name']}-{suffix_index}"
            suffix_index += 1
        record["name"] = import_name
        self.airfoil_library[import_name] = record
        self.selected_airfoil_names.append(import_name)
        self.refresh_airfoil_listboxes()
        self.recompute_airfoils()

    def operating_point(self) -> dict:
        try:
            return {
                "alpha_deg": float(self.alpha_entry_var.get()),
                "airspeed_m_s": float(self.airspeed_var.get()),
                "altitude_m": float(self.altitude_var.get()),
                "temperature_c": float(self.temperature_var.get()),
                "chord_mm": float(self.chord_var.get()),
            }
        except ValueError:
            self.alpha_entry_var.set(f"{self.last_valid_operating_point['alpha_deg']:g}")
            self.airspeed_var.set(f"{self.last_valid_operating_point['airspeed_m_s']:g}")
            self.altitude_var.set(f"{self.last_valid_operating_point['altitude_m']:g}")
            self.temperature_var.set(f"{self.last_valid_operating_point['temperature_c']:g}")
            self.chord_var.set(f"{self.last_valid_operating_point['chord_mm']:g}")
            raise

    def neuralfoil_settings(self) -> dict:
        try:
            return {
                "n_crit": float(self.n_crit_var.get()),
                "xtr_upper": float(self.xtr_upper_var.get()),
                "xtr_lower": float(self.xtr_lower_var.get()),
                "model_size": self.model_size_var.get(),
                "include_360_deg_effects": bool(self.include_360_var.get()),
            }
        except ValueError:
            self.n_crit_var.set(f"{self.last_valid_neuralfoil_settings['n_crit']:g}")
            self.xtr_upper_var.set(f"{self.last_valid_neuralfoil_settings['xtr_upper']:g}")
            self.xtr_lower_var.set(f"{self.last_valid_neuralfoil_settings['xtr_lower']:g}")
            raise

    def on_alpha_scale_changed(self, _value: str) -> None:
        self.alpha_entry_var.set(f"{self.alpha_scale_var.get():g}")
        self.recompute_airfoils()

    def on_alpha_entry_commit(self) -> None:
        try:
            self.alpha_scale_var.set(float(self.alpha_entry_var.get()))
        except ValueError:
            self.alpha_entry_var.set(f"{self.last_valid_operating_point['alpha_deg']:g}")
            self.alpha_scale_var.set(self.last_valid_operating_point["alpha_deg"])
            return
        self.recompute_airfoils()

    def recompute_airfoils(self) -> None:
        try:
            operating_point = self.operating_point()
            neuralfoil_settings = self.neuralfoil_settings()
        except ValueError:
            return
        self.last_valid_operating_point = dict(operating_point)
        self.last_valid_neuralfoil_settings = dict(neuralfoil_settings)
        flow_conditions = compute_flow_conditions(operating_point)
        self.flow_text_var.set(
            f"Re = {flow_conditions['reynolds']:.0f} | M = {flow_conditions['mach']:.3f} | alpha sweep = {DEFAULT_SWEEP_ALPHA_MIN_DEG:.0f}..{DEFAULT_SWEEP_ALPHA_MAX_DEG:.0f} deg"
        )
        if len(self.selected_airfoil_names) == 0:
            self.current_results = None
            self.summary_text.delete("1.0", tk.END)
            self.summary_text.insert("1.0", "No airfoils selected.")
            self.draw_empty_plots()
            return
        records = [self.airfoil_library[name] for name in self.selected_airfoil_names]
        self.current_results = analyze_airfoils(records, operating_point, neuralfoil_settings)
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", self.current_results["summary_text"])
        self.draw_plots()

    def draw_empty_plots(self) -> None:
        for axis in self.plot_axes.values():
            axis.clear()
            axis.grid(True, alpha=0.25)
        self.plot_axes["drag_polar"].set_title("Cd vs Cl")
        self.plot_axes["lift_curve"].set_title("Cl vs Alpha")
        self.plot_axes["efficiency_curve"].set_title("Cl/Cd vs Alpha")
        self.plot_axes["moment_curve"].set_title("Cm vs Alpha")
        self.plot_canvas.draw_idle()

    def plot_curve_with_confidence(self, axis, x_values, y_values, confidence_values, label: str) -> None:
        """Solid where NeuralFoil is confident, faint dashed elsewhere; NaN gaps keep separate stretches unjoined."""
        high = confidence_values >= SUMMARY_CONFIDENCE_MIN
        low = ~high
        # widen the low-confidence mask by one point each side so dashed stretches meet the solid line
        low_drawn = low | np.roll(low, 1) | np.roll(low, -1)
        low_drawn[0], low_drawn[-1] = low[0] or low[1], low[-1] or low[-2]
        high_line = axis.plot(x_values, np.where(high, y_values, np.nan), label=label)[0]
        if np.any(low):
            axis.plot(x_values, np.where(low_drawn, y_values, np.nan), color=high_line.get_color(), alpha=0.3, linestyle="--")

    def draw_plots(self) -> None:
        for axis in self.plot_axes.values():
            axis.clear()
            axis.grid(True, alpha=0.25)
        self.plot_axes["drag_polar"].set_title("Cd vs Cl")
        self.plot_axes["drag_polar"].set_xlabel("Cl")
        self.plot_axes["drag_polar"].set_ylabel("Cd")
        self.plot_axes["lift_curve"].set_title("Cl vs Alpha")
        self.plot_axes["lift_curve"].set_xlabel("Alpha (deg)")
        self.plot_axes["lift_curve"].set_ylabel("Cl")
        self.plot_axes["efficiency_curve"].set_title("Cl/Cd vs Alpha")
        self.plot_axes["efficiency_curve"].set_xlabel("Alpha (deg)")
        self.plot_axes["efficiency_curve"].set_ylabel("Cl/Cd")
        self.plot_axes["moment_curve"].set_title("Cm vs Alpha")
        self.plot_axes["moment_curve"].set_xlabel("Alpha (deg)")
        self.plot_axes["moment_curve"].set_ylabel("Cm")
        for result in self.current_results["results"]:
            label = result["name"]
            sweep = result["sweep"]
            op = result["operating_point"]
            self.plot_curve_with_confidence(self.plot_axes["drag_polar"], sweep["cl"], sweep["cd"], sweep["confidence"], label)
            self.plot_axes["drag_polar"].scatter([op["cl"]], [op["cd"]], s=36)
            self.plot_curve_with_confidence(self.plot_axes["lift_curve"], sweep["alpha_deg"], sweep["cl"], sweep["confidence"], label)
            self.plot_axes["lift_curve"].scatter([op["alpha_deg"]], [op["cl"]], s=36)
            self.plot_curve_with_confidence(self.plot_axes["efficiency_curve"], sweep["alpha_deg"], sweep["cl_cd"], sweep["confidence"], label)
            self.plot_axes["efficiency_curve"].scatter([op["alpha_deg"]], [op["cl_cd"]], s=36)
            self.plot_curve_with_confidence(self.plot_axes["moment_curve"], sweep["alpha_deg"], sweep["cm"], sweep["confidence"], label)
            self.plot_axes["moment_curve"].scatter([op["alpha_deg"]], [op["cm"]], s=36)
        for axis in self.plot_axes.values():
            axis.legend(loc="best")
        self.plot_figure.tight_layout()
        self.plot_canvas.draw_idle()
