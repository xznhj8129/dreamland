"""
Usage:
from libs.views.vlm_view import VLMMainView

VLM view: AeroSandbox vortex-lattice sweep of the current resolved airframe (wings only, see the report header),
point selection by index sliders and the AeroSandbox 3D view of a solved point. The view owns its state; the app
calls mark_stale() whenever the airframe changes.
"""

from __future__ import annotations

import copy
import queue
import re
import threading
import traceback
import tkinter as tk
from tkinter import ttk

from libs.aero import run_vlm_sweep

VLM_DEFAULTS = {
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
VLM_LABELS = {
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
QUEUE_POLL_MS = 100
SELECTOR_AXES = ("altitude", "aoa", "airspeed")
SELECTOR_UNITS = {"altitude": "m", "aoa": "deg", "airspeed": "m/s"}


class VLMMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.vlm_settings = copy.deepcopy(VLM_DEFAULTS)
        self.vlm_vars = {}
        self.thread: threading.Thread | None = None
        self.queue: queue.Queue = queue.Queue()
        self.generation = 0
        self.result = None
        self.status_var = tk.StringVar(value="Idle")
        self.progress_text_var = tk.StringVar(value="0 / 0")
        self.selected_point_var = tk.StringVar(value="No VLM point selected.")
        self.index_vars = {axis: tk.IntVar(value=0) for axis in SELECTOR_AXES}
        self.display_vars = {axis: tk.StringVar(value="") for axis in SELECTOR_AXES}
        self.sliders = {}
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.vlm_view = self.frame

        aero_panes = ttk.Panedwindow(self.frame, orient=tk.HORIZONTAL)
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
        self.run_button = ttk.Button(header_frame, text="Run", command=self.start_run)
        self.run_button.grid(row=0, column=0, sticky="w")
        self.progressbar = ttk.Progressbar(header_frame, orient="horizontal", mode="determinate", maximum=100.0)
        self.progressbar.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(header_frame, textvariable=self.progress_text_var).grid(row=0, column=2, sticky="e")
        ttk.Label(aero_controls_frame, textvariable=self.status_var).grid(row=1, column=0, sticky="w", pady=(0, 8))

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
        self.vlm_vars = {}
        for group_index, (title, keys) in enumerate(grouped_keys):
            group_frame = ttk.LabelFrame(params_grid, text=title, padding=6)
            group_frame.grid(row=group_index // 2, column=group_index % 2, sticky="nsew", padx=4, pady=4)
            group_frame.columnconfigure(1, weight=1)
            for row_index, key in enumerate(keys):
                ttk.Label(group_frame, text=VLM_LABELS[key]).grid(row=row_index, column=0, sticky="w")
                variable = tk.StringVar(value=f"{self.vlm_settings[key]:g}")
                entry = ttk.Entry(group_frame, textvariable=variable)
                entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                self.bind_entry(entry, key, variable)
                self.vlm_vars[key] = variable

        aero_results_frame = ttk.LabelFrame(aero_bottom_frame, text="Results", padding=4)
        aero_results_frame.grid(row=0, column=0, sticky="nsew")
        aero_results_frame.columnconfigure(0, weight=1)
        aero_results_frame.rowconfigure(0, weight=1)
        self.results_text = tk.Text(
            aero_results_frame,
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
        aero_scroll_y = ttk.Scrollbar(aero_results_frame, orient="vertical", command=self.results_text.yview)
        aero_scroll_x = ttk.Scrollbar(aero_results_frame, orient="horizontal", command=self.results_text.xview)
        self.results_text.configure(yscrollcommand=aero_scroll_y.set, xscrollcommand=aero_scroll_x.set)
        self.results_text.grid(row=0, column=0, sticky="nsew")
        aero_scroll_y.grid(row=0, column=1, sticky="ns")
        aero_scroll_x.grid(row=1, column=0, sticky="ew")

        aero_display_frame = ttk.LabelFrame(aero_right_frame, text="Display", padding=6)
        aero_display_frame.grid(row=0, column=0, sticky="nsew")
        aero_display_frame.columnconfigure(0, weight=1)
        aero_display_frame.rowconfigure(2, weight=1)

        aero_selector_frame = ttk.Frame(aero_display_frame)
        aero_selector_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        aero_selector_frame.columnconfigure(0, weight=1)

        altitude_row = ttk.Frame(aero_selector_frame)
        altitude_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        altitude_row.columnconfigure(1, weight=1)
        ttk.Label(altitude_row, text="Altitude").grid(row=0, column=0, sticky="w")
        self.sliders["altitude"] = tk.Scale(
            altitude_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=self.index_vars["altitude"],
            showvalue=False,
            command=self.on_selector_changed,
            background=app.ui_bg,
            foreground=app.ui_fg,
            troughcolor=app.ui_panel,
            activebackground=app.ui_border,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            bd=0,
            relief="flat",
        )
        self.sliders["altitude"].grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(altitude_row, textvariable=self.display_vars["altitude"]).grid(row=0, column=2, sticky="e")

        aoa_row = ttk.Frame(aero_selector_frame)
        aoa_row.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        aoa_row.columnconfigure(1, weight=1)
        ttk.Label(aoa_row, text="AoA").grid(row=0, column=0, sticky="w")
        self.sliders["aoa"] = tk.Scale(
            aoa_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=self.index_vars["aoa"],
            showvalue=False,
            command=self.on_selector_changed,
            background=app.ui_bg,
            foreground=app.ui_fg,
            troughcolor=app.ui_panel,
            activebackground=app.ui_border,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            bd=0,
            relief="flat",
        )
        self.sliders["aoa"].grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(aoa_row, textvariable=self.display_vars["aoa"]).grid(row=0, column=2, sticky="e")

        airspeed_row = ttk.Frame(aero_selector_frame)
        airspeed_row.grid(row=2, column=0, sticky="ew")
        airspeed_row.columnconfigure(1, weight=1)
        ttk.Label(airspeed_row, text="Airspeed").grid(row=0, column=0, sticky="w")
        self.sliders["airspeed"] = tk.Scale(
            airspeed_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=self.index_vars["airspeed"],
            showvalue=False,
            command=self.on_selector_changed,
            background=app.ui_bg,
            foreground=app.ui_fg,
            troughcolor=app.ui_panel,
            activebackground=app.ui_border,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            bd=0,
            relief="flat",
        )
        self.sliders["airspeed"].grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(airspeed_row, textvariable=self.display_vars["airspeed"]).grid(row=0, column=2, sticky="e")

        ttk.Label(aero_display_frame, textvariable=self.selected_point_var).grid(row=1, column=0, sticky="w", pady=(0, 8))

        self.display_body = ttk.Frame(aero_display_frame)
        self.display_body.grid(row=2, column=0, sticky="nsew")
        self.display_body.columnconfigure(0, weight=1)
        self.display_body.rowconfigure(0, weight=1)
        ttk.Button(self.display_body, text="Show", command=self.show_selected).grid(row=0, column=0, sticky="n", pady=(12, 8))
        self.display_label = ttk.Label(
            self.display_body,
            text="Run a sweep, choose a point, then Show.",
            anchor="center",
            justify="center",
        )
        self.display_label.grid(row=1, column=0, sticky="n")

        app.root.update_idletasks()
        aero_panes.sashpos(0, max(app.root.winfo_width() // 3, 280))
        aero_left_panes.sashpos(0, max(app.root.winfo_height() // 2, 240))

    def on_entry_commit(self, key: str, variable: tk.StringVar) -> str:
        try:
            if type(self.vlm_settings[key]) is int:
                value = int(float(variable.get()))
            else:
                value = float(variable.get())
        except ValueError:
            variable.set(f"{self.vlm_settings[key]:g}")
            return "break"
        self.vlm_settings[key] = value
        variable.set(f"{value:g}")
        return "break"

    def on_numeric_key_step(self, key: str, variable: tk.StringVar, delta: int) -> str:
        try:
            if type(self.vlm_settings[key]) is int:
                current_value = int(float(variable.get()))
                variable.set(str(current_value + delta))
            else:
                current_value = float(variable.get())
                variable.set(f"{current_value + delta:g}")
        except ValueError:
            variable.set(f"{self.vlm_settings[key]:g}")
            return "break"
        return self.on_entry_commit(key, variable)

    def bind_entry(self, widget, key: str, variable: tk.StringVar) -> None:
        value_type = "int" if type(self.vlm_settings[key]) is int else "float"
        widget.configure(
            validate="key",
            validatecommand=(self.app.root.register(self.validate_numeric_text), value_type, "%P"),
        )
        widget.bind("<Return>", lambda event, k=key, v=variable: self.on_entry_commit(k, v))
        widget.bind("<KP_Enter>", lambda event, k=key, v=variable: self.on_entry_commit(k, v))
        widget.bind("<FocusOut>", lambda event, k=key, v=variable: self.on_entry_commit(k, v))
        widget.bind("<Up>", lambda event, k=key, v=variable: self.on_numeric_key_step(k, v, 1))
        widget.bind("<Down>", lambda event, k=key, v=variable: self.on_numeric_key_step(k, v, -1))

    def validate_numeric_text(self, value_type: str, proposed_text: str) -> bool:
        if value_type == "int":
            return re.fullmatch(r"[+-]?\d*", proposed_text) is not None
        return re.fullmatch(r"[+-]?((\d+(\.\d*)?)|(\.\d*))?([eE][+-]?\d*)?", proposed_text) is not None


    def mark_stale(self) -> None:
        """The airframe changed: any finished or running sweep no longer matches it."""
        self.generation += 1
        self.result = None
        self.status_var.set("Stale: press Run (or F5 here)")
        self.progress_text_var.set("0 / 0")
        self.progressbar.configure(value=0.0)
        self.selected_point_var.set("No VLM point selected.")
        for axis in SELECTOR_AXES:
            self.sliders[axis].configure(from_=0, to=0)
            self.display_vars[axis].set("")
        self.results_text.delete("1.0", tk.END)
        self.results_text.insert("1.0", "Airframe changed since the last sweep. Press Run.")

    def start_run(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        if self.app.resolved is None:
            self.status_var.set("No valid airframe; fix the definition first.")
            return
        self.generation += 1
        generation = self.generation
        self.queue = queue.Queue()
        self.status_var.set("Running")
        self.progress_text_var.set("0 / 0")
        self.progressbar.configure(value=0.0)
        self.run_button.configure(state="disabled")
        self.results_text.delete("1.0", tk.END)
        self.results_text.insert("1.0", "Running VLM sweep...")
        self.selected_point_var.set("Running VLM sweep...")
        self.thread = threading.Thread(target=self.run_worker, args=(self.app.resolved, copy.deepcopy(self.vlm_settings), generation), daemon=True)
        self.thread.start()
        self.app.root.after(QUEUE_POLL_MS, self.poll_queue)

    def run_worker(self, resolved, settings: dict, generation: int) -> None:
        try:
            result = run_vlm_sweep(resolved, settings, progress_callback=lambda completed, total, status: self.queue.put(("progress", generation, completed, total, status)))
            self.queue.put(("done", generation, result))
        except ValueError as error:  # bad sweep ranges: show the message, not a traceback
            self.queue.put(("invalid", generation, str(error)))
        except Exception:  # thread boundary: hand the traceback to the Tk thread for the error dialog
            self.queue.put(("error", generation, traceback.format_exc()))

    def poll_queue(self) -> None:
        """Drain messages; keep polling until the worker's final message has been handled (never lose a result)."""
        finished = False
        while True:
            try:
                message = self.queue.get_nowait()
            except queue.Empty:
                break
            kind, generation = message[0], message[1]
            if kind != "progress":
                finished = True
                self.run_button.configure(state="normal")
            if generation != self.generation:
                continue
            if kind == "progress":
                _, _, completed, total, status = message
                self.status_var.set(status)
                self.progress_text_var.set(f"{completed} / {total}")
                self.progressbar.configure(value=completed / total * 100.0)
            elif kind == "done":
                self.result = message[2]
                self.status_var.set("Complete")
                self.progressbar.configure(value=100.0)
                self.results_text.delete("1.0", tk.END)
                self.results_text.insert("1.0", self.result.report_text)
                self.configure_selectors()
            elif kind == "invalid":
                self.status_var.set(message[2])
                self.results_text.delete("1.0", tk.END)
                self.results_text.insert("1.0", message[2])
            else:
                self.status_var.set("Error")
                self.app.show_error_dialog(message[2])
        if not finished:
            self.app.root.after(QUEUE_POLL_MS, self.poll_queue)

    def axis_values(self, axis: str) -> tuple[float, ...]:
        return {"altitude": self.result.altitude_values, "aoa": self.result.aoa_values, "airspeed": self.result.airspeed_values}[axis]

    def configure_selectors(self) -> None:
        for axis in SELECTOR_AXES:
            self.sliders[axis].configure(from_=0, to=len(self.axis_values(axis)) - 1, resolution=1)
            self.index_vars[axis].set(0)
        self.on_selector_changed("0")

    def selected_point(self) -> tuple[float, float, float] | None:
        if self.result is None:
            return None
        return tuple(self.axis_values(axis)[self.index_vars[axis].get()] for axis in SELECTOR_AXES)

    def on_selector_changed(self, _value: str) -> None:
        point = self.selected_point()
        if point is None:
            return
        for axis, value in zip(SELECTOR_AXES, point):
            self.display_vars[axis].set(f"{value:g} {SELECTOR_UNITS[axis]}")
        altitude_m, aoa_deg, airspeed_m_s = point
        row = next(row for row in self.result.rows if (row.altitude_m, row.aoa_deg, row.airspeed_m_s) == point)
        self.selected_point_var.set(
            f"Alt {altitude_m:g} m   AoA {aoa_deg:g} deg   V {airspeed_m_s:g} m/s   ->   lift {row.lift_n:.2f} N ({row.load_pct:.0f} % weight), CL {row.cl:.3f}"
        )

    def show_selected(self) -> None:
        point = self.selected_point()
        if point is None:
            self.selected_point_var.set("Run a sweep first.")
            return
        self.result.solutions[point[1]].draw(show_kwargs=dict())
