from __future__ import annotations

import copy
import queue
import re
import threading
import traceback
import tkinter as tk
from tkinter import ttk

from libs.analysis import run_vlm_sweep, vlm_point_key


class VLMMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.aero_view = self.frame

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
        app.aero_run_button = ttk.Button(header_frame, text="Run", command=self.start_run)
        app.aero_run_button.grid(row=0, column=0, sticky="w")
        app.aero_progressbar = ttk.Progressbar(header_frame, orient="horizontal", mode="determinate", maximum=100.0)
        app.aero_progressbar.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(header_frame, textvariable=app.aero_progress_text_var).grid(row=0, column=2, sticky="e")
        ttk.Label(aero_controls_frame, textvariable=app.aero_status_var).grid(row=1, column=0, sticky="w", pady=(0, 8))

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
        app.aero_vlm_vars = {}
        for group_index, (title, keys) in enumerate(grouped_keys):
            group_frame = ttk.LabelFrame(params_grid, text=title, padding=6)
            group_frame.grid(row=group_index // 2, column=group_index % 2, sticky="nsew", padx=4, pady=4)
            group_frame.columnconfigure(1, weight=1)
            for row_index, key in enumerate(keys):
                ttk.Label(group_frame, text=app.aero_vlm_labels[key]).grid(row=row_index, column=0, sticky="w")
                variable = tk.StringVar(value=f"{app.aero_vlm_settings[key]:g}")
                entry = ttk.Entry(group_frame, textvariable=variable)
                entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                self.bind_entry(entry, key, variable)
                app.aero_vlm_vars[key] = variable

        aero_results_frame = ttk.LabelFrame(aero_bottom_frame, text="Results", padding=4)
        aero_results_frame.grid(row=0, column=0, sticky="nsew")
        aero_results_frame.columnconfigure(0, weight=1)
        aero_results_frame.rowconfigure(0, weight=1)
        app.aero_results_text = tk.Text(
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
        aero_scroll_y = ttk.Scrollbar(aero_results_frame, orient="vertical", command=app.aero_results_text.yview)
        aero_scroll_x = ttk.Scrollbar(aero_results_frame, orient="horizontal", command=app.aero_results_text.xview)
        app.aero_results_text.configure(yscrollcommand=aero_scroll_y.set, xscrollcommand=aero_scroll_x.set)
        app.aero_results_text.grid(row=0, column=0, sticky="nsew")
        aero_scroll_y.grid(row=0, column=1, sticky="ns")
        aero_scroll_x.grid(row=1, column=0, sticky="ew")

        aero_display_frame = ttk.LabelFrame(aero_right_frame, text="Display", padding=6)
        aero_display_frame.grid(row=0, column=0, sticky="nsew")
        aero_display_frame.columnconfigure(0, weight=1)
        aero_display_frame.rowconfigure(2, weight=1)
        app.aero_display_frame = aero_display_frame

        aero_selector_frame = ttk.Frame(aero_display_frame)
        aero_selector_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        aero_selector_frame.columnconfigure(0, weight=1)

        altitude_row = ttk.Frame(aero_selector_frame)
        altitude_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        altitude_row.columnconfigure(1, weight=1)
        ttk.Label(altitude_row, text="Altitude").grid(row=0, column=0, sticky="w")
        app.aero_altitude_slider = tk.Scale(
            altitude_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=app.aero_altitude_value_var,
            showvalue=False,
            command=self.on_aero_selector_changed,
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
        app.aero_altitude_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(altitude_row, textvariable=app.aero_altitude_display_var).grid(row=0, column=2, sticky="e")

        aoa_row = ttk.Frame(aero_selector_frame)
        aoa_row.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        aoa_row.columnconfigure(1, weight=1)
        ttk.Label(aoa_row, text="AoA").grid(row=0, column=0, sticky="w")
        app.aero_aoa_slider = tk.Scale(
            aoa_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=app.aero_aoa_value_var,
            showvalue=False,
            command=self.on_aero_selector_changed,
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
        app.aero_aoa_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(aoa_row, textvariable=app.aero_aoa_display_var).grid(row=0, column=2, sticky="e")

        airspeed_row = ttk.Frame(aero_selector_frame)
        airspeed_row.grid(row=2, column=0, sticky="ew")
        airspeed_row.columnconfigure(1, weight=1)
        ttk.Label(airspeed_row, text="Airspeed").grid(row=0, column=0, sticky="w")
        app.aero_airspeed_slider = tk.Scale(
            airspeed_row,
            from_=0,
            to=0,
            orient="horizontal",
            resolution=1.0,
            variable=app.aero_airspeed_value_var,
            showvalue=False,
            command=self.on_aero_selector_changed,
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
        app.aero_airspeed_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(airspeed_row, textvariable=app.aero_airspeed_display_var).grid(row=0, column=2, sticky="e")

        ttk.Label(aero_display_frame, textvariable=app.aero_selected_point_var).grid(row=1, column=0, sticky="w", pady=(0, 8))

        app.aero_display_body = ttk.Frame(aero_display_frame)
        app.aero_display_body.grid(row=2, column=0, sticky="nsew")
        app.aero_display_body.columnconfigure(0, weight=1)
        app.aero_display_body.rowconfigure(0, weight=1)
        ttk.Button(app.aero_display_body, text="Show", command=self.show_selected_aero_vlm).grid(row=0, column=0, sticky="n", pady=(12, 8))
        app.aero_display_label = ttk.Label(
            app.aero_display_body,
            text="Run a sweep, choose a point, then Show.",
            anchor="center",
            justify="center",
        )
        app.aero_display_label.grid(row=1, column=0, sticky="n")

        app.root.update_idletasks()
        aero_panes.sashpos(0, max(app.root.winfo_width() // 3, 280))
        aero_left_panes.sashpos(0, max(app.root.winfo_height() // 2, 240))

    def on_entry_commit(self, key: str, variable: tk.StringVar) -> str:
        try:
            if type(self.app.aero_vlm_settings[key]) is int:
                value = int(float(variable.get()))
            else:
                value = float(variable.get())
        except ValueError:
            variable.set(f"{self.app.aero_vlm_settings[key]:g}")
            return "break"
        self.app.aero_vlm_settings[key] = value
        variable.set(f"{value:g}")
        return "break"

    def on_numeric_key_step(self, key: str, variable: tk.StringVar, delta: int) -> str:
        try:
            if type(self.app.aero_vlm_settings[key]) is int:
                current_value = int(float(variable.get()))
                variable.set(str(current_value + delta))
            else:
                current_value = float(variable.get())
                variable.set(f"{current_value + delta:g}")
        except ValueError:
            variable.set(f"{self.app.aero_vlm_settings[key]:g}")
            return "break"
        return self.on_entry_commit(key, variable)

    def bind_entry(self, widget, key: str, variable: tk.StringVar) -> None:
        value_type = "int" if type(self.app.aero_vlm_settings[key]) is int else "float"
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

    def start_run(self) -> None:
        app = self.app
        if app.aero_vlm_thread is not None and app.aero_vlm_thread.is_alive():
            return
        app.light_recompute()
        generation = app.aero_vlm_generation
        app.aero_vlm_queue = queue.Queue()
        app.aero_status_var.set("Running")
        app.aero_progress_text_var.set("0 / 0")
        if app.aero_progressbar is not None:
            app.aero_progressbar.configure(value=0.0)
        if app.aero_run_button is not None:
            app.aero_run_button.configure(state="disabled")
        if app.aero_results_text is not None:
            app.aero_results_text.delete("1.0", tk.END)
            app.aero_results_text.insert("1.0", "Running VLM sweep...")
        app.aero_selected_point_var.set("Running VLM sweep...")
        analysis_snapshot = copy.deepcopy(app.analysis)
        settings_snapshot = copy.deepcopy(app.aero_vlm_settings)
        app.aero_vlm_thread = threading.Thread(
            target=self.run_worker,
            args=(analysis_snapshot, settings_snapshot, generation),
            daemon=True,
        )
        app.aero_vlm_thread.start()
        app.root.after(app.aero_queue_poll_ms, self.poll_queue)

    def run_worker(self, analysis_snapshot: dict, settings_snapshot: dict, generation: int) -> None:
        try:
            result = run_vlm_sweep(
                analysis_snapshot,
                settings_snapshot,
                progress_callback=lambda completed, total, status: self.app.aero_vlm_queue.put(("progress", generation, completed, total, status)),
                include_vlm_objects=True,
            )
            self.app.aero_vlm_queue.put(("done", generation, result))
        except Exception:
            self.app.aero_vlm_queue.put(("error", generation, "".join(traceback.format_exc())))

    def poll_queue(self) -> None:
        app = self.app
        while True:
            try:
                message = app.aero_vlm_queue.get_nowait()
            except queue.Empty:
                break
            if message[0] == "progress":
                _, generation, completed, total, status = message
                if generation != app.aero_vlm_generation:
                    continue
                app.aero_status_var.set(status)
                app.aero_progress_text_var.set(f"{completed} / {total}")
                if app.aero_progressbar is not None and total > 0:
                    app.aero_progressbar.configure(value=(completed / total) * 100.0)
            elif message[0] == "done":
                _, generation, result = message
                if generation != app.aero_vlm_generation:
                    if app.aero_run_button is not None:
                        app.aero_run_button.configure(state="normal")
                    continue
                app.aero_vlm_result = result
                app.aero_vlm_cache = app.aero_vlm_result["vlm_cache"]
                app.aero_vlm_rows_by_key = {
                    vlm_point_key(row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"]): row
                    for row in app.aero_vlm_result["rows"]
                }
                app.aero_status_var.set("Complete")
                app.aero_progress_text_var.set(f"{len(app.aero_vlm_result['rows'])} / {len(app.aero_vlm_result['rows'])}")
                if app.aero_progressbar is not None:
                    app.aero_progressbar.configure(value=100.0)
                if app.aero_results_text is not None:
                    app.aero_results_text.delete("1.0", tk.END)
                    app.aero_results_text.insert("1.0", app.aero_vlm_result["report_text"])
                self.configure_selectors()
                self.on_aero_selector_changed("0")
                if app.aero_run_button is not None:
                    app.aero_run_button.configure(state="normal")
            elif message[0] == "error":
                _, generation, traceback_text = message
                if generation != app.aero_vlm_generation:
                    if app.aero_run_button is not None:
                        app.aero_run_button.configure(state="normal")
                    continue
                app.aero_status_var.set("Error")
                if app.aero_run_button is not None:
                    app.aero_run_button.configure(state="normal")
                app.show_error_dialog(traceback_text)
        if app.aero_vlm_thread is not None and app.aero_vlm_thread.is_alive():
            app.root.after(app.aero_queue_poll_ms, self.poll_queue)

    def clear_display(self, message: str) -> None:
        app = self.app
        app.aero_vlm_cache = {}
        app.aero_vlm_result = None
        app.aero_vlm_rows_by_key = {}
        app.aero_altitude_values = []
        app.aero_aoa_values = []
        app.aero_airspeed_values = []
        app.aero_altitude_value_var.set(0.0)
        app.aero_aoa_value_var.set(0.0)
        app.aero_airspeed_value_var.set(0.0)
        app.aero_altitude_display_var.set("0.0 m")
        app.aero_aoa_display_var.set("0.0 deg")
        app.aero_airspeed_display_var.set("0.0 m/s")
        app.aero_selected_point_var.set(message)
        app.aero_altitude_slider.configure(from_=0.0, to=0.0, resolution=1.0)
        app.aero_aoa_slider.configure(from_=0.0, to=0.0, resolution=1.0)
        app.aero_airspeed_slider.configure(from_=0.0, to=0.0, resolution=1.0)
        if app.aero_results_text is not None:
            app.aero_results_text.delete("1.0", tk.END)
            app.aero_results_text.insert("1.0", message)
        if app.aero_display_label is not None:
            app.aero_display_label.configure(text=message)
            app.aero_display_label.grid(row=1, column=0, sticky="n")

    def configure_selectors(self) -> None:
        app = self.app
        app.aero_altitude_values = list(app.aero_vlm_result["altitude_values"])
        app.aero_aoa_values = list(app.aero_vlm_result["aoa_values"])
        app.aero_airspeed_values = list(app.aero_vlm_result["airspeed_values"])
        altitude_step = app.aero_altitude_values[1] - app.aero_altitude_values[0] if len(app.aero_altitude_values) > 1 else 1.0
        aoa_step = app.aero_aoa_values[1] - app.aero_aoa_values[0] if len(app.aero_aoa_values) > 1 else 1.0
        airspeed_step = app.aero_airspeed_values[1] - app.aero_airspeed_values[0] if len(app.aero_airspeed_values) > 1 else 1.0
        app.aero_altitude_slider.configure(from_=app.aero_altitude_values[0], to=app.aero_altitude_values[-1], resolution=altitude_step)
        app.aero_aoa_slider.configure(from_=app.aero_aoa_values[0], to=app.aero_aoa_values[-1], resolution=aoa_step)
        app.aero_airspeed_slider.configure(from_=app.aero_airspeed_values[0], to=app.aero_airspeed_values[-1], resolution=airspeed_step)
        app.aero_altitude_value_var.set(app.aero_altitude_values[0])
        app.aero_aoa_value_var.set(app.aero_aoa_values[0])
        app.aero_airspeed_value_var.set(app.aero_airspeed_values[0])

    def selected_key(self):
        app = self.app
        if not app.aero_altitude_values or not app.aero_aoa_values or not app.aero_airspeed_values:
            return None
        altitude_m = float(app.aero_altitude_value_var.get())
        aoa_deg = float(app.aero_aoa_value_var.get())
        airspeed_m_s = float(app.aero_airspeed_value_var.get())
        return (round(float(altitude_m), 6), round(float(aoa_deg), 6), round(float(airspeed_m_s), 6))

    def on_aero_selector_changed(self, _value: str) -> None:
        app = self.app
        if app.aero_altitude_values:
            app.aero_altitude_display_var.set(f"{app.aero_altitude_value_var.get():.1f} m")
        if app.aero_aoa_values:
            app.aero_aoa_display_var.set(f"{app.aero_aoa_value_var.get():.1f} deg")
        if app.aero_airspeed_values:
            app.aero_airspeed_display_var.set(f"{app.aero_airspeed_value_var.get():.1f} m/s")
        key = self.selected_key()
        if key is None or key not in app.aero_vlm_cache:
            if key is None or key not in app.aero_vlm_rows_by_key:
                app.aero_selected_point_var.set("No VLM point selected.")
                return
            altitude_m, aoa_deg, airspeed_m_s = key
            app.aero_selected_point_var.set(
                f"Selected point: Alt {altitude_m:.1f} m   AoA {aoa_deg:.1f} deg   V {airspeed_m_s:.1f} m/s   (loaded rows only; rerun to Show)"
            )
            return
        altitude_m, aoa_deg, airspeed_m_s = key
        app.aero_selected_point_var.set(
            f"Selected point: Alt {altitude_m:.1f} m   AoA {aoa_deg:.1f} deg   V {airspeed_m_s:.1f} m/s"
        )

    def show_selected_aero_vlm(self) -> None:
        key = self.selected_key()
        if key is None or key not in self.app.aero_vlm_cache:
            if key is not None and key in self.app.aero_vlm_rows_by_key:
                self.app.aero_selected_point_var.set("Loaded VLM rows have no cached geometry. Re-run VLM to Show.")
            return
        self.app.aero_vlm_cache[key].draw(show_kwargs=dict())
