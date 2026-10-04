"""
Usage:
from libs.views.sitl_view import SITLMainView

SITL view: flies the project's saved airframe in INAV SITL on JSBSim by running sim_inav.py as a subprocess (the same path as
the command line), streams its output into the summary and plots the bench's flight log (CSV) live while it runs.
The plotted columns are picked on the right.
"""

from __future__ import annotations

import os
import queue
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from libs.controller import controller_names
from libs.simrun import START_OPTIONS, start_reasons
from sim_inav import REPO_DIR, flight_log_path, is_real_fc_diff, tests_for
from sim_script import DEFAULT_DURATION_S, script_log_path

SIM_SCRIPT = REPO_DIR / "sim_inav.py"
SCRIPT_RUNNER = REPO_DIR / "sim_script.py"
INAV_CONTROLLER = "INAV SITL"
SCRIPT_PREFIX = "script: "  # then a script in the project's controllers/ (libs/controller.py)
GENERATED_CONFIG = "generated from the design"
DIFF_PREFIXES = ("real FC diff: ", "SITL diff: ")  # then a diff in the project's inav/
START_LABELS = {"alt": "Altitude (m)", "speed": "Speed (m/s)", "heading": "Heading (deg)", "lat": "Latitude", "lon": "Longitude",
                "pitch": "Pitch (deg)", "roll": "Roll (deg)", "wind-from": "Wind from (deg)", "wind-speed": "Wind (m/s)"}  # sim_inav START_OPTIONS
START_ENTRY_WIDTH = 10
QUEUE_POLL_MS = 100
PLOT_REFRESH_MS = 500
SHUTDOWN_WAIT_S = 5.0  # after SIGINT, for sim_inav to stop its SITL, before it is killed
DEFAULT_X = "t"
DEFAULT_Y = ("agl_m", "ias_kts", "pitch")
SERIES_COLORS = ("#19ff19", "#33ccff", "#ffcc33", "#ff6666", "#cc66ff", "#ffffff")
ZOOM_STEP = 0.8  # one wheel notch in: the x range shrinks to this around the cursor (out: grows by its inverse)
DRAG_MIN_FRACTION = 0.01  # a drag shorter than this of the shown x range is a click, not a zoom
DRAG_SPAN_COLOR = "#33ccff"
PATH_COLUMNS = ("east_m", "north_m", "agl_m")  # the flight log's position, m from home and above ground
PATH_VIEW = (25.0, -60.0)  # elevation, azimuth of the 3D view until it is rotated
PATH_START_COLOR, PATH_END_COLOR = "#19ff19", "#ff4d4d"
PATH_BOX_ZOOM = 0.85  # the 3D box a little smaller than its axes, so the axis labels fit


class SITLMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.process: subprocess.Popen | None = None
        self.output_queue: queue.Queue = queue.Queue()
        self.run_started = 0.0
        self.log_path: Path | None = None  # the running test's flight log, tailed while the sim runs
        self.log_offset = 0
        self.columns: list[str] = []
        self.rows: list[list[float]] = []
        self.plot_dirty = False
        self.x_range: tuple[float, float] | None = None  # the Plots tab's zoom; None = the whole run
        self.drag_start: float | None = None
        self.drag_spans: list = []
        self.path_axes = None  # the 3D view's axes, kept for its rotation across redraws
        self.controller_var = tk.StringVar(value=INAV_CONTROLLER)
        self.duration_var = tk.StringVar(value=f"{DEFAULT_DURATION_S:g}")
        self.scenario_var = tk.StringVar(value="")
        self.start_var = tk.StringVar(value="air")
        self.start_option_vars = {option: tk.StringVar(value="") for option in START_OPTIONS}
        self.start_option_entries: dict[str, ttk.Entry] = {}
        self.config_var = tk.StringVar(value=GENERATED_CONFIG)
        self.status_var = tk.StringVar(value="Idle")
        self.x_var = tk.StringVar(value=DEFAULT_X)
        self.y_vars: dict[str, tk.BooleanVar] = {}
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.sitl_view = self.frame

        panes = ttk.Panedwindow(self.frame, orient=tk.HORIZONTAL)
        panes.grid(row=0, column=0, sticky="nsew")
        left_frame = ttk.Frame(panes, padding=8)
        middle_frame = ttk.Frame(panes, padding=8)
        right_frame = ttk.Frame(panes, padding=8)
        for pane_frame in (left_frame, middle_frame, right_frame):
            pane_frame.columnconfigure(0, weight=1)
        panes.add(left_frame, weight=2)
        panes.add(middle_frame, weight=4)
        panes.add(right_frame, weight=1)

        left_frame.rowconfigure(1, weight=1)
        controls_frame = ttk.LabelFrame(left_frame, text="SITL Run", padding=6)
        controls_frame.grid(row=0, column=0, sticky="ew")
        controls_frame.columnconfigure(1, weight=1)
        ttk.Label(controls_frame, text="Flight controller").grid(row=0, column=0, sticky="w")
        self.controller_box = ttk.Combobox(controls_frame, textvariable=self.controller_var, state="readonly")
        self.controller_box.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
        self.controller_box.bind("<<ComboboxSelected>>", lambda event: self.refresh_controller())
        ttk.Label(controls_frame, text="Scenario").grid(row=1, column=0, sticky="w")
        self.scenario_box = ttk.Combobox(controls_frame, textvariable=self.scenario_var, state="readonly")
        self.scenario_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
        self.scenario_box.bind("<<ComboboxSelected>>", lambda event: self.refresh_start())
        ttk.Label(controls_frame, text="Start").grid(row=3, column=0, sticky="w")
        self.start_box = ttk.Combobox(controls_frame, textvariable=self.start_var, state="readonly")
        self.start_box.grid(row=3, column=1, sticky="ew", padx=(8, 0), pady=2)
        ttk.Label(controls_frame, text="FC settings").grid(row=4, column=0, sticky="w")
        self.config_box = ttk.Combobox(controls_frame, textvariable=self.config_var, state="readonly")
        self.config_box.grid(row=4, column=1, sticky="ew", padx=(8, 0), pady=2)
        ttk.Label(controls_frame, text="Duration (s)").grid(row=5, column=0, sticky="w")
        self.duration_entry = ttk.Entry(controls_frame, textvariable=self.duration_var, width=START_ENTRY_WIDTH)
        self.duration_entry.grid(row=5, column=1, sticky="w", padx=(8, 0), pady=2)
        conditions = ttk.LabelFrame(controls_frame, text="Start conditions (blank = default)", padding=4)
        conditions.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        for index, option in enumerate(START_OPTIONS):
            row, column = divmod(index, 2)
            ttk.Label(conditions, text=START_LABELS[option]).grid(row=row, column=2 * column, sticky="w", padx=(0 if column == 0 else 12, 4))
            entry = ttk.Entry(conditions, textvariable=self.start_option_vars[option], width=START_ENTRY_WIDTH)
            entry.grid(row=row, column=2 * column + 1, sticky="w", pady=1)
            self.start_option_entries[option] = entry
        self.start_box.bind("<<ComboboxSelected>>", lambda event: self.refresh_start_conditions())
        self.run_button = ttk.Button(controls_frame, text="Run", command=self.start_run)
        self.run_button.grid(row=7, column=0, sticky="w", pady=(6, 0))
        ttk.Label(controls_frame, textvariable=self.status_var).grid(row=7, column=1, sticky="w", padx=(8, 0), pady=(6, 0))

        summary_frame = ttk.LabelFrame(left_frame, text="Summary", padding=4)
        summary_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        summary_frame.columnconfigure(0, weight=1)
        summary_frame.rowconfigure(0, weight=1)
        self.summary_text = tk.Text(
            summary_frame,
            wrap="word",
            font=("TkFixedFont", 9),
            background=app.ui_panel,
            foreground=app.ui_fg,
            insertbackground=app.ui_fg,
            selectbackground=app.ui_border,
            selectforeground=app.ui_accent,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            relief="flat",
            state="disabled",
        )
        summary_scroll_y = ttk.Scrollbar(summary_frame, orient="vertical", command=self.summary_text.yview)
        self.summary_text.configure(yscrollcommand=summary_scroll_y.set)
        self.summary_text.grid(row=0, column=0, sticky="nsew")
        summary_scroll_y.grid(row=0, column=1, sticky="ns")

        middle_frame.rowconfigure(0, weight=1)
        middle_frame.columnconfigure(0, weight=1)
        self.center_tabs = ttk.Notebook(middle_frame)
        self.center_tabs.grid(row=0, column=0, sticky="nsew")
        self.figure, self.canvas = self.figure_tab("Plots")
        self.path_figure, self.path_canvas = self.figure_tab("3D View")
        self.center_tabs.bind("<<NotebookTabChanged>>", lambda event: self.draw_plot())
        self.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)

        right_frame.rowconfigure(1, weight=1)
        x_frame = ttk.LabelFrame(right_frame, text="X axis", padding=6)
        x_frame.grid(row=0, column=0, sticky="ew")
        x_frame.columnconfigure(0, weight=1)
        self.x_box = ttk.Combobox(x_frame, textvariable=self.x_var, state="readonly")
        self.x_box.grid(row=0, column=0, sticky="ew")
        self.x_box.bind("<<ComboboxSelected>>", lambda event: self.reset_zoom())
        y_frame = ttk.LabelFrame(right_frame, text="Y values", padding=6)
        y_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        y_frame.columnconfigure(0, weight=1)
        y_frame.rowconfigure(0, weight=1)
        y_canvas = tk.Canvas(y_frame, background=app.ui_bg, highlightthickness=1, highlightbackground=app.ui_border,
                             highlightcolor=app.ui_accent, relief="flat", width=160)
        y_scroll = ttk.Scrollbar(y_frame, orient="vertical", command=y_canvas.yview)
        self.y_list = ttk.Frame(y_canvas)
        self.y_list.bind("<Configure>", lambda event: y_canvas.configure(scrollregion=y_canvas.bbox("all")))
        window_id = y_canvas.create_window((0, 0), window=self.y_list, anchor="nw")
        y_canvas.configure(yscrollcommand=y_scroll.set)
        y_canvas.grid(row=0, column=0, sticky="nsew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        y_canvas.bind("<Configure>", lambda event: y_canvas.itemconfigure(window_id, width=event.width))

        self.output_after_id = self.frame.after(QUEUE_POLL_MS, self.poll_output)
        self.log_after_id = self.frame.after(PLOT_REFRESH_MS, self.poll_log)

    def refresh(self) -> None:
        """Offer the flight controllers (INAV SITL and the project's scripts), the scenarios and starts the current
        airframe can fly, and the FC settings (generated, or the project's real FC diffs)."""
        controllers = [INAV_CONTROLLER, *(SCRIPT_PREFIX + name for name in controller_names(self.app.project.controllers_dir))]
        self.controller_box.configure(values=controllers)
        if self.controller_var.get() not in controllers:
            self.controller_var.set(INAV_CONTROLLER)
        airframe = self.app.resolved.airframe if self.app.resolved is not None else None
        tests = tests_for(airframe) if airframe is not None else []
        self.scenario_box.configure(values=tests)
        if self.scenario_var.get() not in tests:
            self.scenario_var.set(tests[0] if tests else "")
        self.refresh_start()
        configs = [GENERATED_CONFIG, *(DIFF_PREFIXES[not is_real_fc_diff(path)] + path.name for path in sorted(self.app.project.inav_dir.glob("*.txt")))]
        self.config_box.configure(values=configs)
        if self.config_var.get() not in configs:
            self.config_var.set(GENERATED_CONFIG)
        if self.process is None:
            self.status_var.set("Idle" if tests else "No valid airframe")
        self.refresh_controller()

    def script(self) -> str | None:
        """The chosen script's name, or None for INAV SITL."""
        choice = self.controller_var.get()
        return choice.removeprefix(SCRIPT_PREFIX) if choice.startswith(SCRIPT_PREFIX) else None

    def refresh_controller(self) -> None:
        """A script flies itself: scenario and FC settings are INAV's and greyed out; a duration applies instead."""
        script = self.script() is not None
        for box in (self.scenario_box, self.config_box):
            box.state(["disabled"] if script else ["!disabled", "readonly"])
        self.duration_entry.state(["!disabled"] if script else ["disabled"])
        self.refresh_start()

    def refresh_start(self) -> None:
        """Starts for the chosen scenario: hover always stands on the ground; forward flight starts in the air, or on the
        ground where the airframe can take off."""
        airframe = self.app.resolved.airframe if self.app.resolved is not None else None
        if airframe is None or self.script() is None and self.scenario_var.get() == "hover":
            starts = ["ground"] if airframe is not None else []
        else:
            starts = [start for start, reason in start_reasons(airframe).items() if reason is None]
        self.start_box.configure(values=starts)
        if self.start_var.get() not in starts:
            self.start_var.set(starts[0] if starts else "")
        self.refresh_start_conditions()

    def refresh_start_conditions(self) -> None:
        """A ground start stands on its tail at rest: its height, speed and attitude entries are off."""
        ground = self.start_var.get() == "ground"
        for option, (_text, air_only) in START_OPTIONS.items():
            self.start_option_entries[option].state(["disabled"] if air_only and ground else ["!disabled"])

    def start_run(self) -> None:
        test, start, script = self.scenario_var.get(), self.start_var.get(), self.script()
        if self.process is not None or not test and script is None:
            return
        conditions = []
        for option, (_text, air_only) in START_OPTIONS.items():
            text = self.start_option_vars[option].get().strip()
            if not text or air_only and start == "ground":
                continue
            try:
                conditions += [f"--{option}", f"{float(text):g}"]
            except ValueError:
                self.status_var.set(f"{START_LABELS[option]}: {text!r} is not a number")
                return
        duration = self.duration_var.get().strip()
        if script is not None and duration:
            try:
                conditions += ["--duration", f"{float(duration):g}"]
            except ValueError:
                self.status_var.set(f"Duration (s): {duration!r} is not a number")
                return
        self.app.save_project()  # what flies is the file on disk
        project = self.app.project
        if script is not None:
            command = [sys.executable, "-u", str(SCRIPT_RUNNER), str(project.root), script, "--start", start, *conditions]
        else:
            command = [sys.executable, "-u", str(SIM_SCRIPT), test, str(project.root)]
            if test != "hover":
                command += ["--start", start]
            command += conditions
            if self.config_var.get() != GENERATED_CONFIG:
                command += ["--inav-config", str(project.inav_dir / self.config_var.get().split(": ", 1)[1])]
        self.summary_text.configure(state="normal")
        self.summary_text.delete("1.0", tk.END)
        shown = ["python", *(os.path.relpath(part, REPO_DIR) if os.path.isabs(part) else part for part in command[2:])]
        self.summary_text.insert(tk.END, " ".join(shown) + "\n\n")
        self.summary_text.configure(state="disabled")
        self.columns, self.rows, self.log_offset = [], [], 0
        self.x_range = None
        self.log_path = flight_log_path(project, test, start) if script is None else script_log_path(project, script)
        self.run_started = time.time()
        self.draw_plot()
        self.process = subprocess.Popen(command, cwd=REPO_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self.read_output, args=(self.process,), daemon=True).start()
        self.run_button.state(["disabled"])
        self.status_var.set(f"Running {test if script is None else SCRIPT_PREFIX + script}...")

    def read_output(self, process: subprocess.Popen) -> None:
        for line in process.stdout:
            self.output_queue.put(line)
        self.output_queue.put(None)  # end of output: the process is exiting

    def poll_output(self) -> None:
        lines, finished = [], False
        while not self.output_queue.empty():
            line = self.output_queue.get()
            if line is None:
                finished = True
                break
            lines.append(line)
        if lines:
            self.summary_text.configure(state="normal")
            self.summary_text.insert(tk.END, "".join(lines))
            self.summary_text.configure(state="disabled")
            self.summary_text.see(tk.END)
        if finished:
            code = self.process.wait()
            self.process = None
            self.read_log()  # the rest of the log, flushed when the bench closed it
            self.log_path = None
            if self.plot_dirty:
                self.draw_plot()
            self.run_button.state(["!disabled"])
            self.status_var.set(f"Finished (exit code {code})")
        self.output_after_id = self.frame.after(QUEUE_POLL_MS, self.poll_output)

    def poll_log(self) -> None:
        if self.log_path is not None:
            self.read_log()
            if self.plot_dirty:
                self.draw_plot()
        self.log_after_id = self.frame.after(PLOT_REFRESH_MS, self.poll_log)

    def read_log(self) -> None:
        """Append the complete rows written since the last read. A log older than this run is the previous run's,
        which sim_inav removes before the bench starts."""
        try:
            if self.log_path.stat().st_mtime < self.run_started:
                return
            with open(self.log_path, "rb") as file:
                file.seek(self.log_offset)
                chunk = file.read()
        except FileNotFoundError:
            return
        end = chunk.rfind(b"\n") + 1
        if end == 0:
            return
        self.log_offset += end
        lines = chunk[:end].decode().splitlines()
        if not self.columns:
            self.set_columns(lines.pop(0).split(","))
        self.rows.extend([float(value) for value in line.split(",")] for line in lines)
        self.plot_dirty = self.plot_dirty or bool(lines)

    def set_columns(self, columns: list[str]) -> None:
        """Offer the log's columns on the right, keeping the picked ones of an earlier run."""
        self.columns = columns
        self.x_box.configure(values=columns)
        if self.x_var.get() not in columns:
            self.x_var.set(DEFAULT_X)
        for child in self.y_list.winfo_children():
            child.destroy()
        for row, column in enumerate(columns):
            if column not in self.y_vars:
                self.y_vars[column] = tk.BooleanVar(value=column in DEFAULT_Y)
            ttk.Checkbutton(self.y_list, text=column, variable=self.y_vars[column], command=self.draw_plot).grid(row=row, column=0, sticky="w")

    def figure_tab(self, title: str) -> tuple[Figure, FigureCanvasTkAgg]:
        """A center tab holding one matplotlib figure."""
        tab = ttk.Frame(self.center_tabs, padding=4)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        self.center_tabs.add(tab, text=title)
        figure = Figure(figsize=(8.0, 6.0), dpi=100, facecolor=self.app.ui_bg, layout="constrained")
        canvas = FigureCanvasTkAgg(figure, master=tab)
        canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        return figure, canvas

    def draw_plot(self) -> None:
        """Redraw the shown center tab (the other one is drawn when it is shown)."""
        self.plot_dirty = False
        if self.center_tabs.index("current") == 0:
            self.draw_series()
        else:
            self.draw_path()

    def draw_series(self) -> None:
        """One subplot per picked Y column against the X column, sharing the X axis, within the zoom (x_range) with
        each Y axis fitted to what is shown."""
        app = self.app
        self.figure.clear()
        self.drag_spans = []
        picked = [column for column in self.columns if self.y_vars[column].get()]
        if self.rows and picked:
            data = np.array(self.rows)
            x = data[:, self.columns.index(self.x_var.get())]
            shown = np.ones(len(x), dtype=bool) if self.x_range is None else (x >= self.x_range[0]) & (x <= self.x_range[1])
            axes = self.figure.subplots(len(picked), 1, sharex=True, squeeze=False)[:, 0]
            for index, (axis, column) in enumerate(zip(axes, picked)):
                y = data[:, self.columns.index(column)]
                axis.plot(x, y, color=SERIES_COLORS[index % len(SERIES_COLORS)], linewidth=1.2)
                if self.x_range is not None and shown.any():
                    low, high = np.nanmin(y[shown]), np.nanmax(y[shown])
                    margin = (high - low) * 0.05 or max(abs(high) * 0.05, 1e-3)
                    axis.set_ylim(low - margin, high + margin)
                axis.set_ylabel(column, color=app.ui_fg)
                axis.set_facecolor(app.ui_bg)
                axis.tick_params(colors=app.ui_fg)
                axis.grid(color=app.ui_border, alpha=0.4)
                for spine in axis.spines.values():
                    spine.set_color(app.ui_border)
            if self.x_range is not None:
                axes[0].set_xlim(*self.x_range)
            axes[-1].set_xlabel(self.x_var.get(), color=app.ui_fg)
        self.canvas.draw_idle()

    def reset_zoom(self) -> None:
        self.x_range = None
        self.draw_plot()

    def on_scroll(self, event) -> None:
        """Wheel: zoom the x range in (up) or out (down) around the cursor."""
        if event.inaxes is None or event.xdata is None:
            return
        low, high = event.inaxes.get_xlim()
        factor = ZOOM_STEP if event.button == "up" else 1.0 / ZOOM_STEP
        self.x_range = (event.xdata - (event.xdata - low) * factor, event.xdata + (high - event.xdata) * factor)
        self.draw_series()

    def on_press(self, event) -> None:
        """Left button: a double-click shows the whole run again; a press starts a drag across the stretch to zoom to."""
        if event.inaxes is None or event.xdata is None or event.button != 1:
            return
        if event.dblclick:
            self.drag_start = None
            self.reset_zoom()
            return
        self.drag_start = event.xdata

    def on_drag(self, event) -> None:
        """Shade the stretch being dragged across, on every subplot."""
        if self.drag_start is None or event.xdata is None:
            return
        for span in self.drag_spans:
            span.remove()
        self.drag_spans = [axis.axvspan(min(self.drag_start, event.xdata), max(self.drag_start, event.xdata), color=DRAG_SPAN_COLOR, alpha=0.2)
                           for axis in self.figure.axes]
        self.canvas.draw_idle()

    def on_release(self, event) -> None:
        """End of a drag: zoom to the stretch, unless it was too short to be one."""
        if self.drag_start is None:
            return
        start, self.drag_start = self.drag_start, None
        axes = self.figure.axes
        if event.xdata is None or not axes:
            self.draw_series()
            return
        low, high = axes[0].get_xlim()
        if abs(event.xdata - start) > DRAG_MIN_FRACTION * (high - low):
            self.x_range = (min(start, event.xdata), max(start, event.xdata))
        self.draw_series()

    def draw_path(self) -> None:
        """The flight path in 3D (east, north, height above ground), start and end marked, in a box just around it with
        the ground (height 0) at the bottom; drag to rotate, which is kept across redraws."""
        app = self.app
        view = (self.path_axes.elev, self.path_axes.azim) if self.path_axes is not None else PATH_VIEW
        self.path_figure.clear()
        axes = self.path_figure.add_subplot(projection="3d")
        self.path_axes = axes
        axes.set_facecolor(app.ui_bg)
        for axis in (axes.xaxis, axes.yaxis, axes.zaxis):
            axis.set_pane_color((0.0, 0.06, 0.0, 1.0))
            axis.label.set_color(app.ui_fg)
        axes.tick_params(colors=app.ui_fg)
        axes.set_xlabel("east m")
        axes.set_ylabel("north m")
        axes.set_zlabel("height m")
        if self.rows and all(column in self.columns for column in PATH_COLUMNS):
            data = np.array(self.rows)
            east, north, height = (data[:, self.columns.index(column)] for column in PATH_COLUMNS)
            axes.plot(east, north, height, color=SERIES_COLORS[0], linewidth=1.2)
            axes.scatter(east[:1], north[:1], height[:1], color=PATH_START_COLOR, s=30, label="start")
            axes.scatter(east[-1:], north[-1:], height[-1:], color=PATH_END_COLOR, s=30, label="end")
            for limits, values in ((axes.set_xlim, east), (axes.set_ylim, north)):
                low, high = float(np.nanmin(values)), float(np.nanmax(values))
                limits(low, high if high > low else low + 1.0)
            axes.set_zlim(0.0, max(float(np.nanmax(height)), 1.0))  # the ground at the bottom, whatever the path
            legend = axes.legend(loc="upper left", facecolor=app.ui_panel, edgecolor=app.ui_border)
            for text in legend.get_texts():
                text.set_color(app.ui_fg)
        axes.view_init(*view)
        axes.set_box_aspect(None, zoom=PATH_BOX_ZOOM)
        self.path_canvas.draw_idle()

    def shutdown(self) -> None:
        """Stop polling and end a running sim: SIGINT lets sim_inav stop its SITL (own session, not killed with it)."""
        self.frame.after_cancel(self.output_after_id)
        self.frame.after_cancel(self.log_after_id)
        if self.process is None:
            return
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=SHUTDOWN_WAIT_S)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
