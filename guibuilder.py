"""
Usage:
python guibuilder.py
python guibuilder.py aircraft/quadlancer62

Tk front end for plane-first airframes (aircraft projects, see libs/project.py; airframe/2 build files, see PLAN.md):
edit the aircraft definition and see
the drawing, mass/CG, aerodynamics and stability update. Geometry and mass update immediately; aero runs in
the background and replaces the summary when it finishes.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, simpledialog, ttk

from libs.aero import OperatingPointError, analyze_aero, build_asb_airplane
from libs.airframe import SCHEMA, AirframeError, load_parts_data, parse_airframe
from libs.geometry import resolve_airframe
from libs.motorlib import MotorStalled
from libs.project import BUILD_FILE, PROJECTS_DIR, Project, ProjectError, copy_project, open_project
from libs.propulsion import propulsion_summary
from libs.report import build_report_lines
from libs.reportline import ReportLine
from libs.views.aero_analysis_view import AeroAnalysisMainView
from libs.views.cad_view import CADMainView
from libs.views.configuration_view import ConfigurationMainView
from libs.views.part_editor_view import PartEditorMainView
from libs.views.sitl_view import SITLMainView
from libs.views.vlm_view import VLMMainView

DEFAULT_PROJECT = PROJECTS_DIR / "example"
VIEW_OPTIONS = ["Longitudinal", "Front"]
UI_BG = "#000000"
UI_PANEL = "#031a03"
UI_BORDER = "#0a5f0a"
UI_FG = "#67ff67"
UI_DIM = "#2a7a2a"  # disabled fields
UI_LEVEL_COLORS = {"info": "#67ff67", "warning": "#ffd23f", "problem": "#ff4d4d"}  # libs/reportline.py LEVELS
UI_ACCENT = "#19ff19"
UI_CANVAS = "#001000"
UI_DRAW = "#2aff2a"
AERO_QUEUE_POLL_MS = 100


class BuilderApp:
    def __init__(self, root: tk.Tk, project: Project) -> None:
        self.root = root
        self.root.geometry("1600x980")
        self.root.configure(background=UI_BG)
        self.root.report_callback_exception = self.handle_callback_exception
        self.ui_bg = UI_BG
        self.ui_panel = UI_PANEL
        self.ui_border = UI_BORDER
        self.ui_fg = UI_FG
        self.ui_accent = UI_ACCENT
        self.ui_level_colors = UI_LEVEL_COLORS
        self.ui_canvas = UI_CANVAS
        self.ui_draw = UI_DRAW
        self.view_options = VIEW_OPTIONS
        self.parts_data = load_parts_data()
        self.project = project
        with open(project.build_path, "r") as file:
            self.definition = json.load(file)
        self.resolved = None
        self.problems: list[str] = []
        self.propulsion = None  # PropulsionSummary of the last valid powered airframe whose motor turns its prop
        self.aero = None
        self.asb_airplane = None
        self.aero_generation = 0
        self.aero_queue: queue.Queue = queue.Queue()
        self.recompute_status_var = tk.StringVar(value="")
        self.view_mode_var = tk.StringVar(value="Longitudinal")
        self.roll_view_var = tk.DoubleVar(value=0.0)
        self.roll_view_display_var = tk.StringVar(value="0.0 deg")
        self.view_canvas: tk.Canvas | None = None
        self.summary_text: tk.Text | None = None
        self.asb_frame: ttk.Frame | None = None
        self.asb_canvas = None
        self.asb_figure = None
        self.asb_draw_busy = False
        self.asb_last_size = (0, 0)
        self.configuration_view = None
        self.airfoil_view = None
        self.vlm_view = None
        self.sitl_view = None
        self.cad_view = None
        self.part_editor_view = None
        self.part_editor_panel = None
        self.configuration_main_view = None
        self.airfoil_main_view = None  # created on first show, with the airframe's airfoils selected
        self.vlm_main_view = None
        self.sitl_main_view = None
        self.cad_main_view = None
        self.main_view_host = None
        self.part_editor_main_view = None
        self.active_main_view = "Configuration"
        self.setup_theme()
        self.build_menu()
        self.build_layout()
        self.set_title()
        self.root.bind("<F5>", self.on_manual_recompute)
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.poll_after_id = self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_queue)
        self.recompute()
        self.configuration_main_view.rebuild_editors()

    def set_title(self) -> None:
        self.root.title(f"quadrocketbuilder - {self.project.name} ({self.definition['name']})")

    def recompute(self, run_aero: bool = True) -> None:
        """Validate and resolve the definition now; aero follows in a background thread unless run_aero is False."""
        self.aero_generation += 1
        try:
            airframe = parse_airframe(self.definition, self.parts_data, path=f"{self.project.name}/{BUILD_FILE}")
            resolved, problems = resolve_airframe(airframe, self.parts_data)
        except AirframeError as error:
            self.log_error("Definition error", f"{error}\nThe drawing shows the last valid airframe.")
            return
        self.resolved = resolved
        self.problems = problems
        self.propulsion = None
        if airframe.propulsion is not None:
            try:
                self.propulsion = propulsion_summary(resolved, self.parts_data)
            except MotorStalled as error:
                self.log_error("Propulsion", str(error))
        self.aero = None
        self.vlm_main_view.mark_stale()
        self.configuration_main_view.draw_active_view()
        if not run_aero:
            self.recompute_status_var.set("Aero pending")
            return
        self.recompute_status_var.set("Computing aero...")
        threading.Thread(target=self.run_aero, args=(self.aero_generation, resolved), daemon=True).start()

    def run_aero(self, generation: int, resolved) -> None:
        try:
            self.aero_queue.put((generation, analyze_aero(resolved), None, None))
        except OperatingPointError as error:  # show the reason, not a traceback
            self.aero_queue.put((generation, None, str(error), None))
        except Exception:  # thread boundary: hand the traceback to the Tk thread for the summary
            self.aero_queue.put((generation, None, None, traceback.format_exc()))

    def poll_aero_queue(self) -> None:
        while not self.aero_queue.empty():
            generation, aero, invalid_text, error_text = self.aero_queue.get()
            if generation != self.aero_generation:
                continue
            if invalid_text is not None:
                self.log_error("Aero not computed", invalid_text)
            elif error_text is not None:
                self.log_error("Aero failed", error_text)
            else:
                self.aero = aero
                self.write_summary(build_report_lines(self.resolved, aero, self.problems, self.propulsion, sweep=True))
                self.recompute_status_var.set("Up to date")
                self.configuration_main_view.draw_active_view()
        self.poll_after_id = self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_queue)

    def shutdown(self) -> None:
        self.root.after_cancel(self.poll_after_id)
        self.sitl_main_view.shutdown()
        self.root.destroy()

    def write_summary(self, lines: list[ReportLine]) -> None:
        """The report, each line coloured by its level."""
        self.summary_text.delete("1.0", tk.END)
        for line in lines:
            self.summary_text.insert(tk.END, line.text + "\n", line.level)

    def log_error(self, title: str, text: str) -> None:
        """Errors go to the Log tab, never a window or the panels."""
        self.recompute_status_var.set(f"{title} (see Log)")
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')}  {title}\n{text}\n\n", "problem")
        self.log_text.see(tk.END)
        self.left_notebook.select(self.log_tab)

    def on_manual_recompute(self, _event=None) -> str:
        """F5 recomputes whatever the active view shows."""
        if self.active_main_view == "Aero Analysis":
            self.airfoil_main_view.recompute_airfoils()
        elif self.active_main_view == "VLM":
            self.vlm_main_view.start_run()
        else:
            self.recompute()
        return "break"

    def update_asb_view(self) -> None:
        if self.resolved is None:
            return
        self.asb_airplane = build_asb_airplane(self.resolved)
        self.configuration_main_view.draw_asb_view(force=True)

    def reload_library_data(self) -> None:
        self.parts_data = load_parts_data()
        self.recompute()
        self.configuration_main_view.rebuild_editors()

    def handle_callback_exception(self, exc_type, exc_value, exc_traceback) -> None:
        traceback_text = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        self.log_error("Error", traceback_text)

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
        style.map("TEntry", foreground=[("disabled", UI_DIM)])
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
            foreground=[("disabled", UI_DIM), ("readonly", UI_FG)],
            selectbackground=[("readonly", UI_PANEL)],
            selectforeground=[("disabled", UI_DIM), ("readonly", UI_FG)],
            arrowcolor=[("disabled", UI_DIM)],
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
        file_menu.add_command(label="Load", command=self.load_project)
        file_menu.add_command(label="Save", command=self.save_project)
        file_menu.add_command(label="Save As", command=self.save_project_as)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.shutdown)
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
        self.main_view_host = main_view_host

        self.configuration_main_view = ConfigurationMainView(self, main_view_host)
        self.vlm_main_view = VLMMainView(self, main_view_host)
        self.sitl_main_view = SITLMainView(self, main_view_host)
        self.cad_main_view = CADMainView(self, main_view_host)
        self.part_editor_main_view = PartEditorMainView(self, main_view_host)

        bottom_bar = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        bottom_bar.grid(row=1, column=0, sticky="ew")
        ttk.Button(bottom_bar, text="Configuration", command=lambda: self.show_main_view("Configuration")).pack(side="left")
        ttk.Button(bottom_bar, text="Aero Analysis", command=lambda: self.show_main_view("Aero Analysis")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="VLM", command=lambda: self.show_main_view("VLM")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="SITL", command=lambda: self.show_main_view("SITL")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="CAD", command=lambda: self.show_main_view("CAD")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Part Editor", command=lambda: self.show_main_view("Part Editor")).pack(side="left", padx=(8, 0))
        self.show_main_view("Configuration")

    def show_main_view(self, view_name: str) -> None:
        self.active_main_view = view_name
        if view_name == "Aero Analysis":
            if self.airfoil_main_view is None:
                self.airfoil_main_view = AeroAnalysisMainView(self, self.main_view_host)
            self.airfoil_view.tkraise()
            return
        if view_name == "VLM":
            self.vlm_view.tkraise()
            return
        if view_name == "SITL":
            self.sitl_main_view.refresh()
            self.sitl_view.tkraise()
            return
        if view_name == "CAD":
            self.cad_view.tkraise()
            return
        if view_name == "Part Editor":
            self.part_editor_panel.refresh_from_disk()
            self.part_editor_view.tkraise()
            return
        self.configuration_view.tkraise()

    def save_project(self) -> None:
        with open(self.project.build_path, "w") as file:
            json.dump(self.definition, file, indent=4)
        self.set_title()

    def save_project_as(self) -> None:
        """Copy the whole project under a new name, then save the build file there and work on the copy."""
        name = simpledialog.askstring("Save Project As", "New project name:", parent=self.root)
        if not name:
            return
        if Path(name).name != name:
            self.log_error("Save Project As", f"'{name}' is not a plain directory name.")
            return
        try:
            project = copy_project(self.project, PROJECTS_DIR / name)
        except ProjectError as error:
            self.log_error("Save Project As", str(error))
            return
        self.project = project
        self.save_project()
        self.sitl_main_view.refresh()

    def load_project(self) -> None:
        path = filedialog.askdirectory(parent=self.root, title="Load Project", initialdir=str(PROJECTS_DIR), mustexist=True)
        if not path:
            return
        try:
            project = open_project(Path(path))
        except ProjectError as error:
            self.log_error("Load Project", str(error))
            return
        with open(project.build_path, "r") as file:
            data = json.load(file)
        if data.get("schema") != SCHEMA:
            self.log_error("Load Project", f"{project.build_path} is not an {SCHEMA} file (old module-stack configs are not supported).")
            return
        self.definition = data
        self.project = project
        self.set_title()
        self.recompute()
        self.configuration_main_view.rebuild_editors()
        self.sitl_main_view.refresh()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plane-first airframe builder")
    parser.add_argument("project", type=Path, nargs="?", default=DEFAULT_PROJECT, help="aircraft project directory (aircraft/<name>)")
    args = parser.parse_args()
    project = open_project(args.project)
    root = tk.Tk()
    BuilderApp(root, project)
    root.mainloop()


if __name__ == "__main__":
    main()
