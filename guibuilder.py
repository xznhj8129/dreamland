"""
Usage:
python guibuilder.py
python guibuilder.py aircraft/quadlancet62.json

Tk front end for plane-first airframes (airframe/2 files, see PLAN.md): edit the aircraft definition and see
the drawing, mass/CG, aerodynamics and stability update. Geometry and mass update immediately; aero runs in
the background and replaces the summary when it finishes.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from libs.aero import analyze_aero, build_asb_airplane
from libs.airframe import PARTS_PATH, SCHEMA, AirframeError, parse_airframe
from libs.geometry import resolve_airframe
from libs.report import build_report_text
from libs.simlib2 import TestedMotorData
from libs.views.cad_view import CADMainView
from libs.views.configuration_view import ConfigurationMainView
from libs.views.part_editor_view import PartEditorMainView

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_AIRFRAME_PATH = SCRIPT_DIR / "aircraft" / "m62v2.json"
VIEW_OPTIONS = ["Longitudinal", "Front"]
UI_BG = "#000000"
UI_PANEL = "#031a03"
UI_BORDER = "#0a5f0a"
UI_FG = "#67ff67"
UI_ACCENT = "#19ff19"
UI_CANVAS = "#001000"
UI_DRAW = "#2aff2a"
AERO_QUEUE_POLL_MS = 100


def load_parts_data() -> dict:
    with open(PARTS_PATH, "r") as file:
        return json.load(file)


class BuilderApp:
    def __init__(self, root: tk.Tk, airframe_path: Path) -> None:
        self.root = root
        self.root.geometry("1600x980")
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
        self.parts_data = load_parts_data()
        self.motor_options = TestedMotorData.available_motors()
        self.airframe_path = airframe_path
        with open(airframe_path, "r") as file:
            self.definition = json.load(file)
        self.resolved = None
        self.problems: list[str] = []
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
        self.cad_view = None
        self.part_editor_view = None
        self.part_editor_panel = None
        self.configuration_main_view = None
        self.cad_main_view = None
        self.part_editor_main_view = None
        self.active_main_view = "Configuration"
        self.error_dialog: tk.Toplevel | None = None
        self.setup_theme()
        self.build_menu()
        self.build_layout()
        self.set_title()
        self.root.bind("<F5>", self.on_manual_recompute)
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.poll_after_id = self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_queue)
        self.configuration_main_view.rebuild_editors()
        self.recompute()

    def set_title(self) -> None:
        self.root.title(f"quadrocketbuilder - {self.airframe_path.name}")

    def recompute(self, run_aero: bool = True) -> None:
        """Validate and resolve the definition now; aero follows in a background thread unless run_aero is False."""
        self.aero_generation += 1
        try:
            airframe = parse_airframe(self.definition, self.parts_data, path=self.airframe_path.name)
            resolved, problems = resolve_airframe(airframe, self.parts_data)
        except AirframeError as error:
            self.recompute_status_var.set("Definition error")
            self.write_summary(f"Definition error:\n\n{error}\n\nThe drawing shows the last valid airframe.")
            return
        self.resolved = resolved
        self.problems = problems
        self.aero = None
        self.configuration_main_view.draw_active_view()
        if not run_aero:
            self.recompute_status_var.set("Aero pending")
            return
        self.recompute_status_var.set("Computing aero...")
        threading.Thread(target=self.run_aero, args=(self.aero_generation, resolved), daemon=True).start()

    def run_aero(self, generation: int, resolved) -> None:
        try:
            self.aero_queue.put((generation, resolved, analyze_aero(resolved), None))
        except Exception:  # thread boundary: hand the traceback to the Tk thread for the error dialog
            self.aero_queue.put((generation, resolved, None, traceback.format_exc()))

    def poll_aero_queue(self) -> None:
        while not self.aero_queue.empty():
            generation, resolved, aero, error_text = self.aero_queue.get()
            if error_text is not None:
                self.recompute_status_var.set("Aero failed")
                self.show_error_dialog(error_text)
                continue
            if generation != self.aero_generation:
                continue
            self.aero = aero
            self.write_summary(build_report_text(resolved, aero, self.problems, sweep=True))
            self.recompute_status_var.set("Up to date")
            self.configuration_main_view.draw_active_view()
        self.poll_after_id = self.root.after(AERO_QUEUE_POLL_MS, self.poll_aero_queue)

    def shutdown(self) -> None:
        self.root.after_cancel(self.poll_after_id)
        self.root.destroy()

    def write_summary(self, text: str) -> None:
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", text)

    def on_manual_recompute(self, _event=None) -> str:
        self.recompute()
        return "break"

    def update_asb_view(self) -> None:
        if self.resolved is None:
            return
        self.asb_airplane = build_asb_airplane(self.resolved)
        self.configuration_main_view.draw_asb_view(force=True)

    def reload_library_data(self) -> None:
        self.parts_data = load_parts_data()
        self.configuration_main_view.rebuild_editors()
        self.recompute()

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
        file_menu.add_command(label="Save As", command=self.save_json_as)
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

        self.configuration_main_view = ConfigurationMainView(self, main_view_host)
        self.cad_main_view = CADMainView(self, main_view_host)
        self.part_editor_main_view = PartEditorMainView(self, main_view_host)

        bottom_bar = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        bottom_bar.grid(row=1, column=0, sticky="ew")
        ttk.Button(bottom_bar, text="Configuration", command=lambda: self.show_main_view("Configuration")).pack(side="left")
        ttk.Button(bottom_bar, text="CAD", command=lambda: self.show_main_view("CAD")).pack(side="left", padx=(8, 0))
        ttk.Button(bottom_bar, text="Part Editor", command=lambda: self.show_main_view("Part Editor")).pack(side="left", padx=(8, 0))
        self.show_main_view("Configuration")

    def show_main_view(self, view_name: str) -> None:
        self.active_main_view = view_name
        if view_name == "CAD":
            self.cad_view.tkraise()
            return
        if view_name == "Part Editor":
            self.part_editor_panel.refresh_from_disk()
            self.part_editor_view.tkraise()
            return
        self.configuration_view.tkraise()

    def write_definition(self, path: Path) -> None:
        with open(path, "w") as file:
            json.dump(self.definition, file, indent=4)
        self.airframe_path = path
        self.set_title()

    def save_json(self) -> None:
        self.write_definition(self.airframe_path)

    def save_json_as(self) -> None:
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save Airframe",
            initialdir=str(self.airframe_path.parent),
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
        )
        if path:
            self.write_definition(Path(path))

    def load_json(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="Load Airframe",
            initialdir=str(self.airframe_path.parent),
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        with open(path, "r") as file:
            data = json.load(file)
        if data.get("schema") != SCHEMA:
            messagebox.showerror("Load Airframe", f"{Path(path).name} is not an {SCHEMA} file (old module-stack configs are not supported).", parent=self.root)
            return
        self.definition = data
        self.airframe_path = Path(path)
        self.set_title()
        self.configuration_main_view.rebuild_editors()
        self.recompute()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plane-first airframe builder")
    parser.add_argument("airframe", type=Path, nargs="?", default=DEFAULT_AIRFRAME_PATH, help="airframe/2 JSON file")
    args = parser.parse_args()
    root = tk.Tk()
    BuilderApp(root, args.airframe)
    root.mainloop()


if __name__ == "__main__":
    main()
