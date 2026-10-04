"""
Usage:
from libs.views.cad_view import CADMainView

CAD view: STEP/STL solid export and FreeCAD varset (AirframeParams) export of the current resolved airframe.
"""

from __future__ import annotations

import json
import traceback
import tkinter as tk
from tkinter import filedialog, ttk

from libs.cad import build_freecad_exports, export_step, export_stl


STEP_MODE_LABELS = {
    "assembly": "Component assembly (named parts, for CAD)",
    "fused": "Fused OML (single watertight solid, for CFD)",
}


class CADMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        app.cad_view = self.frame

        export_frame = ttk.LabelFrame(self.frame, text="Solid Geometry Export", padding=12)
        export_frame.grid(row=0, column=0, sticky="new", padx=8, pady=8)
        export_frame.columnconfigure(0, weight=1)

        self.step_mode_var = tk.StringVar(value="assembly")
        for row, mode in enumerate(("assembly", "fused")):
            ttk.Radiobutton(
                export_frame,
                text=STEP_MODE_LABELS[mode],
                variable=self.step_mode_var,
                value=mode,
            ).grid(row=row, column=0, sticky="w")

        self.prop_discs_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            export_frame,
            text="Include propeller discs (actuator-disc CFD markers)",
            variable=self.prop_discs_var,
        ).grid(row=2, column=0, sticky="w", pady=(4, 8))

        button_bar = ttk.Frame(export_frame)
        button_bar.grid(row=3, column=0, sticky="w")
        ttk.Button(button_bar, text="Export STEP", command=self.on_export_step).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(button_bar, text="Export STL", command=self.on_export_stl).grid(row=0, column=1, padx=(0, 8))
        ttk.Button(button_bar, text="Export FreeCAD params (JSON)", command=self.on_export_freecad).grid(row=0, column=2)

        self.status_var = tk.StringVar(value="")
        ttk.Label(export_frame, textvariable=self.status_var, wraplength=520, justify="left").grid(
            row=4, column=0, sticky="w", pady=(8, 0)
        )

    def export_dir(self) -> str:
        """The project's cad/ directory, the default place for exports."""
        cad_dir = self.app.project.cad_dir
        cad_dir.mkdir(parents=True, exist_ok=True)
        return str(cad_dir)

    def on_export_step(self) -> None:
        if self.app.resolved is None:
            self.status_var.set("No valid airframe yet; fix the definition first.")
            return
        path = filedialog.asksaveasfilename(
            title="Export STEP",
            defaultextension=".step",
            initialdir=self.export_dir(),
            initialfile=f"{self.app.project.name}.step",
            filetypes=[("STEP files", "*.step *.stp"), ("All files", "*.*")],
        )
        if not path:
            return
        mode = self.step_mode_var.get()
        self.status_var.set(f"Exporting STEP ({mode})...")
        self.frame.update_idletasks()
        try:
            written_mode = export_step(
                self.app.resolved, path, mode=mode, include_prop_discs=self.prop_discs_var.get()
            )
        except Exception as error:
            traceback.print_exc()
            self.status_var.set(f"STEP export failed: {error}")
            return
        note = " (fuse failed, wrote unfused compound)" if written_mode == "compound" else ""
        self.status_var.set(f"STEP written ({written_mode}){note}: {path}")

    def on_export_stl(self) -> None:
        if self.app.resolved is None:
            self.status_var.set("No valid airframe yet; fix the definition first.")
            return
        path = filedialog.asksaveasfilename(
            title="Export STL",
            defaultextension=".stl",
            initialdir=self.export_dir(),
            initialfile=f"{self.app.project.name}.stl",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")],
        )
        if not path:
            return
        self.status_var.set("Exporting STL...")
        self.frame.update_idletasks()
        try:
            written_mode = export_stl(self.app.resolved, path, include_prop_discs=self.prop_discs_var.get())
        except Exception as error:
            traceback.print_exc()
            self.status_var.set(f"STL export failed: {error}")
            return
        note = " (fuse failed, wrote unfused compound)" if written_mode == "compound" else ""
        self.status_var.set(f"STL written{note}: {path}")

    def on_export_freecad(self) -> None:
        if self.app.resolved is None:
            self.status_var.set("No valid airframe yet; fix the definition first.")
            return
        path = filedialog.asksaveasfilename(
            title="Export FreeCAD params",
            defaultextension=".json",
            initialdir=self.export_dir(),
            initialfile=f"{self.app.project.name}_freecad.json",
            filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w") as file:
            json.dump(build_freecad_exports(self.app.resolved), file, indent=4)
        self.status_var.set(f"FreeCAD params written: {path}")
