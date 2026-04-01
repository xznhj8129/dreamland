from __future__ import annotations

from tkinter import ttk


class CADMainView:
    def __init__(self, app, parent) -> None:
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.cad_view = self.frame
        cad_placeholder = ttk.LabelFrame(self.frame, text="CAD", padding=12)
        cad_placeholder.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        cad_placeholder.columnconfigure(0, weight=1)
        cad_placeholder.rowconfigure(0, weight=1)
        ttk.Label(
            cad_placeholder,
            text="CAD tools will live here.",
            anchor="center",
            justify="center",
        ).grid(row=0, column=0, sticky="nsew")

