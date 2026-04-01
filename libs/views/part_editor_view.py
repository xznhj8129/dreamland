from __future__ import annotations

from tkinter import ttk

from libs.library_editors import PartEditorFrame


class PartEditorMainView:
    def __init__(self, app, parent) -> None:
        self.frame = app.part_editor_view = ttk.Frame(parent, padding=8)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.part_editor_panel = PartEditorFrame(self.frame, on_save=app.reload_library_data)
        app.part_editor_panel.grid(row=0, column=0, sticky="nsew")
