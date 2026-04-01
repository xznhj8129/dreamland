from __future__ import annotations

from tkinter import ttk

from libs.library_editors import ModuleEditorFrame


class ModuleEditorMainView:
    def __init__(self, app, parent) -> None:
        self.frame = app.module_editor_view = ttk.Frame(parent, padding=8)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.module_editor_panel = ModuleEditorFrame(self.frame, on_save=app.reload_library_data)
        app.module_editor_panel.grid(row=0, column=0, sticky="nsew")
