from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from libs.graphics import draw_canvas_view


class ConfigurationMainView:
    def __init__(self, app, parent) -> None:
        self.app = app
        self.collapsed_part_ids = set()
        self.view_zoom_scale = 1.0
        self.view_pan_x = 0.0
        self.view_pan_y = 0.0
        self.view_drag_last_xy = None
        self.view_metrics = None
        self.view_scroll_x = None
        self.view_scroll_y = None
        self.frame = ttk.Frame(parent)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)
        app.configuration_view = self.frame

        pane_window = ttk.Panedwindow(self.frame, orient=tk.HORIZONTAL)
        pane_window.grid(row=0, column=0, sticky="nsew")

        left_frame = ttk.Frame(pane_window, padding=8)
        center_frame = ttk.Frame(pane_window, padding=8)
        right_frame = ttk.Frame(pane_window, padding=8)
        pane_window.add(left_frame, weight=1)
        pane_window.add(center_frame, weight=2)
        pane_window.add(right_frame, weight=1)

        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=1)
        center_frame.columnconfigure(0, weight=1)
        center_frame.rowconfigure(0, weight=1)
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(1, weight=1)

        left_header = ttk.LabelFrame(left_frame, text="Recompute", padding=6)
        left_header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        left_header.columnconfigure(1, weight=1)
        ttk.Button(left_header, text="Recompute (F5)", command=app.heavy_recompute).grid(row=0, column=0, sticky="w")
        ttk.Label(left_header, textvariable=app.recompute_status_var).grid(row=0, column=1, sticky="e", padx=(8, 0))

        left_notebook = ttk.Notebook(left_frame)
        left_notebook.grid(row=1, column=0, sticky="nsew")
        summary_tab = ttk.Frame(left_notebook, padding=4)
        controls_tab = ttk.Frame(left_notebook, padding=4)
        summary_tab.columnconfigure(0, weight=1)
        summary_tab.rowconfigure(0, weight=1)
        controls_tab.columnconfigure(0, weight=1)
        left_notebook.add(summary_tab, text="Summary")
        left_notebook.add(controls_tab, text="Controls")

        summary_frame = ttk.LabelFrame(summary_tab, text="Summary", padding=4)
        summary_frame.grid(row=0, column=0, sticky="nsew")
        summary_frame.columnconfigure(0, weight=1)
        summary_frame.rowconfigure(0, weight=1)
        app.summary_text = tk.Text(
            summary_frame,
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
        summary_scroll_y = ttk.Scrollbar(summary_frame, orient="vertical", command=app.summary_text.yview)
        summary_scroll_x = ttk.Scrollbar(summary_frame, orient="horizontal", command=app.summary_text.xview)
        app.summary_text.configure(yscrollcommand=summary_scroll_y.set, xscrollcommand=summary_scroll_x.set)
        app.summary_text.grid(row=0, column=0, sticky="nsew")
        summary_scroll_y.grid(row=0, column=1, sticky="ns")
        summary_scroll_x.grid(row=1, column=0, sticky="ew")

        controls_frame = ttk.LabelFrame(controls_tab, text="Controls", padding=6)
        controls_frame.grid(row=0, column=0, sticky="new")
        controls_frame.columnconfigure(0, weight=1)
        operating_point_frame = ttk.LabelFrame(controls_frame, text="Surface Force Operating Point", padding=6)
        operating_point_frame.grid(row=0, column=0, sticky="ew")
        operating_point_frame.columnconfigure(1, weight=1)
        for row_index, key in enumerate(app.config_force_settings):
            ttk.Label(operating_point_frame, text=app.config_force_labels[key]).grid(row=row_index, column=0, sticky="w")
            entry = ttk.Entry(operating_point_frame, textvariable=app.config_force_vars[key])
            entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            app.bind_value_entry(entry, app.config_force_settings, key, app.config_force_vars[key], "config_force")
        ttk.Label(
            controls_frame,
            text="These values drive the Surface Forces block in Summary.",
        ).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(controls_frame, text="Cheap aero sweep is always recomputed from -45 to 45 deg.").grid(row=2, column=0, sticky="w", pady=(8, 0))

        center_notebook = ttk.Notebook(center_frame)
        center_notebook.grid(row=0, column=0, sticky="nsew")
        simple_tab = ttk.Frame(center_notebook, padding=4)
        asb_tab = ttk.Frame(center_notebook, padding=4)
        simple_tab.columnconfigure(0, weight=1)
        simple_tab.rowconfigure(1, weight=1)
        asb_tab.columnconfigure(0, weight=1)
        asb_tab.rowconfigure(0, weight=1)
        center_notebook.add(simple_tab, text="Simple")
        center_notebook.add(asb_tab, text="ASB")

        graphics_controls = ttk.LabelFrame(simple_tab, text="Graphics", padding=6)
        graphics_controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        graphics_controls.columnconfigure(0, weight=1)
        ttk.Label(graphics_controls, text="X = Front   Y = Right   Z = Up").grid(row=0, column=0, sticky="w")
        button_bar = ttk.Frame(graphics_controls)
        button_bar.grid(row=1, column=0, sticky="w", pady=(6, 0))
        for view_name in app.view_options:
            ttk.Radiobutton(
                button_bar,
                text=view_name,
                value=view_name,
                variable=app.view_mode_var,
                command=self.draw_active_view,
            ).pack(side="left", padx=(0, 8))
        ttk.Button(button_bar, text="Reset View", command=self.reset_view_transform).pack(side="left", padx=(8, 0))
        simple_view_frame = ttk.Frame(simple_tab)
        simple_view_frame.grid(row=1, column=0, sticky="nsew")
        simple_view_frame.columnconfigure(0, weight=1)
        simple_view_frame.rowconfigure(0, weight=1)

        app.view_canvas = self.make_view_canvas(simple_view_frame, "Orthographic View", 0)

        app.roll_frame = ttk.LabelFrame(simple_view_frame, text="Roll", padding=4)
        app.roll_frame.grid(row=0, column=1, sticky="ns", padx=(8, 0))
        app.roll_frame.rowconfigure(1, weight=1)
        ttk.Label(app.roll_frame, text="90 top").grid(row=0, column=0, sticky="n")
        roll_slider = tk.Scale(
            app.roll_frame,
            from_=90.0,
            to=0.0,
            orient="vertical",
            resolution=1.0,
            variable=app.roll_view_var,
            showvalue=False,
            command=self.on_roll_slider_changed,
            background=app.ui_bg,
            foreground=app.ui_fg,
            troughcolor=app.ui_panel,
            activebackground=app.ui_border,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            bd=0,
            relief="flat",
            length=420,
        )
        roll_slider.grid(row=1, column=0, sticky="ns", pady=(6, 6))
        roll_slider.configure(command=self.on_roll_slider_changed)
        ttk.Label(app.roll_frame, text="0 side").grid(row=2, column=0, sticky="s")
        ttk.Label(app.roll_frame, textvariable=app.roll_view_display_var).grid(row=3, column=0, sticky="s", pady=(6, 0))

        app.asb_frame = ttk.LabelFrame(asb_tab, text="ASB View", padding=6)
        app.asb_frame.grid(row=0, column=0, sticky="nsew")
        app.asb_frame.columnconfigure(0, weight=1)
        app.asb_frame.rowconfigure(1, weight=1)
        asb_controls = ttk.Frame(app.asb_frame)
        asb_controls.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(asb_controls, text="Update View", command=app.heavy_recompute).pack(side="left")

        notebook = ttk.Notebook(right_frame)
        notebook.grid(row=0, column=0, rowspan=2, sticky="nsew")
        parameters_tab = ttk.Frame(notebook, padding=4)
        propulsion_tab = ttk.Frame(notebook, padding=4)
        assembly_tab = ttk.Frame(notebook, padding=4)
        parameters_tab.columnconfigure(0, weight=1)
        propulsion_tab.columnconfigure(0, weight=1)
        propulsion_tab.rowconfigure(1, weight=1)
        assembly_tab.columnconfigure(0, weight=1)
        assembly_tab.rowconfigure(1, weight=1)
        notebook.add(parameters_tab, text="Parameters")
        notebook.add(propulsion_tab, text="Propulsion")
        notebook.add(assembly_tab, text="Assembly")

        modules_header = ttk.LabelFrame(parameters_tab, text="Parameters", padding=6)
        modules_header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        modules_header.columnconfigure(1, weight=1)
        ttk.Label(modules_header, text=app.main_driver_labels["assembly_name"]).grid(row=0, column=0, sticky="w")
        assembly_entry = ttk.Entry(modules_header, textvariable=app.assembly_name_var)
        assembly_entry.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
        assembly_entry.bind("<Return>", lambda event: app.on_assembly_name_commit())
        assembly_entry.bind("<KP_Enter>", lambda event: app.on_assembly_name_commit())
        assembly_entry.bind("<FocusOut>", lambda event: app.on_assembly_name_commit())
        ttk.Label(modules_header, text="Standard Profile").grid(row=1, column=0, sticky="w")
        standard_box = ttk.Combobox(modules_header, values=[app.standard_none_option, *app.standard_names], textvariable=app.standard_profile_var, state="readonly")
        standard_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
        standard_box.bind("<<ComboboxSelected>>", lambda event: app.on_standard_profile_selected())
        apply_standard_box = ttk.Checkbutton(modules_header, text="Apply", variable=app.apply_standard_var, command=app.on_apply_standard_changed)
        apply_standard_box.grid(row=1, column=2, sticky="w", padx=(8, 0), pady=2)
        for row_index, key in enumerate(app.main_driver_keys, start=2):
            ttk.Label(modules_header, text=app.main_driver_labels[key]).grid(row=row_index, column=0, sticky="w")
            variable = tk.StringVar(value=app.format_field_value(key, app.current_config[key]))
            entry = ttk.Entry(modules_header, textvariable=variable)
            entry.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            app.bind_value_entry(entry, app.current_config, key, variable, "config")

        app.assembly_inner = ttk.Frame(parameters_tab)
        app.assembly_inner.grid(row=1, column=0, sticky="nsew")

        propulsion_select_frame = ttk.LabelFrame(propulsion_tab, text="Selection", padding=6)
        propulsion_select_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        propulsion_select_frame.columnconfigure(1, weight=1)
        ttk.Label(propulsion_select_frame, text=app.propulsion_labels["battery"]).grid(row=0, column=0, sticky="w")
        battery_box = ttk.Combobox(propulsion_select_frame, values=sorted(app.parts_data["battery"].keys()), textvariable=app.propulsion_battery_var, state="readonly")
        battery_box.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=2)
        battery_box.bind("<<ComboboxSelected>>", lambda event: app.on_propulsion_battery_selected())
        ttk.Label(propulsion_select_frame, text=app.propulsion_labels["motors"]).grid(row=1, column=0, sticky="w")
        motor_box = ttk.Combobox(propulsion_select_frame, values=app.motor_names, textvariable=app.propulsion_motor_var, state="readonly")
        motor_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
        motor_box.bind("<<ComboboxSelected>>", lambda event: app.on_propulsion_motor_selected())
        ttk.Label(propulsion_select_frame, text=app.propulsion_labels["props"]).grid(row=2, column=0, sticky="w")
        app.propulsion_prop_box = ttk.Combobox(propulsion_select_frame, values=[], textvariable=app.propulsion_prop_var, state="readonly")
        app.propulsion_prop_box.grid(row=2, column=1, sticky="ew", padx=(8, 0), pady=2)
        app.propulsion_prop_box.bind("<<ComboboxSelected>>", lambda event: app.on_propulsion_prop_selected())

        propulsion_info_frame = ttk.LabelFrame(propulsion_tab, text="Motor Info", padding=6)
        propulsion_info_frame.grid(row=1, column=0, sticky="nsew")
        propulsion_info_frame.columnconfigure(0, weight=1)
        propulsion_info_frame.rowconfigure(0, weight=1)
        propulsion_info_frame.rowconfigure(1, weight=1)
        app.propulsion_plot_host = ttk.Frame(propulsion_info_frame)
        app.propulsion_plot_host.grid(row=0, column=0, sticky="nsew")
        app.propulsion_plot_host.columnconfigure(0, weight=1)
        app.propulsion_plot_host.rowconfigure(0, weight=1)
        propulsion_text_frame = ttk.Frame(propulsion_info_frame)
        propulsion_text_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        propulsion_text_frame.columnconfigure(0, weight=1)
        propulsion_text_frame.rowconfigure(0, weight=1)
        app.propulsion_text = tk.Text(
            propulsion_text_frame,
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
            height=12,
        )
        propulsion_text_scroll_y = ttk.Scrollbar(propulsion_text_frame, orient="vertical", command=app.propulsion_text.yview)
        app.propulsion_text.configure(yscrollcommand=propulsion_text_scroll_y.set)
        app.propulsion_text.grid(row=0, column=0, sticky="nsew")
        propulsion_text_scroll_y.grid(row=0, column=1, sticky="ns")

        assembly_controls = ttk.Frame(assembly_tab)
        assembly_controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(assembly_controls, text="Add New", command=app.add_new_part).pack(side="left")

        module_scroll_frame = ttk.Frame(assembly_tab)
        module_scroll_frame.grid(row=1, column=0, sticky="nsew")
        module_scroll_frame.columnconfigure(0, weight=1)
        module_scroll_frame.rowconfigure(0, weight=1)
        app.module_canvas = tk.Canvas(
            module_scroll_frame,
            background=app.ui_bg,
            highlightthickness=1,
            highlightbackground=app.ui_border,
            highlightcolor=app.ui_accent,
            relief="flat",
        )
        module_scroll_y = ttk.Scrollbar(module_scroll_frame, orient="vertical", command=app.module_canvas.yview)
        app.module_inner = ttk.Frame(app.module_canvas)
        app.module_inner.bind("<Configure>", lambda event: app.module_canvas.configure(scrollregion=app.module_canvas.bbox("all")))
        app.module_window_id = app.module_canvas.create_window((0, 0), window=app.module_inner, anchor="nw")
        app.module_canvas.configure(yscrollcommand=module_scroll_y.set)
        app.module_canvas.grid(row=0, column=0, sticky="nsew")
        module_scroll_y.grid(row=0, column=1, sticky="ns")
        app.module_canvas.bind("<Configure>", lambda event: app.module_canvas.itemconfigure(app.module_window_id, width=event.width))

    def on_roll_slider_changed(self, _value: str) -> None:
        self.app.roll_view_display_var.set(f"{self.app.roll_view_var.get():.1f} deg")
        self.draw_active_view()

    def reset_view_transform(self) -> None:
        self.view_zoom_scale = 1.0
        self.view_pan_x = 0.0
        self.view_pan_y = 0.0
        self.draw_active_view()

    def clamp_view_pan(self) -> bool:
        if self.view_metrics is None:
            return False
        changed = False
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            min_pan_x = min(0.0, available_x - metrics["content_width"])
            max_pan_x = 0.0
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                min_pan_y = 0.0
                max_pan_y = 0.0
            else:
                min_pan_y = -(content_height - available_y) / 2.0
                max_pan_y = (content_height - available_y) / 2.0
        else:
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            content_width = 2.0 * metrics["content_half_width"]
            if content_width <= available_x:
                min_pan_x = 0.0
                max_pan_x = 0.0
            else:
                min_pan_x = -(content_width - available_x) / 2.0
                max_pan_x = (content_width - available_x) / 2.0
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                min_pan_y = 0.0
                max_pan_y = 0.0
            else:
                min_pan_y = -(content_height - available_y) / 2.0
                max_pan_y = (content_height - available_y) / 2.0
        clamped_pan_x = min(max(self.view_pan_x, min_pan_x), max_pan_x)
        clamped_pan_y = min(max(self.view_pan_y, min_pan_y), max_pan_y)
        if clamped_pan_x != self.view_pan_x:
            self.view_pan_x = clamped_pan_x
            changed = True
        if clamped_pan_y != self.view_pan_y:
            self.view_pan_y = clamped_pan_y
            changed = True
        return changed

    def update_view_scrollbars(self) -> None:
        if self.view_metrics is None or self.view_scroll_x is None or self.view_scroll_y is None:
            return
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if metrics["content_width"] <= available_x:
                self.view_scroll_x.set(0.0, 1.0)
            else:
                first_x = -self.view_pan_x / metrics["content_width"]
                last_x = first_x + available_x / metrics["content_width"]
                self.view_scroll_x.set(first_x, last_x)
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                self.view_scroll_y.set(0.0, 1.0)
            else:
                first_y = ((content_height - available_y) / 2.0 - self.view_pan_y) / content_height
                last_y = first_y + available_y / content_height
                self.view_scroll_y.set(first_y, last_y)
            return
        available_x = metrics["width"] - 2.0 * metrics["pad_x"]
        content_width = 2.0 * metrics["content_half_width"]
        if content_width <= available_x:
            self.view_scroll_x.set(0.0, 1.0)
        else:
            first_x = ((content_width - available_x) / 2.0 - self.view_pan_x) / content_width
            last_x = first_x + available_x / content_width
            self.view_scroll_x.set(first_x, last_x)
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            self.view_scroll_y.set(0.0, 1.0)
        else:
            first_y = ((content_height - available_y) / 2.0 - self.view_pan_y) / content_height
            last_y = first_y + available_y / content_height
            self.view_scroll_y.set(first_y, last_y)

    def on_view_xscroll(self, *args) -> None:
        if self.view_metrics is None:
            return
        metrics = self.view_metrics
        if metrics["mode"] == "Longitudinal":
            content_width = metrics["content_width"]
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if content_width <= available_x:
                return
            if args[0] == "moveto":
                self.view_pan_x = -float(args[1]) * content_width
            else:
                step = 40.0 if args[2] == "units" else available_x * 0.9
                self.view_pan_x -= int(args[1]) * step
        else:
            content_width = 2.0 * metrics["content_half_width"]
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if content_width <= available_x:
                return
            if args[0] == "moveto":
                self.view_pan_x = (content_width - available_x) / 2.0 - float(args[1]) * content_width
            else:
                step = 40.0 if args[2] == "units" else available_x * 0.9
                self.view_pan_x -= int(args[1]) * step
        self.draw_active_view()

    def on_view_yscroll(self, *args) -> None:
        if self.view_metrics is None:
            return
        metrics = self.view_metrics
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            return
        if args[0] == "moveto":
            self.view_pan_y = (content_height - available_y) / 2.0 - float(args[1]) * content_height
        else:
            step = 40.0 if args[2] == "units" else available_y * 0.9
            self.view_pan_y -= int(args[1]) * step
        self.draw_active_view()

    def on_view_drag_start(self, event) -> None:
        self.view_drag_last_xy = (event.x, event.y)

    def on_view_drag_motion(self, event) -> None:
        if self.view_drag_last_xy is None:
            return
        last_x, last_y = self.view_drag_last_xy
        self.view_pan_x += event.x - last_x
        self.view_pan_y += event.y - last_y
        self.view_drag_last_xy = (event.x, event.y)
        self.draw_active_view()

    def on_view_drag_end(self, _event) -> None:
        self.view_drag_last_xy = None

    def zoom_view_at(self, event_x: float, event_y: float, zoom_factor: float) -> None:
        if self.view_metrics is None:
            return
        new_zoom_scale = min(max(self.view_zoom_scale * zoom_factor, 1.0), 25.0)
        if new_zoom_scale == self.view_zoom_scale:
            return
        zoom_ratio = new_zoom_scale / self.view_zoom_scale
        self.view_pan_x = event_x - self.view_metrics["origin_x"] - zoom_ratio * (event_x - self.view_metrics["origin_x"] - self.view_pan_x)
        self.view_pan_y = event_y - self.view_metrics["origin_y"] - zoom_ratio * (event_y - self.view_metrics["origin_y"] - self.view_pan_y)
        self.view_zoom_scale = new_zoom_scale
        self.draw_active_view()

    def on_view_mousewheel(self, event) -> None:
        zoom_factor = 1.1 if event.delta > 0 else 1.0 / 1.1
        self.zoom_view_at(event.x, event.y, zoom_factor)

    def on_view_wheel_up(self, event) -> None:
        self.zoom_view_at(event.x, event.y, 1.1)

    def on_view_wheel_down(self, event) -> None:
        self.zoom_view_at(event.x, event.y, 1.0 / 1.1)

    def draw_active_view(self) -> None:
        app = self.app
        if app.analysis is None:
            return
        if app.view_mode_var.get() == "Front":
            app.roll_frame.grid_remove()
        else:
            app.roll_frame.grid()
        self.view_metrics = draw_canvas_view(
            app.view_canvas,
            app.analysis,
            app.view_mode_var.get(),
            app.roll_view_var.get(),
            app.ui_draw,
            app.ui_bg,
            self.view_zoom_scale,
            self.view_pan_x,
            self.view_pan_y,
        )
        if self.clamp_view_pan():
            self.view_metrics = draw_canvas_view(
                app.view_canvas,
                app.analysis,
                app.view_mode_var.get(),
                app.roll_view_var.get(),
                app.ui_draw,
                app.ui_bg,
                self.view_zoom_scale,
                self.view_pan_x,
                self.view_pan_y,
            )
        self.update_view_scrollbars()

    def make_view_canvas(self, parent: ttk.Frame, title: str, row: int) -> tk.Canvas:
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.grid(row=row, column=0, sticky="nsew", padx=4, pady=4)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        canvas = tk.Canvas(
            frame,
            bg=self.app.ui_canvas,
            highlightthickness=1,
            highlightbackground=self.app.ui_border,
            highlightcolor=self.app.ui_accent,
            relief="flat",
        )
        scroll_x = ttk.Scrollbar(frame, orient="horizontal", command=self.on_view_xscroll)
        scroll_y = ttk.Scrollbar(frame, orient="vertical", command=self.on_view_yscroll)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")
        self.view_scroll_x = scroll_x
        self.view_scroll_y = scroll_y
        canvas.bind("<Configure>", lambda event: self.app.analysis is not None and self.draw_active_view())
        canvas.bind("<ButtonPress-1>", self.on_view_drag_start)
        canvas.bind("<B1-Motion>", self.on_view_drag_motion)
        canvas.bind("<ButtonRelease-1>", self.on_view_drag_end)
        canvas.bind("<MouseWheel>", self.on_view_mousewheel)
        canvas.bind("<Button-4>", self.on_view_wheel_up)
        canvas.bind("<Button-5>", self.on_view_wheel_down)
        return canvas

    def rebuild_module_editor(self) -> None:
        app = self.app
        for child in app.module_inner.winfo_children():
            child.destroy()
        for child in app.assembly_inner.winfo_children():
            child.destroy()
        app.module_inner.columnconfigure(0, weight=1)
        self.collapsed_part_ids.intersection_update({id(part) for part in app.current_config["parts"]})

        config_frame = ttk.LabelFrame(app.assembly_inner, text="Assembly", padding=6)
        config_frame.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        config_frame.columnconfigure(1, weight=1)
        row_index = 0
        for key in app.ordered_keys(app.current_config, app.config_field_order):
            if key in {"parts", *app.main_driver_keys, "standard_profile", "apply_standard", "thrust_axis_tilt_deg", "battery", "motors", "props"}:
                continue
            value = app.current_config[key]
            ttk.Label(config_frame, text=app.field_label(key)).grid(row=row_index, column=0, sticky="w")
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                widget = ttk.Checkbutton(config_frame, variable=variable, command=lambda d=app.current_config, k=key, v=variable: app.on_bool_changed(d, k, v, False))
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
            else:
                variable = tk.StringVar(value=app.format_field_value(key, value))
                widget = ttk.Entry(config_frame, textvariable=variable)
                widget.grid(row=row_index, column=1, sticky="ew", padx=(8, 0), pady=2)
                app.bind_value_entry(widget, app.current_config, key, variable, "config")
            row_index += 1

        for index, part in enumerate(app.current_config["parts"]):
            module_name = part["module"]["name"]
            part_id = id(part)
            is_collapsed = part_id in self.collapsed_part_ids
            frame = ttk.LabelFrame(app.module_inner, padding=4)
            frame.grid(row=index, column=0, sticky="ew", padx=4, pady=(2, 3) if is_collapsed else 4)
            frame.columnconfigure(0, weight=1)

            header_bar = ttk.Frame(frame)
            header_bar.grid(row=0, column=0, sticky="ew", pady=(0, 0) if is_collapsed else (0, 6))
            ttk.Label(header_bar, text=f"{index + 1}. {module_name}", font=("TkDefaultFont", 12, "bold")).pack(side="left")
            ttk.Button(
                header_bar,
                text="+" if is_collapsed else "-",
                width=4,
                command=lambda pid=part_id: self.toggle_part_collapsed(pid),
            ).pack(side="left", padx=(8, 0))

            if is_collapsed:
                continue

            button_bar = ttk.Frame(frame)
            button_bar.grid(row=1, column=0, sticky="ew", pady=(0, 6))
            ttk.Button(button_bar, text="Up", command=lambda idx=index: app.move_part(idx, -1)).pack(side="left")
            ttk.Button(button_bar, text="Down", command=lambda idx=index: app.move_part(idx, 1)).pack(side="left", padx=4)
            ttk.Button(button_bar, text="Delete", command=lambda idx=index: app.delete_part(idx)).pack(side="left")

            load_bar = ttk.Frame(frame)
            load_bar.grid(row=2, column=0, sticky="ew", pady=(0, 6))
            load_bar.columnconfigure(1, weight=1)
            ttk.Label(load_bar, text="Load Module").grid(row=0, column=0, sticky="w")
            module_source_var = tk.StringVar(value=str(part["module"]["name"]))
            module_source_box = ttk.Combobox(load_bar, values=app.module_library_names, textvariable=module_source_var, state="readonly")
            module_source_box.grid(row=0, column=1, sticky="ew", padx=(8, 8))
            ttk.Button(load_bar, text="Load", command=lambda p=part, v=module_source_var: app.on_load_module_from_library(p, v)).grid(row=0, column=2, sticky="e")

            self.build_dict_editor(frame, "Parameters", part["module"], app.module_field_order, app.module_type_options, 3, {"diam", "model"}, "module", part)
            next_row = 4
            if "surfaces" in part:
                self.build_dict_editor(frame, "Surface", part["surfaces"], app.surface_field_order, app.surface_type_options, next_row, {"model"}, "surface", part)
                next_row += 1

            part_keys = []
            if part["module"].get("battery", False) or part["module"].get("payload", False) or part["module"].get("ballast", False):
                part_keys.extend(["x", "y", "z", "pitch", "yaw", "roll"])
            if part["module"].get("payload", False):
                part_keys.append("payload_mass")
            if part["module"].get("ballast", False):
                part_keys.append("ballast_mass")

            if part_keys:
                aux_frame = ttk.LabelFrame(frame, text="Part Fields", padding=6)
                aux_frame.grid(row=next_row, column=0, sticky="ew", pady=(6, 0))
                aux_frame.columnconfigure(1, weight=1)
                for aux_row, key in enumerate(part_keys):
                    value = part[key]
                    ttk.Label(aux_frame, text=app.field_label(key)).grid(row=aux_row, column=0, sticky="w")
                    variable = tk.StringVar(value=app.format_field_value(key, value))
                    entry = ttk.Entry(aux_frame, textvariable=variable)
                    entry.grid(row=aux_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                    app.bind_value_entry(entry, part, key, variable, "part")

    def toggle_part_collapsed(self, part_id: int) -> None:
        if part_id in self.collapsed_part_ids:
            self.collapsed_part_ids.remove(part_id)
        else:
            self.collapsed_part_ids.add(part_id)
        self.rebuild_module_editor()

    def build_dict_editor(
        self,
        parent,
        title: str,
        data: dict,
        preferred_order: list[str],
        type_options: list[str] | None,
        row: int,
        exclude_keys: set[str],
        schema_name: str,
        owner_part: dict,
    ) -> None:
        app = self.app
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        frame.columnconfigure(1, weight=1)
        next_row = self.build_dict_rows(frame, data, preferred_order, type_options, exclude_keys, schema_name, owner_part, ())
        if title == "Surface":
            ttk.Label(frame, text="mid-span chord (mm)").grid(row=next_row, column=0, sticky="w")
            ttk.Label(frame, text=f"{(0.5 * (data['chord_root'] + data['chord_tip']) * 1000.0):g}").grid(row=next_row, column=1, sticky="w", padx=(8, 0), pady=2)

    def build_dict_rows(
        self,
        frame,
        data: dict,
        preferred_order: list[str],
        type_options: list[str] | None,
        exclude_keys: set[str],
        schema_name: str,
        owner_part: dict,
        path: tuple[str, ...],
    ) -> int:
        app = self.app
        visible_keys = [key for key in app.ordered_keys(data, preferred_order) if key not in exclude_keys]
        if path == () and schema_name == "module" and data["type"] == "end" and not data.get("fins", False):
            visible_keys = [key for key in visible_keys if key not in {"wings_n", "wing_phase", "wings_ang", "wings_x"}]
        if path == ("pointed_shape",):
            active_shape_param_keys = set(app.pointed_shape_param_keys[data["shape"]])
            visible_keys = [key for key in visible_keys if key == "shape" or key in active_shape_param_keys]
        if "motor" in data and not data["motor"]:
            visible_keys = [key for key in visible_keys if key not in app.motor_edit_keys]
        next_row = 0
        for key in visible_keys:
            value = data[key]
            if type(value) is dict:
                group = ttk.LabelFrame(frame, text=app.field_label(key), padding=6)
                group.grid(row=next_row, column=0, columnspan=2, sticky="ew", pady=2)
                group.columnconfigure(1, weight=1)
                child_order = app.pointed_shape_field_order if key == "pointed_shape" else []
                self.build_dict_rows(group, value, child_order, None, set(), schema_name, owner_part, path + (key,))
                next_row += 1
                continue
            ttk.Label(frame, text=app.field_label(key)).grid(row=next_row, column=0, sticky="w")
            if key == "type" and type_options is not None:
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=type_options, textvariable=variable, state="readonly")
                widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                if schema_name == "module":
                    widget.bind("<<ComboboxSelected>>", lambda event, p=owner_part, v=variable: app.on_module_type_changed(p, v))
                else:
                    widget.bind("<<ComboboxSelected>>", lambda event, p=owner_part, v=variable: app.on_surface_type_changed(p, v))
                next_row += 1
                continue
            if key == "motor_dir":
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=app.motor_direction_options, textvariable=variable, state="readonly")
                widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                widget.bind("<<ComboboxSelected>>", lambda event, d=data, v=variable: app.on_select_value(d, "motor_dir", v, True))
                next_row += 1
                continue
            if key == "pod_shape":
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=app.pod_shape_options, textvariable=variable, state="readonly")
                widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                widget.bind("<<ComboboxSelected>>", lambda event, d=data, v=variable: app.on_select_value(d, "pod_shape", v, True))
                next_row += 1
                continue
            if path == ("pointed_shape",) and key == "shape":
                variable = tk.StringVar(value=str(value))
                widget = ttk.Combobox(frame, values=app.pointed_shape_options, textvariable=variable, state="readonly")
                widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                widget.bind("<<ComboboxSelected>>", lambda event, d=data, v=variable: app.on_select_value(d, "shape", v, True))
                next_row += 1
                continue
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                widget = ttk.Checkbutton(frame, variable=variable, command=lambda d=data, k=key, v=variable: app.on_bool_changed(d, k, v, True))
                widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
                next_row += 1
                continue
            variable = tk.StringVar(value=app.format_field_value(key, value))
            widget = ttk.Entry(frame, textvariable=variable)
            widget.grid(row=next_row, column=1, sticky="ew", padx=(8, 0), pady=2)
            app.bind_value_entry(widget, data, key, variable, schema_name, key == "name")
            next_row += 1
        return next_row

    def draw_propulsion_info(self) -> None:
        app = self.app
        if app.propulsion_text is None:
            return
        app.refresh_propulsion_controls()
        app.propulsion_text.delete("1.0", tk.END)
        if app.analysis is None or not app.analysis["motor_summary_rows"]:
            app.propulsion_text.insert("1.0", "No propulsion data.")
            if app.propulsion_plot_canvas is not None:
                app.propulsion_plot_canvas.get_tk_widget().destroy()
                app.propulsion_plot_canvas = None
            if app.propulsion_plot_figure is not None:
                plt.close(app.propulsion_plot_figure)
                app.propulsion_plot_figure = None
            return
        row = app.analysis["motor_summary_rows"][0]
        lines = [
            f"Motor                : {row['label']}",
            f"Propeller            : {row['propeller']}",
            f"Pack voltage         : {row['pack_voltage_v']:.2f} V",
            f"Motor KV             : {row['kv']:.0f}",
            f"No-load RPM          : {row['no_load_rpm']:.0f}",
            f"Static prop RPM      : {row['prop_rpm']:.0f}",
            f"Static thrust        : {row['thrust_n_per_motor']:.2f} N",
            f"Static current       : {row['current_a_per_motor']:.2f} A",
            f"Static input power   : {row['input_power_w_per_motor']:.1f} W",
            f"Pitch speed          : {row['pitch_speed_kph']:.1f} km/h",
            f"Efficiency           : {row['efficiency_g_per_w']:.2f} g/W",
        ]
        app.propulsion_text.insert("1.0", "\n".join(lines))
        if app.propulsion_plot_canvas is not None:
            app.propulsion_plot_canvas.get_tk_widget().destroy()
        if app.propulsion_plot_figure is not None:
            plt.close(app.propulsion_plot_figure)
        app.propulsion_plot_figure, axis = plt.subplots(figsize=(5.8, 4.8), dpi=100)
        curve = row["curve"]
        plot_rows = [
            ("Thrust (N)", curve["thrust_n"], "#19ff19"),
            ("RPM / 1000", [value / 1000.0 for value in curve["rpm"]], "#33ccff"),
            ("kW", [value / 1000.0 for value in curve["watts"]], "#ffcc33"),
        ]
        for label, values, color in plot_rows:
            axis.plot(curve["throttle_pct"], values, color=color, linewidth=2.0, label=label)
        axis.set_xlabel("Throttle (%)", color=app.ui_fg)
        axis.set_ylabel("Value", color=app.ui_fg)
        axis.tick_params(colors=app.ui_fg)
        axis.set_facecolor(app.ui_bg)
        axis.grid(color=app.ui_border, alpha=0.4)
        legend = axis.legend(loc="upper left", facecolor=app.ui_panel, edgecolor=app.ui_border, framealpha=1.0)
        for text in legend.get_texts():
            text.set_color(app.ui_fg)
        app.propulsion_plot_figure.patch.set_facecolor(app.ui_bg)
        app.propulsion_plot_figure.tight_layout()
        app.propulsion_plot_canvas = FigureCanvasTkAgg(app.propulsion_plot_figure, master=app.propulsion_plot_host)
        app.propulsion_plot_canvas.draw()
        app.propulsion_plot_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def draw_asb_view(self, force: bool = False) -> None:
        app = self.app
        if app.asb_airplane is None or app.asb_frame is None:
            return
        if app.asb_draw_busy:
            return
        frame_width = max(app.asb_frame.winfo_width(), 200)
        frame_height = max(app.asb_frame.winfo_height(), 200)
        if not force and app.asb_canvas is not None and app.asb_last_size == (frame_width, frame_height):
            return
        app.asb_last_size = (frame_width, frame_height)
        fig_width = frame_width / 100.0
        fig_height = frame_height / 100.0
        app.asb_draw_busy = True
        try:
            if app.asb_canvas is not None:
                app.asb_canvas.get_tk_widget().destroy()
            if app.asb_figure is not None:
                plt.close(app.asb_figure)
            app.asb_figure, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=100, subplot_kw={"projection": "3d"})
            ax.set_facecolor(app.ui_bg)
            ax.tick_params(colors=app.ui_fg)
            ax.xaxis.label.set_color(app.ui_fg)
            ax.yaxis.label.set_color(app.ui_fg)
            ax.zaxis.label.set_color(app.ui_fg)
            ax.xaxis.pane.set_facecolor(app.ui_bg)
            ax.yaxis.pane.set_facecolor(app.ui_bg)
            ax.zaxis.pane.set_facecolor(app.ui_bg)
            ax.xaxis.pane.set_alpha(0.0)
            ax.yaxis.pane.set_alpha(0.0)
            ax.zaxis.pane.set_alpha(0.0)
            app.asb_airplane.draw(
                backend="matplotlib",
                ax=ax,
                use_preset_view_angle="left_isometric",
                set_background_pane_color=app.ui_bg,
                set_background_pane_alpha=0.0,
                set_axis_visibility=False,
                show=False,
            )
            app.asb_figure.patch.set_facecolor(app.ui_bg)
            app.asb_figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
            ax.set_position([0.0, 0.0, 1.0, 1.0])
            app.asb_canvas = FigureCanvasTkAgg(app.asb_figure, master=app.asb_frame)
            app.asb_canvas.draw()
            app.asb_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        finally:
            app.asb_draw_busy = False
