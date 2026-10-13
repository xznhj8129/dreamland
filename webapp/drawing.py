"""Server-side drawing for the browser front end: the Tk drawing code, recorded as plain data.

The Simple drawing is still produced by libs.graphics.draw_canvas_view, unchanged, drawn into a
RecordingCanvas instead of a live Tk canvas. ViewState ports the view fields and commands of the retired Tk
Editor (libs/views/editor_view.py, removed with the Tk frontend), so mode, roll, zoom, pan and the
scrollbars behave as in the Tk view.
"""

from __future__ import annotations

import math
from typing import Any

from libs.airframe import BATTERY_PART_NAME
from libs.geometry import body_section_at, superellipse_radius
from libs.graphics import (
    POINT_MASS_MARK_PX, ROD_COLOR, SECTION_OUTLINE_SAMPLES, convex_hull, draw_canvas_view, longitudinal_transverse,
    front_profile_outlines, part_corners, rounded_rectangle, volume_half_extents,
)
from webapp.drawing_enrich import enrich_front_scene, enrich_longitudinal_scene, roll_detents
from webapp.state import AppState

UI_DRAW = "#2aff2a"  # guibuilder.py UI_DRAW
UI_BG = "#000000"  # guibuilder.py UI_BG
MIN_CANVAS_PX = 10.0  # libs/graphics.py: max(canvas.winfo_width(), 10)

MIN_ZOOM_SCALE = 1.0  # editor_view.py zoom_view_at (line 1389)
MAX_ZOOM_SCALE = 25.0
SCROLL_UNIT_PX = 40.0  # editor_view.py on_view_xscroll/on_view_yscroll
SCROLL_PAGE_FRACTION = 0.9

# Drag handle targets, the definition collections the move op edits (webapp.editor.apply_move).
TAGGED_TARGETS = ("cut", "loft", "spars", "surface", "volume")  # libs/graphics.py canvas tags
ALWAYS_X_TARGETS = ("cut", "loft", "spars", "surface", "rod")  # only the body axis moves them, in either view
HANDLE_OVAL_SAMPLES = 24


def _plain(value: Any) -> Any:
    """A dict/list scalar as passed (tuples become lists so json.dumps needs no custom encoder)."""
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


class RecordingCanvas:
    """A duck-typed canvas stand-in that records create_* calls as JSON-safe scene items.

    Only the calls libs.graphics makes are implemented: winfo_width/winfo_height, delete("all") and
    create_line/polygon/rectangle/oval/text. Options are recorded exactly as passed; none are invented.
    """

    def __init__(self, width: float, height: float) -> None:
        self._width = width
        self._height = height
        self._next_id = 1
        self.items: list[dict[str, Any]] = []

    def winfo_width(self) -> float:
        return self._width

    def winfo_height(self) -> float:
        return self._height

    def delete(self, *args: Any) -> None:
        self.items.clear()

    def create_line(self, *args: Any, **opts: Any) -> int:
        return self._record("line", args, opts)

    def create_polygon(self, *args: Any, **opts: Any) -> int:
        return self._record("polygon", args, opts)

    def create_rectangle(self, *args: Any, **opts: Any) -> int:
        return self._record("rectangle", args, opts)

    def create_oval(self, *args: Any, **opts: Any) -> int:
        return self._record("oval", args, opts)

    def create_text(self, *args: Any, **opts: Any) -> int:
        return self._record("text", args, opts)

    def _record(self, kind: str, args: tuple[Any, ...], opts: dict[str, Any]) -> int:
        item_id = self._next_id
        self._next_id += 1
        coords = list(args[0]) if len(args) == 1 and isinstance(args[0], (list, tuple)) else list(args)
        item = {"kind": kind, "coords": [float(value) for value in coords], "opts": {key: _plain(value) for key, value in opts.items()}}
        self.items.append(item)
        return item_id


class ViewState:
    """One session's view fields; mirrors EditorMainView's view_* fields (libs/views/editor_view.py)."""

    def __init__(self) -> None:
        self.mode = "Longitudinal"
        self.roll_deg = 0.0
        self.zoom_scale = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.metrics: dict[str, Any] | None = None  # the metrics of the last draw, as in the Tk view

    def reset(self) -> None:
        """Port of reset_view_transform (editor_view.py 1257-1261)."""
        self.zoom_scale = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

    def set_mode(self, mode: str) -> None:
        """The view_mode_var radio, applied by draw_active_view (editor_view.py 332-335)."""
        self.mode = mode

    def set_roll(self, roll_deg: float) -> None:
        """Port of on_roll_slider_changed (editor_view.py 1253-1255); the display label comes from draw()."""
        self.roll_deg = roll_deg

    def pan_delta(self, dx: float, dy: float) -> None:
        """Port of the no-target branch of on_view_drag_motion (editor_view.py 374-380)."""
        self.pan_x += dx
        self.pan_y += dy

    def zoom_at(self, x: float, y: float, factor: float) -> None:
        """Port of zoom_view_at (editor_view.py 1386-1396)."""
        metrics = self.metrics
        if metrics is None:
            return
        new_zoom_scale = min(max(self.zoom_scale * factor, MIN_ZOOM_SCALE), MAX_ZOOM_SCALE)
        if new_zoom_scale == self.zoom_scale:
            return
        zoom_ratio = new_zoom_scale / self.zoom_scale
        self.pan_x = x - metrics["origin_x"] - zoom_ratio * (x - metrics["origin_x"] - self.pan_x)
        self.pan_y = y - metrics["origin_y"] - zoom_ratio * (y - metrics["origin_y"] - self.pan_y)
        self.zoom_scale = new_zoom_scale

    def scroll(self, axis: str, mode: str, value: float) -> None:
        """Port of on_view_xscroll / on_view_yscroll (editor_view.py 1345-1384)."""
        metrics = self.metrics
        if metrics is None:
            return
        if axis == "x":
            if metrics["mode"] == "Longitudinal":
                content_width = metrics["content_width"]
                available_x = metrics["width"] - 2.0 * metrics["pad_x"]
                if content_width <= available_x:
                    return
                if mode == "moveto":
                    self.pan_x = -float(value) * content_width
                    return
                step = SCROLL_UNIT_PX if mode == "units" else available_x * SCROLL_PAGE_FRACTION
                self.pan_x -= int(value) * step
                return
            content_width = 2.0 * metrics["content_half_width"]
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if content_width <= available_x:
                return
            if mode == "moveto":
                self.pan_x = (content_width - available_x) / 2.0 - float(value) * content_width
                return
            step = SCROLL_UNIT_PX if mode == "units" else available_x * SCROLL_PAGE_FRACTION
            self.pan_x -= int(value) * step
            return
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            return
        if mode == "moveto":
            self.pan_y = (content_height - available_y) / 2.0 - float(value) * content_height
            return
        step = SCROLL_UNIT_PX if mode == "units" else available_y * SCROLL_PAGE_FRACTION
        self.pan_y -= int(value) * step

    def clamp_pan(self, metrics: dict[str, Any]) -> bool:
        """Port of clamp_view_pan (editor_view.py 1263-1305); True when either pan was clamped."""
        changed = False
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
        clamped_pan_x = min(max(self.pan_x, min_pan_x), max_pan_x)
        clamped_pan_y = min(max(self.pan_y, min_pan_y), max_pan_y)
        if clamped_pan_x != self.pan_x:
            self.pan_x = clamped_pan_x
            changed = True
        if clamped_pan_y != self.pan_y:
            self.pan_y = clamped_pan_y
            changed = True
        return changed

    def scrollbar_fractions(self, metrics: dict[str, Any]) -> dict[str, dict[str, float]]:
        """Scrollbar thumb fractions; port of update_view_scrollbars (editor_view.py 1307-1343)."""
        if metrics["mode"] == "Longitudinal":
            available_x = metrics["width"] - 2.0 * metrics["pad_x"]
            if metrics["content_width"] <= available_x:
                x_first, x_last = 0.0, 1.0
            else:
                x_first = -self.pan_x / metrics["content_width"]
                x_last = x_first + available_x / metrics["content_width"]
            available_y = metrics["height"] - 2.0 * metrics["pad_y"]
            content_height = 2.0 * metrics["content_half_height"]
            if content_height <= available_y:
                y_first, y_last = 0.0, 1.0
            else:
                y_first = ((content_height - available_y) / 2.0 - self.pan_y) / content_height
                y_last = y_first + available_y / content_height
            return {"x": {"first": x_first, "last": x_last}, "y": {"first": y_first, "last": y_last}}
        available_x = metrics["width"] - 2.0 * metrics["pad_x"]
        content_width = 2.0 * metrics["content_half_width"]
        if content_width <= available_x:
            x_first, x_last = 0.0, 1.0
        else:
            x_first = ((content_width - available_x) / 2.0 - self.pan_x) / content_width
            x_last = x_first + available_x / content_width
        available_y = metrics["height"] - 2.0 * metrics["pad_y"]
        content_height = 2.0 * metrics["content_half_height"]
        if content_height <= available_y:
            y_first, y_last = 0.0, 1.0
        else:
            y_first = ((content_height - available_y) / 2.0 - self.pan_y) / content_height
            y_last = y_first + available_y / content_height
        return {"x": {"first": x_first, "last": x_last}, "y": {"first": y_first, "last": y_last}}


def ensure_view(state: AppState) -> ViewState:
    """The one ViewState for an AppState: the server's single browser session (single user)."""
    view = getattr(state, "view", None)
    if not isinstance(view, ViewState):
        view = ViewState()
        state.view = view  # type: ignore[attr-defined]
    return view


def _view_payload(view: ViewState) -> dict[str, Any]:
    return {"mode": view.mode, "roll_deg": view.roll_deg, "zoom_scale": view.zoom_scale, "pan_x": view.pan_x, "pan_y": view.pan_y}


def _drag_axes(view: ViewState, target: str) -> tuple[str | None, str | None]:
    """The definition axes a screen-horizontal / screen-vertical drag edits, as the Tk view maps them.

    Side view: horizontal x, vertical z; front view: horizontal y, vertical z. A side view rolled off zero has no
    unambiguous screen-vertical axis (z and y mix), so vertical is None there. Cuts, spars, surfaces and rods are
    moved only along the body: their horizontal axis is x and they have no vertical axis.
    """
    if target in ALWAYS_X_TARGETS:
        return "x", None
    horizontal = "x" if view.mode == "Longitudinal" else "y"
    vertical = None if view.mode == "Longitudinal" and view.roll_deg != 0.0 else "z"
    return horizontal, vertical


def _handle(view: ViewState, target: str, index: int, kind: str, coords: list[float]) -> dict[str, Any]:
    horizontal, vertical = _drag_axes(view, target)
    return {"target": target, "index": index, "kind": kind, "coords": [float(value) for value in coords], "horizontal": horizontal, "vertical": vertical}


def _circle_points(center_x: float, center_y: float, radius: float) -> list[float]:
    return [value for sample in range(HANDLE_OVAL_SAMPLES)
            for value in (center_x + radius * math.cos(2.0 * math.pi * sample / HANDLE_OVAL_SAMPLES),
                          center_y + radius * math.sin(2.0 * math.pi * sample / HANDLE_OVAL_SAMPLES))]


def _item_outline(item: dict[str, Any]) -> tuple[str, list[float]] | None:
    """One recorded canvas item as a handle outline: text is not an outline, rectangles and ovals become polygons."""
    kind, coords = item["kind"], item["coords"]
    if kind in ("line", "polygon"):
        return kind, list(coords)
    if kind == "rectangle":
        x0, y0, x1, y1 = coords
        return "polygon", [x0, y0, x1, y0, x1, y1, x0, y1]
    if kind == "oval":
        x0, y0, x1, y1 = coords
        return "polygon", _circle_points((x0 + x1) / 2.0, (y0 + y1) / 2.0, max((x1 - x0) / 2.0, (y1 - y0) / 2.0))
    return None


def _tagged_handles(canvas: RecordingCanvas, view: ViewState) -> list[dict[str, Any]]:
    """Handles for the items libs.graphics tagged ("drag", target:<index>), one per outline, in draw order."""
    handles: list[dict[str, Any]] = []
    for item in canvas.items:
        tags = item["opts"].get("tags")
        if not isinstance(tags, list):
            continue
        for tag in tags:
            if not isinstance(tag, str) or ":" not in tag:
                continue
            target, _, suffix = tag.partition(":")
            if target not in TAGGED_TARGETS or not suffix.isdigit():
                continue
            outline = _item_outline(item)
            if outline is None:
                continue
            kind, coords = outline
            handles.append(_handle(view, target, int(suffix), kind, coords))
    return handles


def _longitudinal_map(metrics: dict[str, Any]):
    """The side-view mapping of draw_longitudinal_view: x_to_screen / transverse_to_screen, with the drawn metrics."""
    origin_x = metrics["origin_x"] + metrics["pan_x"]
    origin_y = metrics["origin_y"] + metrics["pan_y"]
    scale, roll_rad = metrics["scale"], metrics["roll_rad"]

    def project(point) -> tuple[float, float]:
        return origin_x + point[0] * scale, origin_y - longitudinal_transverse(point[1], point[2], roll_rad) * scale

    return project


def _front_map(metrics: dict[str, Any]):
    """The front-view mapping of draw_front_view (center_x/center_y from metrics; pan is already folded in)."""
    origin_x = metrics["origin_x"] + metrics["pan_x"]
    origin_y = metrics["origin_y"] + metrics["pan_y"]
    scale = metrics["scale"]

    def project(point) -> tuple[float, float]:
        return origin_x + point[1] * scale, origin_y - point[2] * scale

    return project


def _part_handles(view: ViewState, part, target: str, index: int, project) -> list[dict[str, Any]]:
    """The drawn outline of a placed part (convex hull of its turned box, or the point-mass cross), as handles."""
    if part.size_m is None:
        x, y = project(part.position_m)
        mark = POINT_MASS_MARK_PX
        return [_handle(view, target, index, "line", [x - mark, y, x + mark, y]),
                _handle(view, target, index, "line", [x, y - mark, x, y + mark])]
    outline = convex_hull([project(corner) for corner in part_corners(part)])
    return [] if len(outline) < 2 else [_handle(view, target, index, "polygon", [value for point in outline for value in point])]


def _rod_handles(view: ViewState, reinforcement, project, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """The drawn reinforcement rods: side view lines along x per rod position, front view circles at their y, z."""
    handles = []
    for y, z in reinforcement.pattern.positions_m():
        if view.mode == "Longitudinal":
            x0, t = project((reinforcement.from_x_m, y, z))
            x1, _t = project((reinforcement.to_x_m, y, z))
            handles.append(_handle(view, "rod", 0, "line", [x0, t, x1, t]))
        else:
            center_x, center_y = project((0.0, y, z))
            handles.append(_handle(view, "rod", 0, "polygon", _circle_points(center_x, center_y, reinforcement.pattern.diameter_m / 2.0 * metrics["scale"])))
    return handles


def _front_structure_handles(resolved, view: ViewState, project, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    """Handles for the untagged front-view spars and volumes, computed exactly as draw_front_view draws them.

    Front-view surface panels are tagged by enrich_front_scene (their thickness-band hulls), so their handles come
    from _tagged_handles like the other tagged items.
    """
    handles: list[dict[str, Any]] = []
    surface_indices = {surface_set.name: index for index, surface_set in enumerate(resolved.surfaces)}
    for spar in resolved.spars:
        (x0, y0), (x1, y1) = project(spar.start), project(spar.end)
        handles.append(_handle(view, "spars", surface_indices[spar.surface], "line", [x0, y0, x1, y1]))
    scale = metrics["scale"]
    for index, volume in enumerate(resolved.airframe.volumes):
        if volume.shape == "point":  # not drawn in the front view
            continue
        _half_x, half_y, half_z = volume_half_extents(resolved, volume)
        center_x, center_y = project((volume.x_m, volume.y_m, volume.z_m))
        if volume.shape == "bore":
            coords: list[float] = []
            for sample in range(SECTION_OUTLINE_SAMPLES + 1):
                angle = 2.0 * math.pi * sample / SECTION_OUTLINE_SAMPLES
                radius = superellipse_radius(half_y, half_z, body_section_at(resolved.body, volume.x_m).shape, angle)
                coords += [center_x + radius * math.cos(angle) * scale, center_y - radius * math.sin(angle) * scale]
        elif volume.shape == "cylinder" and volume.axis == "x":
            coords = _circle_points(center_x, center_y, half_y * scale)
        elif volume.shape == "rounded_box" and volume.axis == "x":
            coords = rounded_rectangle(center_x, center_y, half_y * scale, half_z * scale, volume.corner_radius_m * scale)
        else:
            coords = [center_x - half_y * scale, center_y - half_z * scale, center_x + half_y * scale, center_y - half_z * scale,
                      center_x + half_y * scale, center_y + half_z * scale, center_x - half_y * scale, center_y + half_z * scale]
        handles.append(_handle(view, "volume", index, "polygon", coords))
    return handles


def _handles(state: AppState, view: ViewState, metrics: dict[str, Any], canvas: RecordingCanvas) -> list[dict[str, Any]]:
    """Every movable element's screen outline for hit-testing: tagged scene items, then the untagged movable ones.

    Free parts and the battery (placements without a volume), and the reinforcement rods are not tagged by
    libs.graphics, so their outlines are computed from resolved/definition with the metrics the drawing used.
    The move op then applies the drag delta to the definition (webapp.editor.apply_move).
    """
    resolved = state.resolved
    definition = state.definition
    project = _longitudinal_map(metrics) if view.mode == "Longitudinal" else _front_map(metrics)
    handles = _tagged_handles(canvas, view)
    if view.mode != "Longitudinal":
        handles += _front_structure_handles(resolved, view, project, metrics)
    parts = resolved.airframe.parts
    for index, part_data in enumerate(definition.get("parts", [])):
        if "volume" in part_data or "x_m" not in part_data or index >= len(parts):
            continue
        handles += _part_handles(view, parts[index], "part", index, project)
    propulsion = definition.get("propulsion")
    electric = propulsion.get("electric") if isinstance(propulsion, dict) else None
    battery_data = electric.get("battery") if isinstance(electric, dict) else None
    if isinstance(battery_data, dict) and "volume" not in battery_data and "x_m" in battery_data:
        battery = next((part for part in parts if part.name == BATTERY_PART_NAME), None)
        if battery is not None:
            handles += _part_handles(view, battery, "battery", 0, project)
    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        handles += _rod_handles(view, reinforcement, project, metrics)
    for order, handle in enumerate(handles):
        handle["order"] = order
    return handles


def draw(state: AppState, width: float, height: float, control_deflections: dict[int, float]) -> dict[str, Any]:
    """Draw the active view into a RecordingCanvas; the /api/draw payload, all JSON-safe."""
    view = ensure_view(state)
    canvas = RecordingCanvas(max(float(width), MIN_CANVAS_PX), max(float(height), MIN_CANVAS_PX))
    resolved = state.resolved
    if resolved is None:
        # editor_view.py draw_active_view (lines 328-331) returns before drawing when nothing resolved yet.
        return {
            "scene": [],
            "handles": [],
            "section_outlines": [],
            "loft_outlines": [],
            "session_id": state.session_id,
            "metrics": {},
            "view": _view_payload(view),
            "scrollbars": {"x": {"first": 0.0, "last": 1.0}, "y": {"first": 0.0, "last": 1.0}},
            "roll_display": f"{view.roll_deg:.1f} deg",
            "roll_detents": [],
            "width": width,
            "height": height,
        }
    draw_args = (canvas, resolved, state.aero, view.mode, view.roll_deg, UI_DRAW, UI_BG)
    metrics = draw_canvas_view(*draw_args, view.zoom_scale, view.pan_x, view.pan_y, control_deflections)
    if view.clamp_pan(metrics):
        # editor_view.py draw_active_view (lines 337-340): a clamp that bit redraws with the clamped view.
        metrics = draw_canvas_view(*draw_args, view.zoom_scale, view.pan_x, view.pan_y, control_deflections)
    if view.mode == "Longitudinal":
        # The recorded scene gets the browser-only side-view enrichment; the Tk drawing itself is untouched.
        enrich_longitudinal_scene(canvas, resolved, _longitudinal_map(metrics), UI_DRAW, ROD_COLOR)
    else:
        # The front view gets its panel thickness bands; the recorded straight span lines are replaced.
        enrich_front_scene(canvas, resolved, _front_map(metrics), UI_DRAW)
    view.metrics = metrics
    return {
        "scene": canvas.items,
        "handles": _handles(state, view, metrics, canvas),
        "loft_outlines": front_profile_outlines(resolved, [s.x_m for s in resolved.airframe.fuselage.loft_stations], metrics) if view.mode == "Front" else [],
        "section_outlines": front_profile_outlines(resolved, [(s.x0_m + s.x1_m) / 2.0 for s in resolved.sections], metrics) if view.mode == "Front" else [],
        "session_id": state.session_id,
        "metrics": metrics,
        "view": _view_payload(view),
        "scrollbars": view.scrollbar_fractions(metrics),
        "roll_display": f"{view.roll_deg:.1f} deg",
        "roll_detents": roll_detents(resolved),
        "width": width,
        "height": height,
    }
