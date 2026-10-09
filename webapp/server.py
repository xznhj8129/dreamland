"""aiohttp application: one AppState, the REST routes and the /ws/events state push stream."""

from __future__ import annotations

import asyncio
import json
import traceback
from pathlib import Path
from typing import Any, TypeGuard

from aiohttp import WSCloseCode, WSMsgType, web

from libs.airframe import SCHEMA, AirframeError, blank_definition, parse_airframe
from libs.geometry import resolve_airframe
from libs.project import BUILD_FILE, PROJECTS_DIR, ProjectError, copy_project, create_project, open_project
from webapp.drawing import ViewState, draw, ensure_view
from webapp.editor import OpFailed, SessionMismatch, apply_move, apply_op, apply_set, check_session, context
from webapp.library import delete_part, library_payload, save_part
from webapp.model3d import build_model3d
from webapp.serialize import snapshot
from webapp.state import AppState

# guibuilder.py EDITOR_BLOCKS: the definition blocks the editors index directly. Load refuses a definition missing
# any of them. Kept local so the server does not import the Tk frontend; keep in step with guibuilder.py.
EDITOR_BLOCKS = ("name", "tailsitter", "vtol", "powered", "operating_point", "print", "fuselage", "surfaces", "volumes", "parts", "sections")


@web.middleware
async def error_middleware(request: web.Request, handler):
    """The final catch-all: any unexpected command failure is logged and still answers with a snapshot.

    The browser's 500 body then carries the state and the Log entry, instead of an HTML error page
    (guibuilder's report_callback_exception equivalent).
    """
    try:
        return await handler(request)
    except web.HTTPException:
        raise  # deliberate refusals keep their own status
    except Exception:
        state: AppState = request.app["state"]
        state.log_error("Error", traceback.format_exc())
        return web.json_response(snapshot(state), status=500)


def create_app(state: AppState, dist_dir: Path) -> web.Application:
    app = web.Application(middlewares=[error_middleware])
    app["state"] = state
    app["dist_dir"] = dist_dir
    app["websockets"] = set()  # open /ws/events responses, closed on shutdown so cleanup cannot wait on them
    app.router.add_get("/api/state", handle_state)
    app.router.add_post("/api/recompute", handle_recompute)
    app.router.add_post("/api/save", handle_save)
    app.router.add_get("/api/projects", handle_projects)
    app.router.add_post("/api/projects/open", handle_projects_open)
    app.router.add_post("/api/projects/new", handle_projects_new)
    app.router.add_post("/api/save-as", handle_save_as)
    app.router.add_get("/api/editor/context", handle_editor_context)
    app.router.add_post("/api/editor/set", handle_editor_set)
    app.router.add_post("/api/editor/op", handle_editor_op)
    app.router.add_post("/api/editor/move", handle_editor_move)
    app.router.add_get("/api/library", handle_library)
    app.router.add_post("/api/library/save", handle_library_save)
    app.router.add_post("/api/library/delete", handle_library_delete)
    app.router.add_post("/api/draw", handle_draw)
    app.router.add_post("/api/model3d", handle_model3d)
    app.router.add_get("/ws/events", handle_events)
    app.router.add_get("/{tail:.*}", handle_dist)  # after the API routes: SPA fallback
    app.on_startup.append(on_startup)
    return app


def _state(request: web.Request) -> AppState:
    return request.app["state"]


async def close_websockets(app: web.Application) -> None:
    """Close every open /ws/events response so the server can shut down without waiting for clients."""
    sockets: set[web.WebSocketResponse] = app["websockets"]
    if not sockets:
        return
    await asyncio.gather(*(ws.close(code=WSCloseCode.GOING_AWAY, message=b"server exiting") for ws in list(sockets)), return_exceptions=True)


async def on_startup(app: web.Application) -> None:
    app["state"].set_loop(asyncio.get_running_loop())


async def handle_state(request: web.Request) -> web.Response:
    return web.json_response(snapshot(_state(request)))


async def handle_recompute(request: web.Request) -> web.Response:
    """Validate, resolve and push; the optional {run_aero: bool} body lets a drag release run the full aero solve.

    An empty body means run_aero true, as before; the drag motion itself uses /api/editor/move (geometry only).
    """
    state = _state(request)
    run_aero = True
    raw = await request.text()
    if raw.strip():
        try:
            body = json.loads(raw)
        except json.JSONDecodeError as error:
            raise web.HTTPBadRequest(text="body must be a JSON object") from error
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text="body must be a JSON object")
        value = body.get("run_aero", True)
        if type(value) is not bool:
            raise web.HTTPBadRequest(text="run_aero must be a boolean")
        run_aero = value
    state.recompute(run_aero=run_aero)
    return web.json_response(snapshot(state))


async def handle_save(request: web.Request) -> web.Response:
    state = _state(request)
    try:
        state.save()
    except OSError as error:
        state.log_error("Save Project", f"could not write {state.project.build_path}: {error}")
        return _failed(state)
    return web.json_response(snapshot(state))


async def _json_object(request: web.Request) -> dict[str, Any]:
    try:
        body = await request.json()
    except (json.JSONDecodeError, ValueError):
        raise web.HTTPBadRequest(text="body must be a JSON object")
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text="body must be a JSON object")
    return body


def _name(body: dict[str, Any]) -> str | None:
    name = body.get("name")
    return name if isinstance(name, str) else None


def _valid_project_name(name: str | None) -> TypeGuard[str]:
    """A project name is a single plain directory entry: no empty/dot names, separators or NUL."""
    if name is None or not name or name in (".", ".."):
        return False
    return not any(char in name for char in ("/", "\\", "\x00"))


def _failed(state: AppState) -> web.Response:
    """A refused command: the log entry is in this snapshot and the current project/state is unchanged."""
    return web.json_response(snapshot(state), status=400)


async def handle_projects(request: web.Request) -> web.Response:
    """The aircraft projects the server can open (guibuilder's Load opens from PROJECTS_DIR)."""
    names = (
        sorted(entry.name for entry in PROJECTS_DIR.iterdir() if entry.is_dir() and (entry / BUILD_FILE).is_file())
        if PROJECTS_DIR.is_dir()
        else []
    )
    return web.json_response({"projects": names})


async def handle_projects_open(request: web.Request) -> web.Response:
    """File > Load: guibuilder.BuilderApp.load_project, with the picker replaced by a project name."""
    state = _state(request)
    name = _name(await _json_object(request))
    if not _valid_project_name(name):
        state.log_error("Load Project", f"'{name}' is not a plain directory name.")
        return _failed(state)
    try:
        project = open_project(PROJECTS_DIR / name)
    except ProjectError as error:
        state.log_error("Load Project", str(error))
        return _failed(state)
    try:
        with open(project.build_path, "r") as file:
            data = json.load(file)
    except (OSError, json.JSONDecodeError) as error:
        state.log_error("Load Project", f"could not read {project.build_path}: {error}")
        return _failed(state)
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        state.log_error("Load Project", f"{project.build_path} is not an {SCHEMA} file (old module-stack configs are not supported).")
        return _failed(state)
    # The editors index these blocks directly; an incomplete definition with all of them can be opened and
    # repaired, but a file missing one cannot be shown, so the open is refused and the current project stays.
    missing = [key for key in EDITOR_BLOCKS if key not in data]
    if missing:
        state.log_error("Load Project", f"{project.build_path} has no {missing}; it is not an editable {SCHEMA} definition.")
        return _failed(state)
    state.switch_project(project, data)
    return web.json_response(snapshot(state))


async def handle_projects_new(request: web.Request) -> web.Response:
    """File > New: guibuilder.BuilderApp.new_project, with the name dialog on the client."""
    state = _state(request)
    name = _name(await _json_object(request))
    if not _valid_project_name(name):
        state.log_error("New Project", f"'{name}' is not a plain directory name.")
        return _failed(state)
    definition = blank_definition(name)
    try:  # fail before anything is written if the blank no longer matches the schema or the part library
        resolve_airframe(parse_airframe(definition, state.parts_data, path=f"{name}/{BUILD_FILE}"), state.parts_data)
    except AirframeError as error:
        state.log_error("New Project", f"blank definition: {error}")
        return _failed(state)
    try:
        project = create_project(name, definition)
    except ProjectError as error:
        state.log_error("New Project", str(error))
        return _failed(state)
    state.log.clear()
    state.switch_project(project, definition)
    return web.json_response(snapshot(state))


async def handle_save_as(request: web.Request) -> web.Response:
    """File > Save As: guibuilder.BuilderApp.save_project_as (copy the project, keep working on the copy)."""
    state = _state(request)
    name = _name(await _json_object(request))
    if not _valid_project_name(name):
        state.log_error("Save Project As", f"'{name}' is not a plain directory name.")
        return _failed(state)
    try:
        project = copy_project(state.project, PROJECTS_DIR / name)
    except ProjectError as error:
        state.log_error("Save Project As", str(error))
        return _failed(state)
    except OSError as error:  # a copy that fails part-way must be a logged, recoverable state
        state.log_error("Save Project As", f"could not copy {state.project.root} to {PROJECTS_DIR / name}: {error}")
        return _failed(state)
    state.project = project  # same definition and resolved airframe, only the project directory changed
    state.definition_revision += 1  # accepted definition change: the editors rebuild against the copy
    try:
        state.save()
    except OSError as error:
        state.log_error("Save Project As", f"could not write {state.project.build_path}: {error}")
        return _failed(state)
    state.notify()  # the title in the pushed state changed
    return web.json_response(snapshot(state))


async def handle_editor_context(request: web.Request) -> web.Response:
    """Everything the forms need that the state snapshot does not carry: part library, presets, ported Tk constants."""
    return web.json_response(context(_state(request)))


async def handle_editor_set(request: web.Request) -> web.Response:
    """One atomic path edit on the current definition (a field commit or an optional-key removal).

    A command that names another session is refused with the current snapshot before anything runs; a path that
    does not resolve is logged and refused with the untouched snapshot (400). Success recomputes and pushes.
    """
    state = _state(request)
    body = await _json_object(request)
    try:
        apply_set(state, body)
    except SessionMismatch:
        return web.json_response(snapshot(state), status=409)
    except ValueError as error:
        state.log_error("Edit", str(error))
        return _failed(state)
    return web.json_response(snapshot(state))


async def handle_editor_op(request: web.Request) -> web.Response:
    """One structural editor operation (section cuts, presets); unknown ops are refused, refused work is logged.

    Like a set, an op carries the session it was issued from; a mismatch is refused with the current snapshot
    (409) instead of running the op against the project that the browser has since left.
    """
    state = _state(request)
    body = await _json_object(request)
    try:
        apply_op(state, body)
    except SessionMismatch:
        return web.json_response(snapshot(state), status=409)
    except ValueError as error:
        raise web.HTTPBadRequest(text=str(error)) from error
    except OpFailed:
        state.notify()  # the Log entry must reach the open websockets even though nothing recomputed
        return _failed(state)
    return web.json_response(snapshot(state))


async def handle_editor_move(request: web.Request) -> web.Response:
    """One drag motion on the current definition (POST /api/editor/move), applied with the Tk clamps.

    A command from another session is refused with the current snapshot (409). An unknown target, index or path is
    logged and refused with the untouched snapshot (400). An applied move answers with the geometry-recomputed
    snapshot; the browser's release then posts /api/recompute for the full aero solve. With a valid width/height
    (the drag fast path) the move instead answers with the /api/draw payload and suppresses the websocket push,
    so one in-flight drag step costs one request and no client wake-up.
    """
    state = _state(request)
    body = await _json_object(request)
    size = _draw_size(body)
    try:
        apply_move(state, body, notify=size is None)
    except SessionMismatch:
        return web.json_response(snapshot(state), status=409)
    except ValueError as error:
        state.log_error("Move", str(error))
        return _failed(state)
    if size is None:
        return web.json_response(snapshot(state))
    return web.json_response(draw(state, size[0], size[1], {}))


async def handle_library(request: web.Request) -> web.Response:
    """The part library and its form metadata (webapp.library.library_payload), session-agnostic like /api/recompute."""
    return web.json_response(library_payload(_state(request)))


async def handle_library_save(request: web.Request) -> web.Response:
    """One library save (webapp.library.save_part): a New part or an edit, rename or category move.

    bad input is logged and refused with the untouched snapshot (400); a write failure is the middleware's 500.
    """
    state = _state(request)
    body = await _json_object(request)
    try:
        save_part(state, body)
    except ValueError as error:
        state.log_error("Part Library", str(error))
        return _failed(state)
    return web.json_response(snapshot(state))


async def handle_library_delete(request: web.Request) -> web.Response:
    """One library delete (webapp.library.delete_part) naming {category, name}; refusals as save."""
    state = _state(request)
    body = await _json_object(request)
    try:
        delete_part(state, body)
    except ValueError as error:
        state.log_error("Part Library", str(error))
        return _failed(state)
    return web.json_response(snapshot(state))


def _draw_size(body: dict[str, Any]) -> tuple[int, int] | None:
    """The drag fast path's optional width/height: both plain positive ints, or None for the snapshot path."""
    width, height = body.get("width"), body.get("height")
    if type(width) is int and type(height) is int and width > 0 and height > 0:
        return width, height
    return None


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _apply_action(view: ViewState, action: dict[str, Any]) -> None:
    """The Tk view commands (libs/views/editor_view.py), dispatched by the browser's action kind."""
    kind = action.get("kind")
    if kind == "redraw":
        return
    if kind == "reset":
        view.reset()
        return
    if kind == "pan_delta":
        dx, dy = _number(action.get("dx")), _number(action.get("dy"))
        if dx is None or dy is None:
            raise ValueError("pan_delta needs numeric dx and dy")
        view.pan_delta(dx, dy)
        return
    if kind == "zoom_at":
        x, y, factor = _number(action.get("x")), _number(action.get("y")), _number(action.get("factor"))
        if x is None or y is None or factor is None or factor <= 0.0:
            raise ValueError("zoom_at needs numeric x, y and a positive factor")
        view.zoom_at(x, y, factor)
        return
    if kind == "scroll":
        axis, mode, value = action.get("axis"), action.get("mode"), _number(action.get("value"))
        if axis not in ("x", "y") or mode not in ("moveto", "units", "pages") or value is None:
            raise ValueError("scroll needs axis x|y, mode moveto|units|pages and a numeric value")
        view.scroll(axis, mode, value)
        return
    if kind == "set_mode":
        mode = action.get("mode")
        if mode not in ("Longitudinal", "Front"):
            raise ValueError("set_mode needs mode Longitudinal or Front")
        view.set_mode(mode)
        return
    if kind == "set_roll":
        roll_deg = _number(action.get("roll_deg"))
        if roll_deg is None:
            raise ValueError("set_roll needs a numeric roll_deg")
        view.set_roll(roll_deg)
        return
    raise ValueError(f"unknown action {kind!r}")


async def handle_draw(request: web.Request) -> web.Response:
    state = _state(request)
    try:
        body = await request.json()
    except (json.JSONDecodeError, ValueError):
        raise web.HTTPBadRequest(text="body must be a JSON object")
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text="body must be a JSON object")
    # Optional session fence: a stale draw must not move the current session's view, so it is refused before the
    # action runs. The answer is the current snapshot; the draw payload also echoes the session id.
    if "session_id" in body:
        session_id = body["session_id"]
        if type(session_id) is not int or session_id != state.session_id:
            return web.json_response(snapshot(state), status=409)
    width, height = _number(body.get("width")), _number(body.get("height"))
    action = body.get("action")
    if width is None or height is None or not isinstance(action, dict):
        raise web.HTTPBadRequest(text="width, height and action are required")
    view = ensure_view(state)
    try:
        _apply_action(view, action)
    except ValueError as error:
        raise web.HTTPBadRequest(text=str(error)) from error
    return web.json_response(draw(state, width, height, {}))


async def handle_model3d(request: web.Request) -> web.Response:
    """The live 3D view's mesh payload (webapp.model3d.build_model3d) for the current session.

    Like a draw, the command carries the session it was issued from; a mismatch is refused with the current
    snapshot (409). With no resolved airframe yet (or after a definition error that left none), the answer is
    also the snapshot with 409, so the client resyncs instead of showing a stale airframe.
    """
    state = _state(request)
    body = await _json_object(request)
    try:
        check_session(state, body)
    except SessionMismatch:
        return web.json_response(snapshot(state), status=409)
    if state.resolved is None:
        return web.json_response(snapshot(state), status=409)
    return web.json_response(build_model3d(state.resolved, state.aero))


async def handle_events(request: web.Request) -> web.WebSocketResponse:
    state = _state(request)
    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)
    sockets: set[web.WebSocketResponse] = request.app["websockets"]
    sockets.add(ws)

    outbox: asyncio.Queue[None] = asyncio.Queue()

    def notify() -> None:
        outbox.put_nowait(None)

    async def writer() -> None:
        try:
            while True:
                await outbox.get()
                await ws.send_json({"type": "state", **snapshot(state)})
        except (ConnectionError, asyncio.CancelledError):  # the socket closed under us at exit
            pass

    state.add_listener(notify)
    writer_task = asyncio.create_task(writer())
    try:
        await ws.send_json({"type": "state", **snapshot(state)})
        async for message in ws:
            if message.type == WSMsgType.ERROR:
                break
    finally:
        sockets.discard(ws)
        state.remove_listener(notify)
        writer_task.cancel()
    return ws


async def handle_dist(request: web.Request) -> web.StreamResponse:
    tail = request.match_info["tail"]
    if tail.startswith("api/") or tail.startswith("ws/"):
        raise web.HTTPNotFound()
    dist_dir: Path = request.app["dist_dir"]
    root = dist_dir.resolve()
    candidate = (root / tail).resolve() if tail else root / "index.html"
    if tail and candidate.is_file() and candidate.is_relative_to(root):
        return web.FileResponse(candidate)
    # The SPA fallback is for extension-less routes only: a missing asset or file must 404, not answer HTML.
    last_segment = tail.rsplit("/", 1)[-1]
    if tail.startswith("assets/") or "." in last_segment:
        raise web.HTTPNotFound()
    return web.FileResponse(root / "index.html", headers={"Cache-Control": "no-cache"})
