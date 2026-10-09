"""Server-side state for the browser frontend: one project, mirroring guibuilder.BuilderApp.

Every mutation runs on the aiohttp event loop except the aero solve, which runs in a daemon thread and hands
its result back through a queue drained on the loop (guibuilder.BuilderApp.poll_aero_queue).
"""

from __future__ import annotations

import asyncio
import json
import queue
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Any, Callable

from libs.aero import AeroResult, NoLiftingSurfacesError, OperatingPointError, analyze_aero
from libs.airframe import AirframeError, load_parts_data, parse_airframe
from libs.geometry import ResolvedAirframe, resolve_airframe
from libs.motorlib import MotorStalled
from libs.project import BUILD_FILE, Project
from libs.propulsion import PropulsionSummary, propulsion_summary
from libs.report import build_report_lines
from libs.reportline import ReportLine

AeroQueueItem = tuple[int, AeroResult | None, str | None, str | None]


@dataclass(frozen=True)
class LogEntry:
    time: str
    title: str
    text: str


class AppState:
    """The loaded definition, its resolved airframe and the background aero result, for one project."""

    def __init__(self, project: Project) -> None:
        self.project = project
        self.parts_data = load_parts_data()
        with open(project.build_path, "r") as file:
            self.definition: dict[str, Any] = json.load(file)
        self.resolved: ResolvedAirframe | None = None
        self.problems: list[str] = []
        self.propulsion: PropulsionSummary | None = None
        self.aero: AeroResult | None = None
        self.aero_generation = 0
        self.session_id = 0  # bumped on open/new/switch; the browser keys its forms subtree on it so they remount
        self.definition_revision = 0  # bumped on every accepted definition change; editors sync their fields from it
        self.library_revision = 0  # bumped on every accepted part-library save/delete; invalidates the cached editor context
        self.definition_valid = False  # True when the current definition parsed and resolved; False = invalid user text behind a last-valid resolved
        self.status = ""
        self.report: list[ReportLine] = []
        self.log: list[LogEntry] = []
        self._last_log_entry: tuple[str, str] | None = None
        self._aero_queue: queue.Queue[AeroQueueItem] = queue.Queue()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._listeners: list[Callable[[], None]] = []

    @property
    def title(self) -> str:
        return f"quadrocketbuilder - {self.project.name} ({self.definition.get('name', 'unnamed')})"

    def add_listener(self, listener: Callable[[], None]) -> None:
        self._listeners.append(listener)

    def remove_listener(self, listener: Callable[[], None]) -> None:
        self._listeners.remove(listener)

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Start notifying from the loop; also drains anything the aero thread queued before it existed."""
        self._loop = loop
        loop.call_soon(self._aero_ready)

    def notify(self) -> None:
        for listener in list(self._listeners):
            listener()

    def notify_threadsafe(self) -> None:
        loop = self._loop
        if loop is None or loop.is_closed() or not loop.is_running():
            return
        try:
            loop.call_soon_threadsafe(self._aero_ready)
        except RuntimeError:  # loop closed between the check and the call
            pass

    def _aero_ready(self) -> None:
        if self._drain_aero_queue():
            self.notify()

    def _drain_aero_queue(self) -> bool:
        changed = False
        while not self._aero_queue.empty():
            generation, aero, invalid_text, error_text = self._aero_queue.get()
            if generation != self.aero_generation:
                continue
            if invalid_text is not None:
                self.log_error("Aero not computed", invalid_text)
            elif error_text is not None:
                self.log_error("Aero failed", error_text)
            else:
                assert self.resolved is not None and aero is not None
                self.aero = aero
                self.report = build_report_lines(self.resolved, aero, self.problems, self.propulsion, sweep=True)
                self.status = "Up to date"
            changed = True
        return changed

    def log_error(self, title: str, text: str) -> None:
        """Errors go to the Log; an unchanged repeat is already there (guibuilder.BuilderApp.log_error)."""
        self.status = f"{title} (see Log)"
        if (title, text) == self._last_log_entry:
            return
        self._last_log_entry = (title, text)
        self.log.append(LogEntry(time=time.strftime("%H:%M:%S"), title=title, text=text))

    def switch_project(self, project: Project, definition: dict[str, Any]) -> None:
        """Point the state at another project's definition and recompute it (guibuilder.BuilderApp.switch_project).

        Everything the last project showed is dropped first, so no stale resolved data, report or log dedupe
        survives the switch; the drawing view transform resets as in the Tk editor.
        """
        from webapp.drawing import ensure_view  # local: webapp.drawing imports AppState

        self.project = project
        self.definition = definition
        self.session_id += 1
        self.definition_revision += 1
        self.resolved = None
        self.problems = []
        self.propulsion = None
        self.aero = None
        self.report = []
        self.status = ""
        self._last_log_entry = None
        ensure_view(self).reset()
        self.recompute()

    def set_definition(self, definition: dict[str, Any], run_aero: bool = True, notify: bool = True) -> None:
        """Adopt an edited definition (the user's text, valid or not) and recompute.

        The revision bumps whatever the outcome, so the editors rebuild once per accepted edit; a failure keeps
        the invalid text in place with the last valid resolved airframe behind it, as Tk does. Drag motions pass
        run_aero=False: they recompute geometry only, exactly like the Tk drag. notify=False suppresses the
        websocket push (the fast drag path renders the returned drawing itself); the revision still bumps.
        """
        self.definition = definition
        self.definition_revision += 1
        # Keep the historical one-keyword call for notify=True so test/embedding monkeypatches of recompute hold.
        if notify:
            self.recompute(run_aero=run_aero)
        else:
            self.recompute(run_aero=run_aero, notify=False)

    def recompute(self, run_aero: bool = True, notify: bool = True) -> None:
        """Validate and resolve the definition now; aero follows in a background thread unless run_aero is False.

        A failure is logged and leaves the last valid resolved/report/aero in place (guibuilder.recompute plus the
        global safety net in handle_callback_exception); it never propagates to the caller. notify=False skips
        every push for this call (the caller answers with the drawing instead).
        """
        self.aero_generation += 1
        try:
            airframe = parse_airframe(self.definition, self.parts_data, path=f"{self.project.name}/{BUILD_FILE}")
            resolved, problems = resolve_airframe(airframe, self.parts_data)
        except AirframeError as error:
            self.definition_valid = False
            self.log_error("Definition error", f"{error}\nThe drawing shows the last valid airframe.")
            if notify:
                self.notify()
            return
        except Exception:  # unexpected parse/resolve failure: log the traceback, keep the last valid state
            self.definition_valid = False
            self.log_error("Definition error", traceback.format_exc())
            if notify:
                self.notify()
            return
        self.definition_valid = True
        self.resolved = resolved
        self.problems = problems
        self.propulsion = None
        if airframe.propulsion is not None:
            try:
                self.propulsion = propulsion_summary(resolved, self.parts_data)
            except MotorStalled as error:
                self.log_error("Propulsion", str(error))
            except Exception:  # an unexpected propulsion failure must not abort the recompute
                self.log_error("Error", traceback.format_exc())
        self.aero = None
        # The mass/geometry/layout report is rebuilt now; the aero report replaces it when the thread lands.
        self.report = build_report_lines(resolved, None, self.problems, self.propulsion, sweep=True)
        if not resolved.surfaces:
            self.status = "No lifting surfaces"
            if notify:
                self.notify()
            return
        if not run_aero:
            self.status = "Aero pending"
            if notify:
                self.notify()
            return
        self.status = "Computing aero..."
        if notify:
            self.notify()
        threading.Thread(target=self._run_aero, args=(self.aero_generation, resolved), daemon=True).start()

    def _run_aero(self, generation: int, resolved: ResolvedAirframe) -> None:
        try:
            self._aero_queue.put((generation, analyze_aero(resolved), None, None))
        except (OperatingPointError, NoLiftingSurfacesError) as error:  # show the reason, not a traceback
            self._aero_queue.put((generation, None, str(error), None))
        except Exception:  # thread boundary: hand the traceback back for the Log
            self._aero_queue.put((generation, None, None, traceback.format_exc()))
        self.notify_threadsafe()

    def save(self) -> None:
        with open(self.project.build_path, "w") as file:
            json.dump(self.definition, file, indent=4)
