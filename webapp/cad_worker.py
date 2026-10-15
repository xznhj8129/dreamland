"""The CAD worker: one long-lived process that builds and keeps every CAD solid, so the web server never runs OCCT.

An OpenCascade boolean holds the GIL for seconds; run in a thread of the server it starves the event loop, so the UI
lags and progress polls do not get through. Every CAD task (Compute Geometry, STEP, STL, sections, drawings) runs here
instead, at lower CPU priority and pinned to at most WORKER_CPUS logical CPUs (the CAD view's CPU setting), and libs.solid_cache lives in this process: what one task built, the next task of the
same design reuses. Solids never leave the process; tasks take the resolved airframe and send back numbers, file
lists and progress messages, which webapp.cad relays to its progress record. The process starts on the first task
and is restarted after it dies (its built solids are gone then).
"""

from __future__ import annotations

import contextlib
import multiprocessing
import os
import traceback
from collections.abc import Callable
from pathlib import Path
from typing import Any

from libs.geometry import ResolvedAirframe
from libs.solid_cache import design_digest

WORKER_NICE = 10  # added to the server's nice value (at most 19): below the server and the desktop, so a long build does not take the machine
WORKER_CPUS = 2  # logical CPUs the worker (and OpenCascade's threads) may run on: a sustained build never loads the whole CPU
# WORKER_CPUS is also the most the CPU setting allows; it can only go lower. The priority stays fixed: an unprivileged
# process cannot lower its nice value again once raised, so a priority setting could not be undone without a restart.

Progress = Callable[[str, int, int], None]


class WorkerError(RuntimeError):
    """A task failed in the CAD worker (the message is its traceback there), or the worker process died."""


# Tasks, run in the worker process. Each takes the resolved airframe first and the progress callback by keyword.

def _measure(resolved: ResolvedAirframe, parts_data: dict, *, progress: Progress):
    from libs.computed_geometry import exact_structure

    return exact_structure(resolved, parts_data, progress=progress)


def _build(resolved: ResolvedAirframe, parts_data: dict, *, progress: Progress) -> None:
    from libs.computed_geometry import compute_geometry

    compute_geometry(resolved, parts_data, progress=progress)


def _step(resolved: ResolvedAirframe, path: Path, mode: str, include_prop_discs: bool, *, progress: Progress) -> str:
    from libs.cad import export_step

    return export_step(resolved, path, mode=mode, include_prop_discs=include_prop_discs, progress=progress)


def _stl(resolved: ResolvedAirframe, path: Path, include_prop_discs: bool, *, progress: Progress) -> str:
    from libs.cad import export_stl

    return export_stl(resolved, path, include_prop_discs=include_prop_discs, progress=progress)


def _sections(resolved: ResolvedAirframe, directory: Path, export_format: str, *, progress: Progress):
    from libs.cad import export_sections

    return export_sections(resolved, directory, export_format, progress=progress)


def _drawings(resolved: ResolvedAirframe, directory: Path, *, progress: Progress) -> list[Path]:
    from libs.drawings import export_drawings

    return export_drawings(resolved, directory, progress=progress)


TASKS = {"measure": _measure, "build": _build, "step": _step, "stl": _stl, "sections": _sections, "drawings": _drawings}


def worker_cpu_ids(count: int) -> list[int]:
    """The logical CPUs a worker with count CPUs runs on: the first count of the server's own."""
    return sorted(os.sched_getaffinity(0))[:count]


def _pin(pid: int, cpu_ids: list[int]) -> None:
    """Pin every thread of process pid (its OpenCascade threads too: affinity is per thread on Linux)."""
    for tid in os.listdir(f"/proc/{pid}/task"):
        with contextlib.suppress(ProcessLookupError):  # a thread that ended since the listing
            os.sched_setaffinity(int(tid), cpu_ids)


def _serve(connection, cpu_ids: list[int]) -> None:
    """The worker process: one task at a time, progress and the result (or the traceback) sent back."""
    os.nice(WORKER_NICE)
    os.sched_setaffinity(0, cpu_ids)  # threads OCCT starts later inherit it
    while True:
        try:
            task, args = connection.recv()
        except EOFError:  # the server went away
            return

        def progress(label: str, done: int, total: int) -> None:
            connection.send(("progress", label, done, total))

        try:
            result = TASKS[task](*args, progress=progress)
        except Exception:
            connection.send(("error", traceback.format_exc()))
        else:
            connection.send(("done", result))


class CadWorker:
    """The server's handle on the worker process: run one task, relaying its progress; which design the worker's
    solids are of, and whether Compute Geometry completed for it; how many CPUs it may use. Callers serialize the tasks
    (webapp.cad's lock)."""

    def __init__(self) -> None:
        self.cpus = WORKER_CPUS
        self._process = None
        self._connection = None
        self._held_digest: str | None = None  # the design whose solids the worker holds (libs.solid_cache keeps one)
        self._complete_digest: str | None = None  # the design Compute Geometry finished building

    def run(self, task: str, resolved: ResolvedAirframe, *args: Any, progress: Progress) -> Any:
        digest = design_digest(resolved)
        if self._process is None or not self._process.is_alive():
            self._start()
        if digest != self._held_digest:  # the worker's store drops the other design's solids for this one
            self._held_digest, self._complete_digest = digest, None
        self._connection.send((task, (resolved, *args)))
        while True:
            try:
                kind, *payload = self._connection.recv()
            except EOFError as error:
                self._process = self._connection = None
                self._held_digest = self._complete_digest = None
                raise WorkerError("the CAD worker process exited") from error
            if kind == "progress":
                progress(*payload)
            elif kind == "error":
                raise WorkerError(payload[0])
            else:
                if task == "build":
                    self._complete_digest = digest
                return payload[0]

    def holds_complete(self, resolved: ResolvedAirframe) -> bool:
        """True when Compute Geometry finished for this design and the worker still holds its solids."""
        return self._complete_digest is not None and self._complete_digest == design_digest(resolved)

    def set_cpus(self, count: int) -> None:
        """Let the worker use count CPUs (1..WORKER_CPUS); a live worker is re-pinned at once, mid-task too."""
        if not 1 <= count <= WORKER_CPUS:
            raise ValueError(f"the CAD worker may use 1 to {WORKER_CPUS} CPUs, not {count}")
        self.cpus = count
        if self._process is not None and self._process.is_alive():
            _pin(self._process.pid, worker_cpu_ids(count))

    def settings(self) -> dict[str, Any]:
        """What the CAD view shows of the worker: its CPUs (and which), the most it may use, its fixed priority (the
        nice value it runs at: WORKER_NICE above the server's own, at most 19)."""
        alive = self._process is not None and self._process.is_alive()
        nice = os.getpriority(os.PRIO_PROCESS, self._process.pid) if alive else min(19, os.getpriority(os.PRIO_PROCESS, 0) + WORKER_NICE)
        return {"cpus": self.cpus, "max_cpus": WORKER_CPUS, "cpu_ids": worker_cpu_ids(self.cpus), "nice": nice, "alive": alive}

    def _start(self) -> None:
        context = multiprocessing.get_context("spawn")  # a clean interpreter: no server threads or loop inherited
        parent, child = context.Pipe()
        self._process = context.Process(target=_serve, args=(child, worker_cpu_ids(self.cpus)), name="cad-worker", daemon=True)
        self._process.start()
        child.close()
        self._connection = parent
        self._held_digest = self._complete_digest = None
