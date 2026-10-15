"""CAD export and Compute Geometry for the browser frontend, called from a server worker thread.

Every OpenCascade task runs in the CAD worker process (webapp.cad_worker), which holds the built solids; the server
process never imports cadquery, so its event loop (the UI) stays responsive while a build runs. A single
module-level lock serializes the tasks. Each export returns the project-relative files it wrote, the notes to show
and log, and one line of detail for the status line.

The server owns the run, not the browser: the record of the current (or last) run, with every progress step the
worker reported and when, and the last outcome of each export, live here and are what GET /api/cad/state answers,
so a view that was closed while an export ran shows it again when it comes back. A failure is re-raised as
ExportFailed carrying the task and the progress label (the worker's traceback as its cause), so the server logs what
was being made. Varsets are plain JSON from the definition and are written here.
"""

from __future__ import annotations

import contextlib
import subprocess
import threading
import time
from collections.abc import Iterator
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from libs.computed_geometry import MeasuredDesign, measured, save_measured
from libs.geometry import ResolvedAirframe
from libs.project import Project
from webapp.cad_worker import CadWorker


class ExportFailed(RuntimeError):
    """One CAD export failed, with the task and the progress label it failed on in the message."""


# One task at a time: the CAD worker runs one, and its solid store is shared by every request.
_EXPORT_LOCK = threading.Lock()
WORKER = CadWorker()

ESTIMATE_MIN_STEPS = 3  # progress steps of one phase before its pace is taken as an estimate of the rest


@dataclass(frozen=True)
class Step:
    """One progress message of the running task, as the worker sent it, and when it arrived (s since the start)."""
    label: str
    done: int
    total: int
    at_s: float


@dataclass
class Run:
    """The current or last CAD task: step/stl/sections/drawings/varsets/geometry, with the options it was started with."""
    task: str
    options: dict[str, Any]
    started: float  # time.monotonic() at the start
    started_at: float  # time.time() at the start, for the clock time the view shows
    steps: list[Step] = field(default_factory=list)
    state: str = "running"  # running | done | failed
    message: str = ""  # the result detail, or the failure line
    ended_s: float | None = None  # duration once it ended


@dataclass(frozen=True)
class Outcome:
    """The last result of one export task, for that export's row (project: the project it wrote into; message: the
    result line, or for a failure the step it failed at; files: the files this export wrote; notes: its problems, also
    in the Log)."""
    project: str
    ok: bool
    message: str
    files: list[str]
    notes: list[str]
    finished_at: float


# _EXPORT_LOCK serializes the work; this lock keeps the run record and the outcomes consistent between the exporting
# worker thread and the polling route.
_RECORD_LOCK = threading.Lock()
_RUN: Run | None = None
_OUTCOMES: dict[str, Outcome] = {}


def begin(task: str, options: dict[str, Any] | None = None, *, continues: bool = False) -> None:
    """Start the run record for one task; continues keeps the steps and the clock of the run before it (the second
    stage of Compute Geometry)."""
    global _RUN
    with _RECORD_LOCK:
        if continues and _RUN is not None and _RUN.task == task:
            _RUN.state, _RUN.message, _RUN.ended_s = "running", "", None
        else:
            _RUN = Run(task=task, options=dict(options or {}), started=time.monotonic(), started_at=time.time())


def report(label: str, done: int, total: int) -> None:
    """One step of the running task: the CAD worker's progress messages, or libs.varsets here."""
    with _RECORD_LOCK:
        if _RUN is not None:
            _RUN.steps.append(Step(label=label, done=int(done), total=int(total), at_s=time.monotonic() - _RUN.started))


def end(state: str, message: str) -> None:
    """Close the run record: done or failed, with the result detail or the failure line."""
    with _RECORD_LOCK:
        if _RUN is not None:
            _RUN.state, _RUN.message, _RUN.ended_s = state, message, time.monotonic() - _RUN.started


def _phase(step: Step) -> tuple[str, int]:
    """The phase a step belongs to: its verb ("building S03" -> "building") and its total."""
    return step.label.split(" ", 1)[0], step.total


def _estimate(run: Run, now_s: float) -> dict[str, Any] | None:
    """Time left in the current phase at the pace of its steps so far, or None while there are too few to tell.

    Only the current phase: a later phase (writing after building) has its own pace that is not known yet. The
    parts of a phase are not alike (a section with hatches takes longer than a control surface), so this is a pace,
    not a promise; the view says so.
    """
    if run.state != "running" or not run.steps:
        return None
    phase = _phase(run.steps[-1])
    steps = []
    for step in reversed(run.steps):
        if _phase(step) != phase:
            break
        steps.append(step)
    first, last = steps[-1], steps[0]
    if len(steps) < ESTIMATE_MIN_STEPS or last.done <= first.done or last.total <= 0:
        return None
    per_step = (last.at_s - first.at_s) / (last.done - first.done)
    left = per_step * (last.total - last.done) - (now_s - last.at_s)
    return {"phase": phase[0], "left_s": max(0.0, left), "per_step_s": per_step}


def current() -> dict[str, Any]:
    """GET /api/cad/state: the current or last run (with its steps, elapsed time and estimate), the last outcome of
    each export, and the worker's settings."""
    with _RECORD_LOCK:
        run = None
        if _RUN is not None:
            now_s = time.monotonic() - _RUN.started
            run = asdict(_RUN)
            del run["started"]
            run["elapsed_s"] = _RUN.ended_s if _RUN.ended_s is not None else now_s
            run["estimate"] = _estimate(_RUN, now_s)
        outcomes = {task: asdict(outcome) for task, outcome in _OUTCOMES.items()}
    return {"run": run, "outcomes": outcomes, "worker": WORKER.settings()}


def running_task() -> str | None:
    """The task of the run in flight, or None."""
    with _RECORD_LOCK:
        return _RUN.task if _RUN is not None and _RUN.state == "running" else None


class _Export:
    """What one export hands back to _exporting: its result dict (files, notes, detail), set before the block ends."""

    def __init__(self) -> None:
        self.result: dict[str, Any] | None = None


@contextlib.contextmanager
def _exporting(task: str, project: Project | None, options: dict[str, Any] | None = None, *, continues: bool = False) -> Iterator[_Export]:
    """Bracket one task with begin/end, keep its outcome for its card, and name the failure context: "CAD <task>
    export failed while <label>".

    The label is the last step the worker reported, so the message says what the export was making when it raised
    (e.g. "building S03"); an export that fails before its first report reads "while starting". The original error
    stays as the cause, so the server logs the whole chain. project None: Compute Geometry, whose outcome is the
    snapshot's geometry status, not a card.
    """
    begin(task, options, continues=continues)
    export = _Export()
    try:
        yield export
    except Exception as error:
        with _RECORD_LOCK:
            label = _RUN.steps[-1].label if _RUN is not None and _RUN.steps else "starting"
        failure = ExportFailed(f"CAD {task} export failed while {label}")
        end("failed", str(failure))
        if project is not None:
            _keep(task, Outcome(project=project.name, ok=False, message=label, files=[], notes=[], finished_at=time.time()))
        raise failure from error
    result = export.result or {"files": [], "notes": [], "detail": ""}
    end("done", result["detail"])
    if project is not None:
        _keep(task, Outcome(project=project.name, ok=True, message=result["detail"], files=list(result["files"]),
                            notes=list(result["notes"]), finished_at=time.time()))


def _keep(task: str, outcome: Outcome) -> None:
    with _RECORD_LOCK:
        _OUTCOMES[task] = outcome


def output_dir(project: Project, task: str) -> Path:
    """The folder an export task writes into (the CAD view's Open folder)."""
    return {"step": project.cad_dir, "stl": project.cad_dir, "sections": project.sections_dir,
            "drawings": project.cad_dir, "varsets": project.varsets_dir}[task]


def open_folder(project: Project, task: str) -> Path:
    """Open the task's output folder in the desktop's file manager (xdg-open; the server runs on the owner's machine)."""
    directory = output_dir(project, task)
    directory.mkdir(parents=True, exist_ok=True)
    subprocess.Popen(["xdg-open", str(directory)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    return directory


def _relative(project: Project, path: Path) -> str:
    """path as a project-relative POSIX string, the form the /files route and the UI links use (e.g. cad/x.step)."""
    return path.resolve().relative_to(project.root.resolve()).as_posix()


def _files_in(project: Project, directory: Path, suffixes: tuple[str, ...], since: float) -> list[str]:
    """The files with one of suffixes in directory written since the time.time() since: what this export wrote, not the
    parts an earlier export of another design left there."""
    if not directory.is_dir():
        return []
    return [
        _relative(project, path)
        for path in sorted(directory.iterdir())
        if path.is_file() and path.suffix.lower() in suffixes and path.stat().st_mtime >= since
    ]


def measure_geometry(resolved: ResolvedAirframe, project: Project, parts_data: dict) -> MeasuredDesign:
    """Compute Geometry, first stage: build the printable parts and measure the exact printed mass, saved to
    cad/exact_mass.json, so the editor can apply it before the rest is built."""
    with _EXPORT_LOCK, _exporting("geometry", None):
        design = measured(resolved, parts_data, WORKER.run("measure", resolved, parts_data, progress=report))
        save_measured(project.cad_dir, design)
    return design


def build_geometry(resolved: ResolvedAirframe, parts_data: dict) -> None:
    """Compute Geometry, second stage: every other solid the exports use (airframe pieces, fused shape, drawing
    projections), held by the CAD worker so the exports of this design only write files. Its steps continue the
    first stage's run record."""
    with _EXPORT_LOCK, _exporting("geometry", None, continues=True):
        WORKER.run("build", resolved, parts_data, progress=report)


def geometry_current(resolved: ResolvedAirframe) -> bool:
    """True when Compute Geometry finished for this design and the CAD worker still holds its solids."""
    return WORKER.holds_complete(resolved)


def export_step(resolved: ResolvedAirframe, project: Project, mode: str = "assembly", include_prop_discs: bool = False) -> dict[str, Any]:
    """STEP of the whole airframe to cad/<project>.step; returns the mode actually written ("assembly"/"fused"/"compound")."""
    path = project.cad_dir / f"{project.name}.step"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("step", project, {"mode": mode, "include_prop_discs": include_prop_discs}) as export:
        written_mode = WORKER.run("step", resolved, path, mode, include_prop_discs, progress=report)
        follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
        notes = [follows] if follows else []
        detail = f"STEP written ({written_mode}) to {_relative(project, path)}" + (f" — {follows}" if notes else "")
        export.result = {"files": [_relative(project, path)], "notes": notes, "detail": detail, "mode": written_mode}
    return export.result


def export_stl(resolved: ResolvedAirframe, project: Project, include_prop_discs: bool = False) -> dict[str, Any]:
    """The whole airframe as one STL mesh to cad/<project>.stl (always fused where possible)."""
    path = project.cad_dir / f"{project.name}.stl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("stl", project, {"include_prop_discs": include_prop_discs}) as export:
        written_mode = WORKER.run("stl", resolved, path, include_prop_discs, progress=report)
        follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
        notes = [follows] if follows else []
        detail = f"STL written to {_relative(project, path)}" + (f" — {follows}" if notes else "")
        export.result = {"files": [_relative(project, path)], "notes": notes, "detail": detail}
    return export.result


def export_sections(resolved: ResolvedAirframe, project: Project, export_format: str = "step") -> dict[str, Any]:
    """Printable sections S01.. and couplers C01.. as STEP or STL (export_format) into cad/sections/; problems are notes
    (the route logs them)."""
    if export_format not in ("step", "stl"):
        raise ValueError(f"export_format must be 'step' or 'stl', not {export_format!r}")
    directory = project.sections_dir
    directory.mkdir(parents=True, exist_ok=True)
    started = time.time()
    with _EXPORT_LOCK, _exporting("sections", project, {"format": export_format}) as export:
        written, problems = WORKER.run("sections", resolved, directory, export_format, progress=report)
        detail = f"{len(written)} {export_format.upper()} parts written to {_relative(project, directory)}"
        if problems:
            detail += f"; {len(problems)} problems in the Log"
        export.result = {"files": _files_in(project, directory, (f".{export_format}",), started), "notes": list(problems), "detail": detail}
    return export.result


def export_drawings(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """The cutaway, three-view and x-ray drawings (.svg + .png) into cad/."""
    directory = project.cad_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("drawings", project) as export:
        written = WORKER.run("drawings", resolved, directory, progress=report)
        files = [_relative(project, path) for path in written]
        export.result = {"files": files, "notes": [], "detail": "Drawings written: " + ", ".join(files)}
    return export.result


def export_varsets(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """One FreeCAD varset JSON per varset into cad/varsets/ (libs.varsets uses the definition's values)."""
    from libs.varsets import export_varsets as write_varsets

    directory = project.varsets_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("varsets", project) as export:
        written = write_varsets(resolved.airframe, directory, progress=report)
        files = [_relative(project, path) for path in written]
        detail = f"{len(files)} varsets written to {_relative(project, directory)}: " + ", ".join(Path(file).stem for file in files)
        export.result = {"files": files, "notes": [], "detail": detail}
    return export.result
