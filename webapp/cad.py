"""CAD export and Compute Geometry for the browser frontend, called from a server worker thread.

Every OpenCascade task runs in the CAD worker process (webapp.cad_worker), which holds the built solids; the server
process never imports cadquery, so its event loop (the UI) stays responsive while a build runs. A single
module-level lock serializes the tasks. Each export returns the project-relative files it wrote, the notes to show
and log, and one line of detail for the status line. Each task owns the progress record (begin/report/end), fed by
the worker's progress messages, that the views poll through GET /api/cad/progress; a failure is re-raised as
ExportFailed carrying the task and the progress label (the worker's traceback as its cause), so the server logs what
was being made. Varsets are plain JSON from the definition and are written here.
"""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Iterator
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

# The one export in flight, for GET /api/cad/progress. _EXPORT_LOCK serializes the work; this lock keeps the
# record consistent between the exporting worker thread and the polling route.
_PROGRESS_LOCK = threading.Lock()
_PROGRESS: dict[str, Any] = {"running": False, "task": None, "label": None, "done": 0, "total": 0}


def begin(task: str) -> None:
    """Start the progress record for one export (task: step/stl/sections/drawings/varsets/geometry)."""
    with _PROGRESS_LOCK:
        _PROGRESS.update(running=True, task=task, label=None, done=0, total=0)


def report(label: str, done: int, total: int) -> None:
    """One step of the running task: the CAD worker's progress messages, or libs.varsets here."""
    with _PROGRESS_LOCK:
        _PROGRESS.update(label=label, done=int(done), total=int(total))


def end() -> None:
    """Clear the record, whether the export succeeded or failed."""
    with _PROGRESS_LOCK:
        _PROGRESS.update(running=False, task=None, label=None, done=0, total=0)


def current() -> dict[str, Any]:
    """A snapshot of the record: the idle shape when no export runs."""
    with _PROGRESS_LOCK:
        return dict(_PROGRESS)


@contextlib.contextmanager
def _exporting(task: str) -> Iterator[None]:
    """Bracket one export with begin/end and name the failure context: "CAD <task> export failed while <label>".

    The label is read before end() clears the record, so the message says what the export was making when it
    raised (e.g. "building S03"); an export that fails before its first report reads "while starting". The
    original error stays as the cause, so the server logs the whole chain.
    """
    begin(task)
    try:
        yield
    except Exception as error:
        label = current()["label"] or "starting"
        raise ExportFailed(f"CAD {task} export failed while {label}") from error
    finally:
        end()


CAD_DIR = "cad"
SECTIONS_DIR = "sections"
VARSETS_DIR = "varsets"


def _relative(project: Project, path: Path) -> str:
    """path as a project-relative POSIX string, the form the /files route and the UI links use (e.g. cad/x.step)."""
    return path.resolve().relative_to(project.root.resolve()).as_posix()


def _files_in(project: Project, directory: Path, suffixes: tuple[str, ...]) -> list[str]:
    if not directory.is_dir():
        return []
    return [
        _relative(project, path)
        for path in sorted(directory.iterdir())
        if path.is_file() and path.suffix.lower() in suffixes
    ]


def measure_geometry(resolved: ResolvedAirframe, project: Project, parts_data: dict) -> MeasuredDesign:
    """Compute Geometry, first stage: build the printable parts and measure the exact printed mass, saved to
    cad/exact_mass.json, so the editor can apply it before the rest is built."""
    with _EXPORT_LOCK, _exporting("geometry"):
        design = measured(resolved, parts_data, WORKER.run("measure", resolved, parts_data, progress=report))
        save_measured(project.cad_dir, design)
    return design


def build_geometry(resolved: ResolvedAirframe, parts_data: dict) -> None:
    """Compute Geometry, second stage: every other solid the exports use (airframe pieces, fused shape, drawing
    projections), held by the CAD worker so the exports of this design only write files."""
    with _EXPORT_LOCK, _exporting("geometry"):
        WORKER.run("build", resolved, parts_data, progress=report)


def geometry_current(resolved: ResolvedAirframe) -> bool:
    """True when Compute Geometry finished for this design and the CAD worker still holds its solids."""
    return WORKER.holds_complete(resolved)


def export_step(resolved: ResolvedAirframe, project: Project, mode: str = "assembly", include_prop_discs: bool = False) -> dict[str, Any]:
    """STEP of the whole airframe to cad/<project>.step; returns the mode actually written ("assembly"/"fused"/"compound")."""
    path = project.cad_dir / f"{project.name}.step"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("step"):
        written_mode = WORKER.run("step", resolved, path, mode, include_prop_discs, progress=report)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STEP written ({written_mode}) to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail, "mode": written_mode}


def export_stl(resolved: ResolvedAirframe, project: Project, include_prop_discs: bool = False) -> dict[str, Any]:
    """The whole airframe as one STL mesh to cad/<project>.stl (always fused where possible)."""
    path = project.cad_dir / f"{project.name}.stl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("stl"):
        written_mode = WORKER.run("stl", resolved, path, include_prop_discs, progress=report)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STL written to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail}


def export_sections(resolved: ResolvedAirframe, project: Project, export_format: str = "step") -> dict[str, Any]:
    """Printable sections S01.. and couplers C01.. as STEP or STL (export_format) into cad/sections/; problems are notes
    (the route logs them)."""
    if export_format not in ("step", "stl"):
        raise ValueError(f"export_format must be 'step' or 'stl', not {export_format!r}")
    directory = project.sections_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("sections"):
        written, problems = WORKER.run("sections", resolved, directory, export_format, progress=report)
    detail = f"{len(written)} {export_format.upper()} parts written to {_relative(project, directory)}"
    if problems:
        detail += f"; {len(problems)} problems in the Log"
    return {"files": _files_in(project, directory, (f".{export_format}",)), "notes": list(problems), "detail": detail}


def export_drawings(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """The cutaway, three-view and x-ray drawings (.svg + .png) into cad/."""
    directory = project.cad_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("drawings"):
        written = WORKER.run("drawings", resolved, directory, progress=report)
    files = [_relative(project, path) for path in written]
    return {"files": files, "notes": [], "detail": "Drawings written: " + ", ".join(files)}


def export_varsets(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """One FreeCAD varset JSON per varset into cad/varsets/ (libs.varsets uses the definition's values)."""
    from libs.varsets import export_varsets as write_varsets

    directory = project.varsets_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("varsets"):
        written = write_varsets(resolved.airframe, directory, progress=report)
    files = [_relative(project, path) for path in written]
    detail = f"{len(files)} varsets written to {_relative(project, directory)}: " + ", ".join(Path(file).stem for file in files)
    return {"files": files, "notes": [], "detail": detail}
