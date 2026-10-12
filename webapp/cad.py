"""CAD export work for the browser frontend: libs.cad, libs.drawings and libs.varsets run in a worker thread.

libs.cad keeps the printable parts in a module-level cache and OpenCascade booleans are not safe to run twice at
once, so a single module-level lock serializes every export. Each function returns the project-relative files it
wrote, the notes to show and log, and one line of detail for the status line; libs.cad / libs.drawings / libs.varsets
load lazily so server startup does not pay for cadquery. Each export also owns the progress record (begin/report/end)
that the CAD view polls through GET /api/cad/progress while its request is open; a failure is re-raised as
ExportFailed carrying the task and the progress label, so the server logs what was being made.
"""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from libs.geometry import ResolvedAirframe
from libs.project import Project


class ExportFailed(RuntimeError):
    """One CAD export failed, with the task and the progress label it failed on in the message."""


# One export at a time: OCCT and libs.cad._PRINTABLE_PARTS are process globals shared by every request.
_EXPORT_LOCK = threading.Lock()

# The one export in flight, for GET /api/cad/progress. _EXPORT_LOCK serializes the work; this lock keeps the
# record consistent between the exporting worker thread and the polling route.
_PROGRESS_LOCK = threading.Lock()
_PROGRESS: dict[str, Any] = {"running": False, "task": None, "label": None, "done": 0, "total": 0}


def begin(task: str) -> None:
    """Start the progress record for one export (task: step/stl/sections/drawings/varsets)."""
    with _PROGRESS_LOCK:
        _PROGRESS.update(running=True, task=task, label=None, done=0, total=0)


def report(label: str, done: int, total: int) -> None:
    """One step of the running export, called from the worker thread by libs.cad / libs.drawings / libs.varsets."""
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


def export_step(resolved: ResolvedAirframe, project: Project, mode: str = "assembly", include_prop_discs: bool = False) -> dict[str, Any]:
    """STEP of the whole airframe to cad/<project>.step; returns the mode actually written ("assembly"/"fused"/"compound")."""
    from libs.cad import export_step as write_step

    path = project.cad_dir / f"{project.name}.step"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("step"):
        written_mode = write_step(resolved, path, mode=mode, include_prop_discs=include_prop_discs, progress=report)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STEP written ({written_mode}) to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail, "mode": written_mode}


def export_stl(resolved: ResolvedAirframe, project: Project, include_prop_discs: bool = False) -> dict[str, Any]:
    """The whole airframe as one STL mesh to cad/<project>.stl (always fused where possible)."""
    from libs.cad import export_stl as write_stl

    path = project.cad_dir / f"{project.name}.stl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("stl"):
        written_mode = write_stl(resolved, path, include_prop_discs=include_prop_discs, progress=report)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STL written to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail}


def export_sections(resolved: ResolvedAirframe, project: Project, export_format: str = "step") -> dict[str, Any]:
    """Printable sections S01.. and couplers C01.. as STEP or STL (export_format) into cad/sections/; problems are notes
    (the route logs them)."""
    if export_format not in ("step", "stl"):
        raise ValueError(f"export_format must be 'step' or 'stl', not {export_format!r}")
    from libs.cad import export_sections as write_sections

    directory = project.sections_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("sections"):
        written, problems = write_sections(resolved, directory, export_format, progress=report)
    detail = f"{len(written)} {export_format.upper()} parts written to {_relative(project, directory)}"
    if problems:
        detail += f"; {len(problems)} problems in the Log"
    return {"files": _files_in(project, directory, (f".{export_format}",)), "notes": list(problems), "detail": detail}


def export_drawings(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """The cutaway, three-view and x-ray drawings (.svg + .png) into cad/."""
    from libs.drawings import export_drawings as write_drawings

    directory = project.cad_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK, _exporting("drawings"):
        written = write_drawings(resolved, directory, progress=report)
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
