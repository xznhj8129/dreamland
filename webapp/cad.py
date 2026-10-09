"""CAD export work for the browser frontend: libs.cad, libs.drawings and libs.varsets run in a worker thread.

libs.cad keeps the printable parts in a module-level cache and OpenCascade booleans are not safe to run twice at
once, so a single module-level lock serializes every export. Each function returns the project-relative files it
wrote, the notes to show and log, and one line of detail for the status line; libs.cad / libs.drawings / libs.varsets
load lazily so server startup does not pay for cadquery.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

from libs.geometry import ResolvedAirframe
from libs.project import Project

# One export at a time: OCCT and libs.cad._PRINTABLE_PARTS are process globals shared by every request.
_EXPORT_LOCK = threading.Lock()

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


def list_files(project: Project) -> list[str]:
    """Every existing file under the project's cad/, project-relative and newest first (GET /api/cad/files)."""
    directory = project.cad_dir
    if not directory.is_dir():
        return []
    files = [path for path in directory.rglob("*") if path.is_file()]
    files.sort(key=lambda path: (-path.stat().st_mtime, path.relative_to(directory).as_posix()))
    return [_relative(project, path) for path in files]


def export_step(resolved: ResolvedAirframe, project: Project, mode: str = "assembly", include_prop_discs: bool = False) -> dict[str, Any]:
    """STEP of the whole airframe to cad/<project>.step; returns the mode actually written ("assembly"/"fused"/"compound")."""
    from libs.cad import export_step as write_step

    path = project.cad_dir / f"{project.name}.step"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK:
        written_mode = write_step(resolved, path, mode=mode, include_prop_discs=include_prop_discs)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STEP written ({written_mode}) to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail, "mode": written_mode}


def export_stl(resolved: ResolvedAirframe, project: Project, include_prop_discs: bool = False) -> dict[str, Any]:
    """The whole airframe as one STL mesh to cad/<project>.stl (always fused where possible)."""
    from libs.cad import export_stl as write_stl

    path = project.cad_dir / f"{project.name}.stl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK:
        written_mode = write_stl(resolved, path, include_prop_discs=include_prop_discs)
    follows = "The boolean fuse failed; wrote an unfused compound instead." if written_mode == "compound" else ""
    notes = [follows] if follows else []
    detail = f"STL written to {_relative(project, path)}" + (f" — {follows}" if notes else "")
    return {"files": [_relative(project, path)], "notes": notes, "detail": detail}


def export_sections(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """Printable sections S01.. and couplers C01.. (STEP + STL) into cad/sections/; problems are notes (the route logs them)."""
    from libs.cad import export_sections as write_sections

    directory = project.sections_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK:
        written, problems = write_sections(resolved, directory)
    detail = f"{len(written)} parts written to {_relative(project, directory)}"
    if problems:
        detail += f"; {len(problems)} problems in the Log"
    return {"files": _files_in(project, directory, (".step", ".stl")), "notes": list(problems), "detail": detail}


def export_drawings(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """The cutaway, three-view and x-ray drawings (.svg + .png) into cad/."""
    from libs.drawings import export_drawings as write_drawings

    directory = project.cad_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK:
        written = write_drawings(resolved, directory)
    files = [_relative(project, path) for path in written]
    return {"files": files, "notes": [], "detail": "Drawings written: " + ", ".join(files)}


def export_varsets(resolved: ResolvedAirframe, project: Project) -> dict[str, Any]:
    """One FreeCAD varset JSON per varset into cad/varsets/ (libs.varsets uses the definition's values)."""
    from libs.varsets import export_varsets as write_varsets

    directory = project.varsets_dir
    directory.mkdir(parents=True, exist_ok=True)
    with _EXPORT_LOCK:
        written = write_varsets(resolved.airframe, directory)
    files = [_relative(project, path) for path in written]
    detail = f"{len(files)} varsets written to {_relative(project, directory)}: " + ", ".join(Path(file).stem for file in files)
    return {"files": files, "notes": [], "detail": detail}
