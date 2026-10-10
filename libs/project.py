"""
Usage:
from libs.project import open_project
project = open_project(Path("aircraft/m62v1"))
project.build_path  # aircraft/m62v1/aircraft.json

An aircraft project: one directory per aircraft holding everything about it.

aircraft/<project>/
    aircraft.json   airframe/2 build file
    inav/           INAV diffs: real FC diffs, SITL diffs flown as written
      generated/    SITL diffs written from the design or a real FC diff on every run from them
    sitl/           eeprom.bin, sitl.log: the bench's workdir
    jsbsim/         JSBSim export
    logs/           SITL and script flight logs and plots
    cad/            STEP / STL exports, drawings: cutaway, threeview, xray (.svg / .png)
      sections/     printable sections S01.. and couplers C01.., STEP and STL
      varsets/      FreeCAD varsets, one JSON per varset (Varset_FROM_JSON.FCMacro)
    scenarios/      run(flight) scripts under inav/, mcu/, or another controller directory
"""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

PROJECTS_DIR = Path(__file__).resolve().parent.parent / "aircraft"
BUILD_FILE = "aircraft.json"


class ProjectError(Exception):
    pass


@dataclass(frozen=True)
class Project:
    root: Path

    @property
    def name(self) -> str:
        return self.root.name

    @property
    def build_path(self) -> Path:
        return self.root / BUILD_FILE

    @property
    def inav_dir(self) -> Path:
        return self.root / "inav"

    @property
    def generated_inav_dir(self) -> Path:
        return self.inav_dir / "generated"

    @property
    def sitl_dir(self) -> Path:
        return self.root / "sitl"

    @property
    def jsbsim_dir(self) -> Path:
        return self.root / "jsbsim"

    @property
    def logs_dir(self) -> Path:
        return self.root / "logs"

    @property
    def cad_dir(self) -> Path:
        return self.root / "cad"

    @property
    def sections_dir(self) -> Path:
        return self.cad_dir / "sections"

    @property
    def varsets_dir(self) -> Path:
        return self.cad_dir / "varsets"

    @property
    def scenarios_dir(self) -> Path:
        return self.root / "scenarios"


def open_project(path: Path) -> Project:
    project = Project(Path(path).resolve())
    if not project.build_path.is_file():
        raise ProjectError(f"{path} is not an aircraft project: no {BUILD_FILE} in it")
    return project


def copy_project(project: Project, new_root: Path) -> Project:
    """The whole project copied to new_root, which must not exist yet."""
    if new_root.exists():
        raise ProjectError(f"{new_root} already exists")
    shutil.copytree(project.root, new_root)
    return Project(new_root.resolve())


def create_project(name: str, definition: dict) -> Project:
    """A new project directory holding definition as its build file; the standard subdirectories are created empty."""
    if not name or Path(name).name != name:
        raise ProjectError(f"{name!r} is not a plain directory name")
    root = PROJECTS_DIR / name
    if root.exists():
        raise ProjectError(f"{root} already exists")
    try:
        root.mkdir()
        for relative in ("inav/generated", "sitl", "jsbsim", "logs", "cad/sections", "cad/varsets", "scenarios/inav", "scenarios/mcu"):
            (root / relative).mkdir(parents=True)
        with open(root / BUILD_FILE, "w") as file:
            json.dump(definition, file, indent=4)
    except OSError as error:
        shutil.rmtree(root, ignore_errors=True)
        raise ProjectError(f"could not create {root}: {error}") from error
    return Project(root.resolve())
