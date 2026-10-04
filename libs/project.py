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
    sitl/           SITL.elf (pinned), eeprom.bin, sitl.log: the bench's workdir
    jsbsim/         JSBSim export
    logs/           SITL and script flight logs and plots
    cad/            STEP / STL / FreeCAD params
    controllers/    flight controller scripts (libs/controller.py)
"""

from __future__ import annotations

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
    def sitl_path(self) -> Path:
        return self.sitl_dir / "SITL.elf"

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
    def controllers_dir(self) -> Path:
        return self.root / "controllers"


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
