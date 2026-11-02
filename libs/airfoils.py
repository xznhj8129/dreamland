"""Airfoil resolution shared by the definition, aero, CAD and the editor.

A surface's `airfoil` is a name. Names in data/airfoils.json (the imported/built-in library, including the plate
sections) resolve from their DAT text; every other name is handed to AeroSandbox (NACA 4-digit and the UIUC database).
Keeping this in one place is what lets a plate be a surface section without teaching every consumer a second format.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import aerosandbox as asb

from libs.airfoil_analysis import load_airfoil_library, parse_dat_text

# A plate is a non-airfoil section: a straight strip of this chord fraction, optionally with semicircular leading and
# trailing edges whose diameter is the strip thickness (the rounded edge radius is half the thickness).
PLATE_THICKNESS_RATIO = 0.02
PLATE_NAMES = ("plate", "plate-rounded")


@lru_cache(maxsize=1)
def _library() -> dict[str, dict]:
    return load_airfoil_library()


def airfoil(name: str) -> asb.Airfoil:
    """The AeroSandbox airfoil for a surface's name: the local library's DAT text, else AeroSandbox by name."""
    record = _library().get(name)
    if record is None or record.get("source") == "asb":
        return asb.Airfoil(name)
    return asb.Airfoil(name=name, coordinates=parse_dat_text(record["dat_text"]))


@lru_cache(maxsize=1)
def airfoil_names() -> tuple[str, ...]:
    """Names a surface can pick: the local library plus every airfoil in AeroSandbox's UIUC database."""
    names = set(_library())
    try:
        database = Path(asb.__file__).resolve().parent / "geometry" / "airfoil" / "airfoil_database"
        names.update(path.stem for path in database.glob("*.dat"))
    except (AttributeError, OSError, TypeError):
        pass
    return tuple(sorted(names, key=str.lower))
