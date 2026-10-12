"""Move section-midpoint loft profiles and fuselage joints to their independent homes.

Usage: python migrate_loft_stations.py aircraft/example [aircraft/other ...]
All definitions are validated before any are written. Existing CAD exports stay in place.
"""

import argparse
import json
from pathlib import Path

from libs.airframe import load_parts_data, parse_airframe


def migrate_definition(definition: dict) -> None:
    fuselage, sections = definition["fuselage"], definition["sections"]
    if "loft_stations" in fuselage or "joints" in sections:
        raise ValueError("Definition already has independent loft guides or sections.joints")
    bounds = [0.0, *sections["cuts_m"], fuselage["length_m"]]
    if len(sections["per_section"]) != len(bounds) - 1:
        raise ValueError("Section count does not match cut boundaries")
    fuselage["loft_stations"] = [
        {"x_m": (start + end) / 2.0, **row.pop("section", {})}
        for row, start, end in zip(sections["per_section"], bounds, bounds[1:])
    ]
    sections["joints"] = fuselage.pop("joints")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("projects", nargs="+", type=Path)
    args = parser.parse_args()
    parts = load_parts_data()
    prepared = []
    for project in args.projects:
        path = project / "aircraft.json"
        source = path.read_text()
        indent = next(line[:len(line) - len(line.lstrip())] for line in source.splitlines() if line.lstrip().startswith('"'))
        definition = json.loads(source)
        migrate_definition(definition)
        parse_airframe(definition, parts)
        prepared.append((path, definition, indent))
    for path, definition, indent in prepared:
        path.write_text(json.dumps(definition, indent=indent) + "\n")
        print(f"{path}: migrated loft guides and joints")


if __name__ == "__main__":
    main()
