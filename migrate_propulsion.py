"""Move the electric system into propulsion.electric and give every motor mount its type.

Usage: python migrate_propulsion.py aircraft/example [aircraft/other ...]
propulsion.{motor, prop, battery, esc, wiring} move unchanged to propulsion.electric; every mount gets type "electric".
All definitions are validated before any are written.
"""

import argparse
import json
from pathlib import Path

from libs.airframe import load_parts_data, parse_airframe

ELECTRIC_KEYS = ("motor", "prop", "battery", "esc", "wiring")


def migrate_definition(definition: dict) -> None:
    propulsion = definition.get("propulsion")
    if propulsion is None:
        return
    if "electric" in propulsion or any("type" in mount for mount in propulsion["mounts"]):
        raise ValueError("Definition already has propulsion.electric or typed mounts")
    propulsion["mounts"] = [{"type": "electric", **mount} for mount in propulsion["mounts"]]
    propulsion["electric"] = {key: propulsion.pop(key) for key in ELECTRIC_KEYS}


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
        print(f"migrated {path}")


if __name__ == "__main__":
    main()
