"""Check resolved loft and CAD envelopes on a saved aircraft project.

Run: python -m libs.test_body_loft aircraft/example
The body STEP and measurements are retained in the project's cad directory.
"""

import argparse
import math
from pathlib import Path

import cadquery as cq

from libs.airframe import load_airframe, load_parts_data
from libs.cad import _airfoil_section_coordinates, _root_le, build_body_solid
from libs.geometry import body_section_at, resolve_airframe
from libs.project import open_project

CAD_ENVELOPE_TOLERANCE_M = 0.0001


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    args = parser.parse_args()
    project = open_project(args.project)
    parts = load_parts_data()
    frame = load_airframe(project.build_path, parts)
    resolved, _ = resolve_airframe(frame, parts)
    body = resolved.body
    limit = max(max(station.width_m, station.height_m) for station in body.stations)
    measurements = []
    # A root embedded in the body cannot travel farther than the body's transverse envelope.
    for surface in resolved.surfaces:
        coordinates = _airfoil_section_coordinates(surface.definition.airfoil)
        for panel in surface.panels:
            depth = math.dist(_root_le(panel, coordinates, body), panel.attach_le)
            message = f"{surface.name}[{panel.index}]: CAD root embed {depth * 1000:.3f} mm, body envelope {limit * 1000:.3f} mm"
            print(message)
            measurements.append(message)
            assert depth <= limit, message
    # Each loft control is exact before end taper, and joins the adjoining transitions continuously.
    for station in body.loft_stations:
        if body.nose_end_x_m < station.x_m < body.tail_start_x_m:
            actual = body_section_at(body, station.x_m)
            assert actual == station
            left, right = body_section_at(body, station.x_m - 1e-7), body_section_at(body, station.x_m + 1e-7)
            assert max(abs(left.width_m - right.width_m), abs(left.height_m - right.height_m)) < 1e-7
            assert abs(left.shape - right.shape) < 1e-5
    solid = build_body_solid(resolved, 0.0, body.length_m).val()
    project.cad_dir.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(cq.Workplane("XY").newObject([solid]), str(project.cad_dir / "body-loft-check.step"), exportType="STEP")
    bounds = solid.BoundingBox()
    assert solid.isValid() and len(solid.Solids()) == 1
    assert bounds.ylen / 1000 <= max(s.width_m for s in body.stations) + CAD_ENVELOPE_TOLERANCE_M
    assert bounds.zlen / 1000 <= max(s.height_m for s in body.stations) + CAD_ENVELOPE_TOLERANCE_M
    message = f"body CAD: {bounds.xlen:.3f} x {bounds.ylen:.3f} x {bounds.zlen:.3f} mm, {solid.Volume()/1000:.3f} cm3; valid, one solid"
    measurements.append(message)
    (project.cad_dir / "body-loft-check.txt").write_text("\n".join(measurements) + "\n")
    print(message)
    print("body loft checks OK")


if __name__ == "__main__":
    main()
