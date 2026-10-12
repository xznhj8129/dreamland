"""Inspect the normal printable-parts export; preserve CAD baselines under the aircraft project.

Run: python -m libs.test_surface_sections aircraft/example
Export printable sections in the browser first. This does not rebuild the exported parts.
"""

import argparse
import json
import math
from pathlib import Path

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from libs.airframe import load_airframe, load_parts_data
from libs.cad import _clip_span, _spar_hole, _through_holes, build_body_solid, build_panel_solids, build_pod_solid, cut_motor_mount_holes
from libs.geometry import dot, resolve_airframe

SPAN_TOLERANCE_M = 2e-6
VOLUME_TOLERANCE_MM3 = 0.1
INTEGRATION_RELATIVE_TOLERANCE = 1e-8


def volume_mm3(shape) -> float:
    """Adaptive integration: ordinary Shape.Volume changes its estimate when a curved loft is partitioned."""
    properties = GProp_GProps()
    error = BRepGProp.VolumePropertiesGK_s(shape.wrapped, properties, INTEGRATION_RELATIVE_TOLERANCE, True, True)
    assert error >= 0.0, f"volume integration failed: relative error {error}"
    return properties.Mass()


def read_part(directory: Path, name: str):
    shape = cq.importers.importStep(str(directory / f"{name}.step")).val()
    assert shape.isValid() and len(shape.Solids()) == 1, f"{name}: invalid or disconnected printable solid"
    return shape


def inspect_project(project: Path, output: Path | None = None) -> None:
    parts_data = load_parts_data()
    resolved, _ = resolve_airframe(load_airframe(project / "aircraft.json", parts_data), parts_data)
    directory = project / "cad" / "sections"
    output = output if output is not None else project / "cad" / "surface-section-check"
    output.mkdir(parents=True, exist_ok=True)
    definition = json.loads((project / "aircraft.json").read_text())
    baseline_definition = output / "aircraft.json"
    reuse = baseline_definition.exists() and json.loads(baseline_definition.read_text()) == definition
    if baseline_definition.exists() and not reuse:
        raise ValueError("Saved baselines belong to another definition; preserve them and select a new project CAD directory with --output")
    outer_path = output / "outer-body.step"
    if reuse and outer_path.exists():
        outer = cq.importers.importStep(str(outer_path)).val()
    else:
        outer = build_body_solid(resolved, 0.0, resolved.body.length_m).val()
        cq.exporters.export(outer, str(outer_path))
    baseline_definition.write_text(json.dumps(definition, indent=4) + "\n")
    for index in range(len(resolved.airframe.sections.cuts_m) + 1):
        read_part(directory, f"S{index + 1:02d}")
    controls = {(item.surface, item.panel_index): item for item in resolved.controls}
    metrics = []
    for surface_index, surface_set in enumerate(resolved.surfaces, start=1):
        cuts = surface_set.definition.sections.cuts_m
        if not cuts:
            continue
        for panel in surface_set.panels:
            fixed, moving = build_panel_solids(resolved, panel, surface_set.definition.control)
            for motor in resolved.motors:
                if motor.pod_stations and motor.panel is not None and (motor.panel.surface, motor.panel.index) == (panel.surface, panel.index):
                    pod = build_pod_solid(motor).val()
                    fixed = pod if fixed is None else fixed.fuse(pod)
            if moving:
                name = f"CS{controls[(panel.surface, panel.index)].number}"
                control = read_part(directory, name)
                assert math.isclose(control.Volume(), moving[0].Volume(), abs_tol=VOLUME_TOLERANCE_MM3), f"{name}: control split or changed"
                cq.exporters.export(moving[0], str(output / f"{name}-baseline.step"))
            if fixed is None:
                continue
            baseline_path = output / f"W{surface_index}_P{panel.index + 1}-baseline.step"
            if reuse and baseline_path.exists():
                exposed = cq.importers.importStep(str(baseline_path)).val()
            else:
                exposed = fixed.cut(outer)
                holes = _through_holes(resolved, 0.0, resolved.body.length_m)
                if holes:
                    exposed = exposed.cut(*holes)
                exposed = cut_motor_mount_holes(exposed, resolved).clean()
                cq.exporters.export(exposed, str(baseline_path))
            original_volume = volume_mm3(exposed)
            retained_volume = volume_mm3(_clip_span(exposed, panel, None, cuts[0]))
            print(f"Inspecting {panel.surface} P{panel.index + 1}: original fixed material {original_volume:.3f} mm3, root {retained_volume:.3f} mm3", flush=True)
            detached_volume = 0.0
            bounds = [*cuts, surface_set.definition.span_m]
            for index, (low, high) in enumerate(zip(bounds, bounds[1:]), start=2):
                name = f"W{surface_index}_P{panel.index + 1}_S{index}"
                shape = read_part(directory, name)
                spans = [dot(tuple(value / 1000.0 - origin for value, origin in zip(vertex.toTuple(), panel.attach_le)), panel.span_dir)
                         for vertex in shape.Vertices()]
                assert min(spans) >= low - SPAN_TOLERANCE_M and max(spans) <= high + SPAN_TOLERANCE_M, f"{name}: outside span planes"
                assert shape.intersect(outer).Volume() <= VOLUME_TOLERANCE_MM3, f"{name}: buried material retained"
                for spar in resolved.spars:
                    if (spar.surface, spar.panel_index) == (panel.surface, panel.index):
                        assert shape.intersect(_spar_hole(resolved, spar)).Volume() <= VOLUME_TOLERANCE_MM3, f"{name}: blocked spar hole"
                measured_volume = volume_mm3(shape)
                detached_volume += measured_volume
                print(f"{name}: span {min(spans) * 1000.0:.3f}..{max(spans) * 1000.0:.3f} mm, volume {measured_volume:.3f} mm3", flush=True)
                metrics.append({"part": name, "span_start_mm": min(spans) * 1000.0, "span_end_mm": max(spans) * 1000.0,
                                "volume_mm3": measured_volume})
            difference = abs(original_volume - retained_volume - detached_volume)
            print(f"{panel.surface} P{panel.index + 1}: baseline {original_volume:.3f}, root {retained_volume:.3f}, detached {detached_volume:.3f} mm3", flush=True)
            (output / "measurements.json").write_text(json.dumps(metrics, indent=2) + "\n")
            assert difference <= VOLUME_TOLERANCE_MM3, f"{panel.surface} P{panel.index + 1}: {difference} mm3 missing or duplicated"
            print(f"OK {panel.surface} P{panel.index + 1}: valid segments, aligned open spar holes, unchanged controls; material difference {difference:.6f} mm3", flush=True)
    assert metrics, "the project has no exported fixed surface segments to inspect"
    (output / "measurements.json").write_text(json.dumps(metrics, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    parser.add_argument("--output", type=Path, help="Directory for this design's persistent CAD baselines")
    args = parser.parse_args()
    inspect_project(args.project, args.output)


if __name__ == "__main__":
    main()
