"""Headless checks for the duct schema, guide curves, placement checks and the 3D preview meshes.

Run: uv run python -m webapp.test_ducts

Everything runs in memory on the bundled example definition: no server, no CAD and no kernel geometry (the duct's
flow volume mesh is plain geometry, webapp.model3d). The CAD junction gets its own checks in webapp.test_geometry.
"""

from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from libs.airframe import AirframeError, load_parts_data, parse_airframe  # noqa: E402
from libs.ducts import duct_cavity_mesh, naca_duct_profile  # noqa: E402
from libs.geometry import resolve_airframe  # noqa: E402
from webapp.model3d import build_model3d  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
CLEAN = {
    "name": "ESC cooling", "type": "naca", "x_m": 0.32, "clock_deg": 90.0,
    "throat_depth_m": 0.007, "inlet_width_m": 0.003, "tunnel_length_m": 0.04, "wall_m": 0.002,
    "width_to_depth": 4, "length_to_width": 2,
}


def definition_with(duct: dict) -> dict:
    definition = json.loads((EXAMPLE / "aircraft.json").read_text())
    definition["ducts"] = [duct]
    return definition


def check_parse(parts_data: dict) -> dict:
    definition = definition_with(CLEAN)
    duct = parse_airframe(definition, parts_data).ducts[0]
    assert (duct.throat_width_m, duct.ramp_length_m, duct.length_m) == (0.028, 0.056, 0.096)
    assert abs(duct.ramp_angle_deg - 7.125) < 0.001
    for bad, message in (({"type": "scoop"}, "type"), ({"inlet_width_m": 0.03}, "inlet"), ({"wall_m": 0.0}, "wall")):
        edited = definition_with({**CLEAN, **bad})
        try:
            parse_airframe(edited, parts_data)
        except AirframeError as error:
            assert message in str(error), (bad, error)
        else:
            raise AssertionError(f"invalid duct accepted: {bad}")
    # A duct on or close to a section cut is a definition error (the joint band plus the duct's wall).
    clearance = 0.01 + CLEAN["wall_m"]
    for change in ({"x_m": 0.28}, {"x_m": 0.451 - 0.005 - 0.096}):  # spanning the cut; 5 mm short of it
        try:
            parse_airframe(definition_with({**CLEAN, **change}), parts_data)
        except AirframeError as error:
            assert "section cut" in str(error) and "stay clear" in str(error), error
        else:
            raise AssertionError(f"a duct on or close to a cut was accepted: {change}")
    parse_airframe(definition_with({**CLEAN, "x_m": 0.451 - clearance - 0.002 - 0.096}), parts_data)  # just clear
    print("ok   duct parse: ratios derive throat width, ramp length and ramp angle; unknown type, non-positive sizes, an inlet at the throat and a duct on or close to a cut are refused")
    return definition


def check_resolve(parts_data: dict, definition: dict) -> None:
    resolved, problems = resolve_airframe(parse_airframe(definition, parts_data), parts_data)
    assert not problems, problems
    assert [duct.instance_id for duct in resolved.ducts] == ["ESC cooling"]
    depth, half = naca_duct_profile(resolved.airframe.ducts[0], 0.028)
    assert 0.002 < depth < 0.003 and 0.007 < half < 0.008, (depth, half)  # the smooth flare: inlet 1.5 -> 7.75 mm at half the ramp
    depth, half = naca_duct_profile(resolved.airframe.ducts[0], 0.08)
    assert (depth, half) == (0.007, 0.014)
    positions, indices = duct_cavity_mesh(resolved.ducts[0], resolved.body)
    assert len(positions) * 3 % 9 == 0 and len(indices) % 3 == 0 and len(indices) // 3 > 300
    payload = build_model3d(resolved, None)
    ducts = [mesh for mesh in payload["meshes"] if mesh["kind"] == "duct"]
    assert len(ducts) == 1 and ducts[0]["name"] == "ESC cooling" and len(ducts[0]["positions"]) % 3 == 0
    # A radial pattern turns copies about the body axis; each copy is rebuilt on the surface at its own clock.
    radial_resolved, radial_problems = resolve_airframe(parse_airframe(definition_with({**CLEAN, "radial": 2}), parts_data), parts_data)
    assert not radial_problems, radial_problems
    assert [(duct.instance_id, duct.clock_deg) for duct in radial_resolved.ducts] == [("ESC cooling 1/2", 90.0), ("ESC cooling 2/2", 270.0)]
    assert len([mesh for mesh in build_model3d(radial_resolved, None)["meshes"] if mesh["kind"] == "duct"]) == 2
    for bad in ({"radial": 0}, {"radial": 1.5}):
        try:
            parse_airframe(definition_with({**CLEAN, **bad}), parts_data)
        except AirframeError:
            pass
        else:
            raise AssertionError(f"invalid radial pattern accepted: {bad}")
    print(f"ok   resolve and preview: flush lip -> throat depth, constant tunnel, {len(indices) // 3} mesh triangles carried to the 3D payload, radial copies at their own clock")


def check_drawing(parts_data: dict, definition: dict) -> None:
    """The Simple view records the duct's surface cut rim (both projections share libs.graphics.draw_ducts)."""
    from libs.graphics import DUCT_COLOR, draw_ducts
    from webapp.drawing import RecordingCanvas

    resolved, _problems = resolve_airframe(parse_airframe(definition, parts_data), parts_data)
    canvas = RecordingCanvas(800, 400)
    draw_ducts(canvas, resolved, lambda point: (point[0], point[2]))  # side view projection, unrolled
    outlines = [item for item in canvas.items if item["kind"] == "polygon" and item["opts"].get("outline") == DUCT_COLOR]
    assert len(outlines) == 1, [item["opts"].get("outline") for item in canvas.items]
    coords = outlines[0]["coords"]
    xs, zs = coords[0::2], coords[1::2]
    assert len(coords) % 2 == 0 and len(coords) > 80
    assert abs(min(xs) - 0.32) < 1e-9 and abs(max(xs) - 0.416) < 1e-9
    assert min(zs) > 0.02 and max(zs) < 0.05  # the rim rides on the upper skin
    # A duct that does not fit the body has no rim and must never break the drawing; the release flags it instead.
    past_resolved, past_problems = resolve_airframe(parse_airframe(definition_with({**CLEAN, "x_m": 0.63}), parts_data), parts_data)
    assert any("past the body" in problem for problem in past_problems), past_problems
    canvas = RecordingCanvas(800, 400)
    draw_ducts(canvas, past_resolved, lambda point: (point[0], point[2]))
    assert not [item for item in canvas.items if item["opts"].get("outline") == DUCT_COLOR]
    # The drag path adopts a joint-band position with no checks; the release recompute re-runs them (libs.airframe).
    parse_airframe(definition_with({**CLEAN, "x_m": 0.28}), parts_data, duct_joints=False)
    print("ok   Simple view: the duct records one teal surface cut rim, spanning lip to tunnel end on the skin; an off-body duct is skipped, and a drag may adopt a joint band")


def check_body_opening(parts_data: dict, definition: dict) -> None:
    """The 3D preview cuts the mouth's exact profile straight into the skin (the cookie cutter): every sample inside
    the mouth passes through the skin, every sample outside it still hits the skin."""
    from dataclasses import replace

    from libs.ducts import clip_body_mouth, naca_duct_profile
    from libs.geometry import body_surface_point, skin_clock_for_arc, skin_normal
    from webapp.model3d import _body_mesh, _body_meshes

    resolved, _problems = resolve_airframe(parse_airframe(definition, parts_data), parts_data)
    duct = resolved.ducts[0]
    positions, indices = _body_mesh(resolved.body.stations)
    kept = clip_body_mouth(duct, resolved.body, positions, indices)
    triangles = [tuple(kept[offset:offset + 3]) for offset in range(0, len(kept), 3)]
    assert len(triangles) >= len(indices) // 3, "splitting the rim must not lose triangles"

    def skin_hits(point: tuple[float, float, float], normal: tuple[float, float, float]) -> int:
        origin = tuple(point[axis] + 0.002 * normal[axis] for axis in range(3))
        direction = tuple(-value for value in normal)
        hits = 0
        for a, b, c in triangles:
            p0, p1, p2 = positions[a], positions[b], positions[c]
            e1 = tuple(p1[axis] - p0[axis] for axis in range(3))
            e2 = tuple(p2[axis] - p0[axis] for axis in range(3))
            pvec = (direction[1] * e2[2] - direction[2] * e2[1], direction[2] * e2[0] - direction[0] * e2[2], direction[0] * e2[1] - direction[1] * e2[0])
            det = sum(e1[axis] * pvec[axis] for axis in range(3))
            if abs(det) < 1e-14:
                continue
            inv = 1.0 / det
            tvec = tuple(origin[axis] - p0[axis] for axis in range(3))
            u = sum(tvec[axis] * pvec[axis] for axis in range(3)) * inv
            if u < 0.0 or u > 1.0:
                continue
            qvec = (tvec[1] * e1[2] - tvec[2] * e1[1], tvec[2] * e1[0] - tvec[0] * e1[2], tvec[0] * e1[1] - tvec[1] * e1[0])
            v = sum(direction[axis] * qvec[axis] for axis in range(3)) * inv
            if v < 0.0 or u + v > 1.0:
                continue
            t = sum(e2[axis] * qvec[axis] for axis in range(3)) * inv
            if 0.0002 < t < 0.004:
                hits += 1
        return hits

    def sample(s_m: float, across_m: float) -> int:
        clock = skin_clock_for_arc(resolved.body, duct.entry.x_m + s_m, math.radians(duct.clock_deg), across_m)
        point, _radius = body_surface_point(resolved.body, duct.entry.x_m + s_m, clock)
        return skin_hits(point, skin_normal(resolved.body, duct.entry.x_m + s_m, clock))

    footprint = duct.entry.ramp_length_m + duct.entry.tunnel_length_m
    for s_index in range(1, 16):  # inside the whole cut shape (ramp and tunnel): no skin left anywhere
        s_m = footprint * s_index / 16.0
        _depth, half = naca_duct_profile(duct.entry, s_m)
        for a_index in range(5):
            assert sample(s_m, (-half + 2.0 * half * (a_index + 0.5) / 5.0) * 0.85) == 0, (s_m, a_index)
    _depth, half = naca_duct_profile(duct.entry, duct.entry.ramp_length_m / 2.0)
    assert sample(duct.entry.ramp_length_m / 2.0, half + 0.006) >= 1  # beside the ramp the skin is intact
    assert sample(-0.006, 0.0) >= 1  # ahead of the lip
    assert sample(footprint + 0.006, 0.0) >= 1  # aft of the tunnel end
    middle = duct.entry.ramp_length_m + duct.entry.tunnel_length_m / 2.0
    _depth, tunnel_half = naca_duct_profile(duct.entry, middle)
    assert sample(middle, tunnel_half + 0.006) >= 1  # beside the tunnel
    payload = build_model3d(resolved, None)
    body = next(mesh for mesh in payload["meshes"] if mesh["name"] == "body")
    closed = _body_meshes(replace(resolved, ducts=()))[0]  # the same body with the cut shape still closed
    assert body["indices"] != closed["indices"], "the payload body kept the duct's cut shape closed"
    cavity = next(mesh for mesh in payload["meshes"] if mesh["kind"] == "duct")
    assert cavity["opacity"] == 1.0 and len(cavity["positions"]) % 3 == 0
    print(f"ok   3D opening: the surface cut shape (ramp and tunnel) is cut straight in ({len(triangles)} skin triangles after the cut); the skin around it, ahead of the lip and aft of the tunnel end is intact")


def check_placement(parts_data: dict, definition: dict) -> None:
    def problems_for(change: dict, extra: dict | None = None) -> list[str]:
        edited = copy.deepcopy(definition)
        edited["ducts"][0].update(change)
        if extra is not None:
            edited.update(extra)
        _resolved, found = resolve_airframe(parse_airframe(edited, parts_data), parts_data)
        return found

    assert any("past the body length" in problem for problem in problems_for({"x_m": 0.63}))
    assert any("volume" in problem for problem in problems_for({"x_m": 0.0705, "clock_deg": 35.0, "throat_depth_m": 0.012, "tunnel_length_m": 0.01}))
    rods = {"type": "cf_rod", "material": "carbon_rod", "diameter_m": 0.0025, "polar_n": 4, "polar_offset_angle": 45.0,
            "polar_split": 0, "polar_split_angle": 0.0, "radial_position_m": 0.028, "from_x_m": 0.1, "to_x_m": 0.5}
    flagged = problems_for({"x_m": 0.32, "clock_deg": 45.0}, {"fuselage": {**json.loads((EXAMPLE / "aircraft.json").read_text())["fuselage"], "reinforcement": rods}})
    assert any("reinforcement rod" in problem for problem in flagged), flagged
    assert not any("volume" in problem for problem in problems_for({}))
    print("ok   placement checks: the body end, reinforcement rods and interior volumes are flagged; a clean placement is not")


def main() -> None:
    parts_data = load_parts_data()
    definition = check_parse(parts_data)
    check_resolve(parts_data, definition)
    check_body_opening(parts_data, definition)
    check_drawing(parts_data, definition)
    check_placement(parts_data, definition)
    print("duct suite OK")


if __name__ == "__main__":
    main()
