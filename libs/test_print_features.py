"""Exercise printable fastener openings, pod placement and their live previews.

Run: python -m libs.test_print_features aircraft/example --check joints|mounts|views
CAD diagnostics are retained under the project's cad/fastener-check directory.
"""

import argparse
import copy
import json
import math
from pathlib import Path

import cadquery as cq

from libs.airframe import load_parts_data, parse_airframe
from libs.cad import _ring, _screw_holes, build_section_solid, build_pod_solid, cut_motor_mount_holes
from libs.geometry import resolve_airframe
from webapp.drawing import RecordingCanvas
from libs.graphics import draw_canvas_view
from webapp.model3d import build_model3d


def check_joints(definition, parts, output):
    data = copy.deepcopy(definition)
    data['fuselage']['loft_stations'] = [{'x_m': 0.2, 'width_m': 0.07, 'height_m': 0.075, 'shape': 4.0}]
    model, _ = resolve_airframe(parse_airframe(data, parts), parts)
    x = model.airframe.sections.cuts_m[1] + model.airframe.sections.joints.hole_edge_dist_m
    shell = _ring(x - .004, x + .004, (.035, .0375), (.033, .0355), 4.0)
    holes = _screw_holes(model, x, 1)
    opened = shell.cut(*holes)
    cq.exporters.export(opened, str(output / 'squarish-holes.step'))
    # A through hole adds a closed inner wire to both the outer skin and the bore.
    # Counting actual CAD face openings catches blind holes independently of cutter construction.
    skins = [face for face in opened.Faces() if face.geomType() == 'BSPLINE']
    assert len(skins) == 2 and all(len(face.Wires()) == 9 for face in skins), (
        f'squarish screw holes do not all open through skin and bore: {[len(face.Wires()) - 1 for face in skins]}')
    for degrees, hole in zip(model.airframe.sections.joints.hole_angles_deg(), holes):
        axis = next(face for face in hole.Faces() if face.geomType() == 'PLANE').normalAt()
        near_top = abs(math.sin(math.radians(degrees))) > abs(math.cos(math.radians(degrees)))
        assert abs(axis.z if near_top else axis.y) > .99, f'{degrees} degree hole still points radially inward'
    print('OK: all eight squarish joint holes break out')


def check_mounts(definition, parts, output):
    model, _ = resolve_airframe(parse_airframe(definition, parts), parts)
    motor = model.motors[0]
    assert motor.mount_holes is not None
    # Printed material in an earlier section must not be bored by a tail mount.
    first = cq.Solid.makeBox(20, 40, 40, cq.Vector(80, -20, -20))
    cut = cut_motor_mount_holes(first, model)
    assert abs(cut.Volume() - first.Volume()) < 1e-5, 'tail fastener cutters remove material in a distant forward section'
    moved = copy.deepcopy(definition)
    moved['propulsion']['mounts'][0]['x_offset_m'] = .012
    shifted, _ = resolve_airframe(parse_airframe(moved, parts), parts)
    assert abs(shifted.motors[0].position[0] - motor.position[0] - .012) < 1e-9
    pod = build_pod_solid(shifted.motors[0]).val()
    tail, _ = build_section_solid(shifted, len(shifted.airframe.sections.cuts_m), [('pod', pod)])
    cq.exporters.export(tail, str(output / 'tail-mount.step'))
    assert tail.BoundingBox().xmax >= shifted.motors[0].mount_face[0] * 1000 - .01, 'tail pod outside fuselage length was discarded'
    assert tail.isValid() and len(tail.Solids()) == 1, 'extended tail pod is invalid or detached'
    print('OK: tail offset moves motor/pod; pod survives section clipping; holes stop in its section')


def check_views(definition, parts, output):
    model, _ = resolve_airframe(parse_airframe(definition, parts), parts)
    payload = build_model3d(model, None)
    (output / 'model3d.json').write_text(json.dumps(payload))
    assert any(mesh['kind'] == 'feature' for mesh in payload['meshes']), '3D payload contains no mounting cut preview'
    for mode in ('Longitudinal', 'Front'):
        canvas = RecordingCanvas(900, 600)
        draw_canvas_view(canvas, model, None, mode, 0, '#2aff2a', '#000000')
        assert any('feature' in item['opts'].get('tags', ()) for item in canvas.items), f'{mode} drawing hides mounting cuts'
    print('OK: fastener cuts visible in front, side and 3D preview payloads')


CHECKS = {'joints': check_joints, 'mounts': check_mounts, 'views': check_views}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path)
    parser.add_argument('--check', choices=CHECKS, required=True)
    args = parser.parse_args()
    definition = json.loads((args.project / 'aircraft.json').read_text())
    output = args.project / 'cad' / 'fastener-check'
    output.mkdir(parents=True, exist_ok=True)
    parts = load_parts_data()
    CHECKS[args.check](definition, parts, output)


if __name__ == '__main__':
    main()
