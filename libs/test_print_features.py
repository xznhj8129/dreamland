"""Exercise printable fastener openings and their live previews.

Run: python -m libs.test_print_features aircraft/example --check joints|views
CAD diagnostics are retained under the project's cad/fastener-check directory.
"""

import argparse
import copy
import json
import math
from pathlib import Path

import cadquery as cq

from libs.airframe import load_parts_data, parse_airframe
from libs.cad import _ring, _screw_holes
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


CHECKS = {'joints': check_joints, 'views': check_views}


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
