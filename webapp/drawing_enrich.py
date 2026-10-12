"""Longitudinal- and front-view enrichment for the browser drawing, applied after the scene has been recorded.

libs/graphics.py records panel quads, wing spar lines and physical hinge cylinders. The browser adapter
replaces the panel and wing spar outlines with projected airfoil profiles and cylinder cross-sections, and
supplies airfoil profiles at surface print cuts to the shared drawing helpers. The
airfoil coordinates are the exact ones libs.cad lofts the solids from. The draw payload also carries the per-surface
roll-slider detents derived from each surface's span direction.
"""

from __future__ import annotations

import math
from functools import lru_cache
from typing import Any, Callable

from libs.cad import _airfoil_section_coordinates
from libs.geometry import ResolvedAirframe, chord_direction, thickness_direction
from libs.graphics import convex_hull, draw_projected_rod, draw_surface_cut, panel_outline

SPAR_TAG_PREFIX = "spars:"
PROFILE_WIDTH_PX = 2
EDGE_WIDTH_PX = 2
MATCH_DECIMALS = 2  # recorded items are matched with coordinates rounded to 0.01 px

Project = Callable[[tuple[float, float, float]], tuple[float, float]]
Vec = tuple[float, float, float]


@lru_cache(maxsize=None)
def airfoil_profile(airfoil: str) -> tuple[tuple[float, float], ...]:
    """The coordinates libs.cad lofts this airfoil's solids from (libs.cad._airfoil_section_coordinates), cached."""
    return tuple((float(x), float(y)) for x, y in _airfoil_section_coordinates(airfoil))


def section_points(leading_edge, chord_m: float, direction, thickness, coordinates) -> list[tuple[float, float, float]]:
    """One airfoil section as libs.cad.build_wing_solid places it: LE + chord * (x * chord_dir + y * thickness_dir)."""
    return [tuple(le + chord_m * (x * cd + y * td) for le, cd, td in zip(leading_edge, direction, thickness)) for x, y in coordinates]


def _is_spar_item(item: dict[str, Any]) -> bool:
    tags = item["opts"].get("tags")
    return isinstance(tags, list) and any(isinstance(tag, str) and tag.startswith(SPAR_TAG_PREFIX) for tag in tags)


def _rounded(coords) -> tuple[float, ...]:
    return tuple(round(float(value), MATCH_DECIMALS) for value in coords)


def _replace_panel_quad(canvas, panel, surface_index: int, project: Project, ui_draw: str) -> None:
    """Drop the recorded flat surface quad and append the panel's LE and TE span edges in its place.

    The quad is identified by its exact projected outline (the panel_outline draw_longitudinal_view recorded), its
    outline colour and its ("drag", "surface:<index>") tag, so control-surface quads are not touched.
    """
    expected = _rounded(value for point in panel_outline(panel) for value in project(point))
    tag = f"surface:{surface_index}"
    for position, item in enumerate(canvas.items):
        if item["kind"] != "polygon" or len(item["coords"]) != 8:
            continue
        tags = item["opts"].get("tags")
        if not isinstance(tags, list) or tag not in tags or item["opts"].get("outline") != ui_draw:
            continue
        if _rounded(item["coords"]) != expected:
            continue
        del canvas.items[position]
        break
    for root, tip in ((panel.attach_le, panel.tip_le), (panel.attach_te, panel.tip_te)):
        (x0, y0), (x1, y1) = project(root), project(tip)
        canvas.create_line(x0, y0, x1, y1, fill=ui_draw, width=EDGE_WIDTH_PX, tags=("drag", tag))


def enrich_longitudinal_scene(canvas, resolved: ResolvedAirframe, project: Project, ui_draw: str, rod_color: str) -> None:
    """Append the root/tip airfoil profiles and spar cross-sections of the side view, replacing the recorded lines.

    project maps a definition point to screen pixels exactly as draw_longitudinal_view did. The recorded flat panel
    quads and slanted spar lines are dropped; each panel keeps its LE and TE span edges, each spar gets its two
    projected end cross-sections plus their dotted silhouette hull.
    """
    canvas.items[:] = [item for item in canvas.items if not _is_spar_item(item)]

    # Airfoil profiles: root at the panel's true root LE, tip at its LE, both in the panel's chord/normal frame
    # (incidence-aware), exactly as libs.cad.build_wing_solid builds them. Identical projected sections of the
    # same surface (e.g. mirrored panels seen edge-on at roll 0) are appended once.
    seen: set[tuple[int, tuple[float, ...]]] = set()
    for surface_index, surface_set in enumerate(resolved.surfaces):
        for panel in surface_set.panels:
            _replace_panel_quad(canvas, panel, surface_index, project, ui_draw)
            coordinates = airfoil_profile(panel.airfoil)
            direction = chord_direction(panel)
            thickness = thickness_direction(panel)
            for leading_edge, chord_m in ((panel.attach_le, panel.chord_root_m), (panel.tip_le, panel.chord_tip_m)):
                points = [project(point) for point in section_points(leading_edge, chord_m, direction, thickness, coordinates)]
                outline = tuple(value for point in points for value in point)
                key = (surface_index, outline)
                if key in seen:
                    continue
                seen.add(key)
                canvas.create_polygon(outline, outline=ui_draw, fill="", width=PROFILE_WIDTH_PX, tags=("drag", f"surface:{surface_index}"))

    # Spar rods: the cross-section circle at each end, in the plane perpendicular to the rod axis (so the projection
    # is an ellipse when the axis leaves the view plane), plus the dotted hull of both projected circles (the rod's
    # silhouette along its length), replacing the recorded slanted spar line.
    surface_indices = {surface_set.name: index for index, surface_set in enumerate(resolved.surfaces)}
    surface_sets = {surface_set.name: surface_set for surface_set in resolved.surfaces}
    spar_index: dict[tuple[str, int], int] = {}  # build_spars appends by spar index then panel, so rank within the pair is the index
    for spar in resolved.spars:
        key = (spar.surface, spar.panel_index)
        index = spar_index.get(key, 0)
        spar_index[key] = index + 1
        surface_set = surface_sets[spar.surface]
        spars = surface_set.definition.spars
        along_m = spars.x_offset_m + index * spars.spacing_m
        panel = next(panel for panel in surface_set.panels if panel.index == spar.panel_index)
        root = tuple(le + along_m * cd + spars.y_offset_m * td
                     for le, cd, td in zip(panel.attach_le, chord_direction(panel), thickness_direction(panel)))
        tags = ("drag", f"spars:{surface_indices[spar.surface]}")
        draw_projected_rod(canvas, root, spar.end, spar.diameter_m, project, rod_color, tags)
    enrich_surface_cuts(canvas, resolved, project)


def _drop_front_span_line(canvas, panel, project: Project, ui_draw: str) -> None:
    """Drop the untagged straight root_le-to-tip_le line draw_front_view recorded for this panel, when present."""
    expected = _rounded(value for point in (panel.root_le, panel.tip_le) for value in project(point))
    for position, item in enumerate(canvas.items):
        if item["kind"] != "line" or len(item["coords"]) != 4 or item["opts"].get("tags"):
            continue
        if item["opts"].get("fill") != ui_draw:
            continue
        coords = _rounded(item["coords"])
        if coords == expected or coords == expected[2:] + expected[:2]:
            del canvas.items[position]
            return


def enrich_front_scene(canvas, resolved: ResolvedAirframe, project: Project, ui_draw: str) -> None:
    """Append each front-view panel's thickness band, replacing its recorded straight span line.

    The band is the convex hull of the root and tip airfoil sections (the same sections the longitudinal profiles
    use) projected with the front-view mapping and tagged ("drag", "surface:<index>"), so the band itself serves
    as that surface's drag handle.
    """
    for surface_index, surface_set in enumerate(resolved.surfaces):
        for panel in surface_set.panels:
            _drop_front_span_line(canvas, panel, project, ui_draw)
            coordinates = airfoil_profile(panel.airfoil)
            direction = chord_direction(panel)
            thickness = thickness_direction(panel)
            points: list[tuple[float, float]] = []
            for leading_edge, chord_m in ((panel.attach_le, panel.chord_root_m), (panel.tip_le, panel.chord_tip_m)):
                points += [project(point) for point in section_points(leading_edge, chord_m, direction, thickness, coordinates)]
            hull = convex_hull(points)
            canvas.create_polygon([value for point in hull for value in point], outline=ui_draw, fill="", width=PROFILE_WIDTH_PX,
                                  tags=("drag", f"surface:{surface_index}"))
    enrich_surface_cuts(canvas, resolved, project)


def roll_detents(resolved: ResolvedAirframe | None) -> list[dict[str, Any]]:
    """Per-surface roll-slider detents from the first panel's span direction: planform (90 - delta) and tip (-delta).

    delta is the span angle off the horizontal in the front view; each angle is wrapped modulo 180 deg into
    [-90, 90] (the flat wing's planform +90 stays +90, not -90) and rounded to 0.1 deg, and equal angles are
    kept once (the first surface/kind wins).
    """
    if resolved is None:
        return []
    detents: list[dict[str, Any]] = []
    seen: set[float] = set()
    for surface_set in resolved.surfaces:
        span = surface_set.panels[0].span_dir
        delta = math.degrees(math.atan2(span[2], span[1]))
        for kind, angle in (("planform", 90.0 - delta), ("tip", -delta)):
            roll_deg = round(_wrap_roll(angle), 1)
            if roll_deg in seen:
                continue
            seen.add(roll_deg)
            detents.append({"surface": surface_set.name, "kind": kind, "roll_deg": roll_deg})
    return detents


def _wrap_roll(angle: float) -> float:
    """An angle modulo 180 deg, in [-90, 90]: +90 is kept when the angle was positive (a flat wing's planform)."""
    wrapped = (angle + 90.0) % 180.0 - 90.0
    return 90.0 if wrapped == -90.0 and angle > 0.0 else wrapped


def enrich_surface_cuts(canvas, resolved: ResolvedAirframe, project: Project) -> None:
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            span_m = surface_set.definition.span_m
            for cut in surface_set.definition.sections.cuts_m:
                fraction = cut / span_m
                leading_edge = tuple(a + fraction * (b - a) for a, b in zip(panel.attach_le, panel.tip_le))
                chord = panel.chord_root_m + fraction * (panel.chord_tip_m - panel.chord_root_m)
                points = section_points(leading_edge, chord, chord_direction(panel), thickness_direction(panel), airfoil_profile(panel.airfoil))
                draw_surface_cut(canvas, points, project, ("surface-cut",))
