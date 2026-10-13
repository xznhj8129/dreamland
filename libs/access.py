"""Physical cut geometry of the saved Access entries (headless, shared by the CAD exports and the view outlines).

The profile lives in the entry's resolved local frame (libs.geometry.resolve_access): local X is the length, local Y
the width, local +Z the extrusion/cut direction. This module decides the cut extent against the real printed fuselage
material - the section solids libs.body_solids builds (outer body less the inner cavity or the volumes, exactly what
libs.cad hollows its printable sections from) - by intersecting the full profile prism with actual kernel geometry. No second body or volume model is built here, nothing is saved in aircraft.json, and no exporter is imported.

Extents, along the local +Z from the profile plane (metres, positive forward):
- depth: the given positive depth_m.
- wall: through the first encountered wall interval over the whole profile footprint, into the cavity behind it,
  stopping clear of the opposite wall; the profile plane may lie just outside the skin.
- all: from before the first material to past the last, along the cutter line over the footprint.

Hole cutters and hatch opening cutters are the same profile prism; a hatch also carries its retained profile, the
opening eroded by gap_m on each edge (a rounded rectangle whose gap exceeds its corner radius keeps sharp corners).
A miss, a reversed direction or a first wall that cannot be cleared without reaching the opposite wall is reported
on the AccessCut with the entry and instance identity, and no cutter is built.

The AccessSectionMaterial composition records what libs.cad applies: the opening union and every retained hatch
piece with its deterministic part id. Openings that overlap keep their material exactly once: holes always win, an earlier
hatch keeps the overlap and a later hatch keeps only the remaining partition, both with a visible note.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from libs.airframe import AccessEntry
from libs.body_solids import M_TO_MM, printed_section_material, section_bounds
from libs.geometry import AccessInstance, ResolvedAirframe, section_index
from libs.solid_cache import cached

WALL_MARGIN_M = 0.0005  # clear a wall this far into the cavity, and stay this far clear of the opposite wall
BOUNDARY_SAMPLES = 32  # directions probed on the profile outline
GRID_SAMPLES = 3  # interior grid per axis inside the profile bounding box
ACCESS_EPS_MM3 = 1e-3  # an intersection below this volume counts as touching, not cutting


@dataclass(frozen=True)
class AccessCut:
    """One resolved seed or radial copy as physical cut geometry.

    extent_m is (start, end) from the profile plane along the local +Z, metres; cutter is the opening prism (mm,
    aircraft frame); inset_cutter is the hatch retained profile over the same extent (None for a hole); attached is
    True when a hatch's cut floor still lies in printed material (the piece is not separated); problems holds the
    visible diagnostics and is non-empty exactly when no cutter could be built."""
    instance: AccessInstance
    extent_m: tuple[float, float] | None
    cutter: object | None
    inset_cutter: object | None
    attached: bool
    problems: tuple[str, ...]

    @property
    def label(self) -> str:
        return self.instance.instance_id


@dataclass(frozen=True)
class RetainedHatchPiece:
    """One hatch piece kept from the section's original curved material: deterministic part id (H01, _c2 copies,
    _p2 pieces), the entry label, its notes and the solid (cq.Shape, mm)."""
    part_id: str
    label: str
    notes: tuple[str, ...]
    solid: object


@dataclass(frozen=True)
class AccessSectionMaterial:
    """The physical access material of one fuselage section (cq.Shape in mm): the union opening cut from the section
    (opening) and the hatch pieces it keeps (retained). The section print is opening removed from the section body and
    the retained pieces printed separately. notes holds the visible
    diagnostics: unresolved entries, seams, overlaps and piece counts."""
    index: int
    opening: object | None
    retained: tuple[RetainedHatchPiece, ...]
    notes: tuple[str, ...]


def _material_solids(resolved: ResolvedAirframe) -> tuple[tuple[float, float, object], ...]:
    """The real printed section material solids (cq.Shape, mm) with their x bounds, from libs.body_solids (built once
    per design there) for the extent probes and the composition."""
    return tuple((x0, x1, printed_section_material(resolved, index)) for index, (x0, x1) in enumerate(section_bounds(resolved)))


def access_cuts(resolved: ResolvedAirframe) -> tuple[AccessCut, ...]:
    """Every Access seed and radial copy as a physical cut; () without entries, so no CAD work is done. Built once per
    design (libs.solid_cache)."""
    if not resolved.airframe.access:
        return ()
    return cached(resolved, ("access cuts",), lambda: tuple(_build_cut(resolved, instance) for instance in resolved.access))


def access_composition(resolved: ResolvedAirframe) -> dict[int, AccessSectionMaterial]:
    """The true access material per fuselage section: opening and retained hatch pieces. {} without entries; built
    once per design (libs.solid_cache, never saved), so every export shares one physical result."""
    if not resolved.airframe.access:
        return {}
    return cached(resolved, ("access composition",), lambda: _build_composition(resolved))


def _entry_label(entry: AccessEntry, instance: AccessInstance) -> str:
    return instance.instance_id if instance.count > 1 else entry.name


def _profile_geometry(entry: AccessEntry) -> tuple[float, float, float]:
    """(half length, half width, corner radius) of the opening profile, metres."""
    if entry.profile == "slot":
        return entry.length_m / 2.0, entry.width_m / 2.0, entry.width_m / 2.0
    return entry.length_m / 2.0, entry.width_m / 2.0, entry.corner_radius_m


def _inside_profile(u: float, v: float, half_l: float, half_w: float, radius: float) -> bool:
    """Point (u, v) in the profile's local 2D frame; a rounded rectangle with radius 0 is a plain rectangle."""
    if abs(u) > half_l or abs(v) > half_w:
        return False
    inside_l, inside_w = half_l - radius, half_w - radius
    if abs(u) <= inside_l or abs(v) <= inside_w:
        return True
    cu, cv = (math.copysign(inside_l, u), math.copysign(inside_w, v))
    return math.hypot(u - cu, v - cv) <= radius + 1e-12


def _profile_boundary(entry: AccessEntry) -> list[tuple[float, float]]:
    """Boundary and interior sample offsets (u along local X, v along local Y), metres."""
    half_l, half_w, radius = _profile_geometry(entry)
    offsets: list[tuple[float, float]] = [(0.0, 0.0)]
    for index in range(BOUNDARY_SAMPLES):
        angle = 2.0 * math.pi * index / BOUNDARY_SAMPLES
        du, dv = math.cos(angle), math.sin(angle)
        low, high = 0.0, math.hypot(half_l, half_w)
        for _ in range(48):
            middle = (low + high) / 2.0
            if _inside_profile(du * middle, dv * middle, half_l, half_w, radius):
                low = middle
            else:
                high = middle
        offsets.append((du * high, dv * high))
    if GRID_SAMPLES > 1:
        for iu in range(1, GRID_SAMPLES + 1):
            for iv in range(1, GRID_SAMPLES + 1):
                u = half_l * (2.0 * iu / (GRID_SAMPLES + 1) - 1.0)
                v = half_w * (2.0 * iv / (GRID_SAMPLES + 1) - 1.0)
                if _inside_profile(u, v, half_l, half_w, radius):
                    offsets.append((u, v))
    return offsets


def _intersects(first, second) -> bool:
    """True when the boolean common has real volume; a kernel error is left to expose its own traceback."""
    return bool(first.Solids() and second.Solids()) and first.intersect(second).Volume() > ACCESS_EPS_MM3


def _profile_solid(instance: AccessInstance, width_m: float, length_m: float, radius_m: float, start_m: float, end_m: float):
    """The exact profile prism in millimetres: slot/circle (radius = width/2) or rounded rectangle, built as the
    union of its corner discs and central bands, extruded from start to end along the rotated local +Z."""
    import cadquery as cq

    height_mm = (end_m - start_m) * M_TO_MM
    if height_mm <= 0.0:
        return None
    centre = cq.Vector(*(value * M_TO_MM for value in instance.center))
    x_dir = cq.Vector(*instance.axes[0])
    z_dir = cq.Vector(*instance.axes[2])
    plane = cq.Plane(origin=centre + z_dir * (start_m * M_TO_MM), xDir=x_dir, normal=z_dir)
    length, width, radius = length_m * M_TO_MM, width_m * M_TO_MM, radius_m * M_TO_MM
    work = cq.Workplane(plane)
    if radius <= 0:
        return work.rect(length, width).extrude(height_mm).val()
    if abs(length - width) < 1e-8 and abs(radius - width / 2) < 1e-8:
        return work.circle(radius).extrude(height_mm).val()
    if abs(radius - min(length, width) / 2) < 1e-8:
        return work.slot2D(max(length, width), min(length, width), 0 if length >= width else 90).extrude(height_mm).val()
    return work.sketch().rect(length, width).vertices().fillet(radius).finalize().extrude(height_mm).val()


def _build_cut(resolved: ResolvedAirframe, instance: AccessInstance) -> AccessCut:
    entry = instance.entry
    label = _entry_label(entry, instance)
    half_l, half_w, radius = _profile_geometry(entry)
    import cadquery as cq
    materials = _material_solids(resolved)
    corners = []
    for _x0, _x1, material in materials:
        box = material.BoundingBox()
        corners.extend((x, y, z) for x in (box.xmin, box.xmax) for y in (box.ymin, box.ymax) for z in (box.zmin, box.zmax))
    depths = [sum((p / M_TO_MM - c) * d for p, c, d in zip(point, instance.center, instance.cut_direction)) for point in corners]
    low, high = min(depths) - WALL_MARGIN_M, max(depths) + WALL_MARGIN_M
    problems = []
    attached = False
    if entry.extent == "depth":
        start, end = 0.0, entry.depth_m
        floor = _profile_solid(instance, entry.width_m, entry.length_m, radius, end, end + 1e-5)
        attached = any(_intersects(floor, material) for _x0, _x1, material in materials)
    elif entry.extent == "all":
        start, end = min(0.0, low), max(0.0, high)
    else:
        # Intersect the full footprint with actual printed material. A cavity separates the near and far wall
        # pieces. Their exact bounds in the cut frame give a wall-clearing depth over the entire curved profile.
        full = _profile_solid(instance, entry.width_m, entry.length_m, radius, low, high)
        frame = cq.Location(cq.Plane(origin=cq.Vector(*instance.center) * M_TO_MM,
                                     xDir=cq.Vector(*instance.axes[0]), normal=cq.Vector(*instance.axes[2])))
        spans = []
        for _x0, _x1, material in materials:
            for solid in full.intersect(material).Solids():
                box = solid.moved(frame.inverse).BoundingBox()
                spans.append((box.zmin / M_TO_MM, box.zmax / M_TO_MM))
        spans.sort()
        merged = []
        for first, second in spans:
            if merged and first <= merged[-1][1] + 1e-7:
                merged[-1] = (merged[-1][0], max(merged[-1][1], second))
            else:
                merged.append((first, second))
        forward = [iv for iv in merged if iv[1] >= -WALL_MARGIN_M]
        if not forward:
            problems.append(f"{label}: the forward local +Z cuts no material; move the plane outside the skin or reverse the orientation")
        elif len(forward) < 2:
            problems.append(f"{label}: no separated cavity behind the selected wall; reduce the profile or use depth / through-all")
        else:
            start = min(0.0, forward[0][0] - WALL_MARGIN_M)
            end = forward[0][1] + min(WALL_MARGIN_M, (forward[1][0] - forward[0][1]) / 3.0)
    if problems:
        return AccessCut(instance=instance, extent_m=None, cutter=None, inset_cutter=None, attached=False, problems=tuple(problems))
    cutter = _profile_solid(instance, entry.width_m, entry.length_m, radius, start, end)
    inset = None
    if entry.kind == "hatch" and entry.gap_m is not None:
        inset_w, inset_l = entry.width_m - 2.0 * entry.gap_m, entry.length_m - 2.0 * entry.gap_m
        inset_r = max(inset_w / 2.0, 0.0) if entry.profile == "slot" else max(radius - entry.gap_m, 0.0)
        inset = _profile_solid(instance, inset_w, inset_l, inset_r, start, end)
    return AccessCut(instance=instance, extent_m=(start, end), cutter=cutter, inset_cutter=inset, attached=attached, problems=())


def _access_part_id(instance: AccessInstance, piece: int | None = None, pieces: int = 1) -> str:
    """Deterministic safe id: H01, radial copy _c2 (count includes the seed), piece _p2 when one entry keeps more."""
    part_id = f"H{instance.index + 1:02d}"
    if instance.count > 1:
        part_id += f"_c{instance.copy + 1}"
    if pieces > 1 and piece is not None:
        part_id += f"_p{piece + 1}"
    return part_id


def _build_composition(resolved: ResolvedAirframe) -> dict[int, AccessSectionMaterial]:
    cuts = access_cuts(resolved)
    bounds = section_bounds(resolved)
    cuts_by_section: dict[int, list[AccessCut]] = {}
    for cut in cuts:
        if cut.cutter is None:  # unresolved: notes go to the section holding the profile centre
            cuts_by_section.setdefault(section_index(resolved.airframe.sections.cuts_m, cut.instance.center[0]), []).append(cut)
            continue
        box = cut.cutter.BoundingBox()
        for index, (x0, x1) in enumerate(bounds):
            if box.xmax >= x0 * M_TO_MM and box.xmin <= x1 * M_TO_MM:
                cuts_by_section.setdefault(index, []).append(cut)
    materials = dict(enumerate(solid for _x0, _x1, solid in _material_solids(resolved)))
    composition: dict[int, AccessSectionMaterial] = {}
    for index, section_cuts in cuts_by_section.items():
        notes = [problem for cut in section_cuts for problem in cut.problems]
        resolved_cuts = [cut for cut in section_cuts if cut.cutter is not None]
        if not resolved_cuts:
            composition[index] = AccessSectionMaterial(index=index, opening=None, retained=(), notes=tuple(notes))
            continue
        material = materials[index]
        openings = [cut.cutter for cut in resolved_cuts]
        opening = openings[0] if len(openings) == 1 else openings[0].fuse(*openings[1:]).clean()
        holes = [cut.cutter for cut in resolved_cuts if cut.instance.entry.kind == "hole"]
        hole_union = None if not holes else holes[0] if len(holes) == 1 else holes[0].fuse(*holes[1:]).clean()
        retained: list[RetainedHatchPiece] = []
        earlier_openings = []
        for cut in resolved_cuts:
            for cut_station in resolved.airframe.sections.cuts_m:
                if cut.cutter.BoundingBox().xmin < cut_station * M_TO_MM < cut.cutter.BoundingBox().xmax:
                    notes.append(f"{cut.label}: crosses the section seam at {cut_station * M_TO_MM:.1f} mm; couplers and joints are not cut")
            if cut.inset_cutter is None:
                continue
            base = _access_part_id(cut.instance)
            piece = cut.inset_cutter.intersect(material)
            if hole_union is not None and _intersects(piece, hole_union):
                notes.append(f"{base} ({cut.label}): a hole opening overlaps its retained material; the hole owns the overlap")
                piece = piece.cut(hole_union)
            if earlier_openings:
                earlier = earlier_openings[0] if len(earlier_openings) == 1 else earlier_openings[0].fuse(*earlier_openings[1:]).clean()
                if _intersects(piece, earlier):
                    notes.append(f"{base} ({cut.label}): its retained material overlaps an earlier hatch opening; the overlap is kept by the earlier entry so no material is duplicated")
                    piece = piece.cut(earlier)
            earlier_openings.append(cut.cutter)
            solids = piece.Solids() if piece is not None else []
            if not solids:
                notes.append(f"{base} ({cut.label}): no retained material is left (it lies inside another opening)")
                continue
            piece_notes = []
            if cut.attached:
                piece_notes.append(f"{base}: partial depth leaves the hatch attached to the section material; it cannot be removed until the cut reaches the cavity")
            if len(solids) > 1:
                piece_notes.append(f"{base}: the hatch comes out as {len(solids)} separate pieces")
            for number, solid in enumerate(solids):
                retained.append(RetainedHatchPiece(part_id=_access_part_id(cut.instance, number, len(solids)), label=cut.label,
                                                   notes=tuple(piece_notes), solid=solid))
        composition[index] = AccessSectionMaterial(index=index, opening=opening, retained=tuple(retained), notes=tuple(notes))
    by_label: dict[str, list[tuple[int, int, RetainedHatchPiece]]] = {}
    for index in sorted(composition):
        for number, piece in enumerate(composition[index].retained):
            by_label.setdefault(piece.label, []).append((index, number, piece))
    for pieces in by_label.values():
        if len(pieces) < 2:
            continue
        for number, (index, position, piece) in enumerate(pieces, 1):
            base = piece.part_id.split("_p")[0]
            retained = list(composition[index].retained)
            retained[position] = replace(piece, part_id=f"{base}_p{number}")
            composition[index] = replace(composition[index], retained=tuple(retained))
    return composition


def access_outline(instance: AccessInstance) -> tuple[tuple[float, float, float], ...]:
    """The profile outline in the aircraft frame, in metres; shared orthographic/view overlay geometry."""
    return tuple(tuple(c + u * x + v * y for c, x, y in zip(instance.center, instance.axes[0], instance.axes[1]))
                 for u, v in _profile_boundary(instance.entry)[1:BOUNDARY_SAMPLES + 1])

