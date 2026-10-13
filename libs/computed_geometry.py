"""Compute Geometry: build every CAD solid of a design once and measure its exact printed mass (headless).

compute_geometry builds what every export uses (printable parts, the whole-airframe pieces and fused shape, the
drawing projections) into libs.solid_cache, so the exports of that design only write files afterwards, and measures
the printed structure on the built solids (exact_structure). resolve_airframe takes the result in place of its
analytic estimates.

Exact mass, by the estimate each piece replaces and with the density that estimate uses:
- fuselage section n: its printable part S0n less the surface roots outside the fuselage skin, plus its hatch pieces.
  Everything inside the skin is the section's. In a hollow section the material inside the bore (carry-through arms,
  spigot roots) is print bulk density and the rest (the wall) material density; a solid section is bulk density
  throughout. A body pod outside the skin is the section's too, at bulk density.
- surface set: its roots outside the fuselage skin, its detached segments W.. and its controls CS.., bulk density.
- coupler: C0n, material density.
Volumes, centroids and inertia come from OpenCascade's volume properties of the solids themselves.

Only the measured numbers are saved (cad/exact_mass.json, with the design digest and densities they belong to), so
the sim and headless runs use them; solids are never written to or read from disk here.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from libs.access import access_composition
from libs.airframe import Airframe
from libs.body_solids import CUT_OVERSHOOT_MM, build_body_solid, section_bounds
from libs.cad import Progress, airframe_pieces, build_printable_parts, fused_airframe, panel_attachments
from libs.drawings import drawing_geometry
from libs.geometry import ExactPiece, ExactStructure, Inertia, ResolvedAirframe, Vec, resolve_airframe
from libs.solid_cache import cached, design_digest, is_built

M_TO_MM = 1000.0
EXACT_MASS_FILE = "exact_mass.json"
EXACT_MASS_FORMAT = "exact_mass/1"


@dataclass(frozen=True)
class Moments:
    """Mass moments about the origin: mass (kg), first moments (kg m) and raw second moments (kg m^2: sum m xx, yy,
    zz, xy, xz, yz). Additive, so pieces of one part can be added and subtracted before densities differ."""
    mass: float
    first: Vec
    second: tuple[float, float, float, float, float, float]

    def __add__(self, other: Moments) -> Moments:
        return Moments(self.mass + other.mass, tuple(a + b for a, b in zip(self.first, other.first)),
                       tuple(a + b for a, b in zip(self.second, other.second)))

    def __sub__(self, other: Moments) -> Moments:
        return Moments(self.mass - other.mass, tuple(a - b for a, b in zip(self.first, other.first)),
                       tuple(a - b for a, b in zip(self.second, other.second)))

    def scaled(self, factor: float) -> Moments:
        return Moments(self.mass * factor, tuple(value * factor for value in self.first), tuple(value * factor for value in self.second))

    def piece(self, name: str) -> ExactPiece:
        """Mass, centroid and inertia about it per kg (tensor entries, as libs.geometry.Inertia)."""
        if self.mass <= 0.0:
            raise ValueError(f"{name}: the built solids hold no material for it")
        x, y, z = (value / self.mass for value in self.first)
        sxx, syy, szz, sxy, sxz, syz = (value / self.mass for value in self.second)
        cxx, cyy, czz, cxy, cxz, cyz = sxx - x * x, syy - y * y, szz - z * z, sxy - x * y, sxz - x * z, syz - y * z
        inertia: Inertia = (cyy + czz, cxx + czz, cxx + cyy, -cxy, -cxz, -cyz)
        return ExactPiece(self.mass, (x, y, z), inertia)


NO_MOMENTS = Moments(0.0, (0.0, 0.0, 0.0), (0.0,) * 6)


def volume_moments(shape) -> Moments:
    """The shape's volume moments (density 1 kg/m^3: mass = volume in m^3), from OpenCascade's volume properties."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    if shape is None or not shape.Solids():
        return NO_MOMENTS
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    volume_mm3 = props.Mass()
    if volume_mm3 <= 0.0:
        return NO_MOMENTS
    centre = props.CentreOfMass()
    centroid = (centre.X() / M_TO_MM, centre.Y() / M_TO_MM, centre.Z() / M_TO_MM)
    matrix = props.MatrixOfInertia()  # about the centroid, tensor entries (products negative), mm^5
    ixx, iyy, izz = matrix.Value(1, 1), matrix.Value(2, 2), matrix.Value(3, 3)
    ixy, ixz, iyz = matrix.Value(1, 2), matrix.Value(1, 3), matrix.Value(2, 3)
    per_volume = 1.0 / volume_mm3 / M_TO_MM ** 2  # mm^5 / mm^3 -> m^2
    covariance = ((iyy + izz - ixx) / 2.0, (ixx + izz - iyy) / 2.0, (ixx + iyy - izz) / 2.0, -ixy, -ixz, -iyz)
    x, y, z = centroid
    products = (x * x, y * y, z * z, x * y, x * z, y * z)
    volume = volume_mm3 / M_TO_MM ** 3
    return Moments(volume, (volume * x, volume * y, volume * z),
                   tuple(volume * (value * per_volume + product) for value, product in zip(covariance, products)))


def densities(resolved: ResolvedAirframe, parts_data: dict) -> tuple[float, float | None]:
    """(print material density, print bulk density) in kg/m^3, as the analytic estimates use them."""
    return parts_data["material"][resolved.airframe.print.material]["density_kg_m3"], resolved.airframe.print.bulk_density_kg_m3


def _bulk(bulk_density: float | None, what: str) -> float:
    if bulk_density is None:
        raise ValueError(f"print.bulk_density_kg_m3 is needed for the exact mass of {what}")
    return bulk_density


def exact_structure(resolved: ResolvedAirframe, parts_data: dict, *, progress: Progress | None = None) -> ExactStructure:
    """The printed structure measured on the built printable parts (built once per design and densities).

    progress (optional): "building <part>" while the printable parts are built, then "measuring <part>"."""
    material, bulk = densities(resolved, parts_data)
    return cached(resolved, ("exact structure", material, bulk), lambda: _measure(resolved, material, bulk, progress))


def _measure(resolved: ResolvedAirframe, material: float, bulk: float | None, progress: Progress | None) -> ExactStructure:
    airframe = resolved.airframe
    parts = {name: shape for name, shape, _notes in build_printable_parts(resolved, progress=progress)}
    fixed, segments, controls = panel_attachments(resolved)
    hatches = {piece.part_id: index for index, section in access_composition(resolved).items() for piece in section.retained}
    sections = [NO_MOMENTS] * len(section_bounds(resolved))
    surfaces = {surface_set.name: NO_MOMENTS for surface_set in resolved.surfaces}
    couplers: dict[int, Moments] = {}
    wall = airframe.fuselage.section.wall_radial_m
    overshoot = CUT_OVERSHOOT_MM / M_TO_MM
    outer_body = build_body_solid(resolved, 0.0, resolved.airframe.fuselage.length_m).val()

    for count, (name, shape) in enumerate(parts.items(), start=1):
        if progress is not None:
            progress(f"measuring {name}", count, len(parts))
        if name.startswith("S"):
            index = int(name[1:]) - 1
            x0, x1 = section_bounds(resolved)[index]
            hollow = not airframe.sections.per_section[index].solid
            bore = build_body_solid(resolved, x0 - overshoot, x1 + overshoot, inset_m=wall) if hollow else None
            bore = None if bore is None else bore.val()
            box = shape.BoundingBox()
            owners: dict[str | None, list] = {}
            for _label, solid, owner in fixed:
                other = solid.BoundingBox()
                if other.xmax >= box.xmin and other.xmin <= box.xmax:
                    owners.setdefault(owner, []).append(solid)
            taken = None
            attached = NO_MOMENTS
            for owner, solids in owners.items():  # each owner's material outside the skin, once (both panels' roots fused)
                region = (solids[0] if len(solids) == 1 else solids[0].fuse(*solids[1:])).cut(outer_body)
                if taken is not None:
                    region = region.cut(taken)
                taken = region if taken is None else taken.fuse(region)
                moments = volume_moments(shape.intersect(region))
                attached = attached + moments
                if owner is None:  # a body pod: the section's own, printed solid
                    sections[index] = sections[index] + moments.scaled(_bulk(bulk, f"the pod in {name}"))
                else:
                    surfaces[owner] = surfaces[owner] + moments.scaled(_bulk(bulk, f"surface {owner!r}"))
            inside = NO_MOMENTS if bore is None else volume_moments(shape.intersect(bore))
            rest = volume_moments(shape) - attached - inside
            if hollow:
                sections[index] = sections[index] + rest.scaled(material)
                if inside.mass > 0.0:
                    sections[index] = sections[index] + inside.scaled(_bulk(bulk, f"the carry-through in {name}"))
            else:
                sections[index] = sections[index] + rest.scaled(_bulk(bulk, f"solid section {name}"))
        elif name.startswith("H"):
            index = hatches[name]
            density = _bulk(bulk, f"hatch {name}") if airframe.sections.per_section[index].solid else material
            sections[index] = sections[index] + volume_moments(shape).scaled(density)
        elif name.startswith("C") and not name.startswith("CS"):
            couplers[int(name[1:]) - 1] = volume_moments(shape).scaled(material)
    for name, _solid, owner in [*segments, *controls]:
        surfaces[owner] = surfaces[owner] + volume_moments(parts[name]).scaled(_bulk(bulk, f"surface {owner!r}"))

    return ExactStructure(
        sections=tuple(moments.piece(f"section {index + 1}") for index, moments in enumerate(sections)),
        surfaces={name: moments.piece(f"surface {name!r}") for name, moments in surfaces.items()},
        couplers={index: moments.piece(f"coupler {index + 1}") for index, moments in couplers.items()},
    )


def compute_geometry(resolved: ResolvedAirframe, parts_data: dict, *, progress: Progress | None = None) -> ExactStructure:
    """Build every solid the exports of this design use and measure its printed structure; the exports afterwards only
    write files. Returns the exact structure (also held in libs.solid_cache).

    progress (optional): the printable parts ("building", "measuring"), then "building airframe pieces", "fusing"
    and "projecting <drawing>"."""
    exact = exact_structure(resolved, parts_data, progress=progress)
    if progress is not None and not is_built(resolved, ("airframe pieces",)):
        progress("building airframe pieces", 0, 1)
    airframe_pieces(resolved)
    fused_airframe(resolved, progress=progress)
    drawing_geometry(resolved, progress=progress)
    return exact


@dataclass(frozen=True)
class MeasuredDesign:
    """An exact structure with the design digest and densities it was measured for (libs.solid_cache.design_digest)."""
    digest: str
    densities: tuple[float, float | None]
    exact: ExactStructure

    def applies_to(self, resolved: ResolvedAirframe, parts_data: dict) -> bool:
        return self.digest == design_digest(resolved) and self.densities == densities(resolved, parts_data)


def measured(resolved: ResolvedAirframe, parts_data: dict, exact: ExactStructure) -> MeasuredDesign:
    return MeasuredDesign(design_digest(resolved), densities(resolved, parts_data), exact)


def _piece_json(piece: ExactPiece) -> dict:
    return {"mass_kg": piece.mass_kg, "position_m": list(piece.position), "inertia_per_kg": list(piece.inertia_per_kg)}


def _piece(data: dict) -> ExactPiece:
    return ExactPiece(data["mass_kg"], tuple(data["position_m"]), tuple(data["inertia_per_kg"]))


def save_measured(cad_dir: Path, design: MeasuredDesign) -> Path:
    """Write the measured numbers with the design digest and densities they belong to (cad/exact_mass.json)."""
    path = Path(cad_dir) / EXACT_MASS_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    exact = design.exact
    path.write_text(json.dumps({
        "format": EXACT_MASS_FORMAT, "digest": design.digest,
        "material_density_kg_m3": design.densities[0], "bulk_density_kg_m3": design.densities[1],
        "sections": [_piece_json(piece) for piece in exact.sections],
        "surfaces": {name: _piece_json(piece) for name, piece in exact.surfaces.items()},
        "couplers": {str(index): _piece_json(piece) for index, piece in exact.couplers.items()},
    }, indent=2))
    return path


def load_measured(cad_dir: Path) -> MeasuredDesign | None:
    """The saved measurement, whichever design it belongs to; None when there is none (or of another format)."""
    path = Path(cad_dir) / EXACT_MASS_FILE
    if not path.is_file():
        return None
    data = json.loads(path.read_text())
    if data["format"] != EXACT_MASS_FORMAT:
        return None
    return MeasuredDesign(data["digest"], (data["material_density_kg_m3"], data["bulk_density_kg_m3"]), ExactStructure(
        sections=tuple(_piece(piece) for piece in data["sections"]),
        surfaces={name: _piece(piece) for name, piece in data["surfaces"].items()},
        couplers={int(index): _piece(piece) for index, piece in data["couplers"].items()},
    ))


def resolve_project(airframe: Airframe, parts_data: dict, cad_dir: Path) -> tuple[ResolvedAirframe, list[str], bool]:
    """Resolve a project's airframe with its saved exact structure when that belongs to this design, else with the
    analytic estimates: (resolved, problems, exact). The headless runs and the sim fly the numbers the editor shows."""
    resolved, problems = resolve_airframe(airframe, parts_data)
    design = load_measured(cad_dir)
    if design is None or not design.applies_to(resolved, parts_data):
        return resolved, problems, False
    resolved, problems = resolve_airframe(airframe, parts_data, design.exact)
    return resolved, problems, True
