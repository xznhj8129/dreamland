"""Compute Geometry: build every CAD solid of a design once and measure its exact printed mass (headless).

compute_geometry builds what every export uses (printable parts, the whole-airframe pieces and fused shape, the
drawing projections) into libs.solid_cache, so the exports of that design only write files afterwards, and measures
the printed structure on the built solids (exact_structure). resolve_airframe takes the result in place of its
analytic estimates.

Exact mass, by the estimate each piece replaces and with the density that estimate uses, from the volume moments
the section builds recorded as they fused and cut (libs.cad.SectionLedger) and the volumes of the separate parts: no
solid is built only to be measured.
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
from libs.body_solids import section_bounds
from libs.cad import Progress, airframe_pieces, build_printable_parts, fused_airframe, panel_attachments, section_build
from libs.drawings import drawing_geometry
from libs.geometry import ExactPiece, ExactStructure, ResolvedAirframe, resolve_airframe
from libs.solid_cache import cached, design_digest, is_built
from libs.volume_moments import NO_MOMENTS, Moments, volume_moments

EXACT_MASS_FILE = "exact_mass.json"
EXACT_MASS_FORMAT = "exact_mass/1"


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
    """The exact structure from the printable parts and their section ledgers (libs.cad.SectionLedger): sums and
    differences of volume moments the builds recorded, and the volumes of separate parts. No solid is built here."""
    airframe = resolved.airframe
    parts = {name: shape for name, shape, _notes in build_printable_parts(resolved, progress=progress)}
    _fixed, segments, controls = panel_attachments(resolved)
    hatches = {piece.part_id: index for index, section in access_composition(resolved).items() for piece in section.retained}
    count = len(section_bounds(resolved))
    sections = [NO_MOMENTS] * count
    surfaces = {surface_set.name: NO_MOMENTS for surface_set in resolved.surfaces}
    couplers: dict[int, Moments] = {}

    for index in range(count):
        name = f"S{index + 1:02d}"
        if progress is not None:
            progress(f"measuring {name}", index + 1, count)
        ledger = section_build(resolved, index).ledger
        roots = NO_MOMENTS
        for owner, moments in ledger.surfaces.items():
            surfaces[owner] = surfaces[owner] + moments.scaled(_bulk(bulk, f"surface {owner!r}"))
            roots = roots + moments
        rest = ledger.total - roots - ledger.pods
        sections[index] = ledger.pods.scaled(_bulk(bulk, f"the pod in {name}")) if ledger.pods.mass > 0.0 else NO_MOMENTS
        if airframe.sections.per_section[index].solid:
            sections[index] = sections[index] + rest.scaled(_bulk(bulk, f"solid section {name}"))
        else:
            sections[index] = sections[index] + (rest - ledger.spared).scaled(material)
            if ledger.spared.mass > 0.0:
                sections[index] = sections[index] + ledger.spared.scaled(_bulk(bulk, f"the carry-through in {name}"))
    for name, shape in parts.items():
        if name.startswith("H"):
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
