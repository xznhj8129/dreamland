# Plan: plane-first airframe model

Status: draft spec, awaiting approval. Supersedes the Lego module-stack model as the core.

## Decisions (owner, 2026-10-01)

- Design the aircraft as a whole, then cut it into printable sections. The module library is not the core.
- Sectioning: tool proposes cuts automatically; user can move or lock cut lines.
- Printed structure mass: computed from geometry and print settings; any section or part can be overridden with a weighed value.
- `m62v2.json` and `quadlancer62.json` (real built aircraft) are re-entered plane-first as validation cases.
- Component data differences (e.g. battery weight) are data errors, not design variants.
- Keep it parametric and define everything. Tube standards (M62, M75) are presets that fill parameters, not constraints.
- Fuselage cross-section is not limited to circular (rectangular, octagonal, rounded corners later); circular first.
- Reserved internal space as simple envelopes and channels. No cooling flow or insertion-path simulation.
- Current joint: 8 radial screws into a coupler, plus longitudinal spar rods for stiffness (M62 standard in `data/specs.json`).
- The builder's definition is the master. CAD (CadQuery, optionally FreeCAD) is a downstream consumer, not a second aircraft model.
- Generating printable section bodies (shells, bulkheads, mounts, channel cuts) is later CAD work.
- Main design driver after the overall shape: user-defined internal volumes (like battery and payload) with a size (e.g. 20x20x20 mm, or internal diameter x 50 mm), a mass (fixed, e.g. 20 g, or a range, e.g. 10-30 g), dragged to where they are needed. They drive and constrain everything else.
- CadQuery generates the bigger sections and volumes; final fine adjustments may be done in FreeCAD. Not every facet and cut is defined in spec files.
- Old part sums are approximately accurate (m62v2 ~1.1 kg real).

## 1. Aircraft definition (saved file, schema "airframe/2")

Implemented in `libs/airframe.py` (`python -m libs.airframe aircraft/m62v2.json`). SI units (m, kg); angles in degrees; x from the nose tip. Library parts are referenced as `<category>:<name>` from `data/parts.json`. Loading fails on any missing required field, unknown field, bad value or unresolved reference.

```
schema              "airframe/2"
name, notes
preset              optional label (M62, M75); values are always written out in full
operating_point     aoa_deg, airspeed_m_s, altitude_m
print               material (parts.json material), printer_bed_m [x, y, z],
                    surface_infill (required only if a surface has no mass_kg)
fuselage
  length_m
  section           width_m, height_m, shape (superellipse exponent; 2 = circle), wall_radial_m
  nose, tail        profile (pointed_shapes name), length_m, params (exactly the profile's keys)
  joints            type radial_screws, count, screw, insertion_depth_m, coupler_mass_kg (optional, one per cut)
  spar_rods         optional: count, diameter_m, radial_position_m, angle_offset_deg, material
  channels[]        name, diameter_m, from_x_m, to_x_m, angle_deg
surfaces[]          name, x_root_le_m (must be on the body; chords may overhang the tail),
                    layout (H X + 1 3), fin_position (top/bottom), dihedral_deg, root_offset (-1..1),
                    root (body: span from the body surface at the root LE; axis: span from the fuselage axis,
                    panels meet in the centre, e.g. delta tail fins), span_m (per panel, along the panel),
                    chord_root_m, chord_tip_m, sweep_le_deg, incidence_deg, airfoil (AeroSandbox name),
                    mass_kg (whole set; optional, absent = computed)
propulsion
  motor             brand, model, kv, prop (must have bench data in libs/motordata.csv for the battery's cell count;
                    motor mass from the same data, one consistent weight required)
  battery_volume    name of the volume whose part is battery:<name>
  mounts[]          on (nose | tail | surface); for surface: surface, span_position_m (from panel root),
                    x_offset_m (from root LE); direction (tractor | pusher);
                    pod optional: diameter_m, flat_length_m, curve_length_m, profile (parameter-free)
volumes[]           name, x_m (centre), y_m and z_m (default 0), shape:
                      box: size_m [x, y, z], or from the part's dimensions
                      cylinder: diameter_m, length_m, axis x|y|z
                      bore: length_m (fills the internal section)
                      point: mass only
                    mass_kg: number or [min, max], or taken from part (must be > 0)
sections
  cuts_m[]          cut stations (ascending, inside the fuselage); section i spans cut i-1 .. cut i
  mass_kg[]         optional, one per section: weighed printed mass (includes whatever is mounted in it), null = computed shell
```

Body-rooted panels keep the true root edge on the body (follows nose/tail cones) and use an equivalent aero root at the mean body radius under the root chord. Axis-rooted panels start on the axis; Barrowman uses their exposed part.

Converted examples: `aircraft/m62v2.json` and `aircraft/quadlancer62.json` reproduce the old totals (1.1408 / 1.3732 kg, plus props 1.1608 / 1.4292 kg). The old modules are sections (cuts at the module boundaries, module mass as the section's weighed mass), couplers 21.36 g per joint, the battery is the only volume. m62v2 tail fins are axis-rooted, 55 mm span (delta: straight trailing edge, joined at the centre).

## 2. Libraries (data/)

- `parts.json`: batteries (mass required, > 0), ESCs, FCs, cameras, payloads, props (mass), materials.
- Motors: `libs/motordata.csv` (bench data), later `dronebuildcalc` motorlib for untested motors.
- `specs.json`: tube standards, couplers, fasteners, openings, printer bed (exists).
- Airfoils: one source (AeroSandbox names), `data/airfoils.json` only for imported coordinates.
- `modules.json`: kept only as reference geometry for converting the two example aircraft, then retired.

## 3. Derived model (one pass, everything else reads it)

`resolve_airframe(definition, libraries) -> airframe`:

- **Geometry**: body as stations (x, width, height, superellipse shape) with nose/tail roles assigned once; lifting panels with root/tip LE xyz, chords, unit chord and normal vectors, mirror-pair id, exposed root station; motors and prop discs (centre, axis, radius).
- **Mass**: 3D point masses for every printed section, surface set, motor, prop, component; total mass, CG (x, y, z), inertia tensor.
- **Sections**: cut stations, section type (nose, body, wing mount, tail). Each seam is defined once (station, joint, spar rod and channel pass-through); both mating ends derive from it.
- **Checks**: volume envelopes vs each other, wall, spar rods and channels; sections vs printer bed; mass/CG.
- **Mass ranges**: volumes with [min, max] mass give a CG range and a static margin range.

Consumers, each a thin reader: AeroBuildup and VLM (`analysis.py`), Barrowman, Tk drawing (`graphics.py`), CadQuery export (`cad.py`), and later the JSBSim exporter.

## 4. Auto sectioning (then user adjusts)

Constraints, in order:
1. Nose and tail are their own sections.
2. A wing set's mount section contains its full root chord plus margin; no cut through a root chord.
3. No cut through a volume envelope (plus `rules.clearance`).
4. Section length plus coupler depth fits the printer bed (`3dp.constraints`).
5. Locked cuts are kept as-is.

Then minimise section count, then balance lengths. Infeasible combinations (e.g. a battery longer than the bed) fail with the reason.

## 5. Structural mass

- Body: wall shell volume between cuts (cross-section perimeter x radial wall) x material density.
- Couplers and spar rods: from the joint and rod parameters (rod material density).
- Nose/tail: shell of the profile.
- Surfaces: airfoil cross-section area x span x infill x density.
- Any section or surface set can be overridden with a weighed mass.

## 6. Validation

Re-enter `m62v2` and `quadlancer62` plane-first. Compare derived total mass and CG with the module-sum values (m62v2: 1.1408 kg, CG 0.334 m) and with real measured values if available. `aero_check.py` keeps cross-checking aero.

## 7. Phases (approval between phases)

1. Done 2026-10-01: schema loader and validator (`libs/airframe.py`), PLA+ in `data/parts.json` material, the two converted examples in `aircraft/`. Headless only. Prop masses not added (unknown).
2. `resolve_airframe` geometry; AeroBuildup, VLM and Barrowman read it; old module stacking removed from `analysis.py`.
3. Drawing and CAD read the shared geometry.
4. Mass model: computed structure, 3D CG, inertia; propulsion physics fixes from `TODO.md` C.
5. Auto sectioning and coupler generation; printable section STEP/STL.
6. GUI rebuilt around the definition (edit plane parameters, move/lock cuts).
7. Aero refinements from `TODO.md` D.
8. JSBSim export.
