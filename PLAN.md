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
- A motor-wing section (tube, X-wing arms, motor mounts) is printed in one piece (monocoque). Joining long wings: later.
- Printed surface mass: solid airfoil volume x effective bulk density (owner's calibration from a weighed printed segment).
- Main design driver after the overall shape: user-defined internal volumes (like battery and payload) with a size (e.g. 20x20x20 mm, or internal diameter x 50 mm), a mass (fixed, e.g. 20 g, or a range, e.g. 10-30 g), dragged to where they are needed. They drive and constrain everything else.
- CadQuery generates the bigger sections and volumes; final fine adjustments may be done in FreeCAD. Not every facet and cut is defined in spec files.
- Old part sums are approximately accurate (m62v2 ~1.1 kg real).
- 2026-10-03: Goal: a do-it-all design framework for new-generation drone designs: quadcopter, plane, quadplane, rocket boosted, any mix. JSBSim simulates all of it. Next propulsion type: rocket motors, with staging (OpenRocket logic).
- 2026-10-03: Flight controller scripts: Python for now (later ArduPilot/PX4/Arduino-level code). One `Controller` class defines the API; scripts in `aircraft/<project>/controllers/`; setup()/loop() at a chosen rate, lockstep simulated time; hardware units as an Arduino gets them (SPI IMU counts, Pa, NAV-PVT GPS, PWM us). Same flight log and plots as an INAV run.

## 1. Aircraft definition (saved file, schema "airframe/2")

Implemented in `libs/airframe.py` (`python -m libs.airframe aircraft/m62v2/aircraft.json`). Saved as `aircraft/<project>/aircraft.json` (`libs/project.py`). SI units (m, kg); angles in degrees; x from the nose tip. Library parts are referenced as `<category>:<name>` from `data/parts.json`. Loading fails on any missing required field, unknown field, bad value or unresolved reference.

```
schema              "airframe/2"
powered, vtol, tailsitter   true/false; tailsitter needs vtol, vtol needs powered. They drive the JSBSim/INAV export:
                    tailsitter = hover + forward mixer profiles; not vtol = AIRPLANE profile, motors on throttle only;
                    other VTOL types and unpowered airframes are not exported. Hover numbers only for vtol.
name, notes
preset              optional: a data/specs.json preset name (M62, M75). Loading it fills section, joints (not coupler_mass_kg)
                    and the reinforcement pattern (not from_x_m/to_x_m); values are always written out in full, and the
                    fuselage is checked against the preset's limits
operating_point     aoa_deg, airspeed_m_s, altitude_m
print               material (parts.json material), printer_bed_m [x, y, z],
                    bulk_density_kg_m3 (printed mass per solid volume; required only if a surface has no mass_kg or a solid
                    section no empty_mass_kg; 318.3 = single-wall 5 % infill PLA, from a weighed 77 g NACA2408 105 x 400 mm segment)
fuselage
  length_m
  section           width_m, height_m, shape (superellipse exponent; 2 = circle), wall_radial_m
  nose, tail        profile (pointed_shapes name), length_m, params (exactly the profile's keys)
  joints            type radial_screws, count, screw, insertion_depth_m, coupler_mass_kg (optional, one per cut)
  reinforcement     optional longitudinal rods: type cf_rod, material (parts.json), diameter_m, radial_position_m (rod centre
                    from the axis), polar_n (all rods), polar_offset_angle (deg from the horizontal, one position per quadrant,
                    mirrored left/right and top/bottom), polar_split (0 = one rod per position, 2 = a pair at
                    polar_offset_angle +/- polar_split_angle), polar_split_angle (deg), from_x_m, to_x_m;
                    rods inside the outer surface and strictly inside their quadrant
  channels[]        name, diameter_m, from_x_m, to_x_m, angle_deg
surfaces[]          name, x_root_le_m (must be on the body; chords may overhang the tail),
                    layout (H X + 1 3), fin_position (top/bottom), dihedral_deg, root_offset (-1..1),
                    root (body: span from the body surface at the root LE; axis: span from the fuselage axis,
                    panels meet in the centre, e.g. delta tail fins), span_m (per panel, along the panel),
                    chord_root_m, chord_tip_m, sweep_le_deg, incidence_deg, airfoil (AeroSandbox name),
                    mass_kg (whole set; optional, absent = computed)
                    control (optional, absent = fixed; applies to every panel of the set):
                      kind split: chord_start (hinge, chord fraction from LE, 0..1 exclusive),
                        span_start, span_end (panel span fractions from the root, end > start)
                      kind all_moving: pivot (chord fraction from LE)
                      roles [pitch yaw roll flaps] (reference only), deflection_min_deg, deflection_max_deg
                      (positive = TE away from the panel normal, down on a wing), rate_deg_s (placeholder),
                      servos (S1..S12, one per panel in panel order, may be shared)
                    control surfaces are numbered C1.. front to back, then clockwise from the top seen from behind
propulsion          required when powered, absent when not
  motor             manufacturer, name (motorlib, libs/motorlib/data/motordatav2.csv; needs a stator size; mass from it)
  prop              motorlib propeller name (libs/motorlib/data/propellers.csv)
  battery, esc      names of placed parts (battery:<name>, esc:<name>); wires run battery -> ESC -> each motor along
                    the axes, gauges and plugs from data/specs.json wire_paths; their resistance enters the motor model
  mounts[]          on (nose | tail | surface); for surface: surface, span_position_m (from panel root),
                    x_offset_m (from root LE); direction (tractor | pusher);
                    spin, seen from where the thrust points: surface out | in (2+ motors alternating cw/ccw round the
                    axis; out = INAV props out, X: front-right and rear-left cw, multirotor front = aircraft belly),
                    nose/tail cw | ccw;
                    thrust_axis_tilt_deg: thrust line tilted up (+z) from the body axis, e.g. to pass through the CG;
                    pod optional: diameter_m, flat_length_m, curve_length_m, profile (parameter-free)
volumes[]           internal space: name, x_m (centre), y_m and z_m (default 0), shape:
                      box: size_m [x, y, z]
                      cylinder: diameter_m, length_m, axis x|y|z
                      bore: length_m (fills the internal section)
                      point
                    mass_kg optional: its own mass, number or [min, max] (> 0), e.g. ballast; absent = space only
parts[]             library components: name, part (<category>:<name> from data/parts.json, category battery | esc),
                    volume (the one it sits in), x_offset_m, y_offset_m, z_offset_m (from the volume centre,
                    default 0), mass_kg optional (number >= 0 or [min, max]; absent = library mass, required when the
                    library has none; 0 = counted elsewhere, e.g. a weighed section); box = library dimensions
                    (x = l, y = w, z = h), a point without them; must fit its volume, must not overlap other parts
sections
  cuts_m[]          cut stations (ascending, inside the fuselage); section i spans cut i-1 .. cut i
  per_section[]     one per section: solid (true: printed solid less the volumes inside it, x print.bulk_density_kg_m3;
                    false: hollow shell wall), empty_mass_kg (optional: weighed printed mass), complete_mass_kg (optional:
                    weighed printed mass + everything attached in the section, which it replaces: points by position,
                    props with their motor, motors with their mount, surface sets at their root LE)
```

Body-rooted panels keep the true root edge on the body (follows nose/tail cones) and use an equivalent aero root at the mean body radius under the root chord. Axis-rooted panels start on the axis; Barrowman uses their exposed part.

Converted examples: `aircraft/m62v2/` and `aircraft/quadlancer62/` reproduce the old totals (1.1408 / 1.3732 kg, plus props 1.1608 / 1.4292 kg). The old modules are sections (cuts at the module boundaries, module mass as the section's weighed empty mass), couplers 21.36 g per joint, the battery is the only volume. m62v2 tail fins are axis-rooted, 55 mm span (delta: straight trailing edge, joined at the centre).

## 2. Libraries (data/)

- `parts.json`: batteries (mass required, > 0), ESCs, FCs, cameras, payloads, props (mass), materials.
- Motors and props: motorlib (`libs/motorlib`, copied from `dronebuildcalc`), used as published.
- `specs.json`: size presets (`presets`: section, joints, reinforcement pattern in the aircraft's own form, limits checked
  against the aircraft, CAD data), fasteners, openings, printer bed, wiring. Presets are created and saved from the
  GUI's Fuselage tab.
- Airfoils: one source (AeroSandbox names), `data/airfoils.json` only for imported coordinates.
- `modules.json`: retired 2026-10-01 after converting the two example aircraft.

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

- Body, hollow section: wall shell volume between cuts (cross-section perimeter x radial wall) x material density.
- Body, solid section: outer volume between cuts less the volumes inside it (by the share of each volume's x extent) x
  print bulk density.
- Couplers: weighed coupler mass per cut. Rods: rod volume x rod material density, one mass point per section crossed.
- A section's complete mass replaces it and everything attached in it with one weighed point.
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
8. JSBSim export. Milestone 1 (hover in INAV SITL) done 2026-10-01; forward flight and transition next (`TODO.md` F).
