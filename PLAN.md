# Plan: plane-first airframe model

## Design premise: plane first, then cut into sections

The original workflow was Lego-style: print tube sections, couplers, nosecones and endcaps in FreeCAD, then assemble aircraft from those parts. That is backwards. The intended way is to design the aircraft as a whole (tube, nose, tail, wings, motors, battery, ballast) and then cut it into printable sections.

The aircraft definition is now plane-first (schema `airframe/2`, section 1): the GUI, aero, drawing and CAD export read it. `aircraft/m62v2` and `aircraft/quadlancer62` are real aircraft, originally built the Lego way and re-entered plane-first. `aircraft/m62v1` is m62v2 without the horizontal main-wing section: a pure dart. The old module-stack model is retired.

## Decisions (owner, 2026-10-01)

- Design the aircraft as a whole, then cut it into printable sections. The module library is not the core.
- Sectioning: tool proposes cuts automatically; user can move or lock cut lines.
- Printed structure mass: computed from geometry and print settings; any section or part can be overridden with a weighed value.
- `m62v2.json` and `quadlancer62.json` (real built aircraft) are re-entered plane-first as validation cases.
- Component data differences (e.g. battery weight) are data errors, not design variants.
- Keep it parametric and define everything. Tube standards (tube62, tube75) are presets that fill parameters, not constraints.
- Fuselage cross-section is not limited to circular (rectangular, octagonal, rounded corners later); circular first.
- Reserved internal space as simple envelopes and channels. No cooling flow or insertion-path simulation.
- Current joint: 8 radial screws into a coupler, plus longitudinal spar rods for stiffness (tube62 standard in `data/presets/tube62.json`).
- The builder's definition is the master. CAD (CadQuery, optionally FreeCAD) is a downstream consumer, not a second aircraft model.
- Generating printable section bodies (shells, bulkheads, mounts, channel cuts) is later CAD work.
- A motor-wing section (tube, X-wing arms, motor mounts) is printed in one piece (monocoque). Joining long wings: later.
- Printed surface mass: solid airfoil volume x effective bulk density (owner's calibration from a weighed printed segment).
- Main design driver after the overall shape: user-defined internal volumes (like battery and ballast) with a size (e.g. 20x20x20 mm, or internal diameter x 50 mm), a mass (fixed, e.g. 20 g, or a range, e.g. 10-30 g), dragged to where they are needed. They drive and constrain everything else.
- CadQuery generates the bigger sections and volumes; final fine adjustments may be done in FreeCAD. Not every facet and cut is defined in spec files.
- Old part sums are approximately accurate (m62v2 ~1.1 kg real).
- 2026-10-03: Goal: a do-it-all design framework for new-generation drone designs: quadcopter, plane, quadplane, rocket boosted, any mix. JSBSim simulates all of it. Next propulsion type: rocket motors, with staging (OpenRocket logic).
- 2026-10-04: Right pane split into Design (the airframe) and Build (parts, wiring, electronics), towards a full design export per aircraft. Wiring is per aircraft (Build > Wiring: each auto-routed run's gauge and connectors); data/specs.json was an early sketch, not authoritative. Parts need not sit in volumes (volumes are a CAD pointer).
- 2026-10-12: Rocket motors are propulsion beside electric: a mount's motor type is electric or rocket; Propulsion has Electric and Rocket sub-tabs; a rocket motor is loaded into a chosen rocket mount (several mounts, e.g. staging later). Solid hobby motors from a RASP database first; liquid fuel later. MCU scripts ignite (controller.ignite()); the launch run flies to ground impact.
- 2026-10-12: Scenarios may set default start conditions (START, e.g. launch: rail and launcher pitch); the SIM toolbar pre-selects them, the CLI and toolbar override them.
- 2026-10-10: One flight framework: `run(flight)`, scripts in `scenarios/<controller>/`, commands imported from `controllers.inav` or `controllers.mcu`. Shared scheduler, simulation state, conditions and raw sensors. `experiment` is an example scenario. MCU scripts read sensors and write PWM; no backend autopilot or mixer.

## 1. Aircraft definition (saved file, schema "airframe/2")

Implemented in `libs/airframe.py` (`python -m libs.airframe aircraft/m62v2/aircraft.json`). Saved as `aircraft/<project>/aircraft.json` (`libs/project.py`). SI units (m, kg); angles in degrees; x from the nose tip. Library parts are referenced as `<category>:<name>` from `data/parts/<category>.json`. Loading fails on any missing required field, unknown field, bad value or unresolved reference.

```
schema              "airframe/2"
powered, vtol, tailsitter   true/false; tailsitter needs vtol, vtol needs powered and an electric mount; powered = propulsion
                    present (electric and/or rocket). They drive the JSBSim/INAV export:
                    tailsitter = hover + forward mixer profiles; not vtol = AIRPLANE profile, motors on throttle only;
                    other VTOL types are not exported. Hover numbers only for vtol.
name, notes
preset              optional: a data/presets/ preset name (tube62, tube75). Loading it fills section, joints (not coupler_mass_kg)
                    and the reinforcement pattern (not from_x_m/to_x_m); values are always written out in full, and the
                    fuselage is checked against the preset's limits; the preset's spars are what a surface set starts with
operating_point     aoa_deg, airspeed_m_s, altitude_m
print               material (data/parts/material.json), printer_bed_m [x, y, z],
                    bulk_density_kg_m3 (printed mass per solid volume; required only if a surface has no mass_kg or a solid
                    section no empty_mass_kg; 318.3 = single-wall 5 % infill PLA, from a weighed 77 g NACA2408 105 x 400 mm segment)
fuselage
  length_m
  section           width_m, height_m, shape (superellipse exponent; 2 = circle), wall_radial_m
  nose, tail        profile (pointed_shapes name), length_m, params (exactly the profile's keys); tail length_m 0 = flat base
  loft_stations[]   independent shape guides: x_m (ascending, unique, within 0..length_m), optional width_m, height_m,
                    shape; omitted profile keys inherit fuselage.section. Empty = constant default cross-section.
                    Cubic easing joins guides; nose/tail profiles taper the loft. Wall thickness stays global.
  reinforcement     optional longitudinal rods: type cf_rod, material (data/parts/material.json), diameter_m, radial_position_m (rod centre
                    from the axis), polar_n (all rods), polar_offset_angle (deg from the horizontal, one position per quadrant,
                    mirrored left/right and top/bottom), polar_split (0 = one rod per position, 2 = a pair at
                    polar_offset_angle +/- polar_split_angle), polar_split_angle (deg), from_x_m, to_x_m;
                    rods inside the outer surface and strictly inside their quadrant
  channels[]        name, diameter_m, from_x_m, to_x_m, angle_deg
surfaces[]          name, x_root_le_m (must be on the body; chords may overhang the tail),
                    layout (H X + 1 3), fin_position (top/bottom), dihedral_deg, vertical_offset (-1..1),
                    root (body: span from the body surface at the root LE; axis: span from the fuselage axis,
                    panels meet in the centre, e.g. delta tail fins), span_m (per panel, along the panel),
                    chord_root_m, chord_tip_m, sweep_le_deg, incidence_deg, incidence_mode (symmetric: XZ-mirror
                    panels pitched alike, lift like AoA; cant: every panel turned the same way about the body axis,
                    roll / spin; + = anticlockwise seen from behind), airfoil (AeroSandbox name),
                    mass_kg (whole set; optional, absent = computed)
                    sections: cuts_m (ascending span distances from the true panel root, strictly inside span_m);
                      empty = attached fixed panel; first segment stays on the fuselage, later fixed segments export separately;
                      cuts apply to every panel of the set; control surfaces remain unsplit, spar holes remain aligned
                    spars (optional): material, diameter_m, count (per panel), spacing_m, x_offset_m (first spar aft of
                      the root LE along the root chord), y_offset_m (off the chord line through the thickness, + towards
                      the panel normal: up on a wing); straight rods along each panel's span in the chord plane, from
                      closest to the fuselage axis out to the tip; mass with the surface set; holes in the printed parts.
                      Where the first panel's spar line runs on in line into the opposite panel, that pair runs through tip
                      to tip; every other spar runs from its tip inward and stops where it meets another: fit_clearance_m
                      clear of a through spar, or of the plane midway between itself and another stopping spar (dihedral
                      and other sets not in line have no through spar). Holes: inside a carry-through they join the holes
                      they meet (mitred, no sliver); without one each ends a fit clearance short of its rod, never meeting.
                      carry_through (optional): the printed web inside the tube the spars run through, one arm per panel:
                        stem_width_m (across the panel plane), length_m (along x, centred between the spars) out to
                        flare_radius_m from where the arms meet (the axis for a radial set, the middle plane for a dihedral
                        pair; arms mitred there), flaring to root_length_m at the inner wall, wall_fillet_m (front view
                        fillet into the wall: recorded for post-processing, not built; the arms meet the wall square); mass = side profile x stem width x print bulk density,
                        none when its section is weighed
                    root_base (optional, recorded only, not built: needs blending): length_m, width_m, height_m,
                      forward_offset_m, leading_cut_m, trailing_cut_m
                    control (optional, absent = fixed; applies to every panel of the set):
                      kind split: chord_start (hinge, chord fraction from LE, 0..1 exclusive),
                        span_start, span_end (panel span fractions from the root, end > start)
                      kind all_moving: pivot (chord fraction from LE), span_start (optional, 0.0: where the moving
                        surface starts; > 0 leaves a fixed base inboard of it to blend/fillet into the body)
                      align_hinge_vertically (optional, false): straight hinge across span, anchored at its inboard position
                      gap_m (optional, 0.0 = none): material removed between the fixed and the moving surface: the
                        moving surface's hinge edge and its span ends (where fixed material borders them) are inset
                        by it, and the pivot rod, when there is one, sits centred in it
                      rod_diameter_m (optional, none = no rod): the pivot rod the surface turns on, outer diameter
                      rod_x_offset_m, rod_y_offset_m (optional, 0.0): the pivot rod line (and so the rotation axis)
                        shifted along the chord direction (+ aft) and towards the panel normal (+ up on a wing)
                      roles [pitch yaw roll flaps] (reference only), deflection_min_deg, deflection_max_deg
                      (positive = TE away from the panel normal, down on a wing), rate_deg_s (placeholder),
                      servos (S1..S12, one per panel in panel order, may be shared: a panel opposite to the others on its servo is exported reversed, a Y-cable reverser)
                    control surfaces are numbered C1.. front to back, then clockwise from the top seen from behind
propulsion          required when powered, absent when not
  mounts[]          (below) type electric | rocket
  electric          required when a mount is electric (may exist without one): motor, prop, battery, esc, wiring:
  motor             manufacturer, name (motorlib, libs/motorlib/data/motordatav2.csv; needs a stator size; mass from it)
  prop              motorlib propeller name (libs/motorlib/data/propellers.csv)
  battery           the pack, placed like a part (volume with x/y/z_offset_m, or its own x_m, y_m, z_m); model:
                      commercial: part (data/parts/battery.json: type, s, p, mah, mass, dimensions, connector, optional
                        ir_mohm_per_cell, lead_awg + lead_length_mm), mass_kg (optional, weighed)
                      custom_liion: s, p, cell (libs/motorlib/data/cells.csv, with a CELL_FORMATS format in its name, e.g.
                        21700), connector, cell_axis (x | y | z: the cells' length along it), stacking [x, y, z] (cells along
                        each axis; x * y * z = s * p), lead_awg, lead_length_m; size per axis = count * (cell size + 2 mm),
                        mass, capacity, resistance and voltage from the cell
                    its connector is the first plug of the battery run; its leads add to the shared resistance
  esc               name of a placed esc part; wires run battery -> ESC along the axes, ESC -> a surface motor inside its panel (along x to the root
                    mid-chord, out along the mid-chord to the motor's span station), ESC -> a nose/tail motor along the axes
  wiring            battery (the battery -> ESC run), motors[] (ESC -> each electric motor, one per motor in motor order); a run:
                    gauge_awg (motorlib wire gauge), conductors (wires in the cable), plugs[] (data/parts/connector.json
                    names along the run, in order); each motor's run and the shared battery run set its resistance in
                    the motor model (one model per motor)
  rocket            optional: motors[]: mount (mount number, 1-based, a rocket mount; at most one motor per mount; an empty
                    rocket mount fires nothing), motor (data/rocket_motors/<name>.eng); a surface mount loads the motor at
                    every panel
  mounts[]          type electric | rocket. Rocket: on (tail | surface), surface, span_position_m, thrust_axis_tilt_deg,
                    x_offset_m = the loaded motor's aft (nozzle) end from the tail face / surface root LE, positive aft
                    (overhang allowed; the casing must fit the inner wall inside the body); no direction, spin, pod, holes.
                    Electric: on (nose | tail | surface); for surface: surface, span_position_m (from panel root),
                    x_offset_m (all mounts; from surface root LE or nose/tail face, positive aft; moves motor and pod);
                    direction (tractor | pusher);
                    spin, seen from where the thrust points: surface out | in (2+ motors alternating cw/ccw round the
                    axis; out = INAV props out, X: front-right and rear-left cw, multirotor front = aircraft belly),
                    nose/tail cw | ccw;
                    thrust_axis_tilt_deg: thrust line tilted up (+z) from the body axis, e.g. to pass through the CG;
                    clearance_m (electric): added to the motor bell diameter for the mount clearance cut: everything of
                      the carrying wing and pod (or fuselage section) on the motor's side of the mount face, within that
                      diameter, is cut away (a pod can sit recessed into the wing, leaving a channel)
                    pod optional: diameter_m, flat_length_m, curve_length_m, profile (parameter-free)
                    holes optional: type round | slot, count (int), angle_deg (first hole; 0 = +y, positive towards +z),
                      screw (a data/parts/screw.json size, M1.6..M5: its clearance bore is cut), round:
                      pattern_diameter_m; slot: inner_diameter_m, outer_diameter_m (radial slot ends);
                      center_diameter_m + center_depth_m (shaft relief); head bore optional, head_diameter_m and
                      head_depth_m together (heads vary; the editor starts it at the screw table's head diameter; the
                      seat inward from the mounting face: the head bore runs from there on through the material in
                      front, the way the screw goes in). A surface motor's bores go through its whole wing and pod (fused first);
                      a nose or tail motor's through the fuselage section carrying it
volumes[]           internal space: name, x_m (centre), y_m and z_m (default 0), shape:
                      box: size_m [x, y, z]
                      rounded_box: size_m [x, y, z], axis x|y|z, corner_radius_m (the edges along axis rounded; fit and
                        overlap checked as its box)
                      cylinder: diameter_m, length_m, axis x|y|z
                      bore: length_m (fills the internal section)
                      point
                    mass_kg optional: its own mass, number or [min, max] (> 0), e.g. ballast; absent = space only
parts[]             library components: name, part (<category>:<name>; category battery (data/parts/battery.json) or a role:
                    fc | esc | camera | companion | vtx | receiver | gps | servo (data/parts/<role>/<name>.json)),
                    volume (optional, the one it sits in) with x_offset_m, y_offset_m, z_offset_m (from the volume
                    centre, default 0), or without a volume its own x_m, y_m, z_m (y, z default 0), mass_kg optional (number >= 0 or [min, max]; absent = library mass, required when the
                    library has none; 0 = counted elsewhere, e.g. a weighed section); box = library dimensions
                    (x = l, y = w, z = h), a point without them; must fit its volume, must not overlap other parts;
                    yaw_deg, pitch_deg, roll_deg optional (default 0): its turn, about z, y, x, roll first, body axes;
                    the first fc part carries the script controller's IMU (its position and axes)
access[]            saved named holes and hatches (fuselage cuts, retained hatch parts, mass and views); required, may be empty.
                    name (unique in access), kind hole | hatch, profile slot | rounded_rectangle, width_m, length_m
                    (total opening extents; equal slot extents = a circle, a slot's length is its longest dimension),
                    corner_radius_m (rounded_rectangle only; positive, at most half the smaller extent), gap_m
                    (hatch only; >= 0, clearance per edge, must leave material inside the profile),
                    x_m, y_m, z_m (profile centre; y, z optional, default 0), roll_deg, pitch_deg, yaw_deg (each
                    optional, default 0; applied roll first about x, then pitch about y, then yaw about z, as parts),
                    extent depth | wall | all with depth_m (depth only; positive, along the local +Z; wall = into the
                    first wall, all = through the whole body). The profile lies in its local XY plane: local X is its
                    length, local Y its width, local +Z the cut direction. radial (optional): count (integer >= 1,
                    includes the seed), axis x | y | z and x_m, y_m, z_m (a point on the rotation axis); the seed and
                    every copy are turned 360 * k / count deg about that axis line, position and local axes together.
sections
  joints            type radial_screws, count (screws per section end), screw, insertion_depth_m (coupler or spigot into
                    each female end's socket), coupler_wall_m, fit_clearance_m (radial play of coupler, spigot, rods and
                    spars), socket_wall_m (skin left over the coupler at a female end: socket = outer surface less it; at
                    most the section wall), hole_edge_dist_m (from the cut), hole_diameter_m, hole_angle_offset_deg (first
                    hole from +y, the rest evenly round), coupler_mass_kg (optional, one per cut). Sections butt at the cut.
                    Screw axes follow the cut profile's wall normals; mating holes stay coaxial and break through the lofted skin.
                    An integral male (front_male / rear_male) runs insertion_depth_m back into its section, chamfered
                    inside from the fuselage ID at that far end to the coupler ID hole_edge_dist_m short of the fastener
                    hole centre (at the cut). A female end whose body tapers below its socket within the insertion depth
                    is a layout problem (the socket breaks through).
  cuts_m[]          cut stations (ascending, inside the fuselage); section i spans cut i-1 .. cut i; may be empty
                    (one uncut section: the whole fuselage, no couplers)
  per_section[]     one per section: solid (true: printed solid less the volumes inside it, x print.bulk_density_kg_m3;
                    fixed_length (optional bool, default false: preserve section length during automatic editor resizing),
                    false: hollow shell wall), empty_mass_kg (optional: weighed printed mass), complete_mass_kg (optional:
                    weighed printed mass + everything attached in the section, which it replaces: points by position,
                    props with their motor, motors with their mount, surface sets where they meet the body: their exposed root LE)
  cut_joints[]      one per cut: coupler (separate coupler into both sections; sections.joints.coupler_mass_kg at the
                    cut), front_male or rear_male (the section ahead of / behind the cut carries the male end, no coupler),
                    or none (the sections butt with no coupler or spigot; cut them in CAD as needed)
```

Body-rooted panels keep the true root edge on the body (follows nose/tail cones) and use an equivalent aero root at the mean body radius under the root chord. Axis-rooted panels start on the axis; Barrowman uses their exposed part.

Converted examples: `aircraft/m62v2/` and `aircraft/quadlancer62/` reproduce the old totals (1.1408 / 1.3732 kg, plus props 1.1608 / 1.4292 kg). The old modules are sections (cuts at the module boundaries, module mass as the section's weighed empty mass), couplers 21.36 g per joint, the battery is the only volume. m62v2 tail fins are axis-rooted, 55 mm span (delta: straight trailing edge, joined at the centre).

## 2. Libraries (data/)

- `data/parts/`, tables (one file each): batteries (`battery.json`; mass required, > 0; connector: the battery's own plug, first on the battery run), props (mass; linear between listed diameters, outside them the nearest scaled by diameter^3), materials, connectors (contact_resistance_ohm), mounting patterns (`mounting.json`: holes_mm, screw, outline_mm).
- `data/parts/<role>/<name>.json`, one file per product, roles fc, esc, camera, companion, vtx, receiver, gps, servo: generic (true = typical stand-in), mass (g), dimensions l, w, h (mm), mount (mounting.json pattern or null), wiring_origin_mm [x, y, z] (from the box centre, part axes), notes; esc also curr, min_s, max_s. Checked when placed.
- Motors and props: motorlib (`libs/motorlib`, copied from `dronebuildcalc`), used as published.
- `rocket_motors/<Manufacturer>_<Designation>.eng`: solid rocket motors, one RASP file each (thrustcurve.org's format, read
  by OpenRocket and RockSim), used as published (`libs/rocketmotors.py`). First entry: AeroTech F42T.
- `presets/<name>.json`: size presets (section, joints, reinforcement pattern, spars in the aircraft's own form, limits checked
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

Consumers, each a thin reader: AeroBuildup and VLM (`analysis.py`), Barrowman, orthographic drawing (`graphics.py`), CadQuery export (`cad.py`), and later the JSBSim exporter.

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
5. Auto sectioning and coupler generation; printable section STEP/STL. Section and coupler STEP/STL done 2026-10-05 (`libs/cad.py` `export_sections`); auto sectioning not built.
6. GUI rebuilt around the definition (edit plane parameters, move/lock cuts).
7. Aero refinements from `TODO.md` D.
8. JSBSim export. Milestone 1 (hover in INAV SITL) done 2026-10-01; forward flight and transition next (`TODO.md` F).
