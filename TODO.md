- Motors need a template as physical objects with the following physical modeling characteristics:
        mount_height (the part that is bolted onto a drone, this is the motor's x=0, height adds to it)
        mount_diameter 
        base_diameter (the stator cylinder base from the mount end to the start of the motor rotor bell, ie the exposed part) (default 20mm)
        base_height (default 5mm)
        rotor_diameter (motor bell, this is often provided, default 28mm)
        rotot_height (as above, default 8mm)
        shaft_diameter (default 5mm)
        shaft_height (starting at bell) (default 16mm)
        mass (g)
        Everything else is relevant more to propulsion, not display



- Parameters.Assembly box: 
        - remove "thrust_axis_tilt_deg", i think this is FC configuration, not relevant here
        - battery, motors and props are handled in the Propulsion tab, so no need for them


- In Parameters, we add a performance box.
        
- Motor pods need the same shape constructor as ends for the curved section

- Review 2026-10-01 (4 parallel reviews; V = verified by running). Order and plane-first redesign: PLAN.md.

- A. Data model (one source of truth):
        - Surface estimate = solid airfoil volume x print.surface_bulk_density_kg_m3 (318.3: weighed 77 g NACA2408 105 x 400 mm segment, single wall 5 % infill; build_calculator test_area.py's 637 is against half the airfoil area). Estimate vs given: m62v2 main-wing 55.4 / 50 g, qrwing-7in 58.6 / 100 g, tailfin 13.5 / 4 g; quadlancer62 9in-puller 77.8 / 100 g; quadlancer62 new-wing 100 g was a placeholder, now estimated (302.8 g; total 1632 g).
        - Section shell estimate vs weighed: plain tube sections 1.03-1.21x; nose 2.9x; motor-wing section 2.6x (m62v2) and 3.5x (quadlancer62); m62v2 tail with fins 2.4x.
        - Open: m62v2 motor-wing section = section 4 (122 g, old quad-wingmount-v3 module) + qrwing-7in (4 x 25 g, old wing entries), counted as 222 g. Estimate for the one-piece print: tube 47.5 g + wings 58.6 g. Owner to confirm whether the printed piece weighs ~122 g or ~222 g.
        - spar_rods has no rod length/start (assumes full fuselage length); real m62v2/quadlancer62 use 4 x 2.5 mm x 400 mm carbon rods; no carbon material entry.

- B. Geometry (found on the module model; verify on the plane-first model):
        - Incidence on unpaired vertical fins opposite in CAD vs ASB (V: bottom fin chord y -0.086 CAD, +0.087 ASB); drawing applies c*sin(aoa)*cos(roll) to all panels. Emit explicit chord/normal unit vectors per panel.
        - End-cap motor at mid-module: bell pokes through cone, prop disc cuts cone (V: motor_cap).
        - pointed_shapes: tangent ogive L<R leaves open tip; blunted nose_radius >= R math error; bi_conic break_fraction 0 div-by-zero; pod shape params never passed (parabolic pod KeyError 'k_prime').

- C. Physics (mass, propulsion):
        - Hover point below lowest measured throttle: simlib2.py inserts fake (0,0,0) and interpolates linearly (V: m62v2 needs 285 g/motor, lowest data 793 g at 40%; reports 14.4%, 4643 rpm, 89 W, 8.2 min; T~rpm^2 from 40% point gives 7743 rpm, 53 W, 13.7 min). is_extrapolated never set. Flag or extrapolate with stated law.
        - motordata.csv non-monotonic: Uangel A2807 GF7042X2 6S thrust 1948 g at 90%, 1836 g at 100%; max thrust taken from 100% row; thrust->throttle inversion ambiguous.
        - 4 LiPo batteries mass 0.0 in parts.json (GNB 5300/4000, Tattu 6000/4500), accepted silently (V: m62v2 with GNB-5300 total 0.839 kg, hover 26.9 min).
        - Multiple battery bays: mass per bay, Wh for one pack only (V).
        - pack_voltage_v is loaded voltage at lowest data point; LiPo Wh inline 3.7 V/cell; hover/full-throttle time use 100% Wh, no avionics load.
        - No check of total motor current vs battery current (m62v2: 160 A vs 154 A); propulsion warnings always empty.
        - parse_motor_name mangles '-' and spaces: "DarwinFPV 1504-3-3800", "iFlight XING 2RAW 2812-1150" never resolve (V). Case handling inconsistent between find_motor_group and TestedMotorData.
        - Motor mass from iloc[0] of matched rows (Brotherhobby F2008-1000 has 2 weights). Bell geometry uses stator dims.
        - Gravity 9.81 (simlib2.py, propulsion summary) vs 9.80665.

- D. Aero (live: lifting-line wings + AeroBuildup bodies, libs/aero.py analyze_aero, ~1.5 s):
        - Lifting line = asb.LiftingLine's algorithm solved for all AoA at once (lifting_line_sweep). Matches asb.LiftingLine to 1e-14 N and the old on-demand detailed model (~50 s) to 1e-9 on m62v2 and quadlancer62. Replaced AeroBuildup wings (no downwash, no carry-over, camber over-credited): m62v2 at 5 deg / 20 m/s was +27.6 mm, now -5.0 mm.
        - Margins at 20 m/s: m62v2 -4.9 mm at 5 deg; quadlancer62 +6.2 / +7.7 / +7.9 / +5.5 / -6.5 mm at 2 / 4 / 5 / 6 / 8 deg, -6.0 mm at its saved 0 deg / 15 m/s. Owner: no control surfaces, FC always flying, jet-fighter-like stability.
        - Sweep -30..30 deg. Above ~35 deg the wind runs backwards along swept vertical fins in the strip model (negative Re, NaN polars, whole solve NaN). Operating point outside the sweep: summary says why, no aero.
        - m62v2 tail fins carry ~0 lift at 5 deg (0.007 N; AeroBuildup said 0.33 N): downwash from the main wing and X-wing. Wind-aligned wake moves NP only 1.5 mm.
        - NeuralFoil section polars (n_crit 9, Re ~100k) make Cm(AoA) wavy, so NP depends on the fit window (m62v2 at 5 deg: -5.0 mm over +-4 deg, -9 mm over +-2 deg). The sweep table's local NP column is meaningless near stall (dCL/dAoA -> 0).
        - aero_check.py still compares each surface set of the live model against that set alone in the lifting line; differences are now downwash between sets, not errors. Rework: check lifting_line_sweep against asb.LiftingLine; keep VLM columns for information.
        - Imported DAT airfoils cannot be assigned to surfaces (surfaces take AeroSandbox names only).
        - VSPAERO 3.43 (/usr/local/bin/vspaero) diverges (CL ~ -300 at 0 deg) on AeroSandbox export_OpenVSP_vspscript geometry.

- E. GUI:
        - Part Editor (libs/library_editors.py): rename to an existing name overwrites on disk; category change keeps old fields; unsaved edits discarded on selection or view change; own copies of length-field list, palette, ordered_keys, format_field_value, field_label, parsing; NEW_PART_LIBRARY_DEFAULTS hardcoded.

- F. JSBSim export, for INAV SITL via inav_development/inav-jsbsim-bench (MSP_SIMULATOR HITL; INAV does the mixing, the bench writes per-motor commands into JSBSim properties):
        - Have: inertia tensor about the CG (component shapes), per-motor prop spin (propulsion.prop_spin), motor positions and thrust axis, reference area/span/chord.
        - Model files after the bundled F450 (jsbsim/aircraft/F450: Metrics, Mass, Gear, Propulsion, Effectors, Aero); motor inputs as plain properties driving fcs/throttle-pos-norm[i], no internal FCS. Motor order = INAV mmix (quad X: 1 rear-right, 2 front-right, 3 rear-left, 4 front-left, in the MC hover frame).
        - Bench Plant loads bundled JSBSim models only: needs set_aircraft_path / set_engine_path / set_systems_path. No tailsitter vehicle or profile yet (INAV: AIRPLANE mixer profile + MC profile with tailsitter_orientation_offset = ON).
        - Body x = nose = thrust axis, so hover is theta 90 deg: bench rangefinder() returns -1 and truth() Euler angles hit gimbal lock there.
        - Ground contact: it stands on its tail; 3 contact points at the tail (F450 notes 4-point ground trim trouble); bench ground start uses do_simple_trim = 2.
        - Aero tables: lifting line only to +-30 deg; hover and transition need +-90..180 deg (AeroBuildup post-stall), beta, Cmq, Clp, Cnr.
        - Prop: no forward-flight data. Static Ct from motordata.csv (T / (rho n^2 D^4)); Ct(J) only from simlib2's Tyto formula (linear, zero at J ~ p/D); no Cp(J) (electrical power only). Motor block needs Kv, coil resistance, no-load current: no Flashobby A2807 entry in dronebuildcalc motorlib (DYS X2807-1300 60 mOhm / 1.0 A, EMAX ECOII-2807 1300KV 58 mOhm / 1.3 A). Rotor inertia: rod estimate (geometry.rotor_inertia) from parts.json prop mass.
        - No control surfaces in the model; control surface design tools to add.
        - No battery sag in the bench (JSBSim maxvolts only).

- Housekeeping:
        - No requirements file. Deps (aerosandbox, neuralfoil, cadquery) only present in conda env "drone".
        - AGENTS.md lists cad_example.py and library_editors_example.py, which do not exist, and the legacy files below as primary.
        - docs/ui predates the plane-first GUI (Builder V3 window, Controls tab, assembly cards).

- Bring over from cadbuildsystem (predecessor):
        - cadparams/ FreeCAD spreadsheets (AirframeParams, WingVars, ParamAirfoil, CanardsAirfoil). Fix Fuselage_Thickness expr: OD - ID gives 2x wall; libs/cad.py uses (OD - ID) / 2.
        - parttojson.FCMacro (FreeCAD tree to JSON). Hidden objects are silently dropped; link_document and link_path hold each other's values.
        - README class hierarchy (Complex / System / Subsystem / Module / Submodule / Component / Subcomponent) and 2R04B1 assembly tree.
