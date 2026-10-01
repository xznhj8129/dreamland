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
        - Saved configs embed full module blocks under library names that drifted from the library (V: batt-1 mass 0.065 vs 0.089, endcap 0.015/len 0.02 vs 0.027/0.07, saggerwing-7in 0.025 vs 0.059, quad-wingmount-v3 len 0.11 vs 0.15). guibuilder.py normalize_config_from_templates expands references; headless cannot run reference configs (KeyError 'x'). One normalizer for GUI + headless; references + explicit overrides.
        - guibuilder.py apply_template_values fills missing keys from templates and drops unknown keys (V: saggerwing-7in-fwd gets motor_y_pos 0.1 / pod_* 0.028 invented; avionics, diam, model, motor_n dropped). Fail on missing/unknown keys.
        - Operating point (AoA, airspeed, altitude) not saved in config (V: m62v2 saved at 5 deg/20 m/s "stable" reloads at 0/10 "unstable").
        - semispan measured from centerline on end modules, from tube surface on wingmounts (V: tailfin root (0,0), main wing root 0.031). Inflates Barrowman tailfin CNa (6.32 vs ~1.8) and reference area. One definition: exposed span from local body radius.
        - Units/semantics: motor_x_pos is metres on wings, fraction on end caps (ignored there, analysis.py end-motor at mid-module); cg is fraction of length on modules, of MAC on surfaces; OD and battery dims in mm, rest in m; aoa_ang is incidence; wings_ang is dihedral/X angle.
        - modules.json motor_dir: 4 wings use ints (+/-1), rest "front"/"back"; -1 becomes pusher (V). motor_cap has no motor_dir (KeyError). Enum + validation.
        - Fields never read: config vtol, powered, material_density_g_cm3, thrust_axis_tilt_deg; module esc, avionics, model, diam, motor_n; part pitch/yaw/roll; surface ctl_surface, all_moving; parts.json esc section; module-level payload_mass/ballast_mass (UI shows them, edits do nothing, V); wing_phase/phase_deg/dihedral_sign; legacy wings_n path.
        - Unknown wing layout silently becomes "H"/2 wings (layout_wing_count, derive_wings_layout).
        - Coupler hardcoded "joint-ring-c" (M62) at every joint regardless of standard_profile.
        - Computed surface mass is airfoil volume x infill only; printed perimeter walls not modelled (m62v2 main wing: 34 g computed vs 50 g given).
        - spar_rods has no rod length/start (assumes full fuselage length); real m62v2/quadlancet62 use 4 x 2.5 mm x 400 mm carbon rods; no carbon material entry.
        - Save writes vlm_results (not in AGENTS.md contract) and a saved analysis with old schema that is never read.
        - Two names: assembly_name ("default") vs config.name.

- B. Shared geometry model (replaces 3 traversals: graphics.py, build_asb_airplane, cad.py):
        - Nose/tail role of end modules: CAD/ASB use first/second end, drawing uses x_start == 0 (V: no-nose config pinches fuselage mid-length in CAD/ASB; body_x_start > body_x_end). Assign role once; raise on invalid stacks.
        - Incidence on unpaired vertical fins opposite in CAD vs ASB (V: bottom fin chord y -0.086 CAD, +0.087 ASB); drawing applies c*sin(aoa)*cos(roll) to all panels. Emit explicit chord/normal unit vectors per panel.
        - Fused CAD pushes centerline-rooted fins 0.75R past the axis (cad.py root_extend).
        - End-cap motor at mid-module: bell pokes through cone, prop disc cuts cone (V: motor_cap).
        - pointed_shapes: tangent ogive L<R leaves open tip; blunted nose_radius >= R math error; bi_conic break_fraction 0 div-by-zero; pod shape params never passed (parabolic pod KeyError 'k_prime').
        - PROP_SHAFT_LEN_M defined twice; surface_force_rows matches airplane.wings by position.

- C. Physics (mass, propulsion):
        - Hover point below lowest measured throttle: simlib2.py inserts fake (0,0,0) and interpolates linearly (V: m62v2 needs 285 g/motor, lowest data 793 g at 40%; reports 14.4%, 4643 rpm, 89 W, 8.2 min; T~rpm^2 from 40% point gives 7743 rpm, 53 W, 13.7 min). is_extrapolated never set. Flag or extrapolate with stated law.
        - motordata.csv non-monotonic: Uangel A2807 GF7042X2 6S thrust 1948 g at 90%, 1836 g at 100%; max thrust taken from 100% row; thrust->throttle inversion ambiguous.
        - 4 LiPo batteries mass 0.0 in parts.json (GNB 5300/4000, Tattu 6000/4500), accepted silently (V: m62v2 with GNB-5300 total 0.839 kg, hover 26.9 min).
        - Multiple battery bays: mass per bay, Wh for one pack only (V).
        - Wing set mass in total but not in module_rows (V: 0.154 kg of 1.1408 missing).
        - No prop mass in data model.
        - CG x only; no y/z, no inertia tensor (needed for JSBSim).
        - pack_voltage_v is loaded voltage at lowest data point; LiPo Wh inline 3.7 V/cell; hover/full-throttle time use 100% Wh, no avionics load.
        - No check of total motor current vs battery current (m62v2: 160 A vs 154 A); propulsion warnings always empty.
        - parse_motor_name mangles '-' and spaces: "DarwinFPV 1504-3-3800", "iFlight XING 2RAW 2812-1150" never resolve (V). Case handling inconsistent between find_motor_group and TestedMotorData.
        - Motor mass from iloc[0] of matched rows (Brotherhobby F2008-1000 has 2 weights). Bell geometry uses stator dims.
        - Gravity 9.81 (simlib2.py, propulsion summary) vs 9.80665.

- D. Aero (fast = AeroBuildup, live; detailed = lifting-line wings + AeroBuildup bodies, on demand):
        - Fast model has no downwash between surfaces, no wing-body carry-over, and centres its finite-wing correction on 0 deg instead of the zero-lift angle (AeroSandbox TODO), over-crediting camber. m62v2 at 5 deg / 20 m/s: fast NP 0.365 m (+27.6 mm), detailed 0.3325 m (-5.0 mm); quadlancet62 fast +8 mm, detailed about 0. Owner: no control surfaces, FC always flying, jet-fighter-like stability (matches detailed).
        - Detailed model: body-rooted panels carried to the axis (carry-over includes camber lift); bodies from AeroBuildup; totals only (no per-surface split); valid to about +-30 deg (lifting line NaN at 45 deg); ~50-60 s.
        - aero_check.py reference is VLM carried through the body (inviscid; under-reads low-Re camber at 0 deg: m62v2 main wing 0.87 N vs lifting line 1.29 N). Use the carried-through lifting line as the reference instead.
        - Stability verdict depends on operating point; low-Re (45-65k) NeuralFoil section curves are bumpy; local NP per AoA jitters +/- 2-3 cm.
        - Recompute ~0.9 s (fast, 94 AoA points). 2.5 deg sweep step: ~0.35 s.
        - Imported DAT airfoils cannot be assigned to surfaces (surfaces take AeroSandbox names only).
        - VSPAERO 3.43 (/usr/local/bin/vspaero) diverges (CL ~ -300 at 0 deg) on AeroSandbox export_OpenVSP_vspscript geometry.

- E. GUI:
        - Part Editor (libs/library_editors.py): rename to an existing name overwrites on disk; category change keeps old fields; unsaved edits discarded on selection or view change; own copies of length-field list, palette, ordered_keys, format_field_value, field_label, parsing; NEW_PART_LIBRARY_DEFAULTS hardcoded.

- F. JSBSim data not in model: inertia tensor and per-component y/z; empty vs payload/battery split; control surfaces (hinge, chord fraction, limits, mixing); per-motor thrust axis, spin direction, prop Ct/Cp vs J, rotor inertia, voltage sag; CL/CD/Cm vs alpha to +/-90 deg with body; CYb, Clb, Cnb, Cmq, Clp, Cnr, control derivatives; explicit reference area/span/chord; ground contact points.

- Housekeeping:
        - __pycache__/*.pyc committed despite .gitignore.
        - No requirements file. Deps (aerosandbox, neuralfoil, cadquery) only present in conda env "drone".
        - AGENTS.md lists cad_example.py and library_editors_example.py, which do not exist; AGENTS.md caveats still describe linear AoA pass; docs/ui/configuration_view.md says red marker is Static CP; main_window.md calls CAD a placeholder.

- Bring over from cadbuildsystem (predecessor):
        - cadparams/ FreeCAD spreadsheets (AirframeParams, WingVars, ParamAirfoil, CanardsAirfoil). Fix Fuselage_Thickness expr: OD - ID gives 2x wall; libs/cad.py uses (OD - ID) / 2.
        - parttojson.FCMacro (FreeCAD tree to JSON). Hidden objects are silently dropped; link_document and link_path hold each other's values.
        - README class hierarchy (Complex / System / Subsystem / Module / Submodule / Component / Subcomponent) and 2R04B1 assembly tree.
