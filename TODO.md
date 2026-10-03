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

- Add direction arrow triangles to prop circle to show direction of spin (from front)

- Parameters.Assembly box: 
        - "thrust_axis_tilt_deg": per motor mount, thrust tilted up to align it with the CG when motors are not inline (done: mounts[].thrust_axis_tilt_deg)
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

- C. Physics (mass, propulsion):
        - Hover point below lowest measured throttle: simlib2.py inserts fake (0,0,0) and interpolates linearly (V: m62v2 needs 285 g/motor, lowest data 793 g at 40%; reports 14.4%, 4643 rpm, 89 W, 8.2 min; T~rpm^2 from 40% point gives 7743 rpm, 53 W, 13.7 min). is_extrapolated never set. Flag or extrapolate with stated law.
        - motordata.csv non-monotonic: Uangel A2807 GF7042X2 6S thrust 1948 g at 90%, 1836 g at 100%; max thrust taken from 100% row; thrust->throttle inversion ambiguous.
        - 4 LiPo batteries mass 0.0 in parts.json (GNB 5300/4000, Tattu 6000/4500), accepted silently (V: m62v2 with GNB-5300 total 0.839 kg, hover 26.9 min).
        - Multiple battery bays: mass per bay, Wh for one pack only (V).
        - pack_voltage_v is loaded voltage at lowest data point; LiPo Wh inline 3.7 V/cell; hover/full-throttle time use 100% Wh, no avionics load.
        - No check of total motor current vs battery current (m62v2: 160 A vs 154 A); propulsion warnings always empty.
        - Motor mass from iloc[0] of matched rows (Brotherhobby F2008-1000 has 2 weights). Bell geometry uses stator dims.
        - Gravity 9.81 (simlib2.py, propulsion summary) vs 9.80665.

- D. Aero (live: lifting-line wings + AeroBuildup bodies, libs/aero.py analyze_aero, ~1.5 s):
        - Lifting line = asb.LiftingLine's algorithm solved for all AoA at once (lifting_line_sweep). Matches asb.LiftingLine to 1e-14 N and the old on-demand detailed model (~50 s) to 1e-9 on m62v2 and quadlancer62. Replaced AeroBuildup wings (no downwash, no carry-over, camber over-credited): m62v2 at 5 deg / 20 m/s was +27.6 mm, now -5.0 mm.
        - Margins at 20 m/s: m62v2 -4.9 mm at 5 deg; quadlancer62 +6.2 / +7.7 / +7.9 / +5.5 / -6.5 mm at 2 / 4 / 5 / 6 / 8 deg, -6.0 mm at its saved 0 deg / 15 m/s. Owner: no control surfaces, FC always flying, jet-fighter-like stability.
        - Sweep -30..30 deg. Above ~35 deg the wind runs backwards along swept vertical fins in the strip model (negative Re, NaN polars, whole solve NaN). Operating point outside the sweep: summary says why, no aero.
        - m62v2 tail fins carry ~0 lift at 5 deg (0.007 N; AeroBuildup said 0.33 N): downwash from the main wing and X-wing. Wind-aligned wake moves NP only 1.5 mm.
        - NeuralFoil section polars (n_crit 9, Re ~100k) make Cm(AoA) wavy, so NP depends on the fit window (m62v2 at 5 deg: -5.0 mm over +-4 deg, -9 mm over +-2 deg). The sweep table's local NP column is meaningless near stall (dCL/dAoA -> 0).
        - aero_check.py checks lifting_line_sweep against asb.LiftingLine (max difference 2.5e-14 on all three aircraft); per-surface live vs alone vs VLM printed for information. m62v1 tail fins sit in the X-wing downwash (0.06 N live vs 0.26 N alone at 5 deg).
        - Imported DAT airfoils cannot be assigned to surfaces (surfaces take AeroSandbox names only).
        - VSPAERO 3.43 (/usr/local/bin/vspaero) diverges (CL ~ -300 at 0 deg) on AeroSandbox export_OpenVSP_vspscript geometry.

- E. GUI:
        - Part Editor (libs/library_editors.py): rename to an existing name overwrites on disk; category change keeps old fields; unsaved edits discarded on selection or view change; own copies of length-field list, palette, ordered_keys, format_field_value, field_label, parsing; NEW_PART_LIBRARY_DEFAULTS hardcoded.

- F. JSBSim export (libs/jsbsim.py, sim_inav.py), for INAV SITL via inav_development/inav-jsbsim-bench (MSP_SIMULATOR HITL; INAV mixes):
        - Done 2026-10-01, milestone 1 (hover): m62v1 stands on its tail, arms, climbs and holds NAV POSHOLD in INAV SITL; hover motor command 0.226 vs 0.228 predicted from bench data; POSHOLD within 0.6 m.
        - Prop: Ct0 0.137 (all bench rows); Ct(J) linear to zero at J = p/D (Tyto, as simlib2); Cp(J) = APC 9x4.5E shape (JSBSim DJI_9450) scaled; no measured forward-flight data. Rotor inertia: prop only.
        - Aero tables: AeroBuildup body-axis CX..Cn over AoA -180..180 x sideslip -90..90 at the operating airspeed, damping at AoA 0. 2026-10-02 against the real m62v1 flight (log .06, steady forward samples, sim cruise sweep at 22.2 V): speed at the real battery power, AeroBuildup tables bias +0.7 km/h RMS 6.8; live-model tables (lifting line + bodies) bias +4.0 RMS 10.7 and off the power curve's front side below motor 0.62. Both too fast above 650 W (real plateau 112-115 km/h; sim 121-127). AeroBuildup has no downwash between surfaces, so the sim's pitch stability differs from the builder's (simpleplane Cm -0.34 against -0.13 at AoA 0).
        - JSBSim quirks: do_simple_trim (ground trim) segfaults on a nose-up model, so start standing at stance_height_m; BOGEY contacts explode with the body axis vertical, so STRUCTURE contacts.
        - INAV HITL: MSP_SIMULATOR writes acc/gyro/mag past board/tailsitter alignment; the bench (Vehicle.tailsitter) rotates sensors into the hover frame. FC pitch estimate carries ~+1.6 deg bias in hover (likely INAV's one-time tailsitter quaternion rotation at boot); POSHOLD absorbs it.
        - 2026-10-02: m62v1 dash on its real Aug-2025 profile 2 (aircraft-defining lines only, rest INAV 10 defaults): level, stable at full throttle. Motor model now a fitted brushless DC motor (bench thrust within 5 g, RPM ~1.5 %). Real flight log 22/08/2025: 30-36 m/s at 57-77 % motor, pack 21-23 V (not full throttle).
        - 2026-10-02: quadlancer62 takeoff + transition + cruise sweep (generated two-profile config): best range ~60 km/h 2.4 Wh/km, best endurance ~55 km/h 138 W, stall just under 55 km/h (model estimates; no battery sag).
        - quadlancer62 hover with INAV 10 default PIDs oscillates (~1 Hz pitch/yaw, tilt to 30 deg); ANGLE climb steady. Needs a hover tune (bigger inertia, 9in props).
        - Transition in the cruise test: climb stopped first, throttle blended hover -> cruise with the nose coming down, gentler pitch-over: quadlancer62 altitude -0.9 / +2.8 m (was +19 m balloon).
        - Bench: NAV ALTHOLD climbs steadily (0.5-1 m/s) at mid stick (MID_STICK and STICK); NAV POSHOLD holds on m62v1. Unexplained.
        - Real wings have servos (quadlancer wing 90 g with servo, 86 g without; m62v1 V1.3 cross canards). Control surfaces in the model (2026-10-02): lifting-line increments (AeroSandbox flap model: AoA shift deflection x (1 - hinge^2.75)), servo FCS with rate limit, smix from roles with signs from computed moments. Servo masses not modelled.
        - simpleplane (fixed-wing test plane): CG far forward (static margin 88 % c_ref) and heavy for its wing (CL 0.98 for level flight at 20 m/s); dives in the plane test. The plane is to be changed.
        - Fixed-wing INAV tuning (trim, PIDs) to do.
        - No battery sag in the bench (JSBSim maxvolts only).

- G. Aero validation and optimization (decided 2026-10-02):
        - Validation, real world: fly m62v1 in sim at the logged motor command and pack voltage (Projects/M62X4/logs, 2025-08-22 .06: 30-36 m/s GPS at 57-77 % motor, 21-23 V) and compare speed. Pack voltage: sim_inav.py --pack-voltage (export maxvolts).
        - VSPAERO (installed, /usr/local/bin/vspaero): panel/vortex method with viscous corrections, minutes per aircraft, body and wings as one surface; third independent estimate. Fix the divergence on AeroSandbox's export_OpenVSP_vspscript geometry first (see D).
        - Performance evaluator (headless, no INAV): aero tables + motor/prop model -> level trim over speed: stall speed, best-range/endurance speed, Wh/km, hover throttle, static margin, mass. Seconds per design.
        - Optimizer on the evaluator: wing span/chords/sweep/dihedral/incidence, wing position, X/H, battery position, motor/prop from test data; constraints: printer bed, static-margin band, stall speed, hover throttle, wing mass from the print calibration. Best few flown in SITL (sim_inav.py cruise). Optimizers exploit model errors: keep ranges sane, read results as directions.
        - CFD, GPU: FluidX3D (lattice Boltzmann, OpenCL, RTX 3060 12 GB; non-commercial license) on the CAD export STL: AoA sweeps in minutes; uniform grid, so lift/Cm/stall usable, drag indicative only.
          On ice 2026-10-02: cloned to Software/other_software/FluidX3D (unmodified, built; benchmark 2527 MLUPs/s on the 3060). cfd/fluidx3d_setup.cpp written, not compiled or run (free stream at AoA on all faces, aircraft fixed, force + moment about CG to CSV; case file via QRB_CASE). Not done: runner cfd_fluidx3d.py (install setup as src/quadrocket_setup.cpp, enable FORCE_FIELD/EQUILIBRIUM_BOUNDARIES/SUBGRID, BENCHMARK off, export fused STL (binary, mm, geometry frame), average last flow-through, compare with LL/AeroBuildup). Docs: forces up to 2x high in turbulent flow (no wall model). License: no commercial or military use; publish altered source if results are published.
        - CFD, CPU: OpenFOAM (Docker image opencfd/openfoam-default; Mint's packaged version is 2020), simpleFoam k-omega SST, half model (XZ symmetry), 0.5-1M cells: ~10-25 min per AoA on the Ryzen 5 3400G (4 cores), overnight sweeps; fully turbulent RANS misreads laminar bubbles at Re ~100k (transition model is slower). Cloud 32-64 cores (~1-3 $/h) for occasional big runs.
        - CFD is a spot-check referee for the fast models, not part of the optimizer loop.

- Housekeeping:
        - No requirements file. Deps (aerosandbox, neuralfoil, cadquery) only present in conda env "drone".
        - docs/ui predates the plane-first GUI (Builder V3 window, Controls tab, assembly cards).

- Bring over from cadbuildsystem (predecessor):
        - cadparams/ FreeCAD spreadsheets (AirframeParams, WingVars, ParamAirfoil, CanardsAirfoil). Fix Fuselage_Thickness expr: OD - ID gives 2x wall; libs/cad.py uses (OD - ID) / 2.
        - parttojson.FCMacro (FreeCAD tree to JSON). Hidden objects are silently dropped; link_document and link_path hold each other's values.
        - README class hierarchy (Complex / System / Subsystem / Module / Submodule / Component / Subcomponent) and 2R04B1 assembly tree.
