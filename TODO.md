User added TODOs (do not remove/change unless done)

- changing xyz positions entry (which specifically, some must stay mm) from abs mm to relative frac to allow scaling without one-by-one mm adjustment

- rename surface "root offset" to "vertical offset"

- surfaces must not start outside fuselage (seperate bodies)

- scroll zoom still hangs until view change (click-drag)

- fuselage cross-sections with different shapes 

- SIM:
    * Re-design testing scenarios to be scripts with starting conditions, measurement points and goal/stop conditions
    * Scenario method of control for Flight Controller SITL is what a pilot would traditionally use: RC channels.  MSP/Mavlink commands (maybe complicated, not right now)
    * Center pane should have editor for the current INAV Diff
    * Start INAV SITL without flight to allow configurator connection
    * Center pane should have a Scenario text editor where the script can be edited as python
        
- Hatches (for now simple rounded rectangle polar cut with width, length, gap; fasteners and hinges deferred)

- Wiring
    * Auto-conduits from wires
    * Wire placement (think drawio, horizontal/vertical)
    
- Motor mount holes, set in motor mount section; either single-hole D1 or adjustable D1 and D2 diameters (drone motor mount), hole size, center hole diameter + depth, bolt head diameter and depth (remove all material before for bolt entry)

- Clean up summary formatting, find better location/format for stability analysis


---
Agent added TODOs, freeform, remove done ones

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

- In Parameters, we add a performance box.
        
- A. Data model:
        - m62v2 motor-wing section: owner to confirm whether the printed piece weighs ~122 g or ~222 g (counted 222 g: section 4 122 g + qrwing-7in 4 x 25 g).
        - Enter the real carbon rods (4 x 2.5 mm x 400 mm) in m62v2 and quadlancer62 fuselage.reinforcement.
        - Fix data/presets/tube75.json: wall (78.2 - 76.2) / 2 = 1.0 mm against its 1.5 mm minimum; its socket_wall_m is 1.0 mm until then.

- B. Geometry, verify on the plane-first model (found on the old module model):
        - Incidence on unpaired vertical fins opposite in CAD vs ASB (bottom fin chord y -0.086 CAD, +0.087 ASB).
        - End-cap motor at mid-module: bell pokes through cone, prop disc cuts cone.

- C. Physics:
        - Check total motor current against the pack's C rating (motorlib only warns on motor max current).

- D. Aero:
        - Let surfaces use imported DAT airfoils (data/airfoils.json); they take AeroSandbox names only.

- F. JSBSim export (libs/jsbsim.py, sim_inav.py), INAV SITL via inav_development/inav-jsbsim-bench:
        - FC part position and orientation not used in INAV SITL yet (the script controller's IMU uses them): bench change to read sensors at the FC (lever arm, axes) and matching board alignment in the generated diff.
        - Aero tables have no downwash between surfaces (AeroBuildup): the sim's pitch stability differs from the builder's (simpleplane Cm -0.34 against -0.13 at AoA 0).
        - quadlancer62 hover with INAV 10 default PIDs oscillates (~1 Hz pitch/yaw, tilt to 30 deg): needs a hover tune.
        - NAV ALTHOLD climbs steadily (0.5-1 m/s) at mid stick; unexplained.
        - Fixed-wing INAV tuning (trim, PIDs).
        - No battery sag in the bench (JSBSim maxvolts only).
        - Fixed-wing ground start (`--start ground`): export wheels or a launcher (structure contacts only now).

- G. Aero validation and optimization:
        - Fly m62v1 in sim at the logged motor command and pack voltage (Projects/M62X4/logs, 2025-08-22 .06: 30-36 m/s GPS at 57-77 % motor, 21-23 V; sim_inav.py --pack-voltage) and compare speed.
        - VSPAERO (/usr/local/bin/vspaero) as a third estimate: fix its divergence (CL ~ -300 at 0 deg) on AeroSandbox's export_OpenVSP_vspscript geometry first.
        - Performance evaluator (headless, no INAV): aero tables + motor/prop model -> level trim over speed: stall speed, best-range/endurance speed, Wh/km, hover throttle, static margin, mass.
        - Optimizer on the evaluator: wing span/chords/sweep/dihedral/incidence, wing position, X/H, battery position, motor/prop; constraints: printer bed, static-margin band, stall speed, hover throttle, wing mass. Best few flown in SITL (sim_inav.py cruise).
        - CFD, GPU (on ice): FluidX3D (Software/other_software/FluidX3D, built): compile and run cfd/fluidx3d_setup.cpp; runner cfd_fluidx3d.py (install setup as src/quadrocket_setup.cpp, enable FORCE_FIELD/EQUILIBRIUM_BOUNDARIES/SUBGRID, BENCHMARK off, fused binary STL in mm, average the last flow-through, compare with LL/AeroBuildup).
        - CFD, CPU: OpenFOAM (Docker opencfd/openfoam-default) simpleFoam k-omega SST half-model AoA sweeps, overnight.

- Printable sections (libs/cad.py export_sections):
        - m62v2 and m62v1: tailfin root chord (573-673 mm) runs past the 602 mm cut into the tail section (S05 exports as 2 solids).
        - Cut fuselage.channels (a channel has no radial position yet).
        - Blend fin root base (surfaces[].root_base, recorded only) into the body.
        - Build auto sectioning (PLAN.md section 4).

- Bring over from cadbuildsystem (predecessor):
        - parttojson.FCMacro (FreeCAD tree to JSON). Hidden objects are silently dropped; link_document and link_path hold each other's values.
        - README class hierarchy (Complex / System / Subsystem / Module / Submodule / Component / Subcomponent) and 2R04B1 assembly tree.

- Add a requirements file (deps aerosandbox, neuralfoil, cadquery only in conda env "drone").
