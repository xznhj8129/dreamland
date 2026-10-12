User added TODOs (do not remove/change unless done)

- changing xyz positions entry (which specifically, some must stay mm) from abs mm to relative frac to allow scaling without one-by-one mm adjustment

- "[Auto][Override]" Button pairs should be a single boolean [Auto] button everywhere

- CAD: export progress bar per section

- SIM:
    * MSP/Mavlink commands (maybe complicated, not right now)
    * SIM priority: flexibility and realism; editor stability estimate versus JSBSim flight response.
    * Performance flash card: efficiency, stall speed, sustained turn rate, acceleration/speed per axis, range, maximum altitude.
    * Optimization tab: optimize for X within user ranges on part values; big outlines and features first.
        
- Hatches (for now simple rounded rectangle polar cut with width, length, gap; fasteners and hinges deferred)

- Wiring
    * Wire placement (straight horizontal/vertical, only 90deg bends for now)
    * Auto-conduits from wires
    


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

- Geometry, verify on the plane-first model (found on the old module model):
        - Incidence on unpaired vertical fins opposite in CAD vs ASB (bottom fin chord y -0.086 CAD, +0.087 ASB).
        - End-cap motor at mid-module: bell pokes through cone, prop disc cuts cone.

- Physics:
        - Check total motor current against the pack's C rating (motorlib only warns on motor max current).

- Aero:
        - Let surfaces use imported DAT airfoils (data/airfoils.json); they take AeroSandbox names only.

- JSBSim export (libs/jsbsim.py, sim_flight.py), INAV SITL via inav_development/inav-jsbsim-bench:
        - FC part position and orientation not used in INAV SITL yet (the script controller's IMU uses them): bench change to read sensors at the FC (lever arm, axes) and matching board alignment in the generated diff.
        - Validate shared forward lifting-line aero against real flight; sideslip and rate damping remain AeroBuildup, with a blended outer envelope.
        - Expand/validate the 5–50 m/s Reynolds grid and altitude dependence before accepting performance outside it.
        - quadlancer62 hover with INAV 10 default PIDs oscillates (~1 Hz pitch/yaw, tilt to 30 deg): needs a hover tune.
        - NAV ALTHOLD climbs steadily (0.5-1 m/s) at mid stick; unexplained.
        - Fixed-wing INAV tuning (trim, PIDs).
        - No battery sag in the bench (JSBSim maxvolts only).
        - Fixed-wing ground start (`--start ground`): export wheels or a launcher (structure contacts only now).

- Aero validation and optimization:
        - Fly m62v1 in sim at the logged motor command and pack voltage (Projects/M62X4/logs, 2025-08-22 .06: 30-36 m/s GPS at 57-77 % motor, 21-23 V; sim_flight.py --pack-voltage) and compare speed.
        - VSPAERO (/usr/local/bin/vspaero) as a third estimate: fix its divergence (CL ~ -300 at 0 deg) on AeroSandbox's export_OpenVSP_vspscript geometry first.
        - Performance evaluator: stall detection, sustained turns, axis acceleration/rate limits, service ceiling; trim feasibility separate from simple-pilot limits.
        - Optimizer on the evaluator: wing span/chords/sweep/dihedral/incidence, wing position, X/H, battery position, motor/prop; constraints: printer bed, static-margin band, stall speed, hover throttle, wing mass. Best few flown in SITL (sim_flight.py).
        - CFD, GPU (on ice): FluidX3D (Software/other_software/FluidX3D, built): compile and run cfd/fluidx3d_setup.cpp; runner cfd_fluidx3d.py (install setup as src/quadrocket_setup.cpp, enable FORCE_FIELD/EQUILIBRIUM_BOUNDARIES/SUBGRID, BENCHMARK off, fused binary STL in mm, average the last flow-through, compare with LL/AeroBuildup).
        - CFD, CPU: OpenFOAM (Docker opencfd/openfoam-default) simpleFoam k-omega SST half-model AoA sweeps, overnight.

- Printable sections (libs/cad.py export_sections):
        - Cut fuselage.channels (a channel has no radial position yet).
        - Blend fin root base (surfaces[].root_base, recorded only) into the body.
        - Build auto sectioning (PLAN.md section 4).

- Bring over from cadbuildsystem (predecessor):
        - parttojson.FCMacro (FreeCAD tree to JSON). Hidden objects are silently dropped; link_document and link_path hold each other's values.
        - README class hierarchy (Complex / System / Subsystem / Module / Submodule / Component / Subcomponent) and 2R04B1 assembly tree.

- Add a requirements file (deps aerosandbox, neuralfoil, cadquery only in conda env "drone").
