User added TODOs (do not remove/change unless done)


- In Parameters, we add a performance box.

**SIM**
    * MSP/Mavlink commands (maybe complicated, not right now)
    * SIM priority: flexibility and realism; editor stability estimate versus JSBSim flight response.
    * Performance flash card: efficiency, stall speed, sustained turn rate, acceleration/speed per axis, range, maximum altitude.
    * Optimization tab: optimize for X within user ranges on part values; big outlines and features first.
    * remember scenario and settings when switching to other tabs
        
**INAV**
    * Current default configs bad to configure

**Editor**
    - changing xyz positions entry (which specifically, some must stay mm) from abs mm to relative frac to allow scaling without one-by-one mm adjustment
    - Add direction arrow triangles to prop circle to show direction of spin (from front)
    - "plain ass plate" as airfoil, with or without thickness-diameter rounded edges
    - ducts, NACA ducts and intake ducts

**Wiring**
    * Wire placement (straight horizontal/vertical, only 90deg bends for now)
    * Auto-conduits from wires

**UI/Styling**
    - left pane buttons (Summary, Mass, Aero, etc) are wider apart than on the right pane; create one consistent universal "button row" element
    - Look for visual weirdness and inconsistencies, create consistent universal styling/functional elements for reuse across UI
    - Remember open tab/section in editor to return to
    
**Construction**
    - Allow non 3D-printed shell construction methods (such as fiberglass or carbon fiber molding), different materials (what that excludes TBD)
    - Allow one section to be considered a monolithic carbon fiber, fiberglass or metal plate, allowing different material for CNC rather than 3D print. This would be used for thin (1-10mm) reinforcement sections
    - Actual real wing, hatch, section joint/closure mechanisms
    
**CAD**
    - Access holes currently cut straight into (in-line). A proper hatch cuts towards the center. This must be an option (not always applicable to non-fuselage ie wings)
    - Hatches start under by default, should be on top and pointing down
    - Make hatches/holes automatically position clear above skin surface, they always point outside-in
    - Why do them CAD exports have all these loft lines anyway?
    - Swifting off CAD tab interrupts
    - Thicken fuselage line on xray export
    - Tell me we're not exporting parts in CAD

**3D view**
    - turn Cuts & Holes off by default
    - Show/hide motors
    - Show/hide propellers
    
**Motors**
    - show actual parametric motor (open or closed (plane) base; up or reverse shaft) (/media/anon/WD2TB/DataVault/TechProjects/CAD/my_drones/std_parts/parametric motor.FCStd)


**MCP**
    - Most of it already exists as HTTP routes in webapp/server.py
    - MCP server joins the same session as the browser. You’d watch the agent edit the airframe in real time through the existing /ws/events push

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
        - quadlancer62 hover with INAV 10 default PIDs oscillates (~1 Hz pitch/yaw, tilt to 30 deg): needs a hover tune. {aircraft design problem, out of scope}
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

- Rockets (solid motors done 2026-10-12: data/rocket_motors, rocket mounts, JSBSim burn, MCU ignite, mcu/launch):
        - Motor database: more RASP files from thrustcurve.org (only AeroTech F42T so far).
        - Recovery: ejection at the motor's delay, parachute (drag area, deploy, descent rate); minirocket's chute bay ballast stands in.
        - Staging: ignite a mount on another's burnout; separate stages.
        - INAV: no way to ignite a rocket motor from INAV yet (output or mode).
        - Aero past 300 m/s (~Mach 0.9): transonic/supersonic; tables hold the 300 m/s values beyond.
        - Liquid-fuel motors (later; up to space).

- Found designing and flying aircraft/albatross (2026-10-14); [decide] = needs owner input:
        - [decide] In-flight battery power (libs/simrun.py motor_electrics) is I = (throttle x V - rpm / Kv) / R, after the fact: no no-load current (Io; libs/propulsion.py has it, 0.5 A = ~8 W on this motor) and not coupled to the shaft power the JSBSim prop absorbs. At 38 m/s battery 117 W vs drag power ~112 W (overall efficiency ~96 %); efficiency.py range/endurance (343 km, 5.8 h) are therefore optimistic. Which model the flight power should come from.
        - [decide] Drag looks optimistic for printed airframes: glide 18:1 (lifting line + AeroBuildup, no roughness/joint/interference drag). Whether to model an allowance, and from what data (no fudge fits; real logs). {owner: good point, we can look at how OpenRocket does it}
        - [decide] Wing twist/washout: no field. A tapered wing without washout departs at the stall (Albatross rolls 14-20 deg below ~13 m/s in glide.py). Schema field (tip incidence or washout_deg)? {yeah we should add that, different incidence for tip}
        - [decide] Parts in wings: aileron servos placed in the wing stick out of it (9 g servo 28.5 mm tall, wing ~13 mm); part_checks only test fuselage volumes; no servo bays, pockets, wire channels or covers in printed wings.
        - [decide] Printed surface construction: estimate = solid airfoil x bulk density (Albatross wing 431 g for 0.16 m^2); exact CAD mass is solid too. Shell + ribs / spar web / infill choice per surface? {slicers provide estimation code?}
        - [decide] efficiency.py starts its sweep at the aero grid top (48 m/s); any aircraft that cannot hold it ends with no data. Start from the full-throttle level speed instead? (Albatross uses a project override, sweep_from 38.) {definitely needs adjustment}
        - Pitch stability falls off above ~2-3 deg AoA in the lifting line (local NP moves forward with AoA) for a V-tail behind a high wing; check with VLM once the VLM view exists.
        - Cutaway drawing: leader numbers overlap where parts share an x (wing root, tail servos); the key line runs off the image's right edge.
        - Web SIM (webapp/sim.py) always uses SITL port 5760; sim_flight.py now refuses a taken port (--sitl-port to move it).
