# Analysis

## Reading the results

Everything is computed at one **operating point**: angle of attack (AoA), airspeed and altitude, set in the Airframe tab (Operating point) and saved in the aircraft file. Pick a speed the aircraft actually flies at in wing-borne flight. At hover-transition speeds the wings carry almost nothing and the stability numbers say little.

The drawing has three markers:

| Marker | Meaning |
|---|---|
| blue | **CG**, where the aircraft balances |
| red | **Neutral point**, where the aerodynamic forces effectively act when AoA changes |
| green | **Lift center** at the chosen operating point |

**Stability: the red marker must be behind the blue one.** If a gust pitches the nose up, the extra lift acts behind the CG and pushes the nose back down. The distance between them is the **static margin**, reported in metres and as a percentage of the mean wing chord (MAC):

- **negative → unstable.** It will diverge in pitch unless a flight controller holds it constantly. Move mass forward or add tail area further back.
- **0-5 % → marginal.** Flyable with a flight controller, twitchy without one.
- **roughly 5-20 % → normal** for small planes.
- **much more → very stable but sluggish**, and it needs more up-elevator (more drag) to hold the nose up.

The Barrowman CP line is a rocket-style estimate shown as a cross-check; if it disagrees wildly with the neutral point, look closer.

**Lift as % of weight** tells you how much the wings carry at that speed and AoA. 100 % means the wings hold the aircraft up by themselves and the motors only push it forward; much less means the motors are still holding it up.

**AoA sweep table** (−30° to +30°): for each AoA, the lift coefficient (CL), drag coefficient (CD), pitch moment (Cm), lift in newtons and % of weight, the local neutral point, and whether the aircraft wants to pitch nose-up or nose-down. The AoA where Cm changes from nose-up to nose-down is where the aircraft **trims** by itself with no control input. Lift that stops rising as AoA increases is stall.

**Per-surface rows** show which wing or fin set produces how much lift and drag, and where its lift acts (`x_lift`). The `body` row is the fuselage plus motor pods.

## How it calculates

1. **Resolve.** The aircraft definition becomes geometry (body stations, wing panels, motors and pods) and point masses: every printed section, coupler, rod, wing set, motor, prop and volume at its real position. That gives total mass, CG (a CG range when a volume's mass is a range) and the inertia tensor. Printed parts are estimated from geometry (fuselage sections: hollow = shell wall x material density, solid = outer volume less the volumes inside it x the print bulk density; wings and fins: solid airfoil volume x that bulk density, calibrated on a weighed printed segment) and the estimate is shown next to any weighed value. Compute Geometry replaces these estimates with the printed structure measured on the built CAD solids (`libs/computed_geometry.py`): each fuselage section is its printable part less the surface roots outside the skin, plus its hatch pieces (in a hollow section the material inside the bore, i.e. carry-through, at bulk density and the wall at material density; a solid section at bulk density); each surface set is its roots outside the skin, its detached segments and its controls at bulk density; each coupler is its ring at material density. Access openings, holes, sockets and spigots are therefore in the mass. Weighed values still win, with the exact value shown next to them. The exact mass applies only to the design it was measured on. A section can also take a weighed complete mass, which replaces it and everything attached in it.
2. **Propulsion.** motorlib (`libs/motorlib`, the rcplanes.online calculator's motor and prop data and formulas, used as published) for the chosen motor, prop and battery, with the resistance from the cells to each motor worked out from the design (pack, wire runs from the parts' positions, plugs, ESC), gives static thrust, current, power, thrust/weight, hover point (VTOL) and flight time. Throttle is INAV's motor output, i.e. ESC duty.
3. **Aerodynamics (live, ~1.5 s): lifting line + bodies.** AeroSandbox's lifting line for all wings and fins, with body-mounted panels carried through the body (a mirrored pair to its centre plane, a single panel to the axis), plus a slender-body model of the fuselage, pods and motors (`body_loads`, after Jorgensen, NASA TR R-474): slice by slice, a potential force from each slice's area change (sin 2σ, zero side-on, reversed tail first), cross-flow drag of each slice as a cylinder, and axial drag at zero angle from AeroBuildup's skin-friction, base and wave models along the axis. AeroBuildup's own fuselage model is not used: its slender-body terms go with sin σ instead of sin 2σ, which at 90° gave swagger a 3 N·m nose-up moment where the body has 0.6 N·m and made a body fly stable tail first. It includes downwash from front surfaces onto rear ones, lift carried across the body, airfoil camber and stall at the actual Reynolds number (NeuralFoil section polars). It is AeroSandbox's `LiftingLine` algorithm solved for the whole AoA sweep at once; between different surface sets each trailing vortex is smoothed over a quarter of the width of the strip that sheds it, so a wake crossing another surface (canards ahead of fins) does not make the answer jump with the mesh. The neutral point comes from a straight-line fit of pitch moment against lift over ±4° around the operating point.
4. **JSBSim export (`libs/jsbsim.py`).** Mass, CG, inertia, contact points, motors with spin and thrust tilt, the same fitted motor and prop tables, the editor’s lifting-line + body model in forward flight, blended to the outer envelope: AeroBuildup for the wings and `body_loads` for the bodies. The rate damping is AeroBuildup's for the wings plus the bodies' from `body_loads` with the body rotating (each slice sees its own motion, so the nose and tail sweeping across the flow damp a tumble); a rate damping that grows with the square of a large rotation is not in the linear JSBSim tables. The static and control tables are solved with moments about the nose tip and moved to the CG exactly, so a change that only moves masses reuses them; the rate damping is solved about the CG. On an airframe that repeats every quarter turn about its body axis (every finned rocket: `rotationally_symmetric`) sideslip is answered as that same pitch-plane model rolled to the flow direction, because AeroBuildup's own sideslip answer loses the fins' stabilising moment and is unstable where the same airframe's pitch is stable; on other airframes (a wing and a fin) sideslip increments remain AeroBuildup. Control surfaces driven by INAV servo outputs, plus INAV mixer profiles by aircraft type (tailsitter: hover and forward flight; fixed-wing or glider: AIRPLANE) with a servo mix whose signs come from each surface's computed moments. `sim_flight.py --controller inav` flies it in INAV SITL on `inav-jsbsim-bench`; m62v1 hovers at the predicted motor command.
5. **Rocket motors.** Solid motors from `data/rocket_motors/` (RASP thrust curves, as published), loaded into rocket mounts. The report gives liftoff and burnout mass and CG, thrust/weight and the static margin in body calibers; JSBSim burns the propellant along the curve, so mass and CG move during the burn.
6. **Barrowman cross-check.** The OpenRocket method for nose plus fins, with each fin weighted by how much it faces the pitch plane, so vertical fins add no pitch stability.

Other views:

- **Aero Analysis.** 2D airfoil comparison (NeuralFoil).
- **VLM.** AeroSandbox vortex-lattice sweep and 3D view, solved once per AoA (airspeed and altitude only scale the forces). Wings only: VLM ignores the fuselage, so wings mounted on the tube read about half their real lift there; use it for trends and pictures, not absolute numbers.
- **CAD.** STEP/STL export of the outer shape; the printable parts (each section with what is attached in it, hollowed, with joint, rod and spar holes; the couplers); blueprint drawings of those parts: a side cutaway (cut faces hatched), a three-view (side, top, front and an opaque iso view from above, ahead and to the left) and an x-ray view from above, ahead and to the left (skin see-through, parts, rods and spars inside); FreeCAD varsets.

## How far to trust it

The numbers are good for comparing layouts and catching bad ideas, not for certifying an aircraft.

- AeroBuildup alone (the previous live model) has no downwash between surfaces and no lift carried across the body: on m62v2 it says +25 mm margin where the lifting line says −25 mm.
- The section polars at these Reynolds numbers (~100k) are wavy, so the margin moves by a few millimetres with the operating point and the fit window (m62v2 at 5° / 20 m/s: −5 mm over ±4°, −9 mm over ±2°). Treat a margin within about ±10 mm as "near neutral", and evaluate at cruise speed; below ~70k (small chords at 10 m/s) it gets worse.
- Control surfaces use AeroSandbox's flap model: a section AoA shift of deflection × (1 − hinge²·⁷⁵), no hinge-gap or flap pitching-moment effects.
- Not modelled yet: propwash and thrust effects.
- Rocket flight tables are solved up to 300 m/s (about Mach 0.9, the subsonic models' end); faster, JSBSim holds the 300 m/s values. No transonic or supersonic aero, no recovery (parachute, ejection delay) yet.
