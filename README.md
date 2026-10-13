<p align="center">
  <img src="dreamland.png" alt="Dreamland — Unmanned Aerospace Sandbox" width="360">
</p>

<h1 align="center">Dreamland</h1>

<p align="center"><b>Unmanned Aerospace Sandbox</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-GPLv3-blue.svg" alt="License: GPLv3"></a>
  <img src="https://img.shields.io/badge/python-3-3776AB.svg" alt="Python 3">
  <img src="https://img.shields.io/badge/UI-React%20%2B%20TypeScript-61DAFB.svg" alt="React + TypeScript">
  <img src="https://img.shields.io/badge/status-in%20development-orange.svg" alt="Status: in development">
</p>

<p align="center">
  <code>uav</code> · <code>drone</code> · <code>vtol</code> · <code>tailsitter</code> · <code>model-rocketry</code> · <code>aircraft-design</code> · <code>lofting</code> · <code>aerodynamics</code> · <code>aerosandbox</code> · <code>jsbsim</code> · <code>inav</code> · <code>sitl</code> · <code>cadquery</code> · <code>3d-printing</code>
</p>

---

Dreamland is a sandbox design and simulation environment for small unmanned aircraft and rockets. You loft an airframe, attach lifting surfaces, mount propulsion and place real components inside it; Dreamland computes how it balances, how it flies and what it weighs, flies it in simulation against real flight-controller firmware, and generates the printable parts.

It is built for hobby flying: RC planes, drones, gliders, rockets, anything you can imagine.

## Vision

Kerbal Space Program for real hardware. A vehicle is whatever its parts make it: a lofted body with wings and electric motors is a plane, add hover motors and it is a VTOL, swap in a solid motor and it is a rocket. Dreamland derives capability from the design instead of starting from a fixed aircraft type.

One definition is the single source of truth. The same file drives the drawing, the aerodynamics, the simulation model, the flight-controller configuration and the CAD. Nothing is entered twice, and nothing downstream becomes a second copy of the aircraft.

Results are meant to be read by people who build and fly, not by aerodynamicists: does it balance, is it stable, can it hover, how long does it fly, will the parts fit, and what do I print.

## Features

### Airframe design
- **Lofted fuselage**: superellipse cross-sections (circle to rounded box) at independent loft stations, joined with cubic easing and tapered by parametric nose and tail profiles
- **Lifting surfaces**: wing and fin sets in H, X, +, 1 and 3 layouts, body- or axis-rooted, with sweep, dihedral, incidence or cant, airfoils, spars and carry-through webs
- **Control surfaces**: split or all-moving, with hinge geometry, gaps, pivot rods, deflection limits and servo assignment
- **Internal layout**: volumes for battery, ballast and avionics; library parts placed and checked for fit and overlap; wiring placement and routing channels
- **Structure**: longitudinal reinforcement rods, wire channels, cut stations with couplers or integral spigots and radial screw joints
- **Part library**: flight controllers, ESCs, receivers, VTX, GPS, cameras, servos, batteries, props, connectors and materials, with a built-in part editor

### Propulsion
- **Electric**: motor and prop simulation, battery and wiring resistance worked out from the actual wire runs, gauges and connectors
- **Rocket**: solid motors from a RASP thrust-curve database, loaded into tail or surface mounts
- **Gas**: internal combustion propeller engines (coming)
- **Turbine**: hobby jet engines (coming)
- Tractor or pusher, nose, tail or wing-mounted motors, pods, spin direction and thrust-line tilt

### Analysis
- Mass, CG, CG range and inertia; printed-part mass estimated from geometry and print settings, replaceable by weighed values
- Lifting-line aerodynamics for every surface plus body buildup, with NeuralFoil section polars at the real Reynolds number
- Neutral point, static margin, trim and stall across an angle-of-attack sweep, with a Barrowman cross-check
- Thrust, current, hover point, thrust-to-weight and flight time; liftoff and burnout figures for rockets
- 2D airfoil comparison
- VLM analysis
- CFD (coming)

### Simulation
- JSBSim model exported from the design: mass properties, aero tables, motor and prop tables, rocket burns, control surfaces
- **INAV SITL**: the real firmware flies the aircraft, from a generated INAV diff you can read and edit; mixer profiles and servo mix come from the design
- **Custom flight controllers**: Python scripts read simulated raw sensors (IMU, baro, mag, GPS, pitot, rangefinder) and write PWM
- Scenarios: hover, efficiency and range, top speed, glide, control response, stability, rocket launch; live plots and 3D flight path in the browser

### Manufacturing
- STEP and STL of the whole airframe, of each printable section and coupler, and of wing segments
- Blueprint drawings: side cutaway, three-view and x-ray
- FreeCAD varsets for downstream detailing

### Interfaces
- Browser UI: Editor (2D drawing and 3D view), Aero Analysis, SIM, CAD and Part Editor
- Everything also runs headless from the command line

## Getting started

Python 3 with `aerosandbox`, `neuralfoil`, `cadquery`, `jsbsim`, `aiohttp`, `numpy`, `scipy`, `pandas`, `matplotlib` and `threadpoolctl`; Node.js for the browser UI. Simulation also needs `inav-jsbsim-bench`, and INAV runs need an INAV SITL build.

```bash
cd web && npm install && npm run build
```

```bash
python webbuilder.py aircraft/example
```

The full command reference is in [docs/usage.md](docs/usage.md).

## Documentation

| Document | Contents |
|---|---|
| [docs/usage.md](docs/usage.md) | Commands, project layout, flight scenarios |
| [docs/analysis.md](docs/analysis.md) | Reading results, calculation stages, accuracy limits |
| [docs/ui/](docs/ui/) | Browser UI, one file per view |
| [PLAN.md](PLAN.md) | Design decisions and the aircraft definition schema |
| [TODO.md](TODO.md) | Open work |

## Status

Under active development. Results are for comparing designs and catching bad ones before printing, not for certifying an aircraft. The VLM view is planned; propwash, recovery systems and supersonic flight are not modelled yet.

## License

Dreamland is free software, released under the [GNU General Public License v3.0](LICENSE).
