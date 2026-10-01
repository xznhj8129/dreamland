# quadrocketbuilder (GUIBuilder)

A design tool for small tube-bodied drone airframes: a fuselage tube made of modules, wing and fin sets bolted onto it, motors, props, battery and payload inside. You assemble the aircraft from parts you have (or intend to have), and the tool works out mass, balance, propulsion and aerodynamics, draws it, and exports geometry for CAD and printing.

It exists because OpenVSP is unusable for this: the interface is hostile and most of what it outputs is meaningless unless you are an aero engineer. Here the inputs are physical parts and the outputs are a few numbers that answer practical questions. Think OpenRocket, but for tubes with wings, motor pods and VTOL weirdness.

## Design premise: plane first, then cut into sections

The original workflow was Lego-style: print tube sections, couplers, nosecones and endcaps in FreeCAD, then assemble aircraft from those parts. That is backwards. The intended way is to design the aircraft as a whole (tube, nose, tail, wings, motors, battery, payload) and then cut it into printable sections.

The current code still assembles aircraft from a module library (`data/modules.json`). The bundled `m62v2.json` and `quadlancet62.json` are real aircraft built the Lego way. Moving to plane-first is the direction; see `TODO.md`.

## Running it

Dependencies (aerosandbox, neuralfoil, cadquery, pandas, numpy, Tk) live in the conda env `drone`.

```
python guibuilder.py                                   # GUI
python analyze_headless.py --config m62v2.json         # same analysis, text report
python analyze_headless.py --config m62v2.json --aoa 5 --airspeed 20
python analyze_headless.py --config m62v2.json --export-step out.step --export-stl out.stl
python aero_check.py --config m62v2.json quadlancet62.json   # cross-check aero against VLM
```

`data/default.json` is the built-in aircraft. Saved aircraft (`m62v2.json`, `quadlancet62.json`) are loaded with `--config` or File → Open.

## Reading the results

Everything is computed at one **operating point**: angle of attack (AoA), airspeed and altitude, set in the Controls tab (Surface Force Operating Point). Pick a speed the aircraft actually flies at in wing-borne flight. At hover-transition speeds the wings carry almost nothing and the stability numbers say little.

The drawing has three markers:

| Marker | Meaning |
|---|---|
| blue | **CG**, where the aircraft balances |
| red | **Neutral point (fast model)**, where the aerodynamic forces effectively act when AoA changes |
| orange | **Neutral point (detailed model)**, after pressing "Detailed aero" |
| green | **Lift center** at the chosen operating point |

**Stability: the red marker must be behind the blue one.** If a gust pitches the nose up, the extra lift acts behind the CG and pushes the nose back down. The distance between them is the **static margin**, reported in metres and as a percentage of the mean wing chord (MAC):

- **negative → unstable.** It will diverge in pitch unless a flight controller holds it constantly. Move mass forward or add tail area further back.
- **0-5 % → marginal.** Flyable with a flight controller, twitchy without one.
- **roughly 5-20 % → normal** for small planes.
- **much more → very stable but sluggish**, and it needs more up-elevator (more drag) to hold the nose up.

The Barrowman CP line is a rocket-style estimate shown as a cross-check; if it disagrees wildly with the neutral point, look closer.

**Lift as % of weight** tells you how much the wings carry at that speed and AoA. 100 % means the wings hold the aircraft up by themselves and the motors only push it forward; much less means the motors are still holding it up.

**AoA sweep table** (−45° to +45°): for each AoA, the lift coefficient (CL), drag coefficient (CD), pitch moment (Cm), lift in newtons and % of weight, the local neutral point, and whether the aircraft wants to pitch nose-up or nose-down. The AoA where Cm changes from nose-up to nose-down is where the aircraft **trims** by itself with no control input. Lift that stops rising as AoA increases is stall.

**Per-surface rows** show which wing or fin set produces how much lift and drag, and where its lift acts (`x_lift`). The `body` row is the fuselage plus motor pods.

## How it calculates

1. **Assembly.** Modules are stacked nose to tail, and every module, wing set, motor, pod, battery, payload and ballast mass is placed at its real x position. That gives length, total mass and CG.
2. **Propulsion.** Measured bench data (`libs/motordata.csv`) for the chosen motor/prop/cell count gives static thrust, current, power, thrust/weight, hover throttle and flight time.
3. **Aerodynamics, fast (live): AeroSandbox AeroBuildup.** The assembled geometry (fuselage, motor pods, every wing and fin set with its real airfoil) goes through AeroSandbox's AeroBuildup model in one pass over the AoA sweep and the operating point. It includes the fuselage, finite (stubby) wings, sweep, airfoil camber and stall, at the actual Reynolds number. The neutral point comes from a straight-line fit of pitch moment against lift over ±4° around the operating point.
4. **Aerodynamics, detailed (button, ~1 min): lifting line + bodies.** AeroSandbox's lifting line for all wings and fins, with body-mounted panels carried through the body, plus AeroBuildup for the fuselage and pods. This adds what the fast model misses: downwash from front surfaces onto rear ones, lift carried across the body, and correct camber lift on short wings. The summary shows both models side by side.
5. **Barrowman cross-check.** The OpenRocket method for nose plus fins, with each fin weighted by how much it faces the pitch plane, so vertical fins add no pitch stability.

Other views:

- **Aero Analysis.** 2D airfoil comparison (NeuralFoil).
- **VLM.** AeroSandbox vortex-lattice sweep and 3D view, solved once per AoA (airspeed and altitude only scale the forces). Wings only: VLM ignores the fuselage, so wings mounted on the tube read about half their real lift there; use it for trends and pictures, not absolute numbers.
- **CAD.** STEP/STL export of the outer shape and FreeCAD varset values.

## How far to trust it

The numbers are good for comparing layouts and catching bad ideas, not for certifying an aircraft.

- The fast model is good for quick comparisons but is optimistic on stability for layouts with a big forward wing and surfaces behind it (no downwash, no body carry-over): on m62v2 it says +28 mm margin where the detailed model says −5 mm. Use "Detailed aero" before trusting a stability verdict.
- Wing lift agrees within roughly ±15 % between the fast and detailed models below stall for these airframes. `aero_check.py` compares each wing against VLM; the known gaps are listed in `TODO.md`.
- At low speed (Reynolds number below ~70k: small chords at 10 m/s) the airfoil model gets bumpy and the margin jumps around. Evaluate at cruise speed.
- Not modelled yet: propwash and thrust effects, control surfaces. Downwash between wing sets and wing-body carry-over are in the detailed model only.

## Where this is going

1. Get the basic aero right (in progress; see `TODO.md`).
2. Export a JSBSim model from the assembled aircraft.
3. Generate printable module solids (tube walls, couplers, bolt holes, hatches, sockets from `data/specs.json`), so less and less is left to do in FreeCAD. Then print and fly.

File layout and agent rules are in `AGENTS.md`; UI behaviour is in `docs/ui/`.
