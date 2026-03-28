# GUIBuilder

`guibuilder` is a Tk-based aircraft configuration tool for quickly building and inspecting modular tube-bodied aircraft.

More precisely, it is a modular airframe composition and validation tool.

It is for cases where you already have, or already intend to have, a real ecosystem of compatible parts:
- tube diameters
- endcaps
- couplers
- wing mounts
- motor pods
- batteries
- ESCs
- motors and propellers
- payload sections
- printable or purchasable structural modules

It is step 1 of the design pipeline:
- define and validate the aircraft layout here
- turn the resulting dimensions and configuration into parametric CAD inputs
- drive FreeCAD from varsets and spreadsheets
- export directly for 3D printing

So although this is not a solid-modeling environment, it is still part of the CAD toolchain in a practical sense. It is the front-end design and validation tool that exists before the FreeCAD model.

This is not a normal airplane design tool.

A conventional aircraft design flow usually starts from abstract requirements and aerodynamic choices:
- mission and performance targets
- selected airfoil family
- derived wing sizing
- derived propeller and propulsion sizing
- derived fuselage and packaging

This tool starts from the opposite end.

Here, many of the important pieces are already known:
- fuselage tube size
- motors
- propellers
- batteries
- ESCs
- payload modules
- existing printed or purchased structural parts

Many of these parts already exist in real life. The vehicle is assembled from modular pieces, more like a constrained kit of parts than a clean-sheet airplane.

The logic is not:
- derive the parts from the ideal aircraft

The logic is:
- start from the parts, packaging, and constraints that are real
- assemble them into one airframe
- see what sort of vehicle that airframe becomes

So the end product is not the starting point. The sum of real modules is the starting point.

That is the central idea behind this tool.

It can represent something that looks like a normal airplane, but that is not the primary design philosophy. A normal airplane is usually designed top-down from aerodynamic and mission requirements. This tool is for building bottom-up from a modular physical system.

It is meant for early configuration work:
- stack fuselage modules
- attach wing and tail fin sets
- place motors, props, batteries, payload, and ballast
- see the shape immediately in orthographic views
- get a fast first-pass stability readout without waiting for VLM

This is not the detailed aerodynamic solver. It is the fast front-end that answers the first practical questions:
- where is the CG?
- where is the approximate CP?
- is the layout statically stable, marginal, or unstable?
- at a chosen AoA, does the simplified model tend nose-up or nose-down?

The output is therefore meant to be CAD-relevant:
- geometry that is explicit enough to parametrize
- masses and placements that are explicit enough to sanity-check
- fast aerodynamic screening before committing to detailed CAD and print work

## Design Premise

The baseline vehicle here is simple:
- a tube
- modular parts attached to that tube
- motors and propellers providing thrust
- batteries and payload packaged inside known real volume

So the design problem is not “derive the ideal aircraft, then choose parts for it.”

It is:
- start from known modules and known packaging
- assemble them into one airframe
- check whether the resulting object is physically sane
- check whether it is statically sane
- if wings are added, see what starts to happen
- only then decide whether deeper aerodynamic analysis is warranted

That is why this tool is bottom-up rather than top-down. It is for modular real-world hardware composition first, and only later for higher-fidelity aerodynamic investigation.

## What It Builds

The configuration is assembled from modular sections rather than one monolithic airframe description.

The current data flow is:
- `libs/default.json` defines the active aircraft configuration
- `libs/modules.json` defines reusable structural module blocks
- `libs/parts.json` defines named batteries and other part-library data
- `libs/templates.json` defines editable templates and field types used by the GUI

From that, the analysis stage expands the full aircraft into one concrete internal object containing:
- resolved fuselage modules in order
- resolved lifting-surface sets
- battery, payload, ballast, and motor masses
- motor/prop summary rows
- drawing geometry for the orthographic view
- derived geometry such as total length, areas, MAC, CG, and motor count

The important point is that the GUI is modular first, aerodynamic second. The aerodynamic calculations operate on the assembled aircraft, not on vague placeholders.

## Analysis Philosophy

The tool deliberately does not jump straight to VLM.

For early design work, most questions are not:
- exact drag at every condition
- high-fidelity trim
- interference-corrected lift distribution

They are:
- did I place the masses in a sane place?
- did I put enough surface area far enough aft?
- is this shape obviously unstable?
- if I change incidence or move the wing, which way does the pitch tendency move?
- if I add wings, am I starting to meaningfully reduce reliance on pure thrust support?

So the analysis is intentionally staged:

1. Build the full modular aircraft.
2. Run a Barrowman-like quick stability estimate.
3. Run a simple linear AoA-based aerodynamic pass.
4. Stop there, unless and until a more detailed aerodynamic model is warranted.

That keeps the tool fast enough to use interactively while still being grounded in the assembled geometry.

## Stage 1: Full Modular Assembly

Before any aero math, the code resolves the whole aircraft into a concrete assembled representation.

This stage computes:
- fuselage length
- body start/end
- module positions along x
- total mass and total moment
- CG location
- wing and tail surface sets
- planform areas
- main-wing MAC
- battery summary
- static motor summary and static thrust-to-weight

This matters because every later aerodynamic estimate depends on the assembled geometry being real:
- surface x-locations come from the actual module stack
- static margin depends on the actual CG, not on guessed placement
- motor and ballast masses can materially move the CG

## Stage 2: Barrowman-Like Quick Stability Analysis

This is the first aerodynamic pass. It exists to answer the OpenRocket-style question quickly:

“Is this layout statically stable, marginal, or unstable?”

### What It Uses

The current implementation in [`analysis.py`](analysis.py) does a Barrowman-like component buildup:
- a tangent-ogive-like nose contribution
- one fin-set contribution per lifting-surface set
- a weighted combined CP from component `CNa` values

For each surface set, it uses:
- fin count
- semispan
- root chord
- tip chord
- sweep
- body radius
- x-location of the root leading edge

From that it computes:
- component `CNa`
- component `x_cp`
- total `x_cp_total`
- static margin in meters
- static margin as `% MAC`
- verdict: `unstable`, `marginal`, or `stable`

### Why This Exists

This is the fast stability screen.

It is cheap, immediate, and geometry-driven. It is exactly the kind of estimate you want while moving surfaces and masses around in a GUI.

It is not pretending to be a full airplane solver. It is answering a narrower question:
- where is the combined aerodynamic center relative to the CG?

Only after that does it make sense to spend effort on deeper aerodynamic analysis.

### Why It Is “Barrowman-Like” and Not Full OpenRocket

The logic is inspired by the same kind of quick rocket-style normal-force buildup, but it is applied to the assembled modular aircraft geometry already present in this project.

That makes it useful for early screening, but it also means you should read it correctly:
- it is a first-pass CP/static-margin estimate
- it is not a trim solution
- it is not a nonlinear wing-body-tail interaction model
- it does not know about stall or downwash

In short: it tells you whether the shape looks sane before you spend time on anything more expensive.

## Stage 3: Linear Aero Analysis After Barrowman

After the quick CP/static-margin pass, the tool runs a second simplified aerodynamic stage.

This stage answers a different question:

“At a chosen AoA, does the simplified model tend nose-up or nose-down?”

### What It Computes

For each surface set, the current code computes:
- finite-wing lift slope
- mount-angle factor
- area ratio relative to the chosen reference area
- aerodynamic center at quarter-MAC of the trapezoid
- effective incidence from surface incidence plus evaluated AoA

It also includes a crude body normal-force term based on:
- nose `CNa`
- body reference area ratio

From that it sums:
- total `CN`
- total `Cm`
- lift-center x-location
- AoA verdict: `nose-up`, `nose-down`, or `trim`

### Why This Stage Exists

Static margin alone is not enough once you start asking:
- what happens if I change wing incidence?
- what happens if I evaluate 0°, 5°, or 10° AoA?
- is the current setup tending toward nose-up or restoring nose-down behavior?
- if I am brute-forcing the transition from vertical-thrust support into forward lift support, is the added wing area starting to matter in a useful way?

The linear pass is there to give immediate directional insight without reaching for VLM.

It is still fast enough to recompute interactively when the user edits the assembly.

### Why It Is Questionable

This stage is intentionally approximate and the code already treats it that way.

Its limitations are substantial:
- linear small-angle behavior
- no stall
- no downwash model
- no wake interaction
- no thrust effects in pitch
- no propwash coupling
- no nonlinear body lift
- no wing-body interference correction
- no real trim solve

So this stage is useful for trend and sign, not for claiming flight-quality aerodynamic truth.

Read it as:
- “which way does the simplified model push?”

Not as:
- “this is the final airplane answer.”

## Why No VLM Here

The point of `guibuilder` is interactive configuration work.

VLM is valuable later, but it is the wrong first tool for:
- dragging module order around
- shifting ballast
- changing incidence
- swapping a wing set
- checking whether a layout is obviously unstable

This tool exists so you can do those fast iterations first, then move to a higher-fidelity method only when the geometry is worth that effort.

The intended escalation is:
- modular layout first
- quick stability screen second
- simple lift/pitch trend check third
- targeted VLM, AeroSandbox, or airfoil work only when the design has clearly entered that regime

In other words:
- `guibuilder` is for configuration and early aerodynamic screening
- VLM or more detailed aero analysis belongs after this stage, not before it

## Current Output Scope

The summary pane currently emphasizes:
- geometry and mass properties
- battery and motor static information
- static thrust-to-weight
- Barrowman-like CP/static margin
- linear `CN` / `Cm` / lift-center behavior at the chosen AoA

The center view shows:
- orthographic geometry
- `CG` marker in blue
- `CP` marker in red
- `CL@AoA` marker in green when the linear pass yields one

## Practical Interpretation

Use the results in this order:

1. Check the assembled geometry and masses.
2. Check CG.
3. Check static margin and stability verdict.
4. Check the linear AoA `Cm` sign and lift-center movement.
5. Only then decide whether the configuration deserves a more detailed aerodynamic pass.

That is the intended workflow of this program.
