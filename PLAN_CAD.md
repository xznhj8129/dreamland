# Plan: STEP Export from Generated Airframe Geometry

## Goal

Export the assembled airframe as a watertight STEP file suitable for CFD meshing and FreeCAD import. The geometry source is the `analysis` dict produced by `recompute_analysis()`, which already contains fully resolved 3D positions and dimensions for every component.

## Current State

- `libs/cad.py` exports FreeCAD property dicts (parameter spreadsheet values), not solid geometry.
- `libs/views/cad_view.py` is a placeholder.
- `build_asb_airplane()` in `libs/analysis.py` already walks the full geometry tree (fuselage xsecs, wing panels, motor bells, pods) and builds AeroSandbox objects. This is the closest thing to a "geometry assembly" pass and is the reference implementation for traversal order.
- All geometry lives in `analysis`: `module_rows`, `surface_sets` (with `panels`), `motor_draw_rows`, and `internal_draw_rows`.
- Airfoil profiles come from AeroSandbox (`asb.Airfoil(name).coordinates`), giving (x, y) normalized point arrays.
- Nose/tail shapes come from `sample_pointed_profile()` which returns (x, radius) pairs.

## Library Choice: CadQuery (or build123d)

**CadQuery** is the most practical choice:
- Pure Python, wraps OpenCascade (OCP) via cadquery-ocp.
- Native STEP export (`cq.exporters.export(shape, "out.step")`).
- Lofting, revolving, extruding, and boolean ops are first-class.
- `pip install cadquery` (or `pip install build123d` for the newer API on the same kernel).
We use CadQuery.

## Geometry Components to Export

### 1. Fuselage (body of revolution with varying profile)

**Source**: `analysis["module_rows"]` where `module_type` in `{body, end, coupler, wingmount}`

**Approach**: Walk modules front-to-back, collecting (x, radius) stations:
- `end` modules (nose/tail): sample via `sample_pointed_profile()` to get the taper curve.
- `body`/`coupler`/`wingmount` modules: constant radius = `tube_diam / 2`.

Loft circular cross-sections at each station to produce the fuselage solid. This is a surface of revolution, so `revolve()` on the profile spline is the most robust path:
1. Build a 2D spline through all (x, radius) points.
2. Close with the axis (radius=0 line).
3. Revolve 360 degrees around X.

This gives a single watertight solid for the entire fuselage OML.

### 2. Wings and Fins (lofted airfoil sections)

**Source**: `analysis["surface_sets"]`, each containing `panels[]` with root/tip 3D coordinates.

**Per panel**:
1. Get the airfoil (x, y) coordinates from `asb.Airfoil(name).coordinates` (normalized 0-1).
2. Scale root profile by `chord_root`, tip profile by `chord_tip`.
3. Apply twist (AoA incidence) rotation around the leading edge.
4. Translate root profile to `(x_root_le, root_y, root_z)`.
5. Translate tip profile to `(x_root_le + dx_sweep, tip_y, tip_z)`, where `dx_sweep = span * tan(sweep_angle)`.
6. Loft between root and tip wire profiles to create the wing solid.

If the airfoil has zero thickness (flat plate), extrude a thin slab instead.

### 3. Motor Bells (cylinders)

**Source**: `analysis["motor_draw_rows"]`

Each motor is a cylinder:
- Center at `(x_m, y_m, z_m)`.
- Diameter = `motor_bell_diam_m`, length = `motor_bell_len_m`.
- Oriented along X axis.

Simple `Workplane.cylinder()` at each motor position.

### 4. Motor Pods (revolved taper + cylinder)

**Source**: `motor_draw_rows` entries with `pod_len_flat_m > 0` or `pod_len_curve_m > 0`.

Same approach as fuselage: build an (x, radius) profile for the pod (flat section at `pod_radius`, then `sample_pointed_profile()` for the curved taper), revolve around the pod axis. Translate to motor position.

### 5. Propeller Discs (optional, for CFD domain marking)

Thin disc at propeller plane, diameter = `prop_diam_m`. Useful as a boundary marker for actuator disc CFD setups. Optional -- can be a separate export flag.

## Assembly Strategy

Two export modes:

### A. Fused OML (for CFD)
Boolean-union all solids into a single watertight shell. This is what a CFD mesher needs. Run `fuse()` across all components. Export as single STEP compound.

### B. Component Assembly (for CAD)
Keep each component as a named solid in a STEP compound. FreeCAD imports these as separate Part objects, preserving the design tree. No booleans needed.

## Implementation Phases

### Phase 1: Core geometry builder in `libs/cad.py`

Add functions that take the `analysis` dict and return CadQuery solids:

```
build_fuselage_solid(analysis) -> cq.Workplane
build_wing_solid(surface, panel) -> cq.Workplane
build_motor_solid(motor_row) -> cq.Workplane
build_pod_solid(motor_row) -> cq.Workplane
build_airframe_assembly(analysis) -> cq.Assembly
```

Each function is independent and testable. `build_airframe_assembly` composes them.

**Files touched**: `libs/cad.py`

### Phase 2: STEP export function + headless CLI

Add to `libs/cad.py`:
```
export_step(analysis, path, mode="assembly")  # mode = "assembly" | "fused"
```

Add `--export-step` flag to `analyze_headless.py` so the full pipeline is:
```
python analyze_headless.py --config myplane.json --export-step output.step
```

**Files touched**: `libs/cad.py`, `analyze_headless.py`

### Phase 3: CAD view integration

Wire up `libs/views/cad_view.py` with:
- "Export STEP" button that calls `export_step()` with a file dialog.
- Assembly/Fused mode toggle.
- Optional: 3D preview using CadQuery's `show()` or an embedded OCC viewer (stretch goal, not required for export).

**Files touched**: `libs/views/cad_view.py`, `guibuilder.py` (if wiring needed)

### Phase 4: STL convenience export (trivial once STEP works)

CadQuery/OCP can export STL directly from the same solid:
```
cq.exporters.export(shape, "out.stl")
```

Add `--export-stl` to the CLI. This is nearly free once Phase 2 is done.

**Files touched**: `libs/cad.py`, `analyze_headless.py`

## Key Decisions and Risks

| Decision | Rationale |
|---|---|
| CadQuery over FreeCAD scripting | No GUI dependency, cleaner API, same OCC kernel. FreeCAD can import the resulting STEP. |
| Revolve over loft for fuselage | Fuselage is axisymmetric. Revolve is exact and avoids loft artifacts at the nose tip. |
| Loft for wings | Airfoil profiles are arbitrary point sets. Loft between two wire profiles is the standard approach. |
| Per-panel solids, not mirrored | The codebase already expands symmetric surfaces into individual panels with explicit coordinates. No need to use CadQuery's `mirror()`. |

| Risk | Mitigation |
|---|---|
| CadQuery install pulls heavy OCP wheel (~200MB) | One-time cost. No alternative for programmatic STEP without FreeCAD. |
| Airfoil spline closure at trailing edge | AeroSandbox airfoils are already closed loops. Verify by checking first == last point; if not, close explicitly. |
| Nose tip singularity (radius=0) | Start the revolve profile at a small epsilon radius or use a capping operation. `sample_pointed_profile` already handles this. |
| Boolean fuse failures on tangent surfaces | Only needed for CFD fused mode. Can fall back to STEP compound (unfused) if booleans fail on edge cases. |

## Dependencies

```
pip install cadquery
# or
pip install build123d
```

Both pull `cadquery-ocp` (the OpenCascade Python bindings). No other new dependencies.

## Verification

1. Export default config to STEP, open in FreeCAD -- all components present and positioned correctly.
2. Export fused mode, import into GMSH or OpenFOAM snappyHexMesh -- mesh generates without holes.
3. Headless round-trip: `analyze_headless.py --export-step out.step` produces valid file with no errors.
4. Wing geometry spot-check: measure chord and span in FreeCAD, compare to analysis output values.
