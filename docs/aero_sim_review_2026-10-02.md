# Aerodynamics, Flight Simulation & Modeling Review

- **Date:** 2026-10-02
- **Repo:** quadrocketbuilder (working tree, including uncommitted changes)
- **Scope:** aerodynamics, propulsion, flight-simulation export, and shared geometry. GUI excluded by request.
- **Method:** three independent fresh-context engineering reviews (aero core; propulsion; JSBSim/geometry export) plus foreman verification of the severity-1 findings against the AeroSandbox source, the exported XML, and direct JSBSim FDM runs.

Environment notes: the project runs in the conda env `drone` (AeroSandbox 4.2.8, NeuralFoil 0.3.2, JSBSim FDM v1.3.1 Python module). The default interpreter lacks aerosandbox.

---

## Severity 1 — Glaring faults that invalidate results today

### S1-1. Motor bells produce physically impossible drag; ~65% of total cruise drag is an artifact

- **Where:** `libs/aero.py:169-175` (bell modeled as a two-station `asb.Fuselage`), `libs/airframe.py:510-517` (`bench_motor_specs` fills `bell_length_m` from the motordata.csv **"Stator height"** column).
- **What:** For the A2807 the "Stator height" is 7 mm while the real rotating bell is ~25 mm long. The modeled bell is a 28 mm × 7 mm cylinder: fineness ratio 0.25. AeroBuildup's fuselage form-factor fit (Götten et al. 2021, `fuselage_form_factor` in `aero_buildup_submodels/fuselage_aerodynamics_utilities.py`) returns **121** at fr = 0.25 (cs1 = 3.174, cs2 = −2.620, cs3 = 1.096), giving an effective skin-friction coefficient of 0.01496 × 121 ≈ **1.81**.
- **Measured:** each bell drags **0.297–0.298 N** at 20 m/s — more than a flat 28 mm disc facing the wind (0.166 N at Cd 1.1) and ~5–8× a realistic motor envelope (skin ~0.03–0.05 N + base ~0.02 N). Four bells = **1.19 N of the 1.566 N** "body + pods" drag, i.e. **65% of the m62v1's 1.818 N total cruise drag**.
- **Impact:** reported CD ≈ 0.206 where physics says ~0.11–0.13; L/D at the operating point reads 1.46 instead of ~2.3. Every drag-derived quantity inherits this: cruise power, endurance, the JSBSim drag tables, and all SITL results.
- **Fix direction:** model the real motor envelope (bell ~28 × 25 mm), or exclude bells from the fuselage drag model and use a blunt-body Cd × frontal area. Do not feed stator height to a form-factor fit.
- **Verification:** independently reproduced (form-factor formula evaluated; single-bell AeroBuildup run gives fr = 0.25, drag = 0.297 N; flat-disc comparison 0.166 N).
- **Status: fixed 2026-10-02.** `build_asb_airplane` makes each bell part of the body it is mounted on (its pod, else the fuselage end): one extra section at the bell's prop-side face, no isolated bell body. m62v1 at 5°, 20 m/s: bodies 1.566 → 0.298 N, total drag 1.818 → 0.551 N; body lift 0.319 → 0.155 N, so static margins moved too (m62v1 18.1 → 6.3 mm, m62v2 −4.9 → −19.1 mm, quadlancer62 −5.6 → −8.3 mm, quad_monoplane 12.2 → 9.0 mm, m62_canards 19.4 → 1.0 mm). Real bell dimensions are still not in the data (the CSV has stator size only).

---

## Severity 2 — Real modeling mistakes that materially affect accuracy

### S2-1. `carried_to_axis` inflates wing lift far beyond the classical body-carry effect, with the magnitude undisclosed

- **Where:** `libs/aero.py:212-223`.
- **What:** body-rooted panels get a full-root-chord strip appended from the root to the span line's closest approach to the fuselage axis. On the m62v1 X-wing, four strips converge on the same axis point (four coincident inner sections at x = 0.245).
- **Measured at 5°, 20 m/s:** qrwing lift **2.281 N with carry vs 0.998 N without (+129%)**; simpleplane main wing 11.162 vs 8.358 N (+34%). The classical Barrowman image enhancement (1+R/(s+R)) predicts ~+9–31%. The strips lie entirely inside the blocked tube region yet generate lift and profile drag there, and AeroBuildup additionally credits the body its own Jorgensen lift (0.319 N at 5°) — the blocked region is effectively counted twice.
- **Context:** carrying panels through the body is standard wing-body practice (aero_check's own "VLM thru" agrees: 11.387 vs 11.126 N), so the method is defensible — but ~40% of the wing's solved area is inside the body silhouette and this is nowhere surfaced.
- **Fix direction:** keep the carry but disclose the carry area/lift contribution, or use the (1+R/(s+R))-scaled exposed-panel model for lift while carrying only for induced-field consistency.
- **Status: open 2026-10-02.** Numbers reproduced exactly. Not established as a bug: the "exposed only" baseline is two stub panels with free roots and no body wall (root losses a real body prevents), so it is not the reference the classical +9–31 % applies to. Needs a like-for-like comparison before any change to the live model.

### S2-2. The "equal to asb.LiftingLine" claim holds only modulo a 1 mm vortex core that moves lift 17%

- **Where:** `libs/aero.py:36,320` (`LIFTING_LINE_CORE_RADIUS_M = 1e-3`), `aero_check.py:67`.
- **What:** stock `asb.LiftingLine` defaults to `vortex_core_radius = 1e-8` (five orders smaller). The core exists to regularize the constructed four-strips-meet-at-axis geometry.
- **Measured (m62v1 qrwing, 5°):** lift 2.2815 N at 1e-3 vs 1.9434 N at 1e-8 (17%); drag 0.203 vs 0.222. `aero_check.py` passes the *same modified radius* to the reference `asb.LiftingLine`, so the check proves only that the batched code reproduces asb *with this modification* — it is blind to the deviation from stock semantics that the module docstring implies.
- **Fix direction:** verify against stock 1e-8 (handling the meeting-point singularity differently, e.g. not carrying four strips to one point), or document that the reference model is asb.LiftingLine-with-a-1-mm-core and show the sensitivity.
- **Status: open 2026-10-02.** Sensitivity reproduced (m62v1 wing 2.281 → 1.943 N at 1e-8; simpleplane main wing 11.162 → 10.979 N). Decided together with S2-1.
- **Status: fixed 2026-10-15.** Core back to stock 1e-8. Mirrored halves are carried to the XZ plane (ending where the span line nears the axis overlapped them). Between surface sets, trailing lines are smoothed over 0.25 × the shedding strip width (`WAKE_CORE_FRACTION`). The 1 mm core read 20–45 % above `asb.VortexLatticeMethod` on four-fin rockets and made swagger's solve near-singular. `aero_check.py` compares each surface set alone to stock `asb.LiftingLine`.

### S2-3. simlib2 silently synthesizes data below the lowest measured row and presents it as measured

- **Where:** `libs/simlib2.py:149-156` (synthetic 0% anchor: rpm = 0, I = 0, V = first measured V), `:166-173` (`fill_value="extrapolate"` interpolators), `:202-208` (`elec_at_rpm` extrapolation flag).
- **What:** the extrapolation flag only fires for `rpm > max_rpm × 1.05`, so everything below the lowest real bench row passes as measured. Verified: `from_throttle("GF7035", 20)` → 408 g / 5.0 A / 123 W with `is_extrapolated = False` though the lowest measured row is 40% (817 g). Worse, `from_throttle(120)` → 2224 g / 45.3 A / 1074 W, also unflagged (21770 rpm < 1.05 × 20874). The class docstring (`:7-9`) claims "electrical data … is always from measured interpolation, never synthesised" — contradicted by both the 0%-anchor and `_airspeed_current` (`:362-372`), which scales static current by a thrust ratio with a hard 5% floor.
- **Impact:** hover lives in exactly this region, so hover thrust/current/power estimates are interpolation between a synthetic zero and the 40% row, labeled as measurement. This violates the tool's stated "grounded entirely in measured data" philosophy.

### S2-4. No outlier/data validation; the CSV demonstrably contains corrupted rows that flow straight into the fits

- **Where:** `libs/propulsion.py:46-57` (`fit_motor`: Ct₀ averaged over all rows; R and Cp₀ pinned to the *last* row), `libs/simlib2.py:139-200` (`_PropData`/`_fit_k` never check monotonicity).
- **What (verified in `libs/motordata.csv`):**
  - Flashobby A4214 GF1308 6S thrust column is corrupted: 7981 g at 50% → **1167 g at 60%** → 1562 g at 70% (dropped digits; should be ~11670/~15620). `available_motors()` offers A4214 in the picker and `fit_motor` would silently produce a Ct₀ off by ~7× with no warning.
    - **Status: fixed 2026-10-02.** The corruption is the other way round: the 60–100 % rows are right, the rows under ~1000 g lost their decimal point (27.9 → "279", 512.8 → "5128"; the CSV's own efficiency × W gives the true value). 27 rows, all A4214 (GF1308, GF1310, HQ1309 at 6S and 8S): 25 restored by the decimal, lines 261 and 312 from efficiency × W (their digits do not fit the decimal rule). Every row now within 11 % of efficiency × W.
  - A2807 GF7042X2: thrust drops 1948 g (90%) → 1836 g (100%) while RPM and current rise.
  - GEPRC EM2810 (1280 and 1350 KV): empty RPM fields → raw `ValueError: could not convert string to float: ''` (`simlib2.py:141`), not a clean "no data" message; the motor is listed but unusable.

### S2-5. Endurance figures are ideal-battery upper bounds reported as predictions

- **Where:** `libs/propulsion.py:100,157-158` (`pack_wh = mah/1000 × cells × 3.7 × p`; `hover_time = pack_wh / power × 60`), shown to one decimal in `summary_lines` (`:182`).
- **What:** no usable-capacity factor (real ≈80%), no end-of-discharge voltage sag, and `battery["c"]` (C-rating, present in parts.json) is never read. Verified case (m62v1 single-motor tailsitter, A2807 GF7035, lipo-6S-2200): full throttle = 37.9 A / 908 W from a 48.8 Wh nominal pack → 3.2 min claimed, but at 17C load with sag and cutoff the real figure is 20–30% lower. The model will happily report full throttle on a pack it cannot support.

### S2-6. Hover operates below the measured range, where the fit is dominated by an implausible lumped constant — unlabeled

- **Where:** the fit at `libs/propulsion.py:57`; JSBSim write at `libs/jsbsim.py:375`.
- **What:** a 2000 g VTOL (4 motors, 500 g/motor on GF7035) hovers at 10393 rpm — *below* the lowest bench row (13409 rpm). At that rpm, **61% of the predicted current is the fitted `no_load_current_a` = 3.137 A** — not physically plausible as a no-load current for a 2807 motor (~0.8 A typical; the Axisflying 2810 fit gives 2.36 A, A4214 1.65 A). It is a lumped residual absorbing Cp-shape/ESC error. The pair (Cp₀, I₀) reproduces the bench range well, but below it the hover power inherits all the lumped error, and neither the GUI nor the report marks hover as extrapolated. The same inflated 3.137 A is written to JSBSim as `<noloadcurrent>`, so SITL draws ~3 A/motor at any nonzero RPM.

### S2-7. The throttle→duty remapping shifts the whole low-throttle axis by up to ~46%, anchored on evidence not in the repo

- **Where:** `libs/propulsion.py:144-159` (`propulsion_summary` ignores the bench's throttle% column), `:115` (`static_points`: duty = motor voltage ÷ full-throttle-loaded pack voltage), single-row R extraction at `:47`.
- **What:** the bench's 40% row maps to model duty **0.583** (+46% vs the label); 80% → 0.836. The docstring (`:8-10`) states the bench's own throttle% does not fit and that a single flight log (2025-08-22) fits INAV output-as-duty — **that log is not in the repository**, and the duty axis is reconstructed, not measured. If that anchor is wrong, every hover duty, thrust-vs-stick curve, and hover stick position is off by tens of percent. Disclosed and plausibly correct (ESCs do apply throttle curves), but it is the model's single largest calibration assumption and it is unverifiable.

---

## Severity 3 — Suspicious or latent, possibly intentional

### S3-1. VLM view and Configuration view disagree by 2.5× at the same operating point

- **Where:** `run_vlm_sweep` (`libs/aero.py:542-588`) uses `build_asb_airplane` **without** `carried_to_axis`; `analyze_aero` uses the carried wings.
- **What (m62v1, 5°, 20 m/s):** VLM CL = 0.120 vs Configuration CL = 0.302; NP 0.349 m vs 0.274 m; static margin 100% c_ref vs 19.4%. The report header discloses the direction ("body-mounted wings read low") but not that the gap is a factor of 2.5, and the two views' stability numbers are computed on different geometries, so they are not comparable at all.

### S3-2. The Barrowman "cross-check" is applied to configurations its fin theory never covered; its apparent agreement is coincidental

- **Where:** `barrowman_finset` (`libs/aero.py:103-126`), `barrowman()` with `NOSE_CP_FRACTION = 0.466` applied to all nose profiles.
- **What:** the textbook 4N-fin formula is valid for low-AR rocket fins. For the simpleplane's AR 8.6 main wing it yields CNα ≈ 87.7/rad² (real value ~4.9/rad) — 20× out of range — yet the Barrowman CP (0.372 m) lands within 1 cm of the lifting-line NP (0.365 m) because the wing's cna so dominates the sum that CP ≈ wing CP ≈ wing AC. With a canard they diverge 2.6 cm (0.345 vs 0.3185). No clamping or blending found — the values are reported honestly — but the cross-check has almost no validating power for these configurations and its near-agreement on simpleplane/m62v1 could easily be misread as model agreement.

### S3-4. Two mutually inconsistent thrust-vs-airspeed laws exist for the same prop

- **Where:** JSBSim export `ct_table = ct0 × (1 − J/(p/D))` (`libs/jsbsim.py:363`); simlib2 Tyto model (`_tyto_thrust_g`, `libs/simlib2.py:85-90`).
- **What:** the JSBSim law forces thrust to zero exactly at J = p/D = 0.5, while real props of p/D 0.5 cross zero at J ≈ 0.6–0.7 — cruise thrust under-predicted by roughly a third. The Tyto model gives thrust −49% at 15 m/s and exactly zero at pitch speed (verified: 2036 g static → 1040 g at 15 m/s, 0 g at 30.9 m/s) — too steep a decay. Neither agrees with the other; the simlib2 airspeed API has zero callers outside its own `main()`. Both are documented approximations, but the tool ships two contradictory answers to "thrust at cruise".

### S3-5. `rho_at_altitude` has a real thermodynamic bug (dead code)

- **Where:** `libs/simlib2.py:118-124`.
- **What:** pressure is computed with `T_alt = T0 − L·alt` but density divides by ground temperature `273.15 + temp_c` instead of `T_alt`. Verified vs ISA: −6.8% at 3 km, −11.3% at 5 km, −18% at 8 km; `temp_c` never enters the lapse formula. No callers anywhere, so latent — but if ever wired in it silently under-predicts density with altitude.
- **Status: fixed 2026-10-02.** Function removed (no callers).

### S3-6. Battery side uses the full-throttle sagged pack voltage at all duties

- **Where:** `libs/propulsion.py:113-116` (`static_points`).
- **What:** battery current = power / `pack_voltage_v`, fixed at 23.94 V (full-throttle sagged). At hover the real pack sits near 24.5–25 V, so hover current/power are overstated ~2–4%; the bench's own intermediate voltage readings (24.74 → 23.94 V) are used in the fit but discarded for output. Additionally, `export_jsbsim` (`libs/jsbsim.py:402-404`) allows a `pack_voltage_v` override that silently re-drives a 6S-fitted R/I₀/Cp₀ at a different cell count with no warning.

### S3-7. `batterydata.csv` is dead data; li-ion entries in parts.json would crash if selected

- **Where:** no code reads `libs/batterydata.csv` (capacity, C-rating, IR — there is no IR column at all — unused). `motor_model` (`libs/propulsion.py:92-93,100`) reads `battery["mah"]` directly, but every `liion-*` entry in `data/parts.json` has only `s`, `p`, and `cell` → selecting one raises a raw `KeyError`. The promised "battery cell data → sag/IR" path does not exist; sag enters only through the bench's measured top-row voltage.

### S3-8. Hardcoded physical constants without local provenance

- **Where:** `BENCH_AIR_DENSITY_KG_M3 = 1.225` (`libs/propulsion.py:23`) — admitted ("bench conditions were not recorded"), so all Ct/Cp are ISA-sea-level fits used regardless of actual bench air density (a 2% density error scales thrust/power directly). `TYTO_A/TYTO_B` (`libs/simlib2.py:43-44`) are cited to the Tyto blog and compensated per-prop by the honest median `k_corr` (`:186-200`). The JSBSim Cp(J) shape is borrowed from an APC 9×4.5E and J-rescaled (`libs/jsbsim.py:361-364`) — disclosed in the generated XML comment, but Cp(J) shape genuinely varies with p/D.

---

## Severity 4 — Minor

- `libs/aero.py:43` — comment says "tangent ogive" but the 0.466 CP fraction is applied to whatever nose profile the airframe declares (elliptical for m62v1). Value is right for both; comment is misleading.
- `libs/aero.py:579` — `drag_induced_n = abs(cdi)` hides the sign of induced drag in the report.
- `libs/aero.py:550` — `weight_n = sum(resolved.mass_kg) / 2.0 * g` is correct (mass_kg is a (min, max) tuple, so sum/2 = nominal) but reads like a halving fudge; worth a comment.
- `libs/propulsion.py:26,123-125` — silent duty saturation: the RPM grid caps at 1.5 × max bench RPM and `np.interp` clamps beyond it; duty ≳ 1.3 silently returns the grid-max RPM (e.g. 31311 rpm for the A2807) with no error or flag.
- `libs/simlib2.py:145,171,379` — the CSV's W column is parsed (`rpm2w`) but `OpPoint.power_w` is recomputed as V × I; W disagrees with V×A by up to 25% at the lowest published rows (rounding) and is dead weight that could have caught corrupted rows.
- `libs/geometry.py:411` (`panel_strips`) — chord midpoint uses `chord/2` without `cos(incidence)`: sub-mm error in surface-mass CG. **Fixed 2026-10-02:** mid-chord along the chord line (incidence and panel normal), as the panel outline.
- `libs/geometry.py:480` (`shell_between`) — shell area uses `hypot(dx, dw)` but ignores `dh`: tiny mass error on vertically tapered sections. **Fixed 2026-10-02:** slope from the mean of the width and height taper.
- `libs/jsbsim.py:154-157` (`contact_points`) — the tailsitter's wingtip contacts (x ≈ 0.35) sit above the tail contact (x = 0.522), so the m62v1 actually rests on a single tail point; harmless because the hover test starts airborne just above stance height and releases.

---

## Verified clean — what did NOT have problems

- **Lifting-line engine is genuine.** `lifting_line_sweep` (`libs/aero.py:336-398`) was compared term-by-term against `asb.LiftingLine.wing_aerodynamics` (installed AeroSandbox 4.2.8): identical alpha_geometric formula, 2π slope linearization, influence matrix, KJ force + profile drag + Cm couple (including the bound-leg x-component zeroing), Mach/Re with sweep, and control-surface handling. `aero_check.py` confirms agreement to ~1e-14, including symmetric and anti-symmetric ±10° deflections.
- **Signs and units are physically consistent** in the live analysis path: CL positive up, Cm positive nose-up (verified against hand checks on a flat-wing test case); units consistent (m, m², N, Pa); the report's CL/CD/weight-% arithmetic checks out independently.
- **No silent clamps, fallback constants, blended polars, or massaged cross-checks** in the aero core. The ±30° AoA cap, the windowed NP fit, and the NeuralFoil "large" model choice are all disclosed. `airfoil_analysis.py` uses NeuralFoil honestly: correct Re/Mach from a deviated standard atmosphere, mm→m conversion, ±20° sweeps, confidence filtering applied only to summary features, no fake polars.
- **The VLM sweep is the real `asb.VortexLatticeMethod` path** (wings-only, as its header states), solved per AoA with q-scaling for speed/altitude rows.
- **Mass/geometry in `libs/geometry.py` is honest:** no CG or inertia nudges; parallel-axis theorem in `combine_inertia` is correct (shift and product signs right); tensor entries are the standard −Σ m a b; inertias are about the CG in kg·m²; `rotor_inertia` (R²/3, R²/6, R²/6), `ring_inertia`, `cylinder_inertia`, and the bore volume inertia are all correct. Mass points use weighed values where given and disclose estimates; the CG-range enumeration is sound.
- **`libs/report.py` is a pure formatter** of already-computed values — no recomputation, no cosmetic faking.
- **Cross-module consistency is good:** jsbsim consumes the same `resolved` object (CG, reference area/span/chord, motor placements) that aero uses; `xyz_ref = resolved.cg_m`, so the moment reference in the tables matches JSBSim's AERORP = CG. Wing area/span in the XML match `geometry.reference` exactly (0.036 m² / 0.185 m).
- **The SITL harness has no cheating:** no state-teleporting, no forced/held velocity, no integrator resets, no secret throttle — it drives sticks over the bench like a scripted pilot. The "cruise sweep stops on sinking" logic (`sim_inav.py:325-327`) is a legitimate minimum-level-power boundary, not a mask. `battery_power_w` is a separate, disclosed lossless-ESC electrical model consistent with the motor fit.
- **Propulsion fit reproduces the bench honestly within range:** the 4-parameter structured model (Ct₀ mean, effective R, Cp₀, no-load current) reproduces the bench rows to within 4–7% worst relative battery-current misfit; full throttle reproduces the bench top row exactly by construction (2017 g vs 2036 g measured). Units are clean (thrust_g→N via 9.80665; inches→m; Ct = T/(ρn²D⁴), Cp = P/(ρn³D⁵) with n in rps — no 1000×/9.81× slips).

---

## Appendix — verification evidence

**Aero core (independent review):**
- `aero_check.py` run in env `drone` (asb 4.2.8, NeuralFoil 0.3.2): all default airframes at 0/5/10°, max diff to `asb.LiftingLine` 8.9e-15; deflected-control cases 1.8e-15. Exit 0.
- `analyze_airframe.py` on m62v1, simpleplane, m62_canards: outputs numerically sane and independently re-derivable (m62v1 CL 0.3019 = 2.663 N / (245 Pa × 0.036 m²); simpleplane CL 0.78 explained by 3° incidence + naca2412 α0 ≈ −2.1° + AR 8.6 + carry area). One layout note (battery 0.5 mm into wall).
- `run_vlm_sweep` headless on m62v1 (−5…10°): real VLM path executes and reports.
- Instrumented experiments: per-fuselage AeroBuildup decomposition (bells 0.2984 N each), carry vs no-carry lifting-line (2.281 vs 0.998 N; 11.162 vs 8.358 N), core radius 1e-3 vs 1e-8 (2.2815 vs 1.9434 N), flat-wing asb moment sign checks.

**Propulsion (independent review):**
- Re-implemented `fit_motor`/`static_points`/`rpm_for_duty` verbatim and drove them with simlib2's real CSV parsing. A2807/GF7035 fit: R = 0.208 Ω, I₀ = 3.137 A, ct0 = 0.1335, cp0 = 0.0609, worst battery-current misfit 4.0%; full throttle: 2017 g vs 2036 g bench, 37.91 A, 908 W, 20874 rpm. Duty sweep 0→1 smooth, monotone, no negative thrust/power; thrust(0) = 0.
- simlib2 API exercise: `from_throttle` at 10/20/30/40/100/120% and airspeed sweeps; `from_thrust(500 g)` at v = 0/15/30 (v = 30 demands 24628 rpm — 1.18× beyond measured — correctly flagged `extrap = True`).
- CSV audits: 68 motor+prop combos, no duplicate throttle rows; non-monotonic/corrupted rows found (A4214 thrust digits, A2807 GF7042X2 top row); GEPRC EM2810 empty-RPM crash reproduced; W ≈ V×A to rounding.
- `rho_at_altitude` vs a correct ISA implementation at 0–8 km.

**Sim export (independent review):**
- `python analyze_airframe.py aircraft/m62v1.json` in env `drone` exits 0 in ~2 min and prints the full report (m62v1: 1039.4 g, CG x 0.2554 m, CL/CD/Cm 0.302/0.206/−0.080 at AoA 5°/20 m/s, lift 2.66 N = 26.1% weight, static margin 18.1 mm "stable").
- Prior SITL logs (`exports/jsbsim/m62v1/sim_inav_*.csv`): hover holds at pitch 90°, agl ~3.6 m, motors ~1294 µs; cruise at pitch +10…+25°, 20–34 m/s; dash at +5…9°, 41.9 m/s. The bench (`inavbench/vehicles.py:246`) reports truth pitch = JSBSim theta and maps JSBSim +x to INAV's forward.

**Foreman verification:**
- Fuselage form-factor formula evaluated: fr = 0.25 → cs1 = 3.174, cs2 = −2.620, cs3 = 1.096 → ff ≈ 121; single-bell AeroBuildup run: fr = 0.25, drag = 0.297 N; flat 28 mm disc at Cd 1.1 = 0.166 N.

**Known validation gap:** the speed-vs-power validation (TODO.md: bias +0.7 km/h, RMS 6.8 km/h vs the real 2025-08-22 flight) validates only the drag/propulsion path. Nothing validates lift, attitude, stability, or control response in the SITL against real flight; and the "2025-08-22 log" that anchors the throttle→duty remap is not in the repository.
