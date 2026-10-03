"""
Usage:
from libs.jsbsim import export_jsbsim
export = export_jsbsim(resolved, parts_data, Path("exports/jsbsim/m62v1"), "m62v1")

JSBSim model of a resolved airframe (libs/geometry.py), for INAV SITL through inav-jsbsim-bench. INAV does the mixing;
motor i is driven through fcs/throttle-cmd-norm[i], in resolved motor order. Writes:
- <out>/aircraft/<model>/<model>.xml: metrics, mass, contact points, propulsion, throttle curve, aerodynamics
- <out>/engine/<model>_motor.xml, <model>_prop.xml
- <out>/inav_aircraft.cli: INAV mixer settings: a tailsitter gets its hover and forward-flight mixer profiles, a
  fixed-wing (not VTOL) an AIRPLANE profile with the motors on throttle (none when unpowered). Control surfaces get a
  servo mix (smix) from their roles, with each rule's sign from the control's own computed moments (servo_mix). VTOL
  types other than tailsitter are not exported yet (ExportError).

Frames: JSBSim's structural frame is the geometry frame (x aft from the nose, y right, z up, metres). Body x is the nose
and the thrust axis, so hover is pitch +90 deg; INAV flies it as a multirotor with tailsitter_orientation_offset = ON,
whose front is the aircraft belly (-z) and right is +y.

Model choices (best estimates where no data exists):
- Motor: JSBSim brushless DC motor with the constants of libs/propulsion.py (MotorModel): KV, no-load current and a
  resistance (wiring, pack, plugs, ESC and motorlib's loaded winding resistance) that reproduce motorlib's full-throttle
  static point at the nominal pack voltage. One resistance for all currents: motorlib's winding factor grows with
  current, so below full throttle the drop is slightly overstated. INAV's motor output is the ESC duty: motor voltage
  = command x pack voltage. Back-EMF limits the RPM at speed. At zero command the motor sees 0 V, i.e. shorted
  windings (JSBSim's motor model), so the prop brakes instead of windmilling; armed, INAV never commands zero
  (INAV_THROTTLE_IDLE_PCT).
- Prop: static Ct and Cp from motorlib's full-throttle point (constant: JSBSim has no RPM dependence, motorlib's
  thrust and power exponents are ~2 and ~3); Ct(J) and Cp(J) shaped like JSBSim's bundled APC 9x4.5E table, J scaled
  by the pitch ratio over the APC's (zero thrust at J = 1.46 x pitch / diameter). Rotor inertia = prop only.
- Aerodynamics: AeroSandbox AeroBuildup on the whole aircraft over AoA -180..180, sideslip -90..90 (dense near 0) and
  AERO_SPEEDS_M_S (body-axis tables); roll/pitch/yaw damping from AeroBuildup over AoA and airspeed at sideslip 0. It
  has no downwash between surfaces, so its pitch stability differs from the live model's (simpleplane Cm -0.34
  against -0.13 at AoA 0).
- Control surfaces: each control's increments over the undeflected aircraft from the same AeroBuildup (so they stall
  with the base tables) at sideslip 0, the base AoA and airspeed grids, and CONTROL_DEFLECTION_GRID_DEG each way (cut
  at its limits, plus the limits). AeroBuildup skips its finite-wing correction for control surfaces, so each control
  is solved at its deflection x its wing's finite_wing_factor. Increments add up in JSBSim; AeroBuildup's one
  interaction between controls is its aircraft-level induced drag (2 x dL1 x dL2), which they leave out. INAV servo
  output Sn (1000..2000 us, as -1..1 on
  SERVO_PROPERTY) maps -1 / 0 / +1 to deflection_min / 0 / deflection_max, through an actuator rate-limited to
  rate_deg_s.
- Contact points: the tail end and every panel tip trailing edge, as STRUCTURE contacts; JSBSim decides which touch
  (it may lean onto wingtips that end short of the tail); a fixed-wing also gets the nose tip. Springs sized for CONTACT_SUPPORTING_POINTS carrying the weight. Start standing at stance_height_m: JSBSim's ground trim (do_simple_trim) crashes on a nose-up model.
"""

import copy
import math
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb
import jsbsim
import numpy as np

from libs.aero import AERO_MODEL_SIZE, build_asb_airplane, build_hybrid_airplanes, with_side_deflections
from libs.airframe import INCH_M, Propulsion
from libs.geometry import STANDARD_GRAVITY_M_S2, Inertia, ResolvedAirframe, Vec
from libs.propulsion import MotorModel, motor_model, rpm_for_thrust, static_points

PROP_J_TABLE_POINTS = 25
PROP_J_TABLE_END = 1.2  # of the zero-thrust advance ratio
APC_REFERENCE_PROP = "DJI_9450"  # JSBSim's bundled APC 9x4.5E table, for the Ct(J) and Cp(J) shapes
APC_REFERENCE_PITCH_RATIO = 4.5 / 9.0
AERO_ALPHA_DEG = np.concatenate((np.arange(-180.0, -30.0, 10.0), np.arange(-30.0, 31.0, 2.0), np.arange(40.0, 181.0, 10.0)))
AERO_BETA_DEG = np.array([-90.0, -60.0, -45.0, -30.0, -20.0, -15.0, -10.0, -7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0])
# Airspeeds the tables are solved at (Reynolds number: section polars and body drag); JSBSim holds the end values
# beyond them. Independent of the definition's operating airspeed, which is a guess.
AERO_SPEEDS_M_S = np.array([5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0])
FT_M = 0.3048
ALPHA_RAD_ROWS = ("aero/alpha-rad", np.radians(AERO_ALPHA_DEG))  # table_xml (property, breakpoints)
AIRSPEED_FPS = ("velocities/vt-fps", AERO_SPEEDS_M_S / FT_M)
CONTROL_COEFFICIENTS = ("CX", "CY", "CZ", "Cl", "Cm", "Cn")
# Deflections the control tables are solved at, each way. Dense near neutral: a section stalls past ~15-20 deg (and
# AeroSandbox's flap drag grows with deflection squared), so a coarse grid interpolates small deflections from a
# stalled column (half of +-45 deg gave 90x the real adverse yaw and a ninth of the roll at 5 deg).
CONTROL_DEFLECTION_GRID_DEG = (2.5, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0)
CONTROL_FACTOR_PROBE_DEG = 2.0  # finite_wing_factor: twist / control deflection
CONTROL_FACTOR_AOA_DEG = 2.0  # finite_wing_factor: AoA it is taken at (mean over AERO_SPEEDS_M_S)
CONTROL_DERIVATIVE_AOA_DEG = 5.0  # derivatives for the servo mix: mean over AoA within this of zero
CONTROL_ROLE_MIN_FRACTION = 0.05  # a role is mixed when its moment (flaps: lift) is at least this fraction of the control's largest
INAV_SERVO_INPUTS = {"roll": 0, "pitch": 1, "yaw": 2, "flaps": 14}  # smix input sources: stabilized axes, FEATURE_FLAPS
# The tailsitter's hover profile takes the same stabilized inputs in the multirotor frame, as for the motor mixers:
# forward roll = -hover yaw, pitch = pitch, forward yaw = hover roll. (input, rate sign) per role.
HOVER_SERVO_INPUTS = {"roll": (2, -1), "pitch": (1, 1), "yaw": (0, 1), "flaps": (14, 1)}
INAV_SMIX_RATE = 100
SERVO_PROPERTY = "fcs/servo-{index}-norm"
CONTROL_POSITION_PROPERTY = "fcs/control-{number}-pos-deg"
CONTACT_SUPPORTING_POINTS = 3  # contacts assumed to carry the weight when it stands, for the spring rate
CONTACT_STATIC_DEFLECTION_M = 0.002
CONTACT_DAMPING_RATIO = 1.0
CONTACT_FRICTION = (0.8, 0.5)  # static, dynamic (as JSBSim's F450)
CONTACT_TYPE = "STRUCTURE"  # BOGEY is a wheel: its rolling direction degenerates with the body axis vertical and the contact explodes
INAV_THROTTLE_IDLE_PCT = 8  # INAV default; motor output = idle + stick x (1 - idle)
MOTOR_THROTTLE_PROPERTY = "fcs/throttle-cmd-norm[{index}]"
# INAV quad X output order as (front, right) in the hover frame: rear-right, front-right, rear-left, front-left.
INAV_QUAD_X_ORDER = ((False, True), (True, True), (False, False), (True, False))


class ExportError(Exception):
    """The airframe is of a kind this export does not cover yet."""


@dataclass(frozen=True)
class JSBSimExport:
    model_name: str
    model_dir: Path  # holds aircraft/<model>/ and engine/
    inav_cli_path: Path
    motor_properties: tuple[str, ...]  # driven by INAV motor output 1, 2, ... (inav_motor_order)
    inav_mixers: dict[str, tuple[tuple[float, float, float], ...]]  # tailsitter "hover" / "forward", fixed-wing "airplane":
                                                                    # (roll, pitch, yaw) per INAV output, yaw as with
                                                                    # motor_direction_inverted = OFF
    hover_motor_fraction: float | None  # predicted motor command in hover (VTOL)
    hover_stick_us: int | None  # nav_mc_hover_thr (VTOL)
    stance_height_m: float | None  # tailsitter: CG height standing on the contact points, nose up (start here; JSBSim's ground trim crashes on it)
    motor: MotorModel | None  # its constants are the JSBSim motor's (None when unpowered)
    pack_voltage_v: float | None  # JSBSim maxvolts: battery power is V = throttle-pos x pack_voltage_v, I = (V - rpm/Kv) / R
    servo_properties: tuple[str, ...]  # driven by INAV servo output 0, 1, ... (S1, S2, ...), -1..1
    servo_notes: tuple[str, ...]  # control roles left out of the servo mix, and why


@dataclass(frozen=True)
class ControlTable:
    number: int  # control number (C1 = 1)
    label: str
    servo: int  # INAV servo output index (S1 = 0)
    roles: tuple[str, ...]
    deflection_min_deg: float
    deflection_max_deg: float
    rate_deg_s: float
    finite_wing_factor: float  # AeroBuildup is given deflection x this (finite_wing_factor)
    deflection_deg: np.ndarray
    increments: dict[str, np.ndarray]  # CONTROL_COEFFICIENTS (body axes) over the undeflected aircraft: (AERO_SPEEDS_M_S, AERO_ALPHA_DEG, deflection_deg)
    derivatives: dict[str, float]  # per degree of deflection near neutral, mean over the airspeeds and AoA within CONTROL_DERIVATIVE_AOA_DEG of zero


def inav_motor_order(resolved: ResolvedAirframe) -> tuple[int, ...]:
    """Resolved motor index of each INAV motor output: quad X order in the hover frame (front = belly -z, right = +y)
    for 4 motors one per quadrant, else the resolved order."""
    quadrants = [(motor.position[2] < 0.0, motor.position[1] > 0.0) for motor in resolved.motors]
    off_axes = all(abs(motor.position[1]) > 1e-9 and abs(motor.position[2]) > 1e-9 for motor in resolved.motors)
    if len(resolved.motors) == 4 and off_axes and len(set(quadrants)) == 4:
        return tuple(quadrants.index(quadrant) for quadrant in INAV_QUAD_X_ORDER)
    return tuple(range(len(resolved.motors)))


def inav_mixers(resolved: ResolvedAirframe, order: tuple[int, ...]) -> dict[str, tuple[tuple[float, float, float], ...]]:
    """INAV mmix (roll, pitch, yaw) weights per output. Hover (multirotor frame: front = belly, right = +y): roll + for
    left motors, pitch + for rear (dorsal), yaw + for ccw. Forward flight is the same motors seen nose-first: roll
    from prop torque (-hover yaw), pitch unchanged, yaw from left/right thrust (hover roll)."""
    reach_y = max(abs(motor.position[1]) for motor in resolved.motors)
    reach_z = max(abs(motor.position[2]) for motor in resolved.motors)
    hover = tuple(
        (-resolved.motors[index].position[1] / reach_y if reach_y > 0.0 else 0.0,  # motors on the axis have no lever arm
         resolved.motors[index].position[2] / reach_z if reach_z > 0.0 else 0.0,
         1.0 if resolved.motors[index].spin == "ccw" else -1.0)
        for index in order
    )
    return {"hover": hover, "forward": tuple((-yaw, pitch, roll) for roll, pitch, yaw in hover)}


def jsbsim_inertia(inertia: Inertia) -> tuple[float, float, float, float, float, float]:
    """Geometry-frame tensor entries -> JSBSim body frame (x fwd, y right, z down) ixx..iyz with positive products."""
    ixx, iyy, izz, txy, txz, tyz = inertia
    return ixx, iyy, izz, txy, -txz, tyz


def contact_points(resolved: ResolvedAirframe) -> list[Vec]:
    """The tail end and every panel tip trailing edge; a fixed-wing also rests on its nose tip."""
    points = [(resolved.body.length_m, 0.0, 0.0)] + ([] if resolved.airframe.tailsitter else [(0.0, 0.0, 0.0)])
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            points.append((panel.tip_le[0] + panel.chord_tip_m, panel.tip_le[1], panel.tip_le[2]))
    return points


def apc_shape() -> tuple[dict[str, np.ndarray], float]:
    """JSBSim's bundled APC reference prop: {"C_THRUST", "C_POWER"} as (J, C / C(0)) rows, and its zero-thrust J."""
    root = ElementTree.parse(Path(jsbsim.get_default_root_dir()) / "engine" / f"{APC_REFERENCE_PROP}.xml").getroot()
    tables = {table.get("name"): np.array([[float(v) for v in line.split()] for line in table.find("tableData").text.strip().splitlines()]) for table in root.iter("table")}
    ct = tables["C_THRUST"]
    zero_j = float(np.interp(0.0, ct[::-1, 1], ct[::-1, 0]))
    return {name: np.column_stack((table[:, 0], table[:, 1] / table[0, 1])) for name, table in tables.items()}, zero_j


def extended(x: np.ndarray, rows: np.ndarray) -> np.ndarray:
    """Interpolate (x, y) rows, continuing the last segment's slope past the end."""
    (x0, y0), (x1, y1) = rows[-2], rows[-1]
    return np.where(x <= x1, np.interp(x, rows[:, 0], rows[:, 1]), y1 + (x - x1) * (y1 - y0) / (x1 - x0))


def table_rows(rows) -> str:
    return "\n".join("        " + "  ".join(f"{value:.6g}" for value in row) for row in rows)


def body_coefficients(aero: dict, op_point: asb.OperatingPoint, resolved: ResolvedAirframe) -> np.ndarray:
    """(6, points) CONTROL_COEFFICIENTS of an AeroBuildup result, body axes."""
    reference = resolved.reference
    force_scale = np.asarray(op_point.dynamic_pressure()) * reference.area_m2
    lengths = (1.0, 1.0, 1.0, reference.span_m, reference.chord_m, reference.span_m)
    loads = [*aero["F_b"], *aero["M_b"]]
    return np.array([np.asarray(load) / (force_scale * length) for load, length in zip(loads, lengths)])


def aero_tables(resolved: ResolvedAirframe) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Body-axis coefficients on the airspeed x AoA x sideslip grid, and the damping derivatives on the airspeed x AoA
    grid at sideslip 0."""
    airplane = build_asb_airplane(resolved)
    atmosphere = asb.Atmosphere(altitude=resolved.airframe.operating_point.altitude_m)
    speed, alpha, beta = np.meshgrid(AERO_SPEEDS_M_S, AERO_ALPHA_DEG, AERO_BETA_DEG, indexing="ij")
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=speed.ravel(), alpha=alpha.ravel(), beta=beta.ravel())
    values = body_coefficients(asb.AeroBuildup(airplane=airplane, op_point=op_point, model_size=AERO_MODEL_SIZE).run(), op_point, resolved)
    tables = {name: values[index].reshape(speed.shape) for index, name in enumerate(CONTROL_COEFFICIENTS)}
    speed, alpha = np.meshgrid(AERO_SPEEDS_M_S, AERO_ALPHA_DEG, indexing="ij")
    rates = asb.AeroBuildup(
        airplane=airplane, op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=speed.ravel(), alpha=alpha.ravel()), model_size=AERO_MODEL_SIZE,
    ).run_with_stability_derivatives(alpha=False, beta=False, p=True, q=True, r=True)
    dampers = {name: np.asarray(rates[name]).reshape(speed.shape) for name in ("Clp", "Cmq", "Cnr")}
    for name, table in (*tables.items(), *dampers.items()):
        if not np.all(np.isfinite(table)):
            raise ValueError(f"AeroBuildup returned non-finite {name} on the airspeed x AoA grid")
    return tables, dampers


def finite_wing_factor(airplane: asb.Airplane, wing: int, run) -> float:
    """AeroBuildup adds a control's AoA change after its finite-wing correction, so a control acts as if on an
    infinite wing. This is the ratio of the force change from twisting the whole wing to the change from a control
    surface hinged at the leading edge over all of it (the same section AoA change), small angle: the factor that
    puts a control through the wing's finite-wing correction."""
    twisted, hinged = copy.deepcopy(airplane), copy.deepcopy(airplane)
    for section in twisted.wings[wing].xsecs:
        section.twist += CONTROL_FACTOR_PROBE_DEG
    for section in hinged.wings[wing].xsecs[:-1]:  # a section's control surfaces act on the segment after it
        section.control_surfaces = [asb.ControlSurface(symmetric=True, deflection=CONTROL_FACTOR_PROBE_DEG, hinge_point=0.0)]
    base = run(airplane)
    twist_change, hinge_change = (np.linalg.norm(run(deflected)[:3] - base[:3], axis=0) for deflected in (twisted, hinged))
    return float(np.mean(twist_change / hinge_change))


def control_tables(resolved: ResolvedAirframe) -> list[ControlTable]:
    """Each control's increments over the undeflected aircraft, AeroBuildup (as the base tables) at sideslip 0, its
    deflection times its wing's finite_wing_factor. An XZ-mirrored pair is one ASB wing, each side deflected alone
    (SideControlSurface)."""
    if not resolved.controls:
        return []
    airplane = build_asb_airplane(resolved)
    hybrid = build_hybrid_airplanes(resolved)  # its wings are airplane.wings in order, with the control numbers per wing
    atmosphere = asb.Atmosphere(altitude=resolved.airframe.operating_point.altitude_m)
    speed, alpha = np.meshgrid(AERO_SPEEDS_M_S, AERO_ALPHA_DEG, indexing="ij")
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=speed.ravel(), alpha=alpha.ravel())
    probe_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=AERO_SPEEDS_M_S, alpha=np.full(len(AERO_SPEEDS_M_S), CONTROL_FACTOR_AOA_DEG))

    def run(deflected: asb.Airplane, point: asb.OperatingPoint = op_point) -> np.ndarray:
        return body_coefficients(asb.AeroBuildup(airplane=deflected, op_point=point, model_size=AERO_MODEL_SIZE).run(), point, resolved)

    base = run(airplane)
    near = np.abs(AERO_ALPHA_DEG) <= CONTROL_DERIVATIVE_AOA_DEG
    placements = {control.number: control for control in resolved.controls}
    tables = []
    for wing, (owner, numbers) in enumerate(zip(hybrid.owners, hybrid.controls)):
        if not any(numbers):
            continue
        surface_set = resolved.surfaces[owner]
        definition = surface_set.definition.control
        factor = finite_wing_factor(airplane, wing, lambda deflected: run(deflected, probe_point))
        deflections = np.unique([0.0, definition.deflection_min_deg, definition.deflection_max_deg,
                                 *(-d for d in CONTROL_DEFLECTION_GRID_DEG if -d > definition.deflection_min_deg),
                                 *(d for d in CONTROL_DEFLECTION_GRID_DEG if d < definition.deflection_max_deg)])
        sides: list[list[np.ndarray]] = [[], []]  # own, mirror: (6, points) per deflection
        for deflection in deflections:
            solved = float(deflection * factor)
            for side, (number, columns) in enumerate(zip(numbers, sides)):
                if number:
                    columns.append(run(with_side_deflections(airplane, wing, *((solved, 0.0) if side == 0 else (0.0, solved)))) - base if deflection else np.zeros_like(base))
        neutral = int(np.flatnonzero(deflections == 0.0)[0])
        low, high = max(neutral - 1, 0), min(neutral + 1, len(deflections) - 1)  # the columns either side of neutral
        for number, columns in zip(numbers, sides):
            if not number:
                continue
            stacked = np.stack(columns, axis=2).reshape(6, *speed.shape, len(deflections))  # (6, speed, AoA, deflection)
            if not np.all(np.isfinite(stacked)):
                raise ValueError(f"AeroBuildup returned non-finite control increments for C{number}")
            control = placements[number]
            tables.append(ControlTable(
                number=number, label=f"C{number} {control.surface} panel {control.panel_index + 1}", servo=int(control.servo[1:]) - 1,
                roles=definition.roles, deflection_min_deg=definition.deflection_min_deg, deflection_max_deg=definition.deflection_max_deg,
                rate_deg_s=definition.rate_deg_s, finite_wing_factor=factor, deflection_deg=deflections,
                increments={name: stacked[index] for index, name in enumerate(CONTROL_COEFFICIENTS)},
                derivatives={name: float(np.mean(stacked[index][:, near, high] - stacked[index][:, near, low]) / (deflections[high] - deflections[low]))
                             for index, name in enumerate(CONTROL_COEFFICIENTS)},
            ))
    return sorted(tables, key=lambda table: table.number)


def servo_mix(resolved: ResolvedAirframe, tables: list[ControlTable]) -> tuple[list[tuple[int, str, int, str]], list[str]]:
    """(servo, role, rate sign, controls) per mixed role, and notes on roles left out. Signs follow INAV's stabilized
    frame (forward-left-up: + roll = right wing down, + pitch = nose down, + yaw = nose left), i.e. JSBSim body Cl, -Cm,
    -Cn; flaps: + = more lift. A role whose moment is under CONTROL_ROLE_MIN_FRACTION of the control's largest (e.g. pitch
    on a vertical fin) is left out."""
    reference = resolved.reference
    rules: dict[tuple[int, str], tuple[int, list[str]]] = {}
    notes = []
    for table in tables:
        d = table.derivatives
        moments = {"roll": d["Cl"] * reference.span_m, "pitch": -d["Cm"] * reference.chord_m, "yaw": -d["Cn"] * reference.span_m}
        if not table.roles:
            notes.append(f"{table.label}: no role, not mixed")
        for role in table.roles:
            value, largest = (-d["CZ"], max(abs(d["CY"]), abs(d["CZ"]))) if role == "flaps" else (moments[role], max(abs(v) for v in moments.values()))
            if abs(value) < CONTROL_ROLE_MIN_FRACTION * largest:
                notes.append(f"{table.label}: {role} not mixed, its {'lift' if role == 'flaps' else role + ' moment'} is {abs(value) / largest:.0%} of its largest")
                continue
            sign = 1 if value > 0.0 else -1
            existing = rules.setdefault((table.servo, role), (sign, []))
            if existing[0] != sign:
                raise ExportError(f"S{table.servo + 1} drives {', '.join(existing[1])} and {table.label} with opposite {role} effect")
            existing[1].append(table.label)
    return [(servo, role, sign, ", ".join(labels)) for (servo, role), (sign, labels) in sorted(rules.items())], notes


def control_fcs_xml(tables: list[ControlTable]) -> str:
    if not tables:
        return ""
    declared = "".join(f"""
    <property value="0"> {SERVO_PROPERTY.format(index=index)} </property>""" for index in range(max(table.servo for table in tables) + 1))
    components = "".join(f"""
      <fcs_function name="control {table.number} command">
        <function>
          <table>
            <independentVar>{SERVO_PROPERTY.format(index=table.servo)}</independentVar>
            <tableData>
              -1  {table.deflection_min_deg:g}
               0  0
               1  {table.deflection_max_deg:g}
            </tableData>
          </table>
        </function>
        <output>fcs/control-{table.number}-cmd-deg</output>
      </fcs_function>
      <actuator name="control {table.number}">
        <input>fcs/control-{table.number}-cmd-deg</input>
        <rate_limit> {table.rate_deg_s:g} </rate_limit>
        <output>{CONTROL_POSITION_PROPERTY.format(number=table.number)}</output>
      </actuator>""" for table in tables)
    return f"""
  <flight_control name="Control surfaces">{declared}
    <channel name="Servos">{components}
    </channel>
  </flight_control>"""


def table_xml(row: tuple[str, np.ndarray], column: tuple[str, np.ndarray], values: np.ndarray, layer: tuple[str, np.ndarray] | None = None) -> str:
    """A JSBSim table over (property, breakpoints) rows and columns: values (rows, columns), or with a layer
    (layers, rows, columns)."""
    def data(grid: np.ndarray, breakpoint: str) -> str:
        header = "          " + "  ".join(f"{value:.6g}" for value in column[1])
        rows = "\n".join(f"          {key:.6g}  " + "  ".join(f"{value:.6g}" for value in line) for key, line in zip(row[1], grid))
        return f"""
          <tableData{breakpoint}>
{header}
{rows}
          </tableData>"""
    independents = f"""
          <independentVar lookup="row">{row[0]}</independentVar>
          <independentVar lookup="column">{column[0]}</independentVar>"""
    if layer is None:
        body = data(values, "")
    else:
        independents += f"""
          <independentVar lookup="table">{layer[0]}</independentVar>"""
        body = "".join(data(grid, f' breakPoint="{key:.6g}"') for key, grid in zip(layer[1], values))
    return f"""
        <table>{independents}{body}
        </table>"""


def coefficient_xml(name: str, description: str, factors: str, table: str) -> str:
    return f"""
    <function name="aero/coefficient/{name}">
      <description>{description}</description>
      <product>
        {factors}{table}
      </product>
    </function>"""


def control_aero_xml(tables: list[ControlTable], coefficient: str, factors: str) -> str:
    return "".join(coefficient_xml(
        f"{coefficient}_control_{table.number}", f"{coefficient} increment of {table.label} (AeroBuildup, deflection x {table.finite_wing_factor:.3f})", factors,
        table_xml(("aero/alpha-deg", AERO_ALPHA_DEG), (CONTROL_POSITION_PROPERTY.format(number=table.number), table.deflection_deg), table.increments[coefficient], AIRSPEED_FPS),
    ) for table in tables)


def aero_axis_xml(coefficient: str, table: np.ndarray, factors: str) -> str:
    return coefficient_xml(coefficient, f"{coefficient} (AeroBuildup, body axes)", factors,
                           table_xml(ALPHA_RAD_ROWS, ("aero/beta-rad", np.radians(AERO_BETA_DEG)), table, AIRSPEED_FPS))


def engine_xml(resolved: ResolvedAirframe, motor_file: str, prop_file: str) -> str:
    return "".join(f"""
    <engine file="{motor_file}" name="{placement.name}">
      <thruster file="{prop_file}">
        <location unit="M"> <x> {placement.prop_center[0]:.5f} </x> <y> {placement.prop_center[1]:.5f} </y> <z> {placement.prop_center[2]:.5f} </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> {math.degrees(math.atan2(placement.thrust_dir[2], -placement.thrust_dir[0])):.3f} </pitch> <yaw> 0 </yaw> </orient>
        <sense> {1.0 if placement.spin == 'ccw' else -1.0} </sense>
        <p_factor> 0.0 </p_factor>
      </thruster>
    </engine>""" for placement in resolved.motors)


def write_engine_files(propulsion: Propulsion, model: MotorModel, engine_dir: Path, model_name: str, pack_voltage_v: float | None) -> tuple[str, str]:
    """The JSBSim motor and propeller files; returns their names. pack_voltage_v as for export_jsbsim."""
    diameter_m = model.diameter_m
    pack_voltage = model.pack_voltage_v if pack_voltage_v is None else pack_voltage_v
    ct0, cp0 = model.ct0, model.cp0
    shapes, apc_zero_j = apc_shape()
    zero_j = model.pitch_m / diameter_m * apc_zero_j / APC_REFERENCE_PITCH_RATIO
    j_grid = np.linspace(0.0, PROP_J_TABLE_END * zero_j, PROP_J_TABLE_POINTS)
    apc_j = j_grid * apc_zero_j / zero_j
    ct_table = ct0 * extended(apc_j, shapes["C_THRUST"])
    cp_table = cp0 * extended(apc_j, shapes["C_POWER"])
    prop_ixx = propulsion.prop_mass_kg * (diameter_m / 2.0) ** 2 / 3.0
    engine_dir.mkdir(parents=True, exist_ok=True)
    motor_file, prop_file = f"{model_name}_motor", f"{model_name}_prop"
    (engine_dir / f"{motor_file}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {model.label}, {model.prop}, {model.cells}S: motorlib's full-throttle static point ({model.full_rpm:.0f} RPM,
     {model.full_current_a:.1f} A) at {model.pack_voltage_v:.1f} V nominal; resistance = wiring {model.wiring.per_motor_ohm * 1000.0:.1f} mOhm per motor
     (see libs/propulsion.py) + loaded winding; maxvolts = {'nominal pack voltage' if pack_voltage_v is None else 'set for this export'}. -->
<brushless_dc_motor name="{model.label}">
  <velocityconstant> {model.kv_rpm_v:.1f} </velocityconstant>
  <coilresistance> {model.resistance_ohm:.5f} </coilresistance>
  <noloadcurrent> {model.no_load_current_a:.4f} </noloadcurrent>
  <maxvolts> {pack_voltage:.3f} </maxvolts>
</brushless_dc_motor>
""")
    (engine_dir / f"{prop_file}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {model.prop}. Static Ct {ct0:.4f} and Cp {cp0:.4f} from motorlib's full-throttle point; Ct(J) and Cp(J) shaped as
     {APC_REFERENCE_PROP} (APC 9x4.5E), J scaled so thrust is zero at {zero_j:.3f} (the APC's zero-thrust J x the pitch ratio
     over its 0.5), the last segment's slope continued past its table; ixx = prop mass x R^2 / 3. -->
<propeller name="{model.prop}" version="1.1">
  <ixx unit="KG*M2"> {prop_ixx:.6g} </ixx>
  <diameter unit="IN"> {diameter_m / INCH_M:.3f} </diameter>
  <numblades> 2 </numblades>
  <constspeed> 0 </constspeed>
  <table name="C_THRUST" type="internal">
    <tableData>
{table_rows(zip(j_grid, ct_table))}
    </tableData>
  </table>
  <table name="C_POWER" type="internal">
    <tableData>
{table_rows(zip(j_grid, cp_table))}
    </tableData>
  </table>
</propeller>
""")
    return motor_file, prop_file


def export_jsbsim(resolved: ResolvedAirframe, parts_data: dict, out_dir: Path, model_name: str, pack_voltage_v: float | None = None) -> JSBSimExport:
    """pack_voltage_v: battery voltage for the motors (default: nominal). The motor command is the duty, so a lower pack
    gives proportionally less motor voltage per command, as an ESC."""
    airframe = resolved.airframe
    if airframe.vtol and not airframe.tailsitter:
        raise ExportError(f"{airframe.name}: INAV export of VTOL types other than tailsitter is not implemented")
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    aircraft_dir = out_dir / "aircraft" / model_name
    aircraft_dir.mkdir(parents=True, exist_ok=True)
    model = pack_voltage = hover_fraction = hover_stick_us = None
    engines_xml = ""
    if airframe.propulsion is not None:
        model = motor_model(resolved, parts_data)
        pack_voltage = model.pack_voltage_v if pack_voltage_v is None else pack_voltage_v
        motor_file, prop_file = write_engine_files(airframe.propulsion, model, out_dir / "engine", model_name, pack_voltage_v)
        engines_xml = engine_xml(resolved, motor_file, prop_file)
        if airframe.vtol:  # hover: weight shared by the motors; the duty that holds that thrust's static RPM
            hover_fraction = float(static_points(model, rpm_for_thrust(model, weight_n / len(resolved.motors)), pack_voltage).duty)
            idle = INAV_THROTTLE_IDLE_PCT / 100.0
            hover_stick_us = int(round(1000.0 + 1000.0 * (hover_fraction - idle) / (1.0 - idle)))
    cg = resolved.cg_m
    ixx, iyy, izz, ixy, ixz, iyz = jsbsim_inertia(resolved.inertia_kg_m2)
    contacts = contact_points(resolved)
    spring = weight_n / (CONTACT_SUPPORTING_POINTS * CONTACT_STATIC_DEFLECTION_M)
    damping = 2.0 * CONTACT_DAMPING_RATIO * math.sqrt(spring * sum(resolved.mass_kg) / 2.0 / CONTACT_SUPPORTING_POINTS)
    contacts_xml = "".join(f"""
    <contact name="tail {index + 1}" type="{CONTACT_TYPE}">
      <location unit="M"> <x> {x:.5f} </x> <y> {y:.5f} </y> <z> {z:.5f} </z> </location>
      <static_friction> {CONTACT_FRICTION[0]} </static_friction>
      <dynamic_friction> {CONTACT_FRICTION[1]} </dynamic_friction>
      <spring_coeff unit="N/M"> {spring:.3f} </spring_coeff>
      <damping_coeff unit="N/M/SEC"> {damping:.3f} </damping_coeff>
    </contact>""" for index, (x, y, z) in enumerate(contacts))
    tables, rates = aero_tables(resolved)
    controls = control_tables(resolved)
    servo_rules, servo_notes = servo_mix(resolved, controls)
    forces = {"X": "CX", "Y": "CY", "Z": "CZ"}
    moments = {"ROLL": ("Cl", "metrics/bw-ft"), "PITCH": ("Cm", "metrics/cbarw-ft"), "YAW": ("Cn", "metrics/bw-ft")}
    dampers = {"ROLL": ("Clp", "metrics/bw-ft", "aero/bi2vel", "velocities/p-aero-rad_sec"),
               "PITCH": ("Cmq", "metrics/cbarw-ft", "aero/ci2vel", "velocities/q-aero-rad_sec"),
               "YAW": ("Cnr", "metrics/bw-ft", "aero/bi2vel", "velocities/r-aero-rad_sec")}
    axes_xml = "".join(f"""
    <axis name="{axis}">{aero_axis_xml(name, tables[name], "<property>aero/qbar-area</property>")}{control_aero_xml(controls, name, "<property>aero/qbar-area</property>")}
    </axis>""" for axis, name in forces.items())
    for axis, (name, length) in moments.items():
        damper, damper_length, rate_scale, rate = dampers[axis]
        damping_xml = coefficient_xml(
            damper, f"{damper} (AeroBuildup, sideslip 0)",
            f"<property>aero/qbar-area</property> <property>{damper_length}</property> <property>{rate_scale}</property> <property>{rate}</property>",
            table_xml(ALPHA_RAD_ROWS, AIRSPEED_FPS, rates[damper].T),
        )
        axes_xml += f"""
    <axis name="{axis}">{aero_axis_xml(name, tables[name], f"<property>aero/qbar-area</property> <property>{length}</property>")}{damping_xml}{control_aero_xml(controls, name, f"<property>aero/qbar-area</property> <property>{length}</property>")}
    </axis>"""
    reference = resolved.reference
    location = f"<x> {cg[0]:.5f} </x> <y> {cg[1]:.5f} </y> <z> {cg[2]:.5f} </z>"
    (aircraft_dir / f"{model_name}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder export of {airframe.name}; see libs/jsbsim.py for the model choices. -->
<fdm_config name="{model_name}" version="2.0" release="ALPHA">
  <fileheader>
    <author>quadrocketbuilder</author>
    <description>{airframe.name}: {"tailsitter, body x = nose = thrust axis" if airframe.tailsitter else "fixed-wing" if airframe.powered else "glider"}</description>
  </fileheader>
  <metrics>
    <wingarea unit="M2"> {reference.area_m2:.6f} </wingarea>
    <wingspan unit="M"> {reference.span_m:.6f} </wingspan>
    <chord unit="M"> {reference.chord_m:.6f} </chord>
    <location name="AERORP" unit="M"> {location} </location>
    <location name="EYEPOINT" unit="M"> {location} </location>
    <location name="VRP" unit="M"> {location} </location>
  </metrics>
  <mass_balance>
    <ixx unit="KG*M2"> {ixx:.6g} </ixx>
    <iyy unit="KG*M2"> {iyy:.6g} </iyy>
    <izz unit="KG*M2"> {izz:.6g} </izz>
    <ixy unit="KG*M2"> {ixy:.6g} </ixy>
    <ixz unit="KG*M2"> {ixz:.6g} </ixz>
    <iyz unit="KG*M2"> {iyz:.6g} </iyz>
    <emptywt unit="KG"> {sum(resolved.mass_kg) / 2.0:.5f} </emptywt>
    <location name="CG" unit="M"> {location} </location>
  </mass_balance>
  <ground_reactions>{contacts_xml}
  </ground_reactions>
  <propulsion>{engines_xml}
  </propulsion>{control_fcs_xml(controls)}
  <aerodynamics>{axes_xml}
  </aerodynamics>
</fdm_config>
""")

    order = inav_motor_order(resolved)
    mixers = inav_mixers(resolved, order) if airframe.tailsitter else {"airplane": ((0.0, 0.0, 0.0),) * len(order)}
    def mmix_lines(mix: str) -> list[str]:
        return [
            f"mmix {output} 1.000 {roll:+.3f} {pitch:+.3f} {yaw:+.3f}    # {resolved.motors[index].name}, {resolved.motors[index].spin}"
            for output, (index, (roll, pitch, yaw)) in enumerate(zip(order, mixers[mix]))
        ]

    def smix_lines(hover: bool) -> list[str]:
        if not controls:
            return []
        lines = ["smix reset"]
        for rule, (servo, role, sign, labels) in enumerate(servo_rules):
            source, frame_sign = HOVER_SERVO_INPUTS[role] if hover else (INAV_SERVO_INPUTS[role], 1)
            lines.append(f"smix {rule} {servo} {source} {sign * frame_sign * INAV_SMIX_RATE} 0 -1    # S{servo + 1} {role}: {labels}")
        return lines + [f"# {note}" for note in servo_notes]

    cli_path = out_dir / "inav_aircraft.cli"
    cli_path.write_text("\n".join([
        f"# quadrocketbuilder export of {airframe.name}: {'fixed-wing, motors on throttle' if airframe.powered else 'glider'}; servo mix from the",
        "# control roles, signs from each control's computed moments.",
        f"set throttle_idle = {INAV_THROTTLE_IDLE_PCT}",
        "mixer_profile 1",
        "set platform_type = AIRPLANE",
        "mmix reset",
        *mmix_lines("airplane"),
        *smix_lines(hover=False),
        "",
    ]) if not airframe.tailsitter else "\n".join([
        f"# quadrocketbuilder export of {airframe.name}: INAV mixer profiles of the tailsitter, structured as the real m62v1",
        "# config that flew forward (Aug 2025): profile 1 hover (multirotor, tailsitter offset), profile 2 forward flight on",
        "# the motors (multirotor platform, nose-first mix). Motor outputs in INAV quad X order where it applies.",
        f"set throttle_idle = {INAV_THROTTLE_IDLE_PCT}",
        f"set nav_mc_hover_thr = {hover_stick_us}    # predicted motor command {hover_fraction:.3f} for {weight_n / len(resolved.motors):.2f} N per motor",
        "mixer_profile 1",
        "set platform_type = MULTIROTOR",
        "set tailsitter_orientation_offset = ON",
        "set motor_direction_inverted = OFF",
        "set mixer_control_profile_linking = ON",
        "mmix reset",
        *mmix_lines("hover"),
        *smix_lines(hover=True),
        "mixer_profile 2",
        "set platform_type = MULTIROTOR",
        "set tailsitter_orientation_offset = OFF",
        "set motor_direction_inverted = OFF",
        "set mixer_control_profile_linking = ON",
        "mmix reset",
        *mmix_lines("forward"),
        *smix_lines(hover=False),
        "mixer_profile 1",
        "",
    ]))
    return JSBSimExport(
        model_name=model_name, model_dir=out_dir, inav_cli_path=cli_path,
        motor_properties=tuple(MOTOR_THROTTLE_PROPERTY.format(index=index) for index in order), inav_mixers=mixers,
        hover_motor_fraction=hover_fraction, hover_stick_us=hover_stick_us,
        stance_height_m=max(point[0] for point in contacts) - cg[0] if airframe.tailsitter else None, motor=model, pack_voltage_v=pack_voltage,
        servo_properties=tuple(SERVO_PROPERTY.format(index=index) for index in range(max((table.servo for table in controls), default=-1) + 1)),
        servo_notes=tuple(servo_notes),
    )
