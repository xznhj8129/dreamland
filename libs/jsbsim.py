"""
Usage:
from libs.jsbsim import export_jsbsim
export = export_jsbsim(resolved, parts_data, Path("exports/jsbsim/m62v1"), "m62v1")

JSBSim model of a resolved airframe (libs/geometry.py), for INAV SITL through inav-jsbsim-bench. INAV does the mixing;
motor i is driven through fcs/throttle-cmd-norm[i], in resolved motor order. Writes:
- <out>/aircraft/<model>/<model>.xml: metrics, mass, contact points, propulsion, throttle curve, aerodynamics
- <out>/engine/<model>_motor.xml, <model>_prop.xml
- <out>/inav_aircraft.cli: INAV mixer settings for the tailsitter's multirotor (hover) profile

Frames: JSBSim's structural frame is the geometry frame (x aft from the nose, y right, z up, metres). Body x is the nose
and the thrust axis, so hover is pitch +90 deg; INAV flies it as a multirotor with tailsitter_orientation_offset = ON,
whose front is the aircraft belly (-z) and right is +y.

Model choices (best estimates where no data exists):
- Motor: JSBSim brushless DC motor at the rated KV, fitted to the bench rows (fit_motor): duty 1 at full throttle fixes
  the effective resistance (motor, ESC, wiring); the no-load current is fitted to the measured battery currents. A
  throttle table maps the command to motor voltage so static RPM and thrust match the bench at every throttle (thrust ~
  throttle^2 below the lowest row). Back-EMF limits the RPM at speed. No battery sag (bench pack voltage).
- Prop: static Ct from the bench rows; Ct(J) linear to zero at J = pitch / diameter (Tyto formula, as libs/simlib2.py);
  Cp(J) shaped like JSBSim's bundled APC 9x4.5E table, scaled to that zero-thrust J. Rotor inertia = prop only.
- Aerodynamics: AeroSandbox AeroBuildup on the whole aircraft over AoA -180..180 and sideslip -90..90 at the operating
  airspeed (body-axis tables), roll/pitch/yaw damping from AeroBuildup at AoA 0. The live lifting-line model covers only
  +-30 deg, so it is not used here yet.
- Contact points: the aft-most points (tail end, panel tip trailing edges) it stands on, as STRUCTURE contacts; springs
  from the weight. Start standing at stance_height_m: JSBSim's ground trim (do_simple_trim) crashes on a nose-up model.
"""

import math
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb
import jsbsim
import numpy as np

from scipy.optimize import least_squares

from libs.aero import AERO_MODEL_SIZE, build_asb_airplane
from libs.airframe import MOTORDATA_PATH
from libs.geometry import STANDARD_GRAVITY_M_S2, Inertia, ResolvedAirframe, Vec
from libs.simlib2 import TestedMotorData

INCH_M = 0.0254
BENCH_AIR_DENSITY_KG_M3 = 1.225  # ISA sea level; the bench conditions were not recorded
LOW_THROTTLE_THRUST_EXPONENT = 2.0  # thrust ~ throttle^2 below the lowest measured row
THROTTLE_TABLE_POINTS = 21
PROP_J_TABLE_POINTS = 25
PROP_J_TABLE_END = 1.2  # of the zero-thrust advance ratio
APC_REFERENCE_PROP = "DJI_9450"  # JSBSim's bundled APC 9x4.5E table, for the Cp(J) shape
AERO_ALPHA_DEG = np.arange(-180.0, 181.0, 10.0)
AERO_BETA_DEG = np.arange(-90.0, 91.0, 10.0)
CONTACT_STANCE_TOLERANCE_M = 0.005  # points this close to the aft-most one carry the aircraft
CONTACT_STATIC_DEFLECTION_M = 0.002
CONTACT_DAMPING_RATIO = 1.0
CONTACT_FRICTION = (0.8, 0.5)  # static, dynamic (as JSBSim's F450)
CONTACT_TYPE = "STRUCTURE"  # BOGEY is a wheel: its rolling direction degenerates with the body axis vertical and the contact explodes
INAV_THROTTLE_IDLE_PCT = 8  # INAV default; motor output = idle + stick x (1 - idle)
MOTOR_THROTTLE_PROPERTY = "fcs/throttle-cmd-norm[{index}]"
# INAV quad X output order as (front, right) in the hover frame: rear-right, front-right, rear-left, front-left.
INAV_QUAD_X_ORDER = ((False, True), (True, True), (False, False), (True, False))


@dataclass(frozen=True)
class JSBSimExport:
    model_name: str
    model_dir: Path  # holds aircraft/<model>/ and engine/
    inav_cli_path: Path
    motor_properties: tuple[str, ...]  # driven by INAV motor output 1, 2, ... (inav_motor_order)
    inav_mixers: dict[str, tuple[tuple[float, float, float], ...]]  # "hover" / "forward": (roll, pitch, yaw) per INAV
                                                                    # output, yaw as with motor_direction_inverted = OFF
    hover_motor_fraction: float  # predicted motor command in hover
    hover_stick_us: int  # nav_mc_hover_thr
    stance_height_m: float  # CG height when standing on the contact points, nose up (start here; JSBSim's ground trim crashes on it)


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
        (-resolved.motors[index].position[1] / reach_y, resolved.motors[index].position[2] / reach_z, 1.0 if resolved.motors[index].spin == "ccw" else -1.0)
        for index in order
    )
    return {"hover": hover, "forward": tuple((-yaw, pitch, roll) for roll, pitch, yaw in hover)}


def jsbsim_inertia(inertia: Inertia) -> tuple[float, float, float, float, float, float]:
    """Geometry-frame tensor entries -> JSBSim body frame (x fwd, y right, z down) ixx..iyz with positive products."""
    ixx, iyy, izz, txy, txz, tyz = inertia
    return ixx, iyy, izz, txy, -txz, tyz


def contact_points(resolved: ResolvedAirframe) -> list[Vec]:
    candidates = [(resolved.body.length_m, 0.0, 0.0)]
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            candidates.append((panel.tip_le[0] + panel.chord_tip_m, panel.tip_le[1], panel.tip_le[2]))
    aft = max(point[0] for point in candidates)
    stance = [point for point in candidates if point[0] >= aft - CONTACT_STANCE_TOLERANCE_M]
    if len(stance) < 3:
        raise ValueError(f"{resolved.airframe.name}: only {len(stance)} aft-most contact points within {CONTACT_STANCE_TOLERANCE_M} m; it cannot stand on its tail")
    return stance


def thrust_curve(points: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """(throttle fraction, thrust N) from 0 to 1: measured rows, thrust ~ throttle^2 below the lowest."""
    throttle = points["throttle_pct"] / 100.0
    thrust = points["thrust_g"] / 1000.0 * STANDARD_GRAVITY_M_S2
    grid = np.linspace(0.0, 1.0, THROTTLE_TABLE_POINTS)
    low = grid < throttle[0]
    values = np.interp(grid, throttle, thrust)
    values[low] = thrust[0] * (grid[low] / throttle[0]) ** LOW_THROTTLE_THRUST_EXPONENT
    return grid, values


@dataclass(frozen=True)
class MotorFit:
    """DC motor + prop fitted to bench rows: motor current = shaft power / back-EMF + no-load current, motor voltage =
    back-EMF + current x resistance, battery power = motor power (lossless ESC), duty 1 at full throttle."""
    kv_rpm_v: float
    resistance_ohm: float
    no_load_current_a: float
    full_voltage_v: float  # pack voltage under load at full throttle
    ct0: float
    cp0: float
    battery_current_error: float  # largest relative battery-current misfit over the rows


def fit_motor(points: dict[str, np.ndarray], kv_rpm_v: float, diameter_m: float, rho: float) -> MotorFit:
    revs = points["rpm"] / 60.0
    back_emf = points["rpm"] / kv_rpm_v
    voltage, current = points["voltage_v"], points["current_a"]
    ct0 = float(np.mean(points["thrust_g"] / 1000.0 * STANDARD_GRAVITY_M_S2 / (rho * revs ** 2 * diameter_m ** 4)))
    resistance = float((voltage[-1] - back_emf[-1]) / current[-1])

    def motor_current(no_load_a: float) -> tuple[np.ndarray, float]:
        cp0 = (current[-1] - no_load_a) * back_emf[-1] / (rho * revs[-1] ** 3 * diameter_m ** 5)
        return cp0 * rho * revs ** 3 * diameter_m ** 5 / back_emf + no_load_a, cp0

    def battery_current_misfit(no_load: np.ndarray) -> np.ndarray:
        motor_a, _cp0 = motor_current(float(no_load[0]))
        return ((back_emf + motor_a * resistance) * motor_a / voltage - current) / current

    no_load = float(least_squares(battery_current_misfit, [1.0], bounds=([0.0], [float(current[-1])])).x[0])
    _motor_a, cp0 = motor_current(no_load)
    return MotorFit(
        kv_rpm_v=kv_rpm_v, resistance_ohm=resistance, no_load_current_a=no_load, full_voltage_v=float(voltage[-1]),
        ct0=ct0, cp0=float(cp0), battery_current_error=float(np.max(np.abs(battery_current_misfit(np.array([no_load]))))),
    )


def motor_voltage_fraction(fit: MotorFit, rpm: np.ndarray, diameter_m: float, rho: float) -> np.ndarray:
    """Fraction of full voltage that holds each static RPM against the prop."""
    revs = rpm / 60.0
    back_emf = rpm / fit.kv_rpm_v
    with np.errstate(divide="ignore", invalid="ignore"):
        motor_a = np.where(rpm > 0.0, fit.cp0 * rho * revs ** 3 * diameter_m ** 5 / back_emf + fit.no_load_current_a, 0.0)
    return np.where(rpm > 0.0, (back_emf + motor_a * fit.resistance_ohm) / fit.full_voltage_v, 0.0)


def apc_shape() -> tuple[np.ndarray, np.ndarray, float]:
    """(J, Cp / Cp(0), zero-thrust J) of JSBSim's bundled APC reference prop."""
    root = ElementTree.parse(Path(jsbsim.get_default_root_dir()) / "engine" / f"{APC_REFERENCE_PROP}.xml").getroot()
    tables = {table.get("name"): np.array([[float(v) for v in line.split()] for line in table.find("tableData").text.strip().splitlines()]) for table in root.iter("table")}
    ct, cp = tables["C_THRUST"], tables["C_POWER"]
    zero_j = float(np.interp(0.0, ct[::-1, 1], ct[::-1, 0]))
    return cp[:, 0], cp[:, 1] / cp[0, 1], zero_j


def table_rows(rows) -> str:
    return "\n".join("        " + "  ".join(f"{value:.6g}" for value in row) for row in rows)


def aero_tables(resolved: ResolvedAirframe) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Body-axis coefficients on the AoA x sideslip grid (rows AoA), and the damping derivatives at AoA 0."""
    airplane = build_asb_airplane(resolved)
    operating_point = resolved.airframe.operating_point
    atmosphere = asb.Atmosphere(altitude=operating_point.altitude_m)
    alpha, beta = np.meshgrid(AERO_ALPHA_DEG, AERO_BETA_DEG, indexing="ij")
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=alpha.ravel(), beta=beta.ravel())
    aero = asb.AeroBuildup(airplane=airplane, op_point=op_point, model_size=AERO_MODEL_SIZE).run()
    reference = resolved.reference
    force_scale = float(asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s).dynamic_pressure()) * reference.area_m2
    coefficients = {
        "CX": np.asarray(aero["F_b"][0]) / force_scale, "CY": np.asarray(aero["F_b"][1]) / force_scale, "CZ": np.asarray(aero["F_b"][2]) / force_scale,
        "Cl": np.asarray(aero["M_b"][0]) / (force_scale * reference.span_m), "Cm": np.asarray(aero["M_b"][1]) / (force_scale * reference.chord_m),
        "Cn": np.asarray(aero["M_b"][2]) / (force_scale * reference.span_m),
    }
    shape = alpha.shape
    tables = {name: values.reshape(shape) for name, values in coefficients.items()}
    for name, values in tables.items():
        if not np.all(np.isfinite(values)):
            raise ValueError(f"AeroBuildup returned non-finite {name} on the AoA x sideslip grid")
    rates = asb.AeroBuildup(
        airplane=airplane, op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=0.0, beta=0.0), model_size=AERO_MODEL_SIZE,
    ).run_with_stability_derivatives(alpha=False, beta=False, p=True, q=True, r=True)
    return tables, {name: float(rates[name]) for name in ("Clp", "Cmq", "Cnr")}


def aero_axis_xml(name: str, coefficient: str, table: np.ndarray, scale: str) -> str:
    header = "          " + "  ".join(f"{math.radians(b):.6g}" for b in AERO_BETA_DEG)
    rows = "\n".join(f"          {math.radians(a):.6g}  " + "  ".join(f"{v:.6g}" for v in row) for a, row in zip(AERO_ALPHA_DEG, table))
    return f"""
    <function name="aero/coefficient/{coefficient}">
      <description>{coefficient} (AeroBuildup, body axes)</description>
      <product>
        {scale}
        <table>
          <independentVar lookup="row">aero/alpha-rad</independentVar>
          <independentVar lookup="column">aero/beta-rad</independentVar>
          <tableData>
{header}
{rows}
          </tableData>
        </table>
      </product>
    </function>"""


def export_jsbsim(resolved: ResolvedAirframe, parts_data: dict, out_dir: Path, model_name: str) -> JSBSimExport:
    airframe = resolved.airframe
    propulsion = airframe.propulsion
    battery = next(volume for volume in airframe.volumes if volume.name == propulsion.battery_volume)
    cells = int(parts_data["battery"][battery.part.partition(":")[2]]["s"])
    motor = propulsion.motor
    tested = TestedMotorData(brand=motor.brand, motor=motor.model, kv=motor.kv, batt_s=cells, csv_path=MOTORDATA_PATH)
    points = tested.measured_points(motor.prop)
    info = tested.prop_info(motor.prop)
    diameter_m = info["diameter_in"] * INCH_M
    rho = BENCH_AIR_DENSITY_KG_M3

    fit = fit_motor(points, float(motor.kv), diameter_m, rho)
    ct0, cp0 = fit.ct0, fit.cp0
    # Throttle command -> motor voltage: the static RPM that gives the measured thrust at each command.
    throttle_grid, thrust_grid = thrust_curve(points)
    voltage_fraction = motor_voltage_fraction(fit, 60.0 * np.sqrt(thrust_grid / (ct0 * rho * diameter_m ** 4)), diameter_m, rho)
    zero_j = info["pitch_in"] / info["diameter_in"]
    apc_j, apc_cp_shape, apc_zero_j = apc_shape()
    j_grid = np.linspace(0.0, PROP_J_TABLE_END * zero_j, PROP_J_TABLE_POINTS)
    ct_table = ct0 * (1.0 - j_grid / zero_j)
    cp_table = cp0 * np.interp(j_grid * apc_zero_j / zero_j, apc_j, apc_cp_shape)
    prop_ixx = propulsion.prop_mass_kg * (diameter_m / 2.0) ** 2 / 3.0

    # Hover: weight shared by the motors, inverted on the measured thrust curve.
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    hover_fraction = float(np.interp(weight_n / len(resolved.motors), thrust_grid, throttle_grid))
    idle = INAV_THROTTLE_IDLE_PCT / 100.0
    hover_stick_us = int(round(1000.0 + 1000.0 * (hover_fraction - idle) / (1.0 - idle)))

    aircraft_dir = out_dir / "aircraft" / model_name
    engine_dir = out_dir / "engine"
    aircraft_dir.mkdir(parents=True, exist_ok=True)
    engine_dir.mkdir(parents=True, exist_ok=True)
    motor_file, prop_file = f"{model_name}_motor", f"{model_name}_prop"
    (engine_dir / f"{motor_file}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {motor.brand} {motor.model} {motor.kv}KV, {cells}S, fitted to the bench rows: effective resistance
     (motor + ESC + wiring) from duty 1 at full throttle, no-load current from the battery currents (worst misfit
     {fit.battery_current_error * 100.0:.1f} %); maxvolts = pack voltage under load at full throttle. -->
<brushless_dc_motor name="{motor.brand} {motor.model} {motor.kv}KV">
  <velocityconstant> {fit.kv_rpm_v:.1f} </velocityconstant>
  <coilresistance> {fit.resistance_ohm:.5f} </coilresistance>
  <noloadcurrent> {fit.no_load_current_a:.4f} </noloadcurrent>
  <maxvolts> {fit.full_voltage_v:.3f} </maxvolts>
</brushless_dc_motor>
""")
    (engine_dir / f"{prop_file}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {motor.prop}. Static Ct {ct0:.4f} from the bench rows; Ct(J) linear to zero at J = p/D = {zero_j:.3f};
     Cp(J) = {cp0:.4f} x the {APC_REFERENCE_PROP} (APC 9x4.5E) shape scaled to that J; ixx = prop mass x R^2 / 3. -->
<propeller name="{motor.prop}" version="1.1">
  <ixx unit="KG*M2"> {prop_ixx:.6g} </ixx>
  <diameter unit="IN"> {info['diameter_in']:.3f} </diameter>
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

    cg = resolved.cg_m
    ixx, iyy, izz, ixy, ixz, iyz = jsbsim_inertia(resolved.inertia_kg_m2)
    contacts = contact_points(resolved)
    spring = weight_n / (len(contacts) * CONTACT_STATIC_DEFLECTION_M)
    damping = 2.0 * CONTACT_DAMPING_RATIO * math.sqrt(spring * sum(resolved.mass_kg) / 2.0 / len(contacts))
    contacts_xml = "".join(f"""
    <contact name="tail {index + 1}" type="{CONTACT_TYPE}">
      <location unit="M"> <x> {x:.5f} </x> <y> {y:.5f} </y> <z> {z:.5f} </z> </location>
      <static_friction> {CONTACT_FRICTION[0]} </static_friction>
      <dynamic_friction> {CONTACT_FRICTION[1]} </dynamic_friction>
      <spring_coeff unit="N/M"> {spring:.3f} </spring_coeff>
      <damping_coeff unit="N/M/SEC"> {damping:.3f} </damping_coeff>
    </contact>""" for index, (x, y, z) in enumerate(contacts))
    engines_xml = "".join(f"""
    <engine file="{motor_file}" name="{placement.name}">
      <thruster file="{prop_file}">
        <location unit="M"> <x> {placement.prop_center[0]:.5f} </x> <y> {placement.prop_center[1]:.5f} </y> <z> {placement.prop_center[2]:.5f} </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> 0 </pitch> <yaw> 0 </yaw> </orient>
        <sense> {1.0 if placement.spin == 'ccw' else -1.0} </sense>
        <p_factor> 0.0 </p_factor>
      </thruster>
    </engine>""" for placement in resolved.motors)
    throttle_xml = "".join(f"""
      <fcs_function name="esc {index + 1}">
        <function>
          <table>
            <independentVar lookup="row">{MOTOR_THROTTLE_PROPERTY.format(index=index)}</independentVar>
            <tableData>
{table_rows(zip(throttle_grid, voltage_fraction))}
            </tableData>
          </table>
        </function>
        <output>fcs/throttle-pos-norm[{index}]</output>
      </fcs_function>""" for index in range(len(resolved.motors)))
    tables, rates = aero_tables(resolved)
    forces = {"X": "CX", "Y": "CY", "Z": "CZ"}
    moments = {"ROLL": ("Cl", "metrics/bw-ft"), "PITCH": ("Cm", "metrics/cbarw-ft"), "YAW": ("Cn", "metrics/bw-ft")}
    dampers = {"ROLL": ("Clp", "metrics/bw-ft", "aero/bi2vel", "velocities/p-aero-rad_sec"),
               "PITCH": ("Cmq", "metrics/cbarw-ft", "aero/ci2vel", "velocities/q-aero-rad_sec"),
               "YAW": ("Cnr", "metrics/bw-ft", "aero/bi2vel", "velocities/r-aero-rad_sec")}
    axes_xml = "".join(f"""
    <axis name="{axis}">{aero_axis_xml(axis, name, tables[name], "<property>aero/qbar-area</property>")}
    </axis>""" for axis, name in forces.items())
    for axis, (name, length) in moments.items():
        damper, damper_length, rate_scale, rate = dampers[axis]
        axes_xml += f"""
    <axis name="{axis}">{aero_axis_xml(axis, name, tables[name], f"<property>aero/qbar-area</property> <property>{length}</property>")}
    <function name="aero/coefficient/{damper}">
      <description>{damper} = {rates[damper]:.5g} (AeroBuildup at AoA 0)</description>
      <product>
        <property>aero/qbar-area</property> <property>{damper_length}</property> <property>{rate_scale}</property> <property>{rate}</property>
        <value> {rates[damper]:.6g} </value>
      </product>
    </function>
    </axis>"""
    reference = resolved.reference
    location = f"<x> {cg[0]:.5f} </x> <y> {cg[1]:.5f} </y> <z> {cg[2]:.5f} </z>"
    (aircraft_dir / f"{model_name}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder export of {airframe.name}; see libs/jsbsim.py for the model choices. -->
<fdm_config name="{model_name}" version="2.0" release="ALPHA">
  <fileheader>
    <author>quadrocketbuilder</author>
    <description>{airframe.name}: tailsitter, body x = nose = thrust axis</description>
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
  </propulsion>
  <flight_control name="ESC throttle curve">
    <channel name="ESC">{throttle_xml}
    </channel>
  </flight_control>
  <aerodynamics>{axes_xml}
  </aerodynamics>
</fdm_config>
""")

    order = inav_motor_order(resolved)
    mixers = inav_mixers(resolved, order)
    mmix = [
        f"mmix {output} 1.000 {roll:+.3f} {pitch:+.3f} {yaw:+.3f}    # {resolved.motors[index].name}, {resolved.motors[index].spin}"
        for output, (index, (roll, pitch, yaw)) in enumerate(zip(order, mixers["hover"]))
    ]
    cli_path = out_dir / "inav_aircraft.cli"
    cli_path.write_text("\n".join([
        f"# quadrocketbuilder export of {airframe.name}: INAV multirotor (hover) profile of the tailsitter.",
        "# Motor outputs in INAV quad X order where it applies; front = aircraft belly, right = +y.",
        "set platform_type = MULTIROTOR",
        "set tailsitter_orientation_offset = ON",
        "set motor_direction_inverted = OFF",
        f"set throttle_idle = {INAV_THROTTLE_IDLE_PCT}",
        "mmix reset",
        *mmix,
        f"set nav_mc_hover_thr = {hover_stick_us}    # predicted motor command {hover_fraction:.3f} for {weight_n / len(resolved.motors):.2f} N per motor",
        "",
    ]))
    return JSBSimExport(
        model_name=model_name, model_dir=out_dir, inav_cli_path=cli_path,
        motor_properties=tuple(MOTOR_THROTTLE_PROPERTY.format(index=index) for index in order), inav_mixers=mixers,
        hover_motor_fraction=hover_fraction, hover_stick_us=hover_stick_us,
        stance_height_m=max(point[0] for point in contacts) - cg[0],
    )
