"""
Usage:
from libs.jsbsim import export_jsbsim, reuse_or_export
export = export_jsbsim(resolved, parts_data, project.jsbsim_dir, project.name)  # libs/project.py
export, reused = reuse_or_export(resolved, parts_data, project.jsbsim_dir, project.name)  # skips an unchanged one

JSBSim model of a resolved airframe (libs/geometry.py), for INAV SITL through inav-jsbsim-bench. INAV does the mixing;
motor i is driven through fcs/throttle-cmd-norm[i], in resolved motor order. Writes:
- <out>/aircraft/<model>/<model>.xml: metrics, mass, contact points, propulsion, throttle curve, aerodynamics
- <out>/engine/<model>_motor.xml, <model>_prop.xml
- <out>/inav_aircraft.cli: INAV mixer settings: a tailsitter gets its hover and forward-flight mixer profiles, a
  fixed-wing (not VTOL) an AIRPLANE profile with the motors on throttle (none when unpowered). Control surfaces get a
  servo mix (smix) from their roles, with each rule's sign from the control's own computed moments (servo_mix); a
  control opposite to the others on its shared servo is wired reversed (a Y-cable reverser) and marked so. VTOL
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
- Aerodynamics: the editor's lifting line + bodies at forward zero-sideslip points; AeroBuildup sideslip increments,
  outer envelope and rate damping (FlightAero documents the blend). Tables cover AoA -180..180, sideslip -90..90 and
  the solved airspeeds (aero_speeds_m_s). Matching models are consistency, not independent aerodynamic validation.
- Control surfaces: each control's increments from the same blended model at sideslip 0, the base AoA/speed grids
  and CONTROL_DEFLECTION_GRID_DEG. The forward model includes interacting-wing response to the actual deflection;
  the outer AeroBuildup envelope uses finite_wing_factor. Separate control increments add up in JSBSim, omitting
  nonlinear interactions between simultaneous deflections. INAV servo
  output Sn (1000..2000 us, as -1..1 on
  SERVO_PROPERTY) maps -1 / 0 / +1 to deflection_min / 0 / deflection_max, through an actuator rate-limited to
  rate_deg_s.
- Rocket motors: JSBSim's solid rocket engine, thrust looked up by propellant burned, from the RASP curve as
  published, each ROCKET_CURVE_STEP_S step of it (linear in time, as RASP means it) at its mean thrust; Isp = total impulse / (propellant
  mass x g0), so the burn delivers the curve's impulse and ends when its tank (the propellant, a cylindrical grain at
  the casing centre) is empty. No buildup ramp (the curve has its own). The nozzle has zero exit area: the curve is
  measured thrust, used as given. A motor ignites when its throttle command is 1 (RocketEngine.throttle_property) and
  burns to the end. The mass balance is the dry airframe; the propellant mass lives in the tanks.
- Contact points: the tail end and every panel tip trailing edge, as STRUCTURE contacts; JSBSim decides which touch
  (it may lean onto wingtips that end short of the tail); a fixed-wing also gets the nose tip. Springs sized for CONTACT_SUPPORTING_POINTS carrying the weight. Start standing at stance_height_m: JSBSim's ground trim (do_simple_trim) crashes on a nose-up model.
"""

import ast
import copy
import dataclasses
import hashlib
import math
import pickle
import shutil
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb
import jsbsim
import neuralfoil
import numpy as np
import time
from threadpoolctl import threadpool_limits

from libs.aero import AERO_MODEL_SIZE, ForwardAero, build_asb_airplane, with_side_deflections
from libs.airframe import INCH_M, ElectricPropulsion, motor_repo, motorlib_motor
from libs.geometry import STANDARD_GRAVITY_M_S2, Inertia, ResolvedAirframe, RocketPlacement, Vec, dry_mass_properties
from libs.propulsion import MotorModel, motor_models, rpm_for_thrust, static_points
from libs.rocketmotors import RocketMotor

REPO_DIR = Path(__file__).resolve().parent.parent
EXPORT_ROOT = "export_jsbsim"  # the code an export runs is what this function reaches (export_code_digest)
EXPORT_FINGERPRINT_FILE = "export.fingerprint"  # in the export directory, next to the export it describes
EXPORT_CACHE_FILE = "export.pickle"  # the JSBSimExport of that export
PROP_J_TABLE_POINTS = 25
PROP_J_TABLE_END = 1.2  # of the zero-thrust advance ratio
APC_REFERENCE_PROP = "DJI_9450"  # JSBSim's bundled APC 9x4.5E table, for the Ct(J) and Cp(J) shapes
APC_REFERENCE_PITCH_RATIO = 4.5 / 9.0
AERO_ALPHA_DEG = np.concatenate((np.arange(-180.0, -30.0, 10.0), np.arange(-30.0, 31.0, 2.0), np.arange(40.0, 181.0, 10.0)))
AERO_BETA_DEG = np.array([-90.0, -60.0, -45.0, -30.0, -20.0, -15.0, -10.0, -7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0])
# Airspeeds the tables are solved at (Reynolds and Mach number: section polars, body and wave drag); JSBSim holds the
# end values beyond them. Independent of the definition's operating airspeed, which is a guess. An airframe with rocket
# motors adds ROCKET_AERO_SPEEDS_M_S, up to about Mach 0.9 where the subsonic models end (aero_speeds_m_s).
AERO_SPEEDS_M_S = np.array([5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0])
ROCKET_AERO_SPEEDS_M_S = np.array([75.0, 100.0, 150.0, 200.0, 250.0, 300.0])
FT_M = 0.3048
ALPHA_RAD_ROWS = ("aero/alpha-rad", np.radians(AERO_ALPHA_DEG))  # table_xml (property, breakpoints)
AIRSPEED_PROPERTY = "velocities/vt-fps"
CONTROL_COEFFICIENTS = ("CX", "CY", "CZ", "Cl", "Cm", "Cn")
FORWARD_ALPHA_LIMIT_DEG = 30.0
ENVELOPE_ALPHA_END_DEG = 45.0
FORWARD_BETA_LIMIT_DEG = 10.0
ENVELOPE_BETA_END_DEG = 20.0
AERO_PROVENANCE = "Lifting line + bodies in forward flight; AeroBuildup sideslip/rate effects and outer envelope"
# Deflections the control tables are solved at, each way. Dense near neutral: a section stalls past ~15-20 deg (and
# AeroSandbox's flap drag grows with deflection squared), so a coarse grid interpolates small deflections from a
# stalled column (half of +-45 deg gave 90x the real adverse yaw and a ninth of the roll at 5 deg).
CONTROL_DEFLECTION_GRID_DEG = (2.5, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0)
CONTROL_FACTOR_PROBE_DEG = 2.0  # finite_wing_factor: twist / control deflection
CONTROL_FACTOR_AOA_DEG = 2.0  # finite_wing_factor: AoA it is taken at (mean over the solved airspeeds)
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
ENGINE_THRUST_PROPERTY = "propulsion/engine[{index}]/thrust-lbs"
ROCKET_CURVE_STEP_S = 0.002
ROCKET_TABLE_DIGITS = 10  # burned-propellant rows near burnout differ in the 7th digit
KG_PER_LB = 0.45359237
N_PER_LBF = 4.4482216152605
# INAV quad X output order as (front, right) in the hover frame: rear-right, front-right, rear-left, front-left.
INAV_QUAD_X_ORDER = ((False, True), (True, True), (False, False), (True, False))


class ExportError(Exception):
    """The airframe is of a kind this export does not cover yet."""


@dataclass(frozen=True)
class RocketEngine:
    """A loaded rocket motor as a JSBSim engine: set throttle_property to 1 to ignite it; it burns to the end."""
    name: str  # RocketPlacement.name
    mount: int  # mount number
    designation: str
    throttle_property: str
    thrust_property: str  # lbf


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
    motors: tuple[MotorModel, ...]  # per motor in motor order (JSBSim engine order), their constants the JSBSim motors'; () when unpowered
    pack_voltage_v: float | None  # JSBSim maxvolts: battery power is V = throttle-pos x pack_voltage_v, I = (V - rpm/Kv) / R
    servo_properties: tuple[str, ...]  # driven by INAV servo output 0, 1, ... (S1, S2, ...), -1..1
    servo_rules: tuple[tuple[int, str, int, str], ...]  # (servo, role, sign, controls) for INAV
    forward_alpha_limit_deg: float  # full lifting-line correction; tighter for swept strips that reverse flow
    envelope_alpha_end_deg: float  # correction fades to pure AeroBuildup here
    rockets: tuple[RocketEngine, ...]  # in resolved rocket order, after the electric motors' engines
    top_table_speed_m_s: float  # the fastest solved airspeed; JSBSim holds the tables' end values beyond it


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
    increments: dict[str, np.ndarray]  # CONTROL_COEFFICIENTS (body axes) over the undeflected aircraft: (FlightAero.speeds_m_s, AERO_ALPHA_DEG, deflection_deg)
    derivatives: dict[str, float]  # per degree of deflection near neutral, mean over the airspeeds and AoA within CONTROL_DERIVATIVE_AOA_DEG of zero


@dataclass(frozen=True)
class ServoMix:
    rules: tuple[tuple[int, str, int, str], ...]  # (servo, role, rate sign, controls) per mixed role
    reversed: frozenset[int]  # control numbers wired reversed on their shared servo (Y-cable reverser)


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


def aero_speeds_m_s(resolved: ResolvedAirframe) -> np.ndarray:
    """The airspeeds the tables are solved at: AERO_SPEEDS_M_S, and ROCKET_AERO_SPEEDS_M_S when rocket motors are loaded."""
    return np.concatenate((AERO_SPEEDS_M_S, ROCKET_AERO_SPEEDS_M_S)) if resolved.rockets else AERO_SPEEDS_M_S


def airspeed_fps(speeds_m_s: np.ndarray) -> tuple[str, np.ndarray]:
    """table_xml (property, breakpoints) of the solved airspeeds."""
    return AIRSPEED_PROPERTY, speeds_m_s / FT_M


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


def table_rows(rows, digits: int = 6) -> str:
    return "\n".join("        " + "  ".join(f"{value:.{digits}g}" for value in row) for row in rows)


def body_coefficients(aero: dict, op_point: asb.OperatingPoint, resolved: ResolvedAirframe) -> np.ndarray:
    """(6, points) CONTROL_COEFFICIENTS of an AeroBuildup result, body axes."""
    reference = resolved.reference
    force_scale = np.asarray(op_point.dynamic_pressure()) * reference.area_m2
    lengths = (1.0, 1.0, 1.0, reference.span_m, reference.chord_m, reference.span_m)
    loads = [*aero["F_b"], *aero["M_b"]]
    return np.array([np.asarray(load) / (force_scale * length) for load, length in zip(loads, lengths)])


class FlightAero:
    """Full flight envelope with the editor's interacting-wing model at forward, zero-sideslip points.

    AeroBuildup supplies sideslip increments and damping. Normally the forward-model correction fades continuously
    outside ±30 alpha / ±10 beta, reaching pure AeroBuildup at ±45 alpha / ±20 beta. Swept strips that reverse flow
    inside ±30 alpha move the alpha blend earlier. The correction is held at its forward boundary before fading;
    lifting line is never solved in backwards flow.
    """

    def __init__(self, resolved: ResolvedAirframe):
        self.resolved = resolved
        self.speeds_m_s = aero_speeds_m_s(resolved)
        self.forward = ForwardAero(resolved)
        self.airplane = build_asb_airplane(resolved)
        self.atmosphere = asb.Atmosphere(altitude=resolved.airframe.operating_point.altitude_m)
        self.forward_alpha_limit_deg = FORWARD_ALPHA_LIMIT_DEG
        self.envelope_alpha_end_deg = ENVELOPE_ALPHA_END_DEG
        strip_forward = self.forward.mesh.forward
        # Geometry x points aft; a strip's forward vector points toward negative x.
        reverse_alpha_deg = float(np.min(np.degrees(np.arctan2(-strip_forward[:, 0], np.abs(strip_forward[:, 2])))))
        if reverse_alpha_deg <= FORWARD_ALPHA_LIMIT_DEG:
            self.envelope_alpha_end_deg = reverse_alpha_deg
            self.forward_alpha_limit_deg = max(0.0, reverse_alpha_deg - (ENVELOPE_ALPHA_END_DEG - FORWARD_ALPHA_LIMIT_DEG))

    @threadpool_limits.wrap(limits=1, user_api="blas")
    def run(self, speed, alpha, beta=0.0, deflections=None, envelope_deflections=None):
        speed, alpha, beta = np.broadcast_arrays(speed, alpha, beta)
        shape = speed.shape
        speed, alpha, beta = (value.ravel() for value in (speed, alpha, beta))
        airplane = self.airplane
        if deflections:
            envelope_deflections = envelope_deflections or deflections
            for wing, numbers in enumerate(self.forward.hybrid.controls):
                if any(number in deflections for number in numbers):
                    airplane = with_side_deflections(airplane, wing, *(envelope_deflections.get(number, 0.0) for number in numbers))
        weight = (np.clip((self.envelope_alpha_end_deg - np.abs(alpha)) / (self.envelope_alpha_end_deg - self.forward_alpha_limit_deg), 0, 1)
                  * np.clip((ENVELOPE_BETA_END_DEG - np.abs(beta)) / (ENVELOPE_BETA_END_DEG - FORWARD_BETA_LIMIT_DEG), 0, 1))
        exact = (weight == 1) & (beta == 0)
        values = np.empty((6, len(speed)))
        outer = ~exact
        if np.any(outer):
            point = asb.OperatingPoint(atmosphere=self.atmosphere, velocity=speed[outer], alpha=alpha[outer], beta=beta[outer])
            values[:, outer] = body_coefficients(asb.AeroBuildup(airplane=airplane, op_point=point, model_size=AERO_MODEL_SIZE).run(), point, self.resolved)
        for velocity in np.unique(speed[weight > 0]):
            indices = np.flatnonzero((speed == velocity) & (weight > 0))
            angles, inverse = np.unique(np.clip(alpha[indices], -self.forward_alpha_limit_deg, self.forward_alpha_limit_deg), return_inverse=True)
            wings, body = self.forward.run(self.atmosphere, velocity, angles, deflections)
            forward_point = asb.OperatingPoint(atmosphere=self.atmosphere, velocity=velocity, alpha=angles)
            forward_loads = {"F_b": wings.force_body_n + np.asarray(body["F_b"]), "M_b": wings.moment_body_nm + np.asarray(body["M_b"])}
            corrected = body_coefficients(forward_loads, forward_point, self.resolved)
            direct = exact[indices]
            values[:, indices[direct]] = corrected[:, inverse[direct]]
            if np.any(~direct):
                used, mapping = np.unique(inverse[~direct], return_inverse=True)
                point = asb.OperatingPoint(atmosphere=self.atmosphere, velocity=velocity, alpha=angles[used])
                baseline = body_coefficients(asb.AeroBuildup(airplane=airplane, op_point=point, model_size=AERO_MODEL_SIZE).run(), point, self.resolved)
                values[:, indices[~direct]] += (corrected[:, used] - baseline)[:, mapping] * weight[indices[~direct]]
        return values.reshape(6, *shape)


def aero_tables(resolved: ResolvedAirframe, model: FlightAero | None = None) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Body-axis coefficients on the airspeed x AoA x sideslip grid, and the damping derivatives on the airspeed x AoA
    grid at sideslip 0."""
    model = model or FlightAero(resolved)
    airplane, atmosphere = model.airplane, model.atmosphere
    speeds = model.speeds_m_s
    speed, alpha, beta = np.meshgrid(speeds, AERO_ALPHA_DEG, AERO_BETA_DEG, indexing="ij")
    values = model.run(speed, alpha, beta)
    tables = {name: values[index].reshape(speed.shape) for index, name in enumerate(CONTROL_COEFFICIENTS)}
    speed, alpha = np.meshgrid(speeds, AERO_ALPHA_DEG, indexing="ij")
    rates = asb.AeroBuildup(
        airplane=airplane, op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=speed.ravel(), alpha=alpha.ravel()), model_size=AERO_MODEL_SIZE,
    ).run_with_stability_derivatives(alpha=False, beta=False, p=True, q=True, r=True)
    dampers = {name: np.asarray(rates[name]).reshape(speed.shape) for name in ("Clp", "Cmq", "Cnr")}
    for name, table in (*tables.items(), *dampers.items()):
        if not np.all(np.isfinite(table)):
            invalid = np.argwhere(~np.isfinite(table))
            index = invalid[0]
            sideslip = f", sideslip {AERO_BETA_DEG[index[2]]:g} deg" if table.ndim == 3 else ""
            raise ValueError(f"Flight aero returned non-finite {name} at {speeds[index[0]]:g} m/s, "
                             f"AoA {AERO_ALPHA_DEG[index[1]]:g} deg{sideslip}; {len(invalid)} invalid grid points")
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


def control_tables(resolved: ResolvedAirframe, model: FlightAero | None = None) -> list[ControlTable]:
    """Each control's increments over the undeflected aircraft, shared FlightAero at sideslip 0.
    Only the outer-envelope AeroBuildup deflection uses finite_wing_factor. An XZ-mirrored pair is one ASB wing, each side deflected alone
    (SideControlSurface)."""
    if not resolved.controls:
        return []
    model = model or FlightAero(resolved)
    hybrid = model.forward.hybrid
    probe_point = asb.OperatingPoint(atmosphere=model.atmosphere, velocity=model.speeds_m_s, alpha=np.full(len(model.speeds_m_s), CONTROL_FACTOR_AOA_DEG))
    speed, alpha = np.meshgrid(model.speeds_m_s, AERO_ALPHA_DEG, indexing="ij")
    base = model.run(speed, alpha)
    near = np.abs(AERO_ALPHA_DEG) <= CONTROL_DERIVATIVE_AOA_DEG
    placements = {control.number: control for control in resolved.controls}
    tables = []
    for wing, (owner, numbers) in enumerate(zip(hybrid.owners, hybrid.controls)):
        if not any(numbers):
            continue
        surface_set = resolved.surfaces[owner]
        definition = surface_set.definition.control
        factor = finite_wing_factor(model.airplane, wing, lambda plane: body_coefficients(
            asb.AeroBuildup(airplane=plane, op_point=probe_point, model_size=AERO_MODEL_SIZE).run(), probe_point, resolved))
        deflections = np.unique([0.0, definition.deflection_min_deg, definition.deflection_max_deg,
                                 *(-d for d in CONTROL_DEFLECTION_GRID_DEG if -d > definition.deflection_min_deg),
                                 *(d for d in CONTROL_DEFLECTION_GRID_DEG if d < definition.deflection_max_deg)])
        sides: list[list[np.ndarray]] = [[], []]  # own, mirror: (6, points) per deflection
        for deflection in deflections:
            solved = float(deflection * factor)
            for side, (number, columns) in enumerate(zip(numbers, sides)):
                if number:
                    columns.append(model.run(speed, alpha, deflections={number: float(deflection)}, envelope_deflections={number: solved}) - base if deflection else np.zeros_like(base))
        neutral = int(np.flatnonzero(deflections == 0.0)[0])
        low, high = max(neutral - 1, 0), min(neutral + 1, len(deflections) - 1)  # the columns either side of neutral
        for number, columns in zip(numbers, sides):
            if not number:
                continue
            stacked = np.stack(columns, axis=-1).reshape(6, *speed.shape, len(deflections))  # (6, speed, AoA, deflection)
            if not np.all(np.isfinite(stacked)):
                raise ValueError(f"Flight aero returned non-finite control increments for C{number}")
            control = placements[number]
            tables.append(ControlTable(
                number=number, label=f"C{number} {control.surface} panel {control.panel_index + 1}", servo=int(control.servo[1:]) - 1,
                roles=definition.roles, deflection_min_deg=definition.deflection_min_deg, deflection_max_deg=definition.deflection_max_deg,
                rate_deg_s=definition.rate_deg_s, finite_wing_factor=factor, deflection_deg=deflections,
                increments={name: stacked[index] for index, name in enumerate(CONTROL_COEFFICIENTS)},
                derivatives={name: float(np.mean(stacked[index][:, near, high] - stacked[index][:, near, low]) / (deflections[high] - deflections[low]))
                             for index, name in enumerate(CONTROL_COEFFICIENTS)},
            ))
            print(f"    C{number} {control.surface} panel {control.panel_index + 1} solved", flush=True)
    return sorted(tables, key=lambda table: table.number)


def servo_mix(resolved: ResolvedAirframe, tables: list[ControlTable]) -> ServoMix:
    """Rule signs follow INAV's stabilized frame (forward-left-up: + roll = right wing down, + pitch = nose down, + yaw =
    nose left), i.e. JSBSim body Cl, -Cm, -Cn; flaps: + = more lift. A role a control cannot produce (moment under
    CONTROL_ROLE_MIN_FRACTION of its largest, e.g. pitch on a vertical fin) is simply not given to it. On a shared servo
    the first control sets each role's sign; a control opposite to it in every shared role is wired reversed."""
    reference = resolved.reference
    servo_signs: dict[int, dict[str, int]] = {}
    labels: dict[tuple[int, str], list[str]] = {}
    reversed_controls = set()
    for table in tables:
        d = table.derivatives
        moments = {"roll": d["Cl"] * reference.span_m, "pitch": -d["Cm"] * reference.chord_m, "yaw": -d["Cn"] * reference.span_m}
        signs = {}
        for role in table.roles:
            value, largest = (-d["CZ"], max(abs(d["CY"]), abs(d["CZ"]))) if role == "flaps" else (moments[role], max(abs(v) for v in moments.values()))
            if abs(value) >= CONTROL_ROLE_MIN_FRACTION * largest:
                signs[role] = 1 if value > 0.0 else -1
        servo = servo_signs.setdefault(table.servo, {})
        opposite = [role for role in signs if role in servo and signs[role] != servo[role]]
        same = [role for role in signs if role in servo and signs[role] == servo[role]]
        if opposite and same:
            raise ExportError(f"S{table.servo + 1}: {table.label} is opposite to the other controls on it in {', '.join(opposite)} "
                              f"but not in {', '.join(same)}; a reverser cannot wire that")
        reverse = bool(opposite)
        if reverse:
            reversed_controls.add(table.number)
        for role, sign in signs.items():
            servo.setdefault(role, -sign if reverse else sign)
            labels.setdefault((table.servo, role), []).append(f"{table.label}{' (reversed)' if reverse else ''}")
    rules = tuple((servo, role, servo_signs[servo][role], ", ".join(names)) for (servo, role), names in sorted(labels.items()))
    return ServoMix(rules=rules, reversed=frozenset(reversed_controls))


def control_fcs_xml(tables: list[ControlTable], reversed_controls: frozenset[int]) -> str:
    """Each control follows its servo, the other way round when reversed (servo_mix)."""
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
              -1  {table.deflection_max_deg if table.number in reversed_controls else table.deflection_min_deg:g}
               0  0
               1  {table.deflection_min_deg if table.number in reversed_controls else table.deflection_max_deg:g}
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


def control_aero_xml(tables: list[ControlTable], coefficient: str, factors: str, speeds_m_s: np.ndarray) -> str:
    return "".join(coefficient_xml(
        f"{coefficient}_control_{table.number}", f"{coefficient} increment of {table.label} (shared forward model, outer deflection x {table.finite_wing_factor:.3f})", factors,
        table_xml(("aero/alpha-deg", AERO_ALPHA_DEG), (CONTROL_POSITION_PROPERTY.format(number=table.number), table.deflection_deg), table.increments[coefficient], airspeed_fps(speeds_m_s)),
    ) for table in tables)


def aero_axis_xml(coefficient: str, table: np.ndarray, factors: str, speeds_m_s: np.ndarray) -> str:
    return coefficient_xml(coefficient, f"{coefficient} ({AERO_PROVENANCE}, body axes)", factors,
                           table_xml(ALPHA_RAD_ROWS, ("aero/beta-rad", np.radians(AERO_BETA_DEG)), table, airspeed_fps(speeds_m_s)))


def engine_xml(resolved: ResolvedAirframe, files: list[tuple[str, str]]) -> str:
    """One engine per motor, in motor order; files: (motor file, prop file) per motor."""
    return "".join(f"""
    <engine file="{motor_file}" name="{placement.name}">
      <thruster file="{prop_file}">
        <location unit="M"> <x> {placement.prop_center[0]:.5f} </x> <y> {placement.prop_center[1]:.5f} </y> <z> {placement.prop_center[2]:.5f} </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> {math.degrees(math.atan2(placement.thrust_dir[2], -placement.thrust_dir[0])):.3f} </pitch> <yaw> 0 </yaw> </orient>
        <sense> {1.0 if placement.spin == 'ccw' else -1.0} </sense>
        <p_factor> 0.0 </p_factor>
      </thruster>
    </engine>""" for placement, (motor_file, prop_file) in zip(resolved.motors, files, strict=True))


def write_engine_files(electric: ElectricPropulsion, model: MotorModel, engine_dir: Path, file_stem: str, pack_voltage_v: float | None) -> tuple[str, str]:
    """The JSBSim motor and propeller files of one motor model; returns their names. pack_voltage_v as for export_jsbsim."""
    diameter_m = model.diameter_m
    pack_voltage = model.pack_voltage_v if pack_voltage_v is None else pack_voltage_v
    ct0, cp0 = model.ct0, model.cp0
    shapes, apc_zero_j = apc_shape()
    zero_j = model.pitch_m / diameter_m * apc_zero_j / APC_REFERENCE_PITCH_RATIO
    j_grid = np.linspace(0.0, PROP_J_TABLE_END * zero_j, PROP_J_TABLE_POINTS)
    apc_j = j_grid * apc_zero_j / zero_j
    ct_table = ct0 * extended(apc_j, shapes["C_THRUST"])
    cp_table = cp0 * extended(apc_j, shapes["C_POWER"])
    prop_ixx = electric.prop_mass_kg * (diameter_m / 2.0) ** 2 / 3.0
    engine_dir.mkdir(parents=True, exist_ok=True)
    motor_file, prop_file = f"{file_stem}_motor", f"{file_stem}_prop"
    (engine_dir / f"{motor_file}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {model.label}, {model.prop}, {model.cells}S: motorlib's full-throttle static point ({model.full_rpm:.0f} RPM,
     {model.full_current_a:.1f} A) at {model.pack_voltage_v:.1f} V nominal; resistance = wiring {model.wiring.per_motor_ohm * 1000.0:.1f} mOhm (this motor's)
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


def write_rocket_engine_file(motor: RocketMotor, engine_dir: Path, file_stem: str) -> str:
    """The JSBSim solid rocket engine of one motor (see the module notes); returns its file name."""
    times_s = np.array([t for t, _f in motor.curve])
    times = np.union1d(np.arange(0.0, motor.burn_time_s, ROCKET_CURVE_STEP_S), times_s)
    thrust_n = np.interp(times, times_s, [f for _t, f in motor.curve])
    impulse_ns = np.concatenate(([0.0], np.cumsum((thrust_n[1:] + thrust_n[:-1]) / 2.0 * np.diff(times))))
    burned_kg = motor.propellant_kg * impulse_ns / impulse_ns[-1]
    isp_s = impulse_ns[-1] / (motor.propellant_kg * STANDARD_GRAVITY_M_S2)
    # Each step's mean thrust from where it starts burning: JSBSim burns propellant at thrust / Isp, so a row at 0 N
    # would never burn on (no ignition from a 0 N start, no burnout towards a 0 N end); the last step's mean holds
    # until the tank is empty, which ends the burn.
    means = (thrust_n[1:] + thrust_n[:-1]) / 2.0
    rows = [*zip(burned_kg[:-1] / KG_PER_LB, means / N_PER_LBF), (burned_kg[-1] / KG_PER_LB, means[-1] / N_PER_LBF)]
    engine_dir.mkdir(parents=True, exist_ok=True)
    (engine_dir / f"{file_stem}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder: {motor.manufacturer} {motor.designation} (data/rocket_motors/{motor.name}.eng), {impulse_ns[-1]:.3f} Ns, {motor.burn_time_s:g} s.
     Thrust (lbf) against propellant burned (lb): each {ROCKET_CURVE_STEP_S:g} s step of the RASP curve at its mean thrust; Isp = impulse / (propellant x g0). -->
<rocket_engine name="{motor.manufacturer} {motor.designation}">
  <isp> {isp_s:.4f} </isp>
  <thrust_table name="propulsion/thrust_prop_remain" type="internal">
    <tableData>
{table_rows(rows, ROCKET_TABLE_DIGITS)}
    </tableData>
  </thrust_table>
</rocket_engine>
""")
    return file_stem


def write_nozzle_file(engine_dir: Path, file_stem: str) -> str:
    """A zero exit-area nozzle: the motor curves are measured thrust, used without an ambient-pressure correction."""
    engine_dir.mkdir(parents=True, exist_ok=True)
    (engine_dir / f"{file_stem}.xml").write_text("""<?xml version="1.0"?>
<nozzle name="measured thrust">
  <area unit="FT2"> 0.0 </area>
</nozzle>
""")
    return file_stem


def rocket_xml(rockets: tuple[RocketPlacement, ...], files: dict[str, str], nozzle_file: str) -> tuple[str, str]:
    """(engines, tanks): one engine per rocket, fed by its own tank (tank i = rocket i), the propellant as a
    cylindrical grain filling the casing at its centre; files by motor name."""
    engines = "".join(f"""
    <engine file="{files[rocket.motor.name]}" name="{rocket.name}">
      <feed> {index} </feed>
      <thruster file="{nozzle_file}">
        <location unit="M"> <x> {rocket.nozzle[0]:.5f} </x> <y> {rocket.nozzle[1]:.5f} </y> <z> {rocket.nozzle[2]:.5f} </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> {math.degrees(math.atan2(rocket.thrust_dir[2], -rocket.thrust_dir[0])):.3f} </pitch> <yaw> 0 </yaw> </orient>
      </thruster>
    </engine>""" for index, rocket in enumerate(rockets))
    tanks = "".join(f"""
    <tank type="FUEL">  <!-- {rocket.name} propellant -->
      <location unit="M"> <x> {rocket.center[0]:.5f} </x> <y> {rocket.center[1]:.5f} </y> <z> {rocket.center[2]:.5f} </z> </location>
      <radius unit="M"> {rocket.motor.diameter_m / 2.0:.5f} </radius>
      <grain_config type="CYLINDRICAL">
        <length unit="M"> {rocket.motor.length_m:.5f} </length>
        <bore_diameter unit="M"> 0.0 </bore_diameter>
      </grain_config>
      <capacity unit="LBS"> {rocket.motor.propellant_kg / KG_PER_LB:.6f} </capacity>
      <contents unit="LBS"> {rocket.motor.propellant_kg / KG_PER_LB:.6f} </contents>
    </tank>""" for rocket in rockets)
    return engines, tanks


def feed_canonical(digest, value) -> None:
    """Feed a value's exact content to a hash: dataclasses field by field, arrays by their bytes, floats exactly."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        digest.update(type(value).__name__.encode())
        for field in dataclasses.fields(value):
            digest.update(field.name.encode())
            feed_canonical(digest, getattr(value, field.name))
    elif isinstance(value, np.ndarray):
        digest.update(f"array{value.dtype}{value.shape}".encode())
        digest.update(np.ascontiguousarray(value).tobytes())
    elif isinstance(value, (list, tuple)):
        digest.update(f"seq{len(value)}".encode())
        for item in value:
            feed_canonical(digest, item)
    elif isinstance(value, dict):
        digest.update(f"map{len(value)}".encode())
        for key in sorted(value, key=repr):
            feed_canonical(digest, key)
            feed_canonical(digest, value[key])
    else:
        digest.update(f"{type(value).__name__}:{value!r}".encode())


def module_index(path: Path) -> tuple[dict[str, ast.AST], dict[str, tuple[Path, str | None]]]:
    """A libs module's top-level definitions by name, and its libs imports: local name -> (module file, name), name
    None for a module itself."""
    tree = ast.parse(path.read_text())
    definitions, imports = {}, {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            definitions[node.name] = node
        elif isinstance(node, ast.Assign):
            definitions.update((target.id, node) for target in node.targets if isinstance(target, ast.Name))
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            definitions[node.target.id] = node
        elif isinstance(node, ast.ImportFrom):
            package = path.parent if node.level <= 1 else path.parents[node.level - 1]
            base = REPO_DIR if node.level == 0 else package
            module = base.joinpath(*(node.module or "").split(".")) if node.module else base
            if node.level == 0 and not (node.module or "").startswith("libs"):
                continue
            module_file = module.with_suffix(".py") if module.with_suffix(".py").is_file() else module / "__init__.py"
            for alias in node.names:
                submodule = module / f"{alias.name}.py"
                imports[alias.asname or alias.name] = (submodule, None) if submodule.is_file() else (module_file, alias.name)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("libs") and alias.asname:
                    imports[alias.asname] = (REPO_DIR.joinpath(*alias.name.split(".")).with_suffix(".py"), None)
    return definitions, imports


def without_docstrings(node: ast.AST) -> ast.AST:
    node = copy.deepcopy(node)
    for item in ast.walk(node):
        body = getattr(item, "body", None)
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and body and isinstance(body[0], ast.Expr) \
                and isinstance(body[0].value, ast.Constant) and isinstance(body[0].value.value, str):
            item.body = body[1:] or [ast.Pass()]
    return node


def export_code_digest() -> str:
    """Hash of the code an export runs: every top-level function, class and constant in libs reachable by name from
    EXPORT_ROOT, as parsed code without docstrings (comments, docstrings and formatting do not count, nor does code
    the export never reaches: the GUI, the sim runners, the report, other analyses)."""
    indexes: dict[Path, tuple[dict, dict]] = {}
    reached: dict[str, str] = {}
    pending = [(Path(__file__).resolve(), EXPORT_ROOT)]
    while pending:
        path, name = pending.pop()
        key = f"{path.relative_to(REPO_DIR)}:{name}"
        if key in reached or not path.is_file():
            continue
        if path not in indexes:
            indexes[path] = module_index(path)
        definitions, imports = indexes[path]
        if name in imports and name not in definitions:
            reached[key] = ""
            target = imports[name]
            if target[1] is not None:
                pending.append((target[0].resolve(), target[1]))
            continue
        node = definitions.get(name)
        if node is None:
            continue  # a builtin, a local name or a third-party name
        reached[key] = ast.dump(without_docstrings(node))
        for item in ast.walk(node):
            if isinstance(item, ast.Name):
                pending.append((path, item.id))
            elif isinstance(item, ast.Attribute) and isinstance(item.value, ast.Name) and imports.get(item.value.id, (None, ""))[1] is None \
                    and item.value.id in imports:
                pending.append((imports[item.value.id][0].resolve(), item.attr))
    digest = hashlib.sha256()
    for key in sorted(reached):
        digest.update(key.encode())
        digest.update(reached[key].encode())
    return digest.hexdigest()


def export_fingerprint(resolved: ResolvedAirframe, parts_data: dict, out_dir: Path, model_name: str, pack_voltage_v: float | None) -> str:
    """Hash of what decides an export: the resolved airframe as the export gets it (its name, notes, size preset (a
    label and limits) and the operating point's AoA and airspeed left out: they do not change the model), the
    library values it reads beyond that (the ESC, the connectors on the wire runs, motorlib's motor, prop and wire
    rows), the code it runs (export_code_digest), the AeroSandbox and NeuralFoil versions, the model name, the output
    directory and the pack voltage."""
    airframe = resolved.airframe
    operating_point = dataclasses.replace(airframe.operating_point, aoa_deg=0.0, airspeed_m_s=0.0)
    digest = hashlib.sha256(repr((model_name, str(out_dir), pack_voltage_v, asb.__version__, neuralfoil.__version__)).encode())
    feed_canonical(digest, dataclasses.replace(resolved, airframe=dataclasses.replace(airframe, name="", notes="", preset=None, operating_point=operating_point)))
    electric = None if airframe.propulsion is None else airframe.propulsion.electric
    if electric is not None:
        esc = next(part.part for part in airframe.parts if part.name == electric.esc).partition(":")[2]
        runs = (electric.wiring.battery, *electric.wiring.motors)
        plugs = {electric.battery.connector, *(plug for run in runs for plug in run.plugs)}
        gauges = {run.gauge_awg for run in runs} | ({electric.battery.lead_awg} if electric.battery.lead_awg is not None else set())
        repo = motor_repo()
        feed_canonical(digest, {
            "esc": parts_data["esc"][esc], "connectors": {plug: parts_data["connector"][plug] for plug in plugs},
            "motor": repr(motorlib_motor(electric.motor)), "prop": repr(repo.prop(electric.prop)),
            "wires": {gauge: repr(repo.wire(gauge)) for gauge in gauges},
        })
    digest.update(export_code_digest().encode())
    return digest.hexdigest()


def reuse_or_export(resolved: ResolvedAirframe, parts_data: dict, out_dir: Path, model_name: str,
                    pack_voltage_v: float | None = None) -> tuple[JSBSimExport, bool]:
    """(export, reused): the export already in out_dir when nothing it depends on changed (export_fingerprint), otherwise
    a fresh one into an emptied out_dir."""
    fingerprint = export_fingerprint(resolved, parts_data, out_dir, model_name, pack_voltage_v)
    fingerprint_path, cache_path = out_dir / EXPORT_FINGERPRINT_FILE, out_dir / EXPORT_CACHE_FILE
    if fingerprint_path.is_file() and fingerprint_path.read_text() == fingerprint and cache_path.is_file():
        with open(cache_path, "rb") as file:
            return pickle.load(file), True
    shutil.rmtree(out_dir, ignore_errors=True)  # nothing left over from an earlier model name
    print("JSBSim model: rebuilding for the current airframe and code; solving tables, this can take minutes", flush=True)
    started = time.monotonic()
    export = export_jsbsim(resolved, parts_data, out_dir, model_name, pack_voltage_v)
    print(f"JSBSim model: exported in {time.monotonic() - started:.0f} s", flush=True)
    with open(cache_path, "wb") as file:
        pickle.dump(export, file)
    fingerprint_path.write_text(fingerprint)  # last: an export cut short leaves no fingerprint, so it is redone
    return export, False


@threadpool_limits.wrap(limits=1, user_api="blas")
def export_jsbsim(resolved: ResolvedAirframe, parts_data: dict, out_dir: Path, model_name: str, pack_voltage_v: float | None = None) -> JSBSimExport:
    """pack_voltage_v: battery voltage for the motors (default: nominal). The motor command is the duty, so a lower pack
    gives proportionally less motor voltage per command, as an ESC."""
    airframe = resolved.airframe
    if airframe.vtol and not airframe.tailsitter:
        raise ExportError(f"{airframe.name}: INAV export of VTOL types other than tailsitter is not implemented")
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    aircraft_dir = out_dir / "aircraft" / model_name
    aircraft_dir.mkdir(parents=True, exist_ok=True)
    models: tuple[MotorModel, ...] = ()
    pack_voltage = hover_fraction = hover_stick_us = None
    engines_xml = ""
    if resolved.motors:
        models = motor_models(resolved, parts_data)
        pack_voltage = models[0].pack_voltage_v if pack_voltage_v is None else pack_voltage_v
        distinct = list(dict.fromkeys(models))  # motors wired alike share their files
        written = [write_engine_files(airframe.propulsion.electric, model, out_dir / "engine", f"{model_name}_{index + 1}", pack_voltage_v) for index, model in enumerate(distinct)]
        engines_xml = engine_xml(resolved, [written[distinct.index(model)] for model in models])
        if airframe.vtol:  # hover: weight shared by the motors; the mean of the duties that hold that thrust's static RPM
            hover_fraction = float(np.mean([static_points(model, rpm_for_thrust(model, weight_n / len(models)), pack_voltage).duty for model in models]))
            idle = INAV_THROTTLE_IDLE_PCT / 100.0
            hover_stick_us = int(round(1000.0 + 1000.0 * (hover_fraction - idle) / (1.0 - idle)))
    rocket_engines: tuple[RocketEngine, ...] = ()
    tanks_xml = ""
    if resolved.rockets:
        nozzle_file = write_nozzle_file(out_dir / "engine", f"{model_name}_nozzle")
        motors = {rocket.motor.name: rocket.motor for rocket in resolved.rockets}
        files = {name: write_rocket_engine_file(motor, out_dir / "engine", f"{model_name}_rocket_{index + 1}") for index, (name, motor) in enumerate(motors.items())}
        rockets_xml, tanks_xml = rocket_xml(resolved.rockets, files, nozzle_file)
        engines_xml += rockets_xml
        rocket_engines = tuple(RocketEngine(name=rocket.name, mount=rocket.mount, designation=rocket.motor.designation,
                                            throttle_property=MOTOR_THROTTLE_PROPERTY.format(index=len(models) + index),
                                            thrust_property=ENGINE_THRUST_PROPERTY.format(index=len(models) + index))
                               for index, rocket in enumerate(resolved.rockets))
    cg = resolved.cg_m  # the aero reference point; the mass balance is the dry airframe
    dry_kg, dry_cg, dry_inertia = dry_mass_properties(resolved)
    ixx, iyy, izz, ixy, ixz, iyz = jsbsim_inertia(dry_inertia)
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
    aero_model = FlightAero(resolved)
    print("  body aero tables...", flush=True)
    stage = time.monotonic()
    tables, rates = aero_tables(resolved, aero_model)
    print(f"  body aero tables done ({time.monotonic() - stage:.0f} s)", flush=True)
    print(f"  control tables ({len(resolved.controls)} controls)...", flush=True)
    stage = time.monotonic()
    controls = control_tables(resolved, aero_model)
    print(f"  control tables done ({time.monotonic() - stage:.0f} s)", flush=True)
    servos = servo_mix(resolved, controls)
    speeds = aero_model.speeds_m_s
    forces = {"X": "CX", "Y": "CY", "Z": "CZ"}
    moments = {"ROLL": ("Cl", "metrics/bw-ft"), "PITCH": ("Cm", "metrics/cbarw-ft"), "YAW": ("Cn", "metrics/bw-ft")}
    dampers = {"ROLL": ("Clp", "metrics/bw-ft", "aero/bi2vel", "velocities/p-aero-rad_sec"),
               "PITCH": ("Cmq", "metrics/cbarw-ft", "aero/ci2vel", "velocities/q-aero-rad_sec"),
               "YAW": ("Cnr", "metrics/bw-ft", "aero/bi2vel", "velocities/r-aero-rad_sec")}
    axes_xml = "".join(f"""
    <axis name="{axis}">{aero_axis_xml(name, tables[name], "<property>aero/qbar-area</property>", speeds)}{control_aero_xml(controls, name, "<property>aero/qbar-area</property>", speeds)}
    </axis>""" for axis, name in forces.items())
    for axis, (name, length) in moments.items():
        damper, damper_length, rate_scale, rate = dampers[axis]
        damping_xml = coefficient_xml(
            damper, f"{damper} (AeroBuildup, sideslip 0)",
            f"<property>aero/qbar-area</property> <property>{damper_length}</property> <property>{rate_scale}</property> <property>{rate}</property>",
            table_xml(ALPHA_RAD_ROWS, airspeed_fps(speeds), rates[damper].T),
        )
        axes_xml += f"""
    <axis name="{axis}">{aero_axis_xml(name, tables[name], f"<property>aero/qbar-area</property> <property>{length}</property>", speeds)}{damping_xml}{control_aero_xml(controls, name, f"<property>aero/qbar-area</property> <property>{length}</property>", speeds)}
    </axis>"""
    reference = resolved.reference
    location = f"<x> {cg[0]:.5f} </x> <y> {cg[1]:.5f} </y> <z> {cg[2]:.5f} </z>"
    dry_location = f"<x> {dry_cg[0]:.5f} </x> <y> {dry_cg[1]:.5f} </y> <z> {dry_cg[2]:.5f} </z>"
    (aircraft_dir / f"{model_name}.xml").write_text(f"""<?xml version="1.0"?>
<!-- quadrocketbuilder export of {airframe.name}; see libs/jsbsim.py for the model choices. -->
<fdm_config name="{model_name}" version="2.0" release="ALPHA">
  <fileheader>
    <author>quadrocketbuilder</author>
    <description>{airframe.name}: {"tailsitter, body x = nose = thrust axis" if airframe.tailsitter else "fixed-wing" if resolved.motors else "rocket" if resolved.rockets else "glider"}</description>
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
    <emptywt unit="KG"> {dry_kg:.5f} </emptywt>
    <location name="CG" unit="M"> {dry_location} </location>
  </mass_balance>
  <ground_reactions>{contacts_xml}
  </ground_reactions>
  <propulsion>{engines_xml}{tanks_xml}
  </propulsion>{control_fcs_xml(controls, servos.reversed)}
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
        for rule, (servo, role, sign, labels) in enumerate(servos.rules):
            source, frame_sign = HOVER_SERVO_INPUTS[role] if hover else (INAV_SERVO_INPUTS[role], 1)
            lines.append(f"smix {rule} {servo} {source} {sign * frame_sign * INAV_SMIX_RATE} 0 -1    # S{servo + 1} {role}: {labels}")
        return lines

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
        stance_height_m=max(point[0] for point in contacts) - cg[0] if airframe.tailsitter else None, motors=models, pack_voltage_v=pack_voltage,
        servo_properties=tuple(SERVO_PROPERTY.format(index=index) for index in range(max((table.servo for table in controls), default=-1) + 1)),
        servo_rules=servos.rules,
        forward_alpha_limit_deg=aero_model.forward_alpha_limit_deg,
        envelope_alpha_end_deg=aero_model.envelope_alpha_end_deg,
        rockets=rocket_engines,
        top_table_speed_m_s=float(speeds[-1]),
    )
