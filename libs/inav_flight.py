"""Shared RC pilots and flight measurements for Python INAV scenarios."""

import math
from dataclasses import dataclass

from inavbench import Bench, Catapult
from libs.jsbsim import INAV_THROTTLE_IDLE_PCT
from libs.simrun import FT, KNOT_M_S

STAND_S = 2.0
FORWARD_MIXER_PROFILE = 1  # INAV mixer profile 2
TAKEOFF_ALT_M = 60.0  # --start ground: the climb ends here, then the transition
GLIDE_SPEED_GAIN_DEG_PER_M_S = 3.0  # the gliding pilot's pitch per m/s of airspeed above its target
TAKEOFF_CLIMB_STICK_ABOVE_HOVER = 0.15
TAKEOFF_MAX_S = 30.0
LAUNCH_WAIT_S = 1.0  # the bench's own launch recipe: arm and run the motor up while held (examples/launch.py:51)
LAUNCH_THROTTLE_CATAPULT = 0.8  # examples/launch.py:51
LAUNCH_THROTTLE_RAIL = 1.0
CLIMB_STOP_S = 1.0  # hover throttle before the transition, so it does not carry the climb into the pitch-over
TRANSITION_DONE_PITCH_DEG = 20.0
TRANSITION_DONE_CLIMB_M_S = 1.0
TRANSITION_MOTOR = 0.70  # motor command the throttle blends to as the nose comes down in the transition
ATTITUDE_SETTLE_DEG = 3.0  # arm once INAV's roll/pitch agree with truth this closely after the profile switch
ATTITUDE_SETTLE_MAX_S = 20.0
TOP_SPEED_MAX_S = 60.0
TOP_SPEED_MIN_S = 15.0
TOP_SPEED_PLATEAU_S = 5  # stop once airspeed rose less than TOP_SPEED_PLATEAU_M_S over this many seconds
TOP_SPEED_PLATEAU_M_S = 0.3
TOP_SPEED_CLIMB_LIMIT_M_S = 0.5
TOP_SPEED_ROLL_LIMIT_DEG = 5.0
LEVEL_THROTTLE_START = 0.5  # throttle stick the speed loop starts from
LEVEL_THROTTLE_P_PER_M_S = 0.05
LEVEL_THROTTLE_I_PER_M_S_S = 0.02
LEVEL_THROTTLE_RAMP_PER_S = 1.0  # the throttle stick moves at most this much per second (0 to full in 1 s)
PILOT_PITCH_LIMITS_DEG = (-15.0, 80.0)


@dataclass
class AltitudePilot:
    """Script pilot on the rate sticks (ACRO): altitude via pitch, wings level, heading hold. Stick pitch + is nose
    down, roll + right, yaw + nose right (RC conventions); JSBSim q is nose-up positive. Yaw is a rate request from the
    heading error alone: INAV's yaw rate loop does the damping, and a rate term of the pilot's own on top of it, through
    the bench's loop delay, sustains a 5 Hz yaw oscillation once something kicks it (the transition does)."""
    target_alt_m: float
    target_heading_deg: float
    alt_gain_deg_per_m: float = 2.0
    climb_gain_deg_per_m_s: float = 2.0
    pitch_limits_deg: tuple[float, float] = PILOT_PITCH_LIMITS_DEG
    angle_gain_per_deg: float = 0.02
    rate_gain_per_deg_s: float = 0.002

    def pitch_command_deg(self, fdm) -> float:
        climb_m_s = -fdm["velocities/v-down-fps"] * FT
        return self.alt_gain_deg_per_m * (self.target_alt_m - fdm["position/h-agl-ft"] * FT) - self.climb_gain_deg_per_m_s * climb_m_s

    def sticks(self, fdm) -> dict[str, float]:
        pitch_cmd = min(max(self.pitch_command_deg(fdm), self.pitch_limits_deg[0]), self.pitch_limits_deg[1])
        heading_error = (self.target_heading_deg - fdm["attitude/psi-deg"] + 180.0) % 360.0 - 180.0
        pitch = -(self.angle_gain_per_deg * (pitch_cmd - fdm["attitude/theta-deg"]) - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/q-rad_sec"]))
        roll = self.angle_gain_per_deg * -fdm["attitude/phi-deg"] - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/p-rad_sec"])
        yaw = self.angle_gain_per_deg * heading_error
        return {axis: min(max(value, -1.0), 1.0) for axis, value in (("pitch", pitch), ("roll", roll), ("yaw", yaw))}


@dataclass
class SpeedPilot(AltitudePilot):
    """AltitudePilot that holds an airspeed with pitch instead of an altitude (gliding): nose up when fast, down when slow."""
    target_speed_m_s: float = 0.0
    speed_gain_deg_per_m_s: float = GLIDE_SPEED_GAIN_DEG_PER_M_S

    def pitch_command_deg(self, fdm) -> float:
        return self.speed_gain_deg_per_m_s * (fdm["velocities/vt-fps"] * FT - self.target_speed_m_s)


def motor_fraction(us: int) -> float:
    return (us - 1000) / 1000.0


def report(bench: Bench, phase: str, motor_count: int) -> None:
    truth = bench.truth()
    altitude_m, _ = bench.altitude()
    motors = [motor_fraction(us) for us in bench.motors[:motor_count]]
    print(
        f"{bench.elapsed:5.1f}s {phase:<9} agl {truth['agl_m']:6.2f} m (FC {altitude_m:6.2f})  tilt {90.0 - truth['pitch']:5.1f} deg  "
        f"drift {math.hypot(truth['north_m'], truth['east_m']):5.2f} m  climb {-truth['vd']:+5.2f} m/s  "
        f"FC roll {bench.fc.roll:+6.1f} pitch {bench.fc.pitch:+6.1f}  motors {' '.join(f'{m:.3f}' for m in motors)}  "
        f"{'ARMED' if bench.fc.armed else 'disarmed'}"
    )


def fly(bench: Bench, seconds: float, phase: str, motor_count: int) -> list[float]:
    """Run, reporting once a second; returns the mean motor command per report."""
    means = []
    for _ in range(int(round(seconds))):
        bench.run(1.0)
        report(bench, phase, motor_count)
        means.append(sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count)
    return means


@dataclass(frozen=True)
class LevelPoint:
    target_m_s: float
    motor: float
    airspeed_m_s: float
    aoa_deg: float
    pitch_deg: float
    power_w: float
    altitude_error_m: float
    climb_m_s: float


@dataclass
class SpeedHold:
    """Throttle stick (PI) holding a target airspeed, from LEVEL_THROTTLE_START at release, its changes limited to
    LEVEL_THROTTLE_RAMP_PER_S."""
    target_m_s: float
    integral: float = 0.0
    stick: float = LEVEL_THROTTLE_START

    def throttle(self, airspeed_m_s: float, dt: float) -> float:
        error = self.target_m_s - airspeed_m_s
        wanted = LEVEL_THROTTLE_START + LEVEL_THROTTLE_P_PER_M_S * error + LEVEL_THROTTLE_I_PER_M_S_S * self.integral
        step = LEVEL_THROTTLE_RAMP_PER_S * dt
        stick = min(max(wanted, self.stick - step, 0.0), self.stick + step, 1.0)
        if stick == wanted:  # no wind-up while the ramp or the stops hold the stick back
            self.integral += error * dt
        self.stick = stick
        return stick


def enter_forward_profile(bench: Bench) -> None:
    """Tailsitter: switch to mixer profile 2 and wait for INAV's attitude estimate to swing round to its sensor frame."""
    bench.mode("MIXER PROFILE 2")
    if not bench.run(5.0, until=lambda b: b.mixer_profile == FORWARD_MIXER_PROFILE):
        raise RuntimeError(f"INAV did not switch to mixer profile 2 (active profile index {bench.mixer_profile})")

    def attitude_settled(b: Bench) -> bool:
        truth = b.truth()
        return abs(b.fc.roll - truth["roll"]) < ATTITUDE_SETTLE_DEG and abs(b.fc.pitch - truth["pitch"]) < ATTITUDE_SETTLE_DEG

    # the sensor frame changes with the profile; INAV's attitude estimate has to swing round to it
    if not bench.run(ATTITUDE_SETTLE_MAX_S, until=attitude_settled):
        raise RuntimeError(f"INAV attitude did not settle: FC roll {bench.fc.roll:+.1f} pitch {bench.fc.pitch:+.1f}, truth {bench.truth()}")
    print(f"profile 2 active, INAV attitude settled (roll {bench.fc.roll:+.1f}, pitch {bench.fc.pitch:+.1f}) after {bench.elapsed:.1f} s")


class ForwardFlight:
    """Level forward flight: the one part of a scenario that depends on the airframe type. Scenarios set the throttle
    (and in controls one stick) through sticks(), every tick.
    Fixed-wing: INAV ANGLE, plus NAV ALTHOLD when powered, sticks centred; the throttle stick sets the power
    (PLANE_TEMPLATE). Tailsitter: its forward mixer profile in ACRO, the script pilot holding altitude, wings level
    and heading."""

    def __init__(self, airframe) -> None:
        self.tailsitter = airframe.tailsitter
        self.powered = airframe.powered
        self.pilot: AltitudePilot | None = None

    def air_start_speed_m_s(self, operating_m_s: float) -> float:
        """Airspeed of the held air start. A tailsitter starts from rest: held at speed, INAV gets moving GPS with an
        IMU at rest, and its multirotor attitude estimate (the forward profile is a multirotor platform) drifts. A
        fixed-wing starts at the operating airspeed."""
        return 0.0 if self.tailsitter else operating_m_s

    def hands_on(self, bench: Bench, on: bool) -> None:
        """A stick step flies the aircraft directly: a fixed-wing leaves NAV ALTHOLD (it turns the pitch stick into a
        climb-rate request) for plain ANGLE while on; a tailsitter's pilot is overridden on the stepped axis by sticks()."""
        if not self.tailsitter and self.powered:
            bench.mode("NAV ALTHOLD", on=not on)

    def hold_airborne(self, bench: Bench) -> None:
        """Take up the hold of an air start, without arming (a launcher hand-over is already armed): a fixed-wing in
        INAV ANGLE, plus NAV ALTHOLD when powered; a tailsitter in its forward mixer profile in ACRO with the script
        pilot holding altitude and heading."""
        if self.tailsitter:
            fdm = bench.plant.fdm
            self.pilot = AltitudePilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
        else:
            bench.mode("ANGLE")
            if self.powered:
                bench.mode("NAV ALTHOLD")

    def start_airborne(self, bench: Bench) -> None:
        """From the held start (level at START_ALT_M and the start airspeed): arm, take up the hold, release."""
        if self.tailsitter:
            enter_forward_profile(bench)
        bench.arm()
        self.hold_airborne(bench)
        bench.release()

    def launch_off(self, bench: Bench) -> None:
        """A fixed wing on its launcher, the bench's own plane recipe (examples/launch.py): launch() arms and runs
        the motor up while held, the catapult fires or the rail's own thrust runs it up, and it is off. Off the rail
        it hands straight to the ordinary air-start hold (hold_airborne; arm() is not run again, it would zero the
        launch throttle)."""
        launcher = bench.plant.rail.launcher
        bench.launch(throttle=LAUNCH_THROTTLE_CATAPULT if isinstance(launcher, Catapult) else LAUNCH_THROTTLE_RAIL,
                     wait_s=LAUNCH_WAIT_S, modes=())
        truth = bench.truth()
        print(f"{bench.elapsed:5.1f}s off the {'catapult' if isinstance(launcher, Catapult) else 'rail'}: "
              f"airspeed {truth['ias_kts'] * KNOT_M_S:.1f} m/s, height {truth['agl_m']:.1f} m")
        # bench.launch(..., modes=()) keeps it in ACRO through the hold and the rail run; never switch to ANGLE
        # before the vehicle is off the rail: ANGLE would wind up trying to right itself against the rail attitude.
        self.hold_airborne(bench)

    def take_off(self, bench: Bench, export) -> None:
        """Tailsitter, standing: arm, climb in ANGLE straight to TAKEOFF_ALT_M, switch to the forward profile in ACRO;
        the pilot pitches over and holds that altitude while the throttle blends from hover to TRANSITION_MOTOR as the
        nose comes down (vertical thrust stays near the weight), until level and settled."""
        fdm = bench.plant.fdm
        hover_stick = (export.hover_stick_us - 1000) / 1000.0
        idle = INAV_THROTTLE_IDLE_PCT / 100.0
        bench.release()  # stand on the tail contacts
        bench.run(STAND_S)
        bench.mode("ANGLE")
        bench.arm()
        start = bench.elapsed
        bench.sticks(throttle=min(1.0, hover_stick + TAKEOFF_CLIMB_STICK_ABOVE_HOVER))
        max_tilt = 0.0

        def climbed(b: Bench) -> bool:
            nonlocal max_tilt
            max_tilt = max(max_tilt, 90.0 - b.truth()["pitch"])
            return b.truth()["agl_m"] >= TAKEOFF_ALT_M

        if not bench.run(TAKEOFF_MAX_S, until=climbed):
            raise RuntimeError(f"takeoff did not reach {TAKEOFF_ALT_M} m in {TAKEOFF_MAX_S} s: agl {bench.truth()['agl_m']:.1f} m")
        print(f"takeoff: {TAKEOFF_ALT_M:.0f} m in {bench.elapsed - start:.1f} s, max tilt {max_tilt:.1f} deg")
        bench.sticks(throttle=hover_stick)
        bench.run(CLIMB_STOP_S)

        # transition: forward profile, ACRO; the pilot pitches over gently and holds this altitude while the throttle
        # blends from hover to TRANSITION_MOTOR as the nose comes down (vertical thrust stays near the weight)
        pilot = AltitudePilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
        cruise_stick = (TRANSITION_MOTOR - idle) / (1.0 - idle)
        bench.mode("ANGLE", on=False)
        bench.mode("MIXER PROFILE 2")
        start = bench.elapsed
        lowest = highest = bench.truth()["agl_m"]
        while True:
            nose_down = min(max(1.0 - fdm["attitude/theta-deg"] / 90.0, 0.0), 1.0)
            bench.sticks(throttle=hover_stick + (cruise_stick - hover_stick) * nose_down, **pilot.sticks(fdm))
            bench.run(0.02)
            altitude = bench.truth()["agl_m"]
            lowest, highest = min(lowest, altitude), max(highest, altitude)
            if (bench.mixer_profile == FORWARD_MIXER_PROFILE and abs(fdm["attitude/theta-deg"]) < TRANSITION_DONE_PITCH_DEG
                    and abs(bench.truth()["vd"]) < TRANSITION_DONE_CLIMB_M_S):
                break
            if bench.elapsed - start > TAKEOFF_MAX_S:
                raise RuntimeError(f"transition did not settle: pitch {fdm['attitude/theta-deg']:.1f} deg, climb {-bench.truth()['vd']:+.1f} m/s, profile {bench.mixer_profile}")
        print(f"transition: level and settled after {bench.elapsed - start:.1f} s at {fdm['velocities/vt-fps'] * FT:.1f} m/s; "
              f"altitude {lowest - pilot.target_alt_m:+.1f} / {highest - pilot.target_alt_m:+.1f} m around {pilot.target_alt_m:.1f} m")
        # the heading taken nose-up at the start of the transition is meaningless (Euler heading near 90 deg pitch): the
        # pilot would hunt for it with the yaw stick; hold the heading the aircraft levelled out on instead
        self.pilot = AltitudePilot(target_alt_m=pilot.target_alt_m, target_heading_deg=fdm["attitude/psi-deg"])

    def glide(self, bench: Bench, speed_m_s: float) -> SpeedPilot:
        """Rate mode for both types with the gliding pilot holding speed_m_s: a fixed-wing leaves ANGLE (and NAV
        ALTHOLD); motors are left at idle by the caller's throttle."""
        if not self.tailsitter:
            bench.mode("ANGLE", on=False)
            if self.powered:
                bench.mode("NAV ALTHOLD", on=False)
        self.pilot = SpeedPilot(target_alt_m=0.0, target_heading_deg=bench.plant.fdm["attitude/psi-deg"], target_speed_m_s=speed_m_s)
        return self.pilot

    def sticks(self, bench: Bench, throttle: float, override: dict[str, float] | None = None) -> None:
        axes = self.pilot.sticks(bench.plant.fdm) if self.pilot is not None else {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        bench.sticks(throttle=throttle, **(axes | (override or {})))


def level_line(bench: Bench, motor_count: int) -> str:
    """One status line of forward flight: speed, height, attitude, AoA, motors, rpm, profile."""
    fdm = bench.plant.fdm
    motors = " ".join(f"{motor_fraction(us):.2f}" for us in bench.motors[:motor_count])
    rpm = f"  rpm {fdm['propulsion/engine[0]/propeller-rpm']:6.0f}" if motor_count else ""
    return (f"{bench.elapsed:5.1f}s airspeed {fdm['velocities/vt-fps'] * FT:5.1f} m/s  agl {fdm['position/h-agl-ft'] * FT:6.1f} m  "
            f"climb {-fdm['velocities/v-down-fps'] * FT:+5.1f}  pitch {fdm['attitude/theta-deg']:+5.1f}  roll {fdm['attitude/phi-deg']:+6.1f}  "
            f"heading {fdm['attitude/psi-deg']:5.1f}  AoA {fdm['aero/alpha-deg']:+5.1f}  motors {motors}{rpm}  "
            + (f"profile {bench.mixer_profile + 1 if bench.mixer_profile is not None else 1}  {'ARMED' if bench.fc.armed else 'disarmed'}"
               if isinstance(bench, Bench) else "direct pilot"))


def accelerate(bench: Bench, flight: ForwardFlight, motor_count: int) -> float:
    """Full throttle in level flight until the airspeed stops rising (TOP_SPEED_PLATEAU_M_S over TOP_SPEED_PLATEAU_S,
    after TOP_SPEED_MIN_S); returns the top speed."""
    fdm = bench.plant.fdm
    speeds = []
    level = []
    start = bench.elapsed
    next_report = start + 1.0
    while bench.elapsed - start < TOP_SPEED_MAX_S:
        flight.sticks(bench, 1.0)
        bench.run(0.02)
        if bench.elapsed < next_report:
            continue
        next_report += 1.0
        speeds.append(fdm["velocities/vt-fps"] * FT)
        level.append(abs(bench.truth()["vd"]) < TOP_SPEED_CLIMB_LIMIT_M_S and abs(bench.truth()["roll"]) < TOP_SPEED_ROLL_LIMIT_DEG)
        print(level_line(bench, motor_count))
        if (bench.elapsed - start >= TOP_SPEED_MIN_S and len(speeds) > TOP_SPEED_PLATEAU_S
                and abs(speeds[-1] - speeds[-1 - TOP_SPEED_PLATEAU_S]) < TOP_SPEED_PLATEAU_M_S
                and all(level[-TOP_SPEED_PLATEAU_S:])):
            return sum(speeds[-TOP_SPEED_PLATEAU_S:]) / TOP_SPEED_PLATEAU_S
    raise RuntimeError(f"Full-throttle level speed did not settle in {TOP_SPEED_MAX_S:g} s; last speed {speeds[-1]:.1f} m/s, climb {-bench.truth()['vd']:+.1f} m/s")

