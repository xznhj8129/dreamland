"""Raw simulated hardware readings, mount geometry, and PWM units shared by every controller."""

import math
import random
from dataclasses import dataclass

# The simulated parts (datasheet scales)
STANDARD_GRAVITY_M_S2 = 9.80665
ACCEL_LSB_PER_G = 4096.0  # ICM-20602 class SPI IMU, accelerometer at +-8 g
GYRO_LSB_PER_DPS = 16.4  # its gyro at +-2000 deg/s
INT16_RANGE = (-32768, 32767)  # IMU registers saturate here
MAG_LSB_PER_GAUSS = 1090.0  # HMC5883L at its default gain
EARTH_FIELD_GAUSS = 0.5  # the simulated field: level, pointing north, no dip
AIR_DENSITY_KG_M3 = 1.225  # pitot: differential pressure 0.5 rho CAS^2
RANGEFINDER_MAX_M = 10.0  # VL53L1X class, looking down the body's -z
PWM_MIN_US, PWM_MID_US, PWM_MAX_US = 1000, 1500, 2000
GPS_FIX_3D = 3  # u-blox NAV-PVT fixType
GPS_SATELLITES = 14
KNOT_M_S = 0.514444
FT = 0.3048


def _noisy(value: float, sigma: float) -> float:
    return value + random.gauss(0.0, sigma) if sigma else value


def _cross(a, b) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _int16(value: float) -> int:
    return int(min(max(round(value), INT16_RANGE[0]), INT16_RANGE[1]))


class Imu:
    """SPI IMU: accel() and gyro() return raw int16 counts (ACCEL_LSB_PER_G, GYRO_LSB_PER_DPS). Level and at rest it
    reads +ACCEL_LSB_PER_G on z; gyro x + = rolling right, y + = nose down, z + = nose left (in its own axes)."""

    def __init__(self) -> None:
        self._accel = (0, 0, 0)
        self._gyro = (0, 0, 0)
        self._offset_m = (0.0, 0.0, 0.0)  # from the CG, body axes (x forward, y left, z up)
        self._rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))  # its axes in the body's (rows)
        self._previous: tuple[int, tuple[float, float, float]] | None = None  # (time_us, rad/s) of the last sample

    def _mount(self, offset_m: tuple[float, float, float], rotation: tuple[tuple[float, float, float], ...]) -> None:
        """Where the runner put it: offset from the CG and its axes, both in the body's (x forward, y left, z up)."""
        self._offset_m, self._rotation = offset_m, rotation

    def accel(self) -> tuple[int, int, int]:
        return self._accel

    def gyro(self) -> tuple[int, int, int]:
        return self._gyro

    def _sample(self, plant, noise: dict, time_us: int) -> None:
        fdm = plant.fdm
        # Physical body axes, independent of the firmware's active mixer/sensor frame.
        if plant.rail and plant.rail.held:
            phi, theta = fdm["attitude/phi-rad"], fdm["attitude/theta-rad"]
            accel_g = [math.sin(theta), math.sin(phi) * math.cos(theta), math.cos(phi) * math.cos(theta)]
        else:
            accel_g = [fdm["accelerations/Nx"], -fdm["accelerations/Ny"], fdm["accelerations/Nz"]]
        rate = [fdm["velocities/p-rad_sec"], -fdm["velocities/q-rad_sec"], -fdm["velocities/r-rad_sec"]]
        previous, self._previous = self._previous, (time_us, tuple(rate))
        lever = _cross(rate, _cross(rate, self._offset_m))  # centripetal, then angular acceleration from the last sample
        if previous is not None and time_us > previous[0]:
            angular_accel = [(now - before) / ((time_us - previous[0]) / 1e6) for now, before in zip(rate, previous[1])]
            lever = [c + a for c, a in zip(lever, _cross(angular_accel, self._offset_m))]
        accel_g = [a + extra / STANDARD_GRAVITY_M_S2 for a, extra in zip(accel_g, lever)]
        own = [[sum(row[axis] * vector[i] for i, row in enumerate(self._rotation)) for axis in range(3)] for vector in (accel_g, rate)]  # R^T v
        self._accel = tuple(_int16(_noisy(g, noise.get("accel_g", 0.0)) * ACCEL_LSB_PER_G) for g in own[0])
        self._gyro = tuple(_int16(_noisy(math.degrees(w), noise.get("gyro_dps", 0.0)) * GYRO_LSB_PER_DPS) for w in own[1])


class Baro:
    """Barometer: pressure() in Pa, as a BMP280 driver returns it."""

    def __init__(self) -> None:
        self._pa = 0.0

    def pressure(self) -> float:
        return self._pa

    def _sample(self, plant, noise: dict) -> None:
        self._pa = _noisy(plant.baro_pa(), noise.get("baro_pa", 0.0))


class Mag:
    """Magnetometer: read() returns raw counts (MAG_LSB_PER_GAUSS), x forward, y left, z up."""

    def __init__(self) -> None:
        self._counts = (0, 0, 0)

    def read(self) -> tuple[int, int, int]:
        return self._counts

    def _sample(self, plant, noise: dict) -> None:
        fdm = plant.fdm
        phi, theta, psi = fdm["attitude/phi-rad"], fdm["attitude/theta-rad"], fdm["attitude/psi-rad"]
        north = (math.cos(theta) * math.cos(psi),
                 math.cos(phi) * math.sin(psi) - math.sin(phi) * math.sin(theta) * math.cos(psi),
                 -math.cos(phi) * math.sin(theta) * math.cos(psi) - math.sin(phi) * math.sin(psi))
        self._counts = tuple(_int16(_noisy(unit * EARTH_FIELD_GAUSS, noise.get("mag_gauss", 0.0)) * MAG_LSB_PER_GAUSS) for unit in north)


@dataclass(frozen=True)
class GpsFix:
    """One solution, in u-blox NAV-PVT units."""
    itow_ms: int  # time of the solution
    fix_type: int  # GPS_FIX_3D
    num_sv: int
    lat: int  # 1e-7 deg
    lon: int  # 1e-7 deg
    height_msl_mm: int
    vel_n_mm_s: int
    vel_e_mm_s: int
    vel_d_mm_s: int
    ground_speed_mm_s: int
    heading_motion: int  # 1e-5 deg


class Gps:
    """GPS receiver: read() returns the GpsFix that arrived since the last read, or None (a serial parser's view).
    Solutions arrive at the controller's gps_hz."""

    def __init__(self) -> None:
        self._new: GpsFix | None = None
        self.last: GpsFix | None = None  # the latest solution, read or not

    def read(self) -> GpsFix | None:
        fix, self._new = self._new, None
        return fix

    def _sample(self, plant, noise: dict, time_ms: int) -> None:
        gps = plant.gps()
        position_m, speed_m_s = noise.get("gps_m", 0.0), noise.get("gps_m_s", 0.0)
        north_m, east_m = _noisy(0.0, position_m), _noisy(0.0, position_m)
        lat_deg = gps["lat_e7"] / 1e7 + north_m / 111320.0
        lon_deg = gps["lon_e7"] / 1e7 + east_m / (111320.0 * math.cos(math.radians(lat_deg)))
        vel_n, vel_e, vel_d = (_noisy(cm_s / 100.0, speed_m_s) for cm_s in gps["vel_ned_cms"])
        self._new = self.last = GpsFix(
            itow_ms=time_ms, fix_type=GPS_FIX_3D, num_sv=GPS_SATELLITES, lat=round(lat_deg * 1e7), lon=round(lon_deg * 1e7),
            height_msl_mm=round(_noisy(gps["alt_cm"] / 100.0, position_m) * 1000.0), vel_n_mm_s=round(vel_n * 1000.0), vel_e_mm_s=round(vel_e * 1000.0),
            vel_d_mm_s=round(vel_d * 1000.0), ground_speed_mm_s=round(math.hypot(vel_n, vel_e) * 1000.0),
            heading_motion=round(math.degrees(math.atan2(vel_e, vel_n)) % 360.0 * 1e5),
        )


class Pitot:
    """Airspeed sensor: pressure() is the differential pressure in Pa (0.5 rho CAS^2)."""

    def __init__(self) -> None:
        self._pa = 0.0

    def pressure(self) -> float:
        return self._pa

    def _sample(self, plant, noise: dict) -> None:
        cas_m_s = plant.airspeed_cms() / 100.0
        self._pa = _noisy(0.5 * AIR_DENSITY_KG_M3 * cas_m_s * cas_m_s, noise.get("pitot_pa", 0.0))


class Rangefinder:
    """Downward rangefinder: distance_mm() along the body's -z, -1 out of range (RANGEFINDER_MAX_M, or tilted past 60 deg)."""

    def __init__(self) -> None:
        self._mm = -1

    def distance_mm(self) -> int:
        return self._mm

    def _sample(self, plant, noise: dict) -> None:
        fdm = plant.fdm
        tilt = math.cos(fdm["attitude/phi-rad"]) * math.cos(fdm["attitude/theta-rad"])
        if tilt < 0.5:
            self._mm = -1
            return
        distance_m = fdm["position/h-agl-ft"] * FT / tilt
        self._mm = -1 if distance_m > RANGEFINDER_MAX_M else max(0, round(_noisy(distance_m * 1000, noise.get("range_mm", 0.0))))


@dataclass(frozen=True)
class TruthState:
    """What the simulator knows the aircraft is doing: no sensor, no noise, nothing a real flight controller has. For
    scripting around a flight (stop conditions, events, scoring, logging), never for control."""
    lat_deg: float
    lon_deg: float
    north_m: float  # from home
    east_m: float
    agl_m: float  # above the ground
    roll_deg: float  # + = right wing down
    pitch_deg: float  # + = nose up
    yaw_deg: float  # heading, 0..360 from north
    vn_m_s: float  # velocity north, east, down
    ve_m_s: float
    vd_m_s: float
    airspeed_m_s: float  # calibrated
    aoa_deg: float
    sideslip_deg: float


class Hardware:
    """The same simulated sensors and PWM outputs for every flight controller."""

    gps_hz = 10.0
    sensor_noise: dict = {}

    def __init__(self, motor_count: int, servo_count: int) -> None:
        self.imu, self.baro, self.mag, self.gps = Imu(), Baro(), Mag(), Gps()
        self.pitot, self.rangefinder = Pitot(), Rangefinder()
        self.esc = [PWM_MIN_US] * motor_count
        self.servo = [PWM_MID_US] * servo_count
        self._time_us = 0
        self._next_gps_us = 0
        self._logged: dict[str, float] = {}
        self.truth: TruthState | None = None  # set before run(flight)

    def _sample(self, plant, time_us: int) -> None:
        """Fresh readings from the plant at time_us, and its truth (the runner's call, before run(flight) and every simulator tick)."""
        self._time_us = time_us
        state, fdm = plant.truth(), plant.fdm
        self.truth = TruthState(
            lat_deg=state["lat"], lon_deg=state["lon"], north_m=state["north_m"], east_m=state["east_m"], agl_m=state["agl_m"],
            roll_deg=state["roll"], pitch_deg=state["pitch"], yaw_deg=state["yaw"], vn_m_s=state["vn"], ve_m_s=state["ve"], vd_m_s=state["vd"],
            airspeed_m_s=state["ias_kts"] * KNOT_M_S, aoa_deg=fdm["aero/alpha-deg"], sideslip_deg=fdm["aero/beta-deg"],
        )
        noise = self.sensor_noise
        self.imu._sample(plant, noise, time_us)
        for part in (self.baro, self.mag, self.pitot, self.rangefinder):
            part._sample(plant, noise)
        if time_us >= self._next_gps_us:
            self._next_gps_us += round(1e6 / self.gps_hz)
            self.gps._sample(plant, noise, time_us // 1000)

    def _outputs_us(self) -> tuple[list[int], list[int]]:
        """esc[] and servo[] clipped to PWM_MIN_US .. PWM_MAX_US, as the plant takes them."""
        def clip(values):
            return [int(min(max(value, PWM_MIN_US), PWM_MAX_US)) for value in values]
        return clip(self.esc), clip(self.servo)

    def _sensor_columns(self) -> dict[str, float]:
        """What the script's hardware read this step, for the flight log."""
        ax, ay, az = self.imu.accel()
        gx, gy, gz = self.imu.gyro()
        mx, my, mz = self.mag.read()
        fix = self.gps.last
        columns = {"imu_ax": ax, "imu_ay": ay, "imu_az": az, "imu_gx": gx, "imu_gy": gy, "imu_gz": gz, "baro_pa": self.baro.pressure(),
                   "mag_x": mx, "mag_y": my, "mag_z": mz, "pitot_pa": self.pitot.pressure(), "range_mm": self.rangefinder.distance_mm()}
        if fix is not None:
            columns |= {"gps_lat": fix.lat, "gps_lon": fix.lon, "gps_height_mm": fix.height_msl_mm, "gps_speed_mm_s": fix.ground_speed_mm_s,
                        "gps_heading": fix.heading_motion}
        return columns
