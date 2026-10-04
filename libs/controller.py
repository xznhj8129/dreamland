"""
Usage, in aircraft/<project>/controllers/<name>.py:
from libs.controller import Controller

class Level(Controller):
    rate_hz = 200                         # loop() runs this often, in simulated time

    def setup(self):                      # once, before the first loop()
        self.start_pressure = self.baro.pressure()

    def loop(self):
        ax, ay, az = self.imu.accel()     # raw counts, like reading the chip over SPI
        self.servo[0] = 1500              # pulse widths in us, like Servo.writeMicroseconds
        self.esc[0] = 1600
        self.log("ax", ax)                # an extra flight-log column

Run: python sim_script.py aircraft/<project> <name> (see sim_script.py), or pick the script in the GUI's SITL view.

A flight controller script, talking to its hardware the way a microcontroller does: raw sensor readings in the units
the parts deliver, pulse widths out, time from micros()/millis(). The hardware is simulated from the JSBSim plant
(inavbench.vehicles.Plant), mounted level and facing forward, so its axes are the body's: x forward, y left, z up.
"""

import importlib.util
import math
import random
from dataclasses import dataclass

# The simulated parts (datasheet scales)
ACCEL_LSB_PER_G = 4096.0  # ICM-20602 class SPI IMU, accelerometer at +-8 g
GYRO_LSB_PER_DPS = 16.4  # its gyro at +-2000 deg/s
INT16_RANGE = (-32768, 32767)  # IMU registers saturate here
MAG_LSB_PER_GAUSS = 1090.0  # HMC5883L at its default gain
EARTH_FIELD_GAUSS = 0.5  # the simulated field: level, pointing north, no dip
PLANT_MAG_UNITS = 16000.0  # the size of inavbench Plant.mag's field vector
AIR_DENSITY_KG_M3 = 1.225  # pitot: differential pressure 0.5 rho CAS^2
RANGEFINDER_MAX_M = 10.0  # VL53L1X class, looking down the body's -z
PWM_MIN_US, PWM_MID_US, PWM_MAX_US = 1000, 1500, 2000
GPS_FIX_3D = 3  # u-blox NAV-PVT fixType
GPS_SATELLITES = 14


def _noisy(value: float, sigma: float) -> float:
    return value + random.gauss(0.0, sigma) if sigma else value


def _int16(value: float) -> int:
    return int(min(max(round(value), INT16_RANGE[0]), INT16_RANGE[1]))


class Imu:
    """SPI IMU: accel() and gyro() return raw int16 counts (ACCEL_LSB_PER_G, GYRO_LSB_PER_DPS). Level and at rest it
    reads +ACCEL_LSB_PER_G on z; gyro x + = rolling right, y + = nose down, z + = nose left."""

    def __init__(self) -> None:
        self._accel = (0, 0, 0)
        self._gyro = (0, 0, 0)

    def accel(self) -> tuple[int, int, int]:
        return self._accel

    def gyro(self) -> tuple[int, int, int]:
        return self._gyro

    def _sample(self, plant, noise: dict) -> None:
        self._accel = tuple(_int16(_noisy(mg / 1000.0, noise.get("accel_g", 0.0)) * ACCEL_LSB_PER_G) for mg in plant.acc_mg())
        self._gyro = tuple(_int16(_noisy(dps16 / 16.0, noise.get("gyro_dps", 0.0)) * GYRO_LSB_PER_DPS) for dps16 in plant.gyro_dps16())


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
        scale = EARTH_FIELD_GAUSS / PLANT_MAG_UNITS
        self._counts = tuple(_int16(_noisy(unit * scale, noise.get("mag_gauss", 0.0)) * MAG_LSB_PER_GAUSS) for unit in plant.mag())


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
        cm = plant.rangefinder_cm(RANGEFINDER_MAX_M)
        self._mm = -1 if cm < 0 else max(0, round(_noisy(cm * 10.0, noise.get("range_mm", 0.0))))


class Controller:
    """A flight controller script: write setup() and loop(). loop() runs rate_hz times per simulated second; before
    each call the sensors hold fresh readings, and after it esc[] and servo[] go to the aircraft.

    Hardware: imu, baro, mag, gps, pitot, rangefinder (see their classes for units).
    Outputs: esc[i] (motor i in the export's order) and servo[i] (S1 = servo[0]), pulse widths in us, PWM_MIN_US ..
    PWM_MAX_US; they keep their value between loops.
    Time: micros(), millis() (simulated). log(name, value) adds a flight-log column; stop() ends the run.
    sensor_noise: Gaussian standard deviations, in the physical units before the part's conversion, by name: accel_g,
    gyro_dps, baro_pa, mag_gauss, gps_m, gps_m_s, pitot_pa, range_mm."""

    rate_hz = 100.0
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
        self._stopped = False

    def setup(self) -> None:
        pass

    def loop(self) -> None:
        pass

    def micros(self) -> int:
        return self._time_us

    def millis(self) -> int:
        return self._time_us // 1000

    def log(self, name: str, value: float) -> None:
        self._logged[name] = value

    def stop(self) -> None:
        self._stopped = True

    def _sample(self, plant, time_us: int) -> None:
        """Fresh readings from the plant at time_us (the runner's call, before setup() and every loop())."""
        self._time_us = time_us
        noise = self.sensor_noise
        for part in (self.imu, self.baro, self.mag, self.pitot, self.rangefinder):
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


class ScriptError(Exception):
    pass


def controller_names(directory) -> list[str]:
    """The scripts in a project's controllers/ directory, by name."""
    return sorted(path.stem for path in directory.glob("*.py")) if directory.is_dir() else []


def load_controller(directory, name: str) -> type[Controller]:
    """The one Controller subclass defined in directory/<name>.py."""
    path = directory / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"controller_script_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    found = [item for item in vars(module).values()
             if isinstance(item, type) and issubclass(item, Controller) and item is not Controller and item.__module__ == module.__name__]
    if len(found) != 1:
        raise ScriptError(f"{path}: needs exactly one Controller subclass, found {[item.__name__ for item in found]}")
    return found[0]
