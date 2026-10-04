"""
Usage:
    from rcplanes_multirotor import DataRepo, MotorSim

    sim = MotorSim(
        cell="Molicel INR21700-P45B 7/10C",
        controller="APD 80F3x",
        motor_mfg="28",
        motor_type="X3115-900",
        propeller="APC E 10x5",
        wire_gauge=18,
        cells=6,
        parallel=1,
        altitude_m=0.0,
        airspeed=0.0,
        temp_c=20.0,
        wire_temp_c=20.0,
        temp_motor_c=20.0,
        wire_length_m=0.5,
    )
    print(dict(sim.static)["thrust_g"])
    print(dict(sim.from_thrust_g(1000))["rpm"])

cell, controller and wire_gauge may be None when the part is not known: its resistance is then left out (0), and a
missing cell needs cell_volt (nominal volts per cell) instead. external_resistance_ohm (default 0) is added in series
with them, for resistance worked out elsewhere (e.g. from wire routes and connectors).
"""
import csv
import math
from dataclasses import dataclass
from pathlib import Path


DATA_DIR = Path(__file__).parent / "data"
MANUFACTURERS = DATA_DIR / "manufacturers.csv"
G_ACCEL = 9.80665
COPPER_RHO = 1.71e-8
TEMP_COEFF = 0.00382
JS_MAGIC_FACTOR = (12.0 ** -5) * 1e-9


class MotorStalled(RuntimeError):
    """The motor cannot turn the prop: its resistance drop eats the whole supply (overloaded)."""


def _read_csv(path: Path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"missing header row in {path}")
        return list(reader)


@dataclass(frozen=True)
class Cell:
    name: str
    cap_mah: float
    res_ohm: float
    weight_g: float
    volt: float
    max_i: float


@dataclass(frozen=True)
class Controller:
    name: str
    res_ohm: float
    imax: float
    weight_g: float


@dataclass(frozen=True)
class Wire:
    gauge: int
    area_mm2: float
    area_m2: float


@dataclass(frozen=True)
class Prop:
    name: str
    tconst: float
    pconst: float
    diam_in: float
    pitch_in: float
    texp: float
    pexp: float
    blades: int


@dataclass(frozen=True)
class Motor:
    mfgcode: str
    name: str
    kv: float
    rm_ohm: float
    io: float
    imax: float
    weight_g: float
    pwr: float
    ki: float
    krpm: float
    rk: float
    vo: float
    stator_diameter_mm: float | None  # motordatav2.csv only
    stator_height_mm: float | None

    @property
    def display_name(self) -> str:
        return f"{self.name} (Kv: {self.kv})"


class DataRepo:
    def __init__(self, data_dir: Path = DATA_DIR):
        self.data_dir = data_dir
        self.manufacturer_by_name, self.manufacturer_by_idx = self._load_manufacturers()
        self.cells = self._load_cells()
        self.controllers = self._load_controllers()
        self.wires = self._load_wires()
        self.props = self._load_props()
        self.motors = self._load_motors()

        self.cell_by_name = {c.name: c for c in self.cells}
        self.controller_by_name = {c.name: c for c in self.controllers}
        self.wire_by_name = {f"AWG {w.gauge}": w for w in self.wires}
        self.prop_by_name = {p.name: p for p in self.props}
        self.motors_by_mfg = {}
        for m in self.motors:
            self.motors_by_mfg.setdefault(m.mfgcode, []).append(m)
        self.motor_by_key = {(m.mfgcode, m.name): m for m in self.motors}

    def _load_manufacturers(self):
        by_name = {}
        by_idx = {}
        if MANUFACTURERS.exists():
            rows = _read_csv(MANUFACTURERS)
            for row in rows:
                name = row["Name"].strip()
                idx = str(row["index"]).strip()
                by_name[name.lower()] = idx
                by_idx[idx] = name
        return by_name, by_idx

    def _load_cells(self):
        rows = _read_csv(self.data_dir / "cells.csv")
        cells = []
        for row in rows:
            cells.append(
                Cell(
                    name=row["Name"].strip(),
                    cap_mah=float(row["Cap"]),
                    res_ohm=float(row["Res"]) / 1000.0,
                    weight_g=float(row["Wgt"]) * 28.34952,
                    volt=float(row["Volt"]),
                    max_i=float(row["MaxI"]),
                )
            )
        return cells

    def _load_controllers(self):
        rows = _read_csv(self.data_dir / "controllers.csv")
        controllers = []
        for row in rows:
            controllers.append(
                Controller(
                    name=row["Name"].strip(),
                    res_ohm=float(row["Ri"]) / 1000.0,
                    imax=float(row["imax"]),
                    weight_g=float(row["wgt"]),
                )
            )
        return controllers

    def _load_wires(self):
        rows = _read_csv(self.data_dir / "wires.csv")
        wires = []
        for row in rows:
            area_mm2 = float(row["wire_area"])
            gauge = int(row["Name"].split()[-1]) if " " in row["Name"] else int(row["Name"])
            wires.append(
                Wire(
                    gauge=gauge,
                    area_mm2=area_mm2,
                    area_m2=area_mm2 / 1_000_000.0,
                )
            )
        return wires

    def _load_props(self):
        rows = _read_csv(self.data_dir / "propellers.csv")
        props = []
        for row in rows:
            props.append(
                Prop(
                    name=row["Name"].strip(),
                    tconst=float(row["Tconst"]),
                    pconst=float(row["Pconst"]),
                    diam_in=float(row["Diam"]),
                    pitch_in=float(row["Pitch"]),
                    texp=float(row["Texp"]),
                    pexp=float(row["Pexp"]),
                    blades=int(row["Blades"]),
                )
            )
        return props

    def _load_motors(self):
        motor_file_v2 = self.data_dir / "motordatav2.csv"
        motor_file = motor_file_v2 if motor_file_v2.exists() else (self.data_dir / "motors.csv")
        rows = _read_csv(motor_file)
        motors = []
        for row in rows:
            if "mfgcode" in row and row["mfgcode"]:
                mfgcode = str(row["mfgcode"]).strip()
            elif "manufacturer" in row and row["manufacturer"]:
                mfgcode = self.manufacturer_by_name.get(str(row["manufacturer"]).strip().lower(), "")
            else:
                mfgcode = ""
            rm_raw = row.get("Rm")
            if rm_raw is None or str(rm_raw).strip() == "":
                rm_ohm = 0.0
            else:
                rm_val = float(rm_raw)
                rm_ohm = rm_val / 1000.0 if rm_val < 1000.0 else rm_val / 1_000_000.0
            weight_g = None
            if "EstimatedWeight_g" in row and row["EstimatedWeight_g"]:
                weight_g = float(row["EstimatedWeight_g"])
            elif "Wgt" in row and row["Wgt"]:
                weight_g = float(row["Wgt"]) * 28.34952
            else:
                weight_g = 0.0
            motors.append(
                Motor(
                    mfgcode=mfgcode,
                    name=row["Name"].strip(),
                    kv=float(row["Kv"]),
                    rm_ohm=rm_ohm,
                    io=float(row["Io"]) if row.get("Io") else 0.0,
                    imax=float(row["Imax"]) if row.get("Imax") else 0.0,
                    weight_g=weight_g,
                    pwr=float(row["Pwr"]) if row.get("Pwr") else 0.0,
                    ki=float(row["Ki"]) if row.get("Ki") else 0.0,
                    krpm=float(row["Krpm"]) if row.get("Krpm") else 0.0,
                    rk=float(row["Rk"]) if row.get("Rk") else 0.0,
                    vo=float(row["Vo"]) if row.get("Vo") else 0.0,
                    stator_diameter_mm=float(row["StatorDiameter_mm"]) if row.get("StatorDiameter_mm") else None,
                    stator_height_mm=float(row["StatorHeight_mm"]) if row.get("StatorHeight_mm") else None,
                )
            )
        return motors

    def cell(self, name: str) -> Cell:
        return self.cell_by_name[name]

    def controller(self, name: str) -> Controller:
        return self.controller_by_name[name]

    def wire(self, name: str) -> Wire:
        if isinstance(name, int):
            gauge = name
        else:
            gauge = int(str(name).replace("AWG", "").strip())
        return next(w for w in self.wires if w.gauge == gauge)

    def prop(self, name: str) -> Prop:
        return self.prop_by_name[name]

    def motor(self, mfg: str, motor_name: str) -> Motor:
        return self.motor_by_key[(mfg, motor_name)]

    def motors_for(self, mfg: str):
        return tuple(self.motors_by_mfg.get(mfg, ()))


@dataclass(frozen=True)
class MotorInputs:
    cell: str | None
    controller: str | None
    motor_mfg: str
    motor_type: str
    propeller: str
    wire_gauge: int | None
    cells: int
    parallel: int
    motors: int
    altitude_m: float
    airspeed: float
    temp_c: float
    wire_temp_c: float
    temp_motor_c: float
    wire_length_m: float
    cell_volt: float | None
    external_resistance_ohm: float


class MotorSim:
    def __init__(self, repo: DataRepo | None = None, **kwargs):
        if repo is None:
            repo = DataRepo()
        kwargs.setdefault("motors", 1)
        kwargs.setdefault("airspeed", 0.0)
        kwargs.setdefault("cell_volt", None)
        kwargs.setdefault("external_resistance_ohm", 0.0)
        self.inputs = MotorInputs(**kwargs)
        if self.inputs.cell is None:
            if self.inputs.cell_volt is None:
                raise ValueError("cell_volt is required when cell is None")
            self.cell = Cell(name="not modelled", cap_mah=math.nan, res_ohm=0.0, weight_g=math.nan, volt=self.inputs.cell_volt, max_i=math.inf)
        else:
            self.cell = repo.cell(self.inputs.cell)
        self.controller = Controller(name="not modelled", res_ohm=0.0, imax=math.inf, weight_g=math.nan) if self.inputs.controller is None else repo.controller(self.inputs.controller)
        self.wire_main = None if self.inputs.wire_gauge is None else repo.wire(self.inputs.wire_gauge)
        self.prop = repo.prop(self.inputs.propeller)
        req_mfg = str(self.inputs.motor_mfg)
        if req_mfg.lower() in repo.manufacturer_by_name:
            req_mfg = repo.manufacturer_by_name[req_mfg.lower()]
        motors = repo.motors_for(req_mfg)
        chosen = None
        for m in motors:
            if m.name == self.inputs.motor_type:
                chosen = m
                break
        if chosen is None:
            matches = [m for m in repo.motors if m.name == self.inputs.motor_type]
            if len(matches) == 1:
                chosen = matches[0]
            elif not matches:
                raise ValueError(f"motor '{self.inputs.motor_type}' not found (code '{self.inputs.motor_mfg}')")
            else:
                raise ValueError(f"motor '{self.inputs.motor_type}' is ambiguous without correct mfg code")
        self.motor = chosen

        P0, T0, L, g, M, R = 101325.0, 288.15, 0.0065, 9.80665, 0.0289644, 8.31447
        temp_k = 273.15 + self.inputs.temp_c
        temp_at_alt = T0 - L * self.inputs.altitude_m
        pressure_pa = P0 * (temp_at_alt / T0) ** ((g * M) / (R * L))
        self.air_density = (pressure_pa) / (287.05 * temp_k)
        self.wire_res_main = 0.0 if self.wire_main is None else self._wire_resistance(self.wire_main, self.inputs.wire_length_m, self.inputs.wire_temp_c)
        self.wire_res_aux = self.wire_res_main

        self._static = self._solve_static()

    def _wire_resistance(self, wire: Wire, length_m: float, temp_c: float):
        res_20c = COPPER_RHO * (length_m * 2.0) / wire.area_m2
        return res_20c * (1.0 + TEMP_COEFF * (temp_c - 20.0))

    def _solve_static(self):
        alt_density = self.air_density
        num_cells = self.inputs.cells
        cells_parallel = self.inputs.parallel
        motors = self.inputs.motors

        pack_r = num_cells * self.cell.res_ohm / cells_parallel
        pack_cap = self.cell.cap_mah * cells_parallel
        pack_v = self.cell.volt * num_cells
        pack_wgt = self.cell.weight_g * num_cells * cells_parallel

        total_r = pack_r + self.controller.res_ohm + self.wire_res_main + self.wire_res_aux + self.inputs.external_resistance_ohm

        eff_diam_in = self.prop.diam_in * (self.prop.blades / 2.0) ** 0.2
        pitch_ratio = self.prop.pitch_in / self.prop.diam_in if self.prop.diam_in > 0 else 0.0
        eff_pitch = self.prop.pitch_in if pitch_ratio < 0.6 else self.prop.diam_in * 0.6

        motor_kv_timed = self.motor.kv
        eff_gear_ratio = 1.0
        motor_io_eff = self.motor.io + max(0.0, (pack_v - self.motor.vo)) / self.motor.rk

        tempg = (
            (alt_density / 1.225)
            * self.prop.pconst
            * self.motor.ki
            * (eff_diam_in ** 4)
            * self.prop.pitch_in
            * JS_MAGIC_FACTOR
            * ((motor_kv_timed / eff_gear_ratio) ** self.prop.pexp)
        )
        # (pack_v - I * total_r)^2 = (I - motor_io_eff) / tempg
        if total_r == 0.0:
            imotor = motor_io_eff + tempg * pack_v * pack_v
        else:
            disc_b = (-2 * total_r * pack_v - 1.0 / tempg) / (total_r * total_r)
            disc_c = (pack_v * pack_v + motor_io_eff / tempg) / (total_r * total_r)
            disc = (disc_b * disc_b / 4.0) - disc_c
            if disc < 0:
                imotor = motor_io_eff
            else:
                z = math.sqrt(disc)
                i1 = -disc_b / 2.0 + z
                i2 = -disc_b / 2.0 - z
                imotor = i2 if i2 > 1e-6 else i1
        motor_current = imotor

        temp_c_motor = self.inputs.temp_motor_c
        motor_rmt = self.motor.rm_ohm * (1.0 + TEMP_COEFF * (temp_c_motor - 20.0))

        volt_drop_ext = motor_current * total_r
        volts_to_motor = max(0.0, pack_v - volt_drop_ext)
        empirical_factor = (1.0 + 0.039 * (motor_current ** 2) / (self.motor.weight_g * 2.0)) * self.motor.krpm
        volt_drop_int = motor_current * motor_rmt * empirical_factor
        v_bemf = volts_to_motor - volt_drop_int
        motor_rpm = motor_kv_timed * v_bemf if motor_kv_timed > 0 and v_bemf > 0 else 0.0
        if motor_rpm <= 0.0:
            raise MotorStalled(f"motor overloaded, cannot turn the prop: {motor_current:.0f} A would drop the {pack_v:.1f} V supply to {v_bemf:.1f} V back-EMF (kv {motor_kv_timed:g})")

        fe_loss = volts_to_motor * motor_io_eff
        cu_loss_factor = empirical_factor / self.motor.krpm if self.motor.krpm else 1.0
        cu_loss = (motor_current ** 2) * motor_rmt * cu_loss_factor
        watts_in = volts_to_motor * motor_current
        watts_out = max(0.0, watts_in - (fe_loss + cu_loss))
        eff_pct = (watts_out / watts_in * 100.0) if watts_in > 1e-6 else 0.0

        prop_rpm = motor_rpm
        pitch_speed_mph = prop_rpm * self.prop.pitch_in * 0.00094697

        rpm_k = prop_rpm / 1000.0
        thrust_g_single = (
            (alt_density / 1.225)
            * self.prop.tconst
            * eff_pitch
            * (eff_diam_in ** 3)
            * (rpm_k ** self.prop.texp)
            * (28.34952 * 0.858 / 10000.0)
            if rpm_k > 0
            else 0.0
        )
        thrust_total_g = thrust_g_single * motors

        motor_wgt_total = self.motor.weight_g * motors
        esc_wgt_total = self.controller.weight_g * motors
        sys_wgt_g = (motor_wgt_total + esc_wgt_total + pack_wgt) * 1.1

        total_current = motor_current * motors
        duration_min = (pack_cap / 1000.0) / total_current * 60.0 * 0.9 if total_current > 1e-6 else 0.0

        warnings = []
        if motor_current > self.motor.imax:
            warnings.append("Motor Max Current exceeded!")
        if motor_current > self.controller.imax:
            warnings.append("ESC Max Current exceeded!")
        if total_current > self.cell.max_i * cells_parallel:
            warnings.append("Battery Max Current exceeded")

        return {
            "pack_r_ohm": pack_r,
            "pack_cap_mah": pack_cap,
            "pack_v": pack_v,
            "pack_wgt_g": pack_wgt,
            "controller_res_ohm": self.controller.res_ohm,
            "controller_imax": self.controller.imax,
            "controller_wgt_g": self.controller.weight_g,
            "motor_current_a": motor_current,
            "motor_rpm": motor_rpm,
            "prop_rpm": prop_rpm,
            "thrust_g": thrust_total_g,
            "thrust_n": thrust_total_g / 1000.0 * G_ACCEL,
            "pitch_speed_mph": pitch_speed_mph,
            "pitch_speed_kph": pitch_speed_mph * 1.609344,
            "efficiency_pct": eff_pct,
            "input_power_w": watts_in,
            "output_power_w": watts_out,
            "duration_min": duration_min,
            "power_system_weight_g": sys_wgt_g,
            "pack_voltage_v": pack_v,
            "wire_res_main_ohm": self.wire_res_main,
            "wire_res_aux_ohm": self.wire_res_aux,
            "motor_rmt": motor_rmt,
            "motor_io_eff": motor_io_eff,
            "warnings": warnings,
            "motor_limit_a": self.motor.imax,
            "battery_max_a": self.cell.max_i * cells_parallel,
            "alt_density": alt_density,
            "eff_diam_in": eff_diam_in,
            "eff_pitch_for_thrust": eff_pitch,
        }

    @property
    def static(self):
        return tuple(self._static.items())

    def from_thrust_g(self, thrust_g: float):
        static_thrust_g = float(self._static["thrust_g"])
        static_rpm = float(self._static["prop_rpm"])
        rpm = static_rpm * math.pow(thrust_g / static_thrust_g, 1.0 / self.prop.texp) if thrust_g > 0.0 else 0.0
        return self._target_payload(rpm, thrust_g)

    def from_rpm(self, rpm: float):
        static_rpm = float(self._static["prop_rpm"])
        static_thrust_g = float(self._static["thrust_g"])
        thrust_g = static_thrust_g * math.pow(rpm / static_rpm, self.prop.texp) if rpm > 0.0 else 0.0
        return self._target_payload(rpm, thrust_g)

    def from_throttle(self, throttle: float):
        target_rpm = throttle * float(self._static["prop_rpm"])
        return self.from_rpm(target_rpm)

    def _target_payload(self, rpm: float, thrust_g: float):
        static_rpm = float(self._static["prop_rpm"])
        rpm_ratio = rpm / static_rpm if static_rpm > 0.0 else 0.0
        shaft_w = float(self._static["output_power_w"]) * math.pow(rpm_ratio, self.prop.pexp) if rpm > 0.0 else 0.0
        elec_w = float(self._static["input_power_w"]) * math.pow(rpm_ratio, self.prop.pexp) if rpm > 0.0 else 0.0
        amps = float(self._static["motor_current_a"]) * math.pow(rpm_ratio, self.prop.pexp) if rpm > 0.0 else 0.0
        eff_pct = shaft_w / elec_w * 100.0 if elec_w > 0.0 else 0.0
        pack_v = float(self._static["pack_voltage_v"])
        payload = {
            "rpm": rpm,
            "thrust_g": thrust_g,
            "shaft_w": shaft_w,
            "elec_w": elec_w,
            "current_a": amps,
            "eff_pct": eff_pct,
            "pack_v": pack_v,
        }
        return tuple(payload.items())
