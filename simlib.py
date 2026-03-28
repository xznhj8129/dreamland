import pandas as pd
import math
import numpy as np
import itertools
from scipy.interpolate import interp1d
from pathlib import Path

INCH_TO_M = 0.0254
PITCH_SPEED_COEFF = 0.00042333333333333336  # m/s per (RPM * inch)
PROP_SPEED_OF_SOUND = 343.0
PROP_DEFAULT_SLIP = 0.20
PROP_DEFAULT_ETA_M = 0.80
PROP_DEFAULT_RHO = 1.225
PROP_DEFAULT_MACH_LIMIT = 0.9
PROP_DEFAULT_V_PER_CELL = 4.0
SCRIPT_DIR = Path(__file__).resolve().parent
MOTORDATA_PATH = SCRIPT_DIR / "motordata.csv"
BATTERYDATA_PATH = SCRIPT_DIR / "batterydata.csv"

def decay_factor(V, V_pitch):
    """
    Very-rough (how rough?) prop thrust derate versus flight speed.

    - Quadratic fall-off from 100 % at 0 m/s to 30 % at the pitch speed.
    - Linear fade from 30 % to 0 % between 1.0 × and 1.3 × pitch speed.
    - Zero (or treat as drag) beyond 1.3 × pitch speed.
    """
    if V <= V_pitch:                       # 0 - 1 × Vpitch
        slip = 1.0 - (V / V_pitch)
        return 0.30 + 0.70 * slip**2       # 1 → 0.30

    V_zero = 1.3 * V_pitch                # ~where thrust really hits zero
    if V < V_zero:                         # 1.0 - 1.3 × Vpitch
        return 0.30 * (1.0 - (V - V_pitch) / (V_zero - V_pitch))

    return 0.0                             # ≥ 1.3 × Vpitch → no thrust

def prop_thrust(RPM: float, diameter_in: float, pitch_in: float, v: float) -> float:
    """
    https://www.tytorobotics.com/blogs/articles/how-to-calculate-propeller-thrust
    """
    if pitch_in <= 0:
        raise ValueError("pitch_in must be > 0")

    thrust_n = 4.392e-8 * (
        RPM * (math.pow(diameter_in, 3.5)) / math.sqrt(pitch_in)
        * (4.233e-4 * RPM * pitch_in - v)
        ) 
    thrust_g = thrust_n / 9.81 * 1000
    
    thrust_g = max(thrust_g, 0.0)
    return thrust_g


def rpm_from_thrust(thrust_g: float, diameter_in: float, pitch_in: float, v: float) -> float:
    """
    Invert prop_thrust() to get RPM from desired thrust (in grams).

    Forward model (thrust in N):
        T = K * RPM * (C * RPM - v)
        where:
            K = 4.392e-8 * d^3.5 / sqrt(pitch)
            C = 4.233e-4 * pitch
            d = diameter_in (in)
            pitch = pitch_in (in)
            v = forward airspeed (m/s)

    After algebra:
        RPM = ( v + sqrt( v^2 + 4 * C * T / K ) ) / ( 2 * C )

    Parameters
    ----------
    thrust_g : float
        Desired thrust in grams ( ≥ 0 ).
    diameter_in : float
        Prop diameter in inches.
    pitch_in : float
        Prop pitch in inches (> 0).
    v : float
        Forward airspeed (m/s).

    Returns
    -------
    float
        Required RPM (rev/min).
    """
    if pitch_in <= 0:
        raise ValueError("pitch_in must be > 0")
    if diameter_in <= 0:
        raise ValueError("diameter_in must be > 0")
    if thrust_g < 0:
        raise ValueError("thrust_g must be >= 0")

    # Convert thrust grams -> Newtons
    thrust_n = thrust_g / 1000.0 * 9.81
    if thrust_n == 0:
        return 0.0

    K = 4.392e-8 * (diameter_in ** 3.5) / math.sqrt(pitch_in)
    C = 4.233e-4 * pitch_in

    # RPM = (v + sqrt(v^2 + 4*C*T/K)) / (2*C)
    inside = v * v + 4.0 * C * thrust_n / K
    RPM = (v + math.sqrt(inside)) / (2.0 * C)
    
    return max(0.0,RPM)

def prop_thrust_slip(RPM, diameter_in, pitch_in, v, slip=0.2):
    if pitch_in <= 0: raise ValueError("pitch_in > 0")
    K = 4.392e-8 * (diameter_in ** 3.5) / math.sqrt(pitch_in)
    Ceff = (1.0 - slip) * 4.233e-4
    thrust_n = K * RPM * (Ceff * RPM * pitch_in - v)
    thrust_g = max(thrust_n / 9.81 * 1000.0, 0.0)
    return thrust_g

def rpm_from_thrust_slip(thrust_g, diameter_in, pitch_in, v, slip=0.2):
    if pitch_in <= 0 or diameter_in <= 0 or thrust_g < 0:
        raise ValueError("bad inputs")
    T = thrust_g / 1000.0 * 9.81
    if T == 0: return 0.0
    K = 4.392e-8 * (diameter_in ** 3.5) / math.sqrt(pitch_in)
    Ceff = (1.0 - slip) * 4.233e-4
    inside = v*v + 4.0 * Ceff * T / K
    RPM = (v + math.sqrt(inside)) / (2.0 * Ceff * pitch_in)
    return max(0.0, RPM)

def _base_loaded_rpm_frac(d_in: float, p_over_d: float) -> float:
    if d_in < 5.5:   base = 0.85
    elif d_in < 6.5: base = 0.78
    elif d_in < 7.5: base = 0.68
    else:            base = 0.62
    penalty = 0.06 * max(0.0, p_over_d - 1.0)
    return max(0.55, base - penalty)

def _cp_from_p_over_d(p_over_d: float) -> float:
    pts = [
        (0.5, 0.080),
        (1.0, 0.100),
        (1.4, 0.130),
        (1.8, 0.160),
        (2.2, 0.180),
    ]
    if p_over_d <= pts[0][0]:
        return pts[0][1]
    if p_over_d >= pts[-1][0]:
        return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= p_over_d <= x1:
            t = (p_over_d - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return 0.11

def speed_from_rpm_pitch_mps(rpm: float, pitch_in: float, slip: float = 0.0) -> float:
    return (1.0 - slip) * PITCH_SPEED_COEFF * rpm * pitch_in

def rpm_from_speed_pitch_mps(speed_mps: float, pitch_in: float, slip: float = 0.0) -> float:
    denom = (1.0 - slip) * PITCH_SPEED_COEFF * pitch_in
    if denom <= 0:
        raise ValueError("pitch_in must be > 0")
    return speed_mps / denom

def prop_performance_summary(cfg: dict) -> dict:
    required = ["diameter_in", "pitch_in", "kv", "cells"]
    missing = [k for k in required if k not in cfg]
    if missing:
        raise ValueError(f"Missing required fields: {', '.join(missing)}")
    d_in = float(cfg["diameter_in"])
    p_in = float(cfg["pitch_in"])
    kv = float(cfg["kv"])
    cells = float(cfg["cells"])

    volts_per_cell = float(cfg.get("volts_per_cell", PROP_DEFAULT_V_PER_CELL))
    slip = float(cfg.get("slip", PROP_DEFAULT_SLIP))
    eta_m = float(cfg.get("eta_m", PROP_DEFAULT_ETA_M))
    rho = float(cfg.get("rho", PROP_DEFAULT_RHO))
    mach_limit = float(cfg.get("mach_limit", PROP_DEFAULT_MACH_LIMIT))
    speed_of_sound = float(cfg.get("speed_of_sound", PROP_SPEED_OF_SOUND))
    calib = cfg.get("calibration", None)
    measured_rpm = cfg.get("measured_rpm", None)
    measured_input_w = cfg.get("measured_input_w", None)
    measured_thrust_n = cfg.get("measured_thrust_n", None)
    measured_v_bus = cfg.get("measured_v_bus", None)
    measured_eff_pct = cfg.get("measured_efficiency_pct", None)

    if d_in <= 0 or p_in <= 0 or kv <= 0 or cells <= 0:
        raise ValueError("diameter_in, pitch_in, kv, and cells must be > 0")

    v_bus = measured_v_bus if measured_v_bus is not None else cells * volts_per_cell
    p_over_d = p_in / d_in
    n0_rpm = kv * v_bus

    d_m = d_in * INCH_TO_M
    n0_rps = n0_rpm / 60.0
    tip_mach_n0 = (math.pi * d_m * n0_rps) / speed_of_sound
    rpm_tip_limit = (mach_limit * speed_of_sound * 60.0) / (math.pi * d_m)

    cp_scale = 1.0
    frac_scale = 1.0
    if calib:
        d_ref, p_ref, v_meas, i_meas, rpm_meas = calib
        pd_ref = p_ref / d_ref
        cp_ref_base = _cp_from_p_over_d(pd_ref)
        frac_ref_base = _base_loaded_rpm_frac(d_ref, pd_ref)
        n0_ref = kv * v_meas
        if n0_ref > 0:
            frac_scale = max(0.7, min(1.1, rpm_meas / (n0_ref * frac_ref_base)))
        pin_req = v_meas * i_meas
        d_m_ref = d_ref * INCH_TO_M
        n_ref = rpm_meas / 60.0
        cp_needed = (pin_req * eta_m) / (rho * (n_ref**3) * (d_m_ref**5)) if rho > 0 else 0.0
        cp_scale = max(0.5, min(2.0, cp_needed / cp_ref_base))

    base_frac = _base_loaded_rpm_frac(d_in, p_over_d)
    frac_loaded = base_frac * frac_scale
    n_loaded = n0_rpm * frac_loaded
    n_loaded_capped = min(n_loaded, rpm_tip_limit)

    # If measured data is provided, trust it to anchor the estimates.
    if measured_rpm is not None:
        n_loaded_capped = float(measured_rpm)
        n_loaded = n_loaded_capped
        frac_loaded = n_loaded_capped / n0_rpm if n0_rpm else 0.0

    pitch_speed_ideal = speed_from_rpm_pitch_mps(n_loaded_capped, p_in, slip=0.0)
    pitch_speed_slip = speed_from_rpm_pitch_mps(n_loaded_capped, p_in, slip=slip)
    tip_mach_loaded = (math.pi * d_m * (n_loaded_capped / 60.0)) / speed_of_sound

    cp = _cp_from_p_over_d(p_over_d) * cp_scale
    n_loaded_hz = n_loaded_capped / 60.0
    shaft_w = rho * (n_loaded_hz ** 3) * (d_m ** 5) * cp
    elec_w = shaft_w / eta_m if eta_m > 0 else float("inf")
    amps = elec_w / v_bus if v_bus > 0 else float("inf")
    static_thrust_g = prop_thrust(n_loaded_capped, d_in, p_in, v=0.0)
    static_thrust_n = static_thrust_g / 1000.0 * 9.81

    if measured_input_w is not None:
        elec_w = float(measured_input_w)
        amps = elec_w / v_bus if v_bus > 0 else float("inf")
        eff_frac = (measured_eff_pct / 100.0) if measured_eff_pct is not None else eta_m
        shaft_w = elec_w * eff_frac if math.isfinite(elec_w) else elec_w
        if shaft_w == 0 and elec_w > 0 and eff_frac == 0:
            shaft_w = elec_w
    if measured_thrust_n is not None:
        static_thrust_n = float(measured_thrust_n)
        static_thrust_g = static_thrust_n / 9.81 * 1000.0

    return {
        "diameter_in": d_in,
        "pitch_in": p_in,
        "kv": kv,
        "cells": cells,
        "volts_per_cell": volts_per_cell,
        "v_bus": v_bus,
        "p_over_d": p_over_d,
        "cp": cp,
        "cp_scale": cp_scale,
        "frac_loaded": frac_loaded,
        "n0_rpm": n0_rpm,
        "n_loaded_rpm": n_loaded_capped,
        "n_loaded_uncapped_rpm": n_loaded,
        "rpm_tip_limit": rpm_tip_limit,
        "tip_mach_no_load": tip_mach_n0,
        "tip_mach_loaded": tip_mach_loaded,
        "pitch_speed_mps": pitch_speed_ideal,
        "pitch_speed_slip_mps": pitch_speed_slip,
        "pitch_speed_kmh": pitch_speed_ideal * 3.6,
        "pitch_speed_slip_kmh": pitch_speed_slip * 3.6,
        "shaft_power_w": shaft_w,
        "input_power_w": elec_w,
        "current_a": amps,
        "static_thrust_g": static_thrust_g,
        "static_thrust_n": static_thrust_n,
        "mach_limit": mach_limit,
        "speed_of_sound": speed_of_sound,
        "slip": slip,
        "eta_m": eta_m,
        "rho": rho,
    }
    
class MotorData:
    def __init__(self, brand, motor, prop, kv, serie):
        self.brand = brand
        self.motor = motor
        self.prop = prop
        self.kv = int(kv)
        self.serie = int(serie)
        self.voltage = 3.7 * self.serie

        motor_data = pd.read_csv(MOTORDATA_PATH)
        group = motor_data[(motor_data['Brand'] == self.brand) & 
                           (motor_data['Motor'] == self.motor) & 
                           (motor_data['Prop'] == self.prop) & 
                           (motor_data['KV'] == self.kv) & 
                           (motor_data['Data Batt S'] == self.serie)]

        if group.empty:
            raise RuntimeError(f"No data found for {brand} {motor} {prop} {kv} {serie}S")
            
        self.throttle = group['T'].str.rstrip('%').astype(float).values
        self.thrust = group['Thrust g'].values
        self.max_thrust = float(self.thrust.max())
        self.current = group['A'].values
        self.max_draw = float(self.current.max())
        self.rpm = group['RPM'].values
        self.max_rpm = float(self.rpm.max())
        self.watts = group['W'].values
        self.max_watts = float(self.watts.max())
        self.motor_batt_s = group['Data Batt S'].iloc[0]
        self.efficiency = group['Efficiency'].values
        self.motor_weight = group['Weight g'].iloc[0]
        self.stator_diameter = group['Stator Diameter'].iloc[0]
        self.stator_height = group['Stator height'].iloc[0]
        self.kv = group['KV'].iloc[0]
        self.prop_diameter = group['Data P size'].iloc[0]
        self.prop_pitch = group['Data P pitch'].iloc[0]

    def thrust_to_throttle(self, thrust_g):
        f = interp1d(self.thrust, self.throttle, fill_value='extrapolate')
        return f(thrust_g)

    def thrust_to_current(self, thrust_g):
        f = interp1d(self.thrust, self.current, fill_value='extrapolate')
        return f(thrust_g)

    def thrust_to_rpm(self, thrust_g):
        f = interp1d(self.thrust, self.rpm, fill_value='extrapolate')
        return f(thrust_g)

    def thrust_to_watts(self, thrust_g):
        f = interp1d(self.thrust, self.watts, fill_value='extrapolate')
        return f(thrust_g)

    def rpm_to_efficiency(self, rpm):
        f = interp1d(self.rpm, self.efficiency, fill_value='extrapolate')
        return f(rpm)

    def thrust_to_efficiency(self, thrust_g):
        f = interp1d(self.thrust, self.efficiency, fill_value='extrapolate')
        return f(thrust_g)

    # Example plot function
    def plot_thrust_to(self, target_variable, thrust_values):
        true_values = interp1d(thrust_values, self.thrust, getattr(self, target_variable))
        predicted_values = getattr(self, f'thrust_to_{target_variable}')(thrust_values)

        # Plot true vs predicted values
        #plt.figure(figsize=(8, 6))
        #plt.plot(thrust_values, true_values, 'o-', label='True Values', color='blue')
        #plt.plot(thrust_values, predicted_values, 'x-', label='Interpolated Values', color='red')
        #plt.title(f'Interpolated vs True Values for {target_variable} ({self.brand} {self.motor} {self.prop})')
        #plt.xlabel('Thrust (g)')
        #plt.ylabel(target_variable.capitalize())
        #plt.legend()
        #plt.grid(True)
        #plt.savefig(f'{brand} {motor} {prop} {target_variable}.png')
        #plt.show()


    def motor_perf_thrust(self, thr_g): # interpolation for thrust g
        if thr_g > self.max_thrust:
            #print('Error: over maximum thrust')
            return math.inf, math.inf, math.inf, math.inf, math.inf
        rpm = max(0.0,self.thrust_to_rpm(thr_g)) #rpm
        elec_eff = max(0.0,self.rpm_to_efficiency(rpm)) #g/w
        power_per_motor = max(0.0,thr_g / elec_eff) #max(0.0,self.thrust_to_watts(thr_g)) #w
        #current_per_motor = max(0.1,self.thrust_to_current(thr_g)) #a
        current_per_motor = power_per_motor / self.voltage
        throttle = max(0.0,self.thrust_to_throttle(thr_g)) # %

        if power_per_motor > self.max_watts:
            print('Error: over maximum power')
            return math.inf, math.inf, math.inf, math.inf, math.inf

        if current_per_motor > self.max_draw:
            print('Error: over maximum draw')
            return math.inf, math.inf, math.inf, math.inf, math.inf
        
        if rpm > self.max_rpm:
            print('Error: over maximum rpm')
            return math.inf, math.inf, math.inf, math.inf, math.inf

        return power_per_motor, current_per_motor, elec_eff, throttle, rpm

    def motor_perf_thrust2(self, thr_g, airspd): # interpolation for thrust g with airspeed decay
        # Track caps for the caller
        self.last_caps = {
            "req_thr_g": thr_g,
            "used_thr_g": 0.0,
            "thr_capped": False,
            "power_capped": False,
            "current_capped": False,
            "rpm_capped": False,
        }
        if not math.isfinite(thr_g) or thr_g <= 0:
            self.last_capped = True
            return 0.0, 0.0, 0.0, 0.0, 0.0

        req_thr = thr_g
        used_thr = min(req_thr, self.max_thrust)
        if used_thr < req_thr:
            self.last_caps["thr_capped"] = True

        rpm = max(0.0, self.thrust_to_rpm(used_thr))
        if rpm > self.max_rpm:
            rpm = self.max_rpm
            self.last_caps["rpm_capped"] = True

        elec_eff = max(1e-9, self.thrust_to_efficiency(used_thr)) # g/W, clamp to avoid div/0
        power_per_motor = max(0.0, used_thr / elec_eff)
        if power_per_motor > self.max_watts:
            power_per_motor = self.max_watts
            self.last_caps["power_capped"] = True

        current_per_motor = power_per_motor / self.voltage
        if current_per_motor > self.max_draw:
            current_per_motor = self.max_draw
            power_per_motor = current_per_motor * self.voltage
            self.last_caps["current_capped"] = True

        throttle = max(0.0, min(100.0, self.thrust_to_throttle(used_thr))) # %

        self.last_caps["req_thr_g"] = req_thr
        self.last_caps["used_thr_g"] = used_thr
        self.last_caps["used_watts"] = power_per_motor
        self.last_caps["used_current"] = current_per_motor
        self.last_caps["used_rpm"] = rpm
        self.last_capped = any([
            self.last_caps["thr_capped"],
            self.last_caps["power_capped"],
            self.last_caps["current_capped"],
            self.last_caps["rpm_capped"],
        ])

        return power_per_motor, current_per_motor, elec_eff, throttle, rpm

class LiIonBatteryData:    
    def __init__(self, brand, model, cellformat, serie, parallel):
        self.brand = brand
        self.model = model
        self.format = int(cellformat)
        self.parallel = int(parallel)
        self.serie = int(serie)
        self.max_v = 4.2
        self.nominal_v = 3.6
        self.min_v = 2.5

        batt_data = pd.read_csv(BATTERYDATA_PATH)
        group = batt_data[(batt_data['Brand'] == self.brand) & 
                           (batt_data['Name'] == self.model) & 
                           (batt_data['Format'] == self.format)]

        if group.empty:
            raise RuntimeError(f"No data found for {brand} {model} {cellformat}")
        self.cell_weight = group['Weight'].iloc[0] / 1000.0
        self.cell_c = group['C-rating'].iloc[0]
        self.cell_cap_a = group['CapacityA'].iloc[0]
        self.cell_cap_wh = group['CapacityWh'].iloc[0]
        self.pack_weight = self.cell_weight * serie * parallel
        self.pack_amps = self.cell_cap_a * parallel
        self.pack_wh = self.cell_cap_wh * parallel
        self.pack_current = self.cell_c * self.pack_amps
        self.cell_len = 0.070 if self.format=="21700" else 0.065
        self.cell_width = 0.021 if self.format=="21700" else 0.018 
        self.pack_len = self.cell_len * self.parallel
        self.pack_width = -1
