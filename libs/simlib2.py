"""
simlib2.py  -- Tested motor/prop performance library

Grounded entirely in measured test-stand data.  Will not guess about
untested propellers -- if you don't have bench data, you don't have data.

Forward-flight thrust uses the corrected Tyto formula (k * Tyto with
airspeed v-term).  Electrical data (current, power) is always from
measured interpolation, never synthesised.

Query from any of:
    from_throttle(prop, %)  |  from_thrust(prop, grams)  |  from_rpm(prop, RPM)

Usage:
    motor = TestedMotorData("Uangel", "A2807", kv=1300, batt_s=6)
    motor.available_props()        # -> ['GF7042X2']

    op = motor.from_throttle("GF7042X2", 75)
    op = motor.from_thrust("GF7042X2", 500, airspeed_mps=15)
    op = motor.from_rpm("GF7042X2", 18000)
    motor.from_throttle("UNTESTED", 75)   # -> ValueError

Run directly for test:  python simlib2.py
"""

import math
import csv
import numpy as np
from dataclasses import dataclass
from scipy.interpolate import interp1d
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
INCH_TO_M = 0.0254
RHO_SL    = 1.225       # kg/m^3  sea-level ISA
SOUND_SL  = 343.0       # m/s     sea-level 20 C

# Tyto Robotics thrust formula coefficients
# Source: https://www.tytorobotics.com/blogs/articles/how-to-calculate-propeller-thrust
# T(N) = TYTO_A * RPM * d^3.5 / sqrt(p) * (TYTO_B * RPM * p - v)
TYTO_A = 4.392e-8
TYTO_B = 4.233e-4       # also: pitch_speed_mps = TYTO_B * RPM * pitch_in


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class OpPoint:
    """Motor/prop operating state at one condition."""
    throttle_pct: float
    rpm: float
    thrust_g: float
    current_a: float
    power_w: float           # electrical (V * I)
    efficiency_gw: float     # thrust grams per electrical watt
    voltage: float
    airspeed_mps: float
    pitch_speed_mps: float   # theoretical zero-thrust speed at this RPM
    tip_mach: float
    prop_name: str
    is_extrapolated: bool    # outside measured range or airspeed > 0

    @property
    def thrust_n(self) -> float:
        return self.thrust_g / 1000.0 * 9.81

    def summary(self) -> str:
        tag = " *" if self.is_extrapolated else ""
        return (
            f"{self.throttle_pct:5.1f}%  {self.rpm:7.0f} RPM  "
            f"{self.thrust_g:7.0f}g  {self.current_a:5.1f}A  "
            f"{self.power_w:6.0f}W  {self.efficiency_gw:.2f} g/W  "
            f"M{self.tip_mach:.2f}{tag}"
        )


# ---------------------------------------------------------------------------
# Tyto formula helpers
# ---------------------------------------------------------------------------

def _tyto_thrust_g(rpm, d_in, p_in, v=0.0):
    """Raw (uncorrected) Tyto thrust in grams."""
    K = TYTO_A * (d_in ** 3.5) / math.sqrt(p_in)
    C = TYTO_B * p_in
    t_n = K * rpm * (C * rpm - v)
    return max(0.0, t_n / 9.81 * 1000.0)


def _solve_rpm(thrust_g, d_in, p_in, v, k):
    """Solve  k * Tyto(RPM, d, p, v) = thrust_g  for RPM (quadratic)."""
    if thrust_g <= 0:
        return 0.0
    t_n = thrust_g / 1000.0 * 9.81
    K = k * TYTO_A * (d_in ** 3.5) / math.sqrt(p_in)
    C = TYTO_B * p_in
    if K <= 0 or C <= 0:
        return 0.0
    disc = v * v + 4.0 * C * t_n / K
    if disc < 0:
        return 0.0
    return max(0.0, (v + math.sqrt(disc)) / (2.0 * C))


def pitch_speed_mps(rpm, pitch_in):
    """Pitch speed in m/s (theoretical zero-thrust speed at this RPM)."""
    return TYTO_B * rpm * pitch_in


def tip_mach_at(rpm, diameter_in, sos=SOUND_SL):
    """Prop tip Mach number."""
    return math.pi * (diameter_in * INCH_TO_M) * rpm / (60.0 * sos)


def rho_at_altitude(alt_m, temp_c=15.0):
    """ISA air density at altitude."""
    T0, L, g, M, R = 288.15, 0.0065, 9.80665, 0.0289644, 8.31447
    T_k = 273.15 + temp_c
    T_alt = T0 - L * alt_m
    P = 101325.0 * (T_alt / T0) ** ((g * M) / (R * L))
    return P / (287.05 * T_k)


# ---------------------------------------------------------------------------
# Internal per-prop data
# ---------------------------------------------------------------------------

class _PropData:
    """Measured data + interpolation for one motor+prop combination."""

    def __init__(self, name, rows):
        self.name = name
        self.d_in = float(rows[0]["Data P size"])
        self.p_in = float(rows[0]["Data P pitch"])

        data = sorted(rows, key=lambda r: float(r["T"].rstrip("%")))
        throttle = [float(r["T"].rstrip("%")) for r in data]
        rpm      = [float(r["RPM"]) for r in data]
        thrust_g = [float(r["Thrust g"]) for r in data]
        current  = [float(r["A"]) for r in data]
        voltage  = [float(r["V"]) for r in data]
        watts    = [float(r["W"]) for r in data]
        eff_gw   = [float(r["Efficiency"]) for r in data]

        # Anchor at 0%: motor off = everything zero (voltage = no-load battery)
        if throttle[0] > 0:
            throttle.insert(0, 0.0)
            rpm.insert(0, 0.0)
            thrust_g.insert(0, 0.0)
            current.insert(0, 0.0)
            voltage.insert(0, voltage[0])  # battery voltage at no load
            watts.insert(0, 0.0)
            eff_gw.insert(0, 0.0)

        self.throttle = np.array(throttle)
        self.rpm      = np.array(rpm)
        self.thrust_g = np.array(thrust_g)
        self.current  = np.array(current)
        self.voltage  = np.array(voltage)
        self.watts    = np.array(watts)
        self.eff_gw   = np.array(eff_gw)

        kw = dict(fill_value="extrapolate", kind="linear")
        self.t2rpm   = interp1d(self.throttle, self.rpm, **kw)
        self.rpm2t   = interp1d(self.rpm, self.throttle, **kw)
        self.rpm2i   = interp1d(self.rpm, self.current, **kw)
        self.rpm2v   = interp1d(self.rpm, self.voltage, **kw)
        self.rpm2w   = interp1d(self.rpm, self.watts, **kw)
        self.rpm2thr = interp1d(self.rpm, self.thrust_g, **kw)
        self.thr2rpm = interp1d(self.thrust_g, self.rpm, **kw)

        self.max_throttle = float(self.throttle.max())
        self.max_rpm      = float(self.rpm.max())
        self.max_thrust_g = float(self.thrust_g.max())
        self.max_current  = float(self.current.max())
        self.max_watts    = float(self.watts.max())
        # Measured bounds (excluding synthetic zero point) for extrapolation flag
        meas = self.throttle > 0
        self._meas_min_rpm = float(self.rpm[meas].min()) if meas.any() else 0.0

        self._fit_k()

    def _fit_k(self):
        """Fit k so that  k * Tyto(RPM, d, p, 0) ~ measured thrust."""
        mask = self.throttle >= 30.0
        if mask.sum() < 2:
            mask = np.ones(len(self.throttle), dtype=bool)
        tyto = np.array([
            _tyto_thrust_g(r, self.d_in, self.p_in)
            for r in self.rpm[mask]
        ])
        meas = self.thrust_g[mask]
        ok = tyto > 1.0
        if ok.sum() == 0:
            self.k_corr = 1.0
            return
        self.k_corr = float(np.median(meas[ok] / tyto[ok]))

    def elec_at_rpm(self, rpm):
        """Interpolate throttle, current, voltage from measured data."""
        thr = float(np.clip(self.rpm2t(rpm), 0, 100))
        cur = float(max(0.0, self.rpm2i(rpm)))
        vol = float(max(0.01, self.rpm2v(rpm)))
        extrap = rpm > self.max_rpm * 1.05
        return thr, cur, vol, extrap


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class TestedMotorData:
    """
    Motor/prop performance from measured test-stand data only.

    Loads ALL tested prop combinations for one motor from the CSV.
    Queries require a prop name -- if you don't have data for it, you
    get a ValueError, not a guess.

    Forward-flight thrust extrapolation uses the Tyto formula with a
    per-prop correction factor k fitted to the static measured data.
    Current at airspeed is scaled by thrust ratio (prop unloads in
    forward flight -> less torque -> less current).
    """

    def __init__(
        self,
        brand: str,
        motor: str,
        kv: int,
        batt_s: int,
        csv_path: str = "motordata.csv",
        speed_of_sound: float = SOUND_SL,
        verbose: bool = False,
    ):
        self.brand = brand
        self.motor_name = motor
        self.kv = int(kv)
        self.batt_s = int(batt_s)
        self.speed_of_sound = speed_of_sound
        self.verbose = verbose

        csv_p = Path(csv_path)
        if not csv_p.is_absolute():
            csv_p = Path(__file__).parent / csv_p

        self._props: dict[str, _PropData] = {}
        self._load_all(csv_p)

        if not self._props:
            raise ValueError(
                f"No data for {brand} {motor} {kv}KV {batt_s}S in {csv_p.name}"
            )

    # -- CSV loading -------------------------------------------------------

    def _load_all(self, path):
        by_prop: dict[str, list] = {}
        with path.open() as f:
            for r in csv.DictReader(f):
                if (r["Brand"] == self.brand
                        and r["Motor"] == self.motor_name
                        and int(r["KV"]) == self.kv
                        and int(r["Data Batt S"]) == self.batt_s):
                    by_prop.setdefault(r["Prop"], []).append(r)

        for name, rows in by_prop.items():
            p = _PropData(name, rows)
            self._props[name] = p
            resid = ""
            tyto_check = np.array([
                _tyto_thrust_g(r, p.d_in, p.p_in) for r in p.rpm[p.throttle >= 30]
            ])
            meas_check = p.thrust_g[p.throttle >= 30]
            ok = tyto_check > 1.0
            if ok.sum() > 0:
                r = (tyto_check[ok] * p.k_corr - meas_check[ok]) / meas_check[ok] * 100
                resid = f"  residual: mean {r.mean():+.1f}%, worst {r[np.argmax(np.abs(r))]:+.1f}%"
            if self.verbose:
                print(
                    f"  {len(rows)} pts: {self.brand} {self.motor_name} + {name} "
                    f"{self.kv}KV {self.batt_s}S  "
                    f"[{p.thrust_g[p.thrust_g > 0].min():.0f}-{p.max_thrust_g:.0f}g, "
                    f"{p._meas_min_rpm:.0f}-{p.max_rpm:.0f} RPM]  "
                    f"k={p.k_corr:.3f}{resid}"
                )

    # -- Public: discovery -------------------------------------------------

    @staticmethod
    def available_motors(csv_path: str = "motordata.csv") -> list[dict]:
        """List all motor+prop combos in the CSV.

        Returns list of dicts with keys: brand, motor, kv, batt_s, prop, points.
        """
        csv_p = Path(csv_path)
        if not csv_p.is_absolute():
            csv_p = Path(__file__).parent / csv_p
        combos: dict[tuple, int] = {}
        with csv_p.open() as f:
            for r in csv.DictReader(f):
                key = (r["Brand"], r["Motor"], int(r["KV"]),
                       int(r["Data Batt S"]), r["Prop"])
                combos[key] = combos.get(key, 0) + 1
        return [
            {"brand": k[0], "motor": k[1], "kv": k[2],
             "batt_s": k[3], "prop": k[4], "points": v}
            for k, v in sorted(combos.items())
        ]

    def available_props(self) -> list[str]:
        """List of prop names with test data for this motor."""
        return list(self._props.keys())

    def prop_info(self, prop_name: str) -> dict:
        """Summary dict for a tested prop."""
        p = self._get_prop(prop_name)
        return {
            "name": p.name,
            "diameter_in": p.d_in,
            "pitch_in": p.p_in,
            "k_corr": p.k_corr,
            "points": len(p.throttle),
            "thrust_range_g": (float(p.thrust_g.min()), p.max_thrust_g),
            "rpm_range": (p._meas_min_rpm, p.max_rpm),
            "max_current_a": p.max_current,
            "max_watts": p.max_watts,
        }

    def measured_points(self, prop_name: str) -> dict[str, np.ndarray]:
        """The measured rows for a prop (no synthetic 0 % anchor): throttle %, rpm, thrust g, current A, voltage V, watts W."""
        p = self._get_prop(prop_name)
        measured = p.throttle > 0
        return {
            "throttle_pct": p.throttle[measured], "rpm": p.rpm[measured], "thrust_g": p.thrust_g[measured],
            "current_a": p.current[measured], "voltage_v": p.voltage[measured], "watts": p.watts[measured],
        }

    def _get_prop(self, prop_name: str) -> _PropData:
        if prop_name not in self._props:
            raise ValueError(
                f"No test data for '{prop_name}' on {self.brand} "
                f"{self.motor_name} {self.kv}KV {self.batt_s}S.  "
                f"Tested props: {self.available_props()}"
            )
        return self._props[prop_name]

    # -- Corrected thrust (Tyto + k) ---------------------------------------

    def _corrected_thrust(self, rpm, p, v=0.0):
        raw = _tyto_thrust_g(rpm, p.d_in, p.p_in, v)
        return raw * p.k_corr

    def _corrected_rpm_for_thrust(self, thrust_g, p, v=0.0):
        return _solve_rpm(thrust_g, p.d_in, p.p_in, v, p.k_corr)

    # -- Current adjustment for airspeed -----------------------------------

    def _airspeed_current(self, cur_static, thrust, rpm, p, v):
        """
        Scale current by thrust ratio in forward flight.
        Prop unloads -> less torque -> less current.
        At v=0 returns cur_static unchanged.
        """
        if v <= 0:
            return cur_static
        thr_static = float(max(1.0, p.rpm2thr(rpm)))
        ratio = max(0.05, thrust / max(1.0, thr_static))
        return cur_static * ratio

    # -- Build OpPoint -----------------------------------------------------

    def _build_op(self, throttle, rpm, thrust_g, current, voltage,
                  airspeed, prop_name, extrapolated):
        p = self._props[prop_name]
        pwr = voltage * current
        eff = thrust_g / pwr if pwr > 0.1 else 0.0
        ps = pitch_speed_mps(rpm, p.p_in)
        tm = tip_mach_at(rpm, p.d_in, self.speed_of_sound)
        return OpPoint(
            throttle_pct=throttle, rpm=rpm, thrust_g=thrust_g,
            current_a=current, power_w=pwr, efficiency_gw=eff,
            voltage=voltage, airspeed_mps=airspeed,
            pitch_speed_mps=ps, tip_mach=tm,
            prop_name=prop_name, is_extrapolated=extrapolated,
        )

    # ======================================================================
    # Public API
    # ======================================================================

    def from_throttle(self, prop_name, throttle_pct, airspeed_mps=0.0):
        """Operating point at a given throttle percentage."""
        p = self._get_prop(prop_name)
        rpm = max(0.0, float(p.t2rpm(throttle_pct)))

        if airspeed_mps == 0.0:
            thrust = float(max(0.0, p.rpm2thr(rpm)))
        else:
            thrust = self._corrected_thrust(rpm, p, airspeed_mps)

        thr, cur, vol, extrap = p.elec_at_rpm(rpm)
        cur = self._airspeed_current(cur, thrust, rpm, p, airspeed_mps)

        return self._build_op(
            throttle_pct, rpm, thrust, cur, vol,
            airspeed_mps, prop_name, extrap or airspeed_mps > 0,
        )

    def from_thrust(self, prop_name, thrust_g, airspeed_mps=0.0):
        """Operating point for a desired thrust (grams)."""
        p = self._get_prop(prop_name)

        if airspeed_mps == 0.0:
            rpm = float(max(0.0, p.thr2rpm(thrust_g)))
        else:
            rpm = self._corrected_rpm_for_thrust(thrust_g, p, airspeed_mps)

        thr, cur, vol, extrap = p.elec_at_rpm(rpm)
        cur = self._airspeed_current(cur, thrust_g, rpm, p, airspeed_mps)

        return self._build_op(
            thr, rpm, thrust_g, cur, vol,
            airspeed_mps, prop_name, extrap or airspeed_mps > 0,
        )

    def from_rpm(self, prop_name, rpm, airspeed_mps=0.0):
        """Operating point at a specific RPM."""
        p = self._get_prop(prop_name)

        if airspeed_mps == 0.0:
            thrust = float(max(0.0, p.rpm2thr(rpm)))
        else:
            thrust = self._corrected_thrust(rpm, p, airspeed_mps)

        thr, cur, vol, extrap = p.elec_at_rpm(rpm)
        cur = self._airspeed_current(cur, thrust, rpm, p, airspeed_mps)

        return self._build_op(
            thr, rpm, thrust, cur, vol,
            airspeed_mps, prop_name, extrap or airspeed_mps > 0,
        )

    def sweep_airspeed(self, prop_name, throttle_pct=100.0,
                       max_speed_mps=None, n_points=25):
        """Thrust/power curve across airspeeds at fixed throttle."""
        static = self.from_throttle(prop_name, throttle_pct)
        if max_speed_mps is None:
            max_speed_mps = static.pitch_speed_mps * 1.05
        speeds = np.linspace(0, max_speed_mps, n_points)
        return [self.from_throttle(prop_name, throttle_pct, float(v))
                for v in speeds]

    def tip_limited_rpm(self, prop_name, mach_limit=0.9):
        """Max RPM before prop tip exceeds given Mach number."""
        p = self._get_prop(prop_name)
        return mach_limit * self.speed_of_sound * 60.0 / (math.pi * p.d_in * INCH_TO_M)


# ---------------------------------------------------------------------------
# CLI test
# ---------------------------------------------------------------------------

def main():
    import os
    os.chdir(Path(__file__).parent)

    HR = "=" * 72

    print(HR)
    print("simlib2 test -- TestedMotorData")
    print(HR)

    # -- What's in the CSV? ------------------------------------------------
    print("\n--- Available motors ---")
    print(f"  {'Brand':<14} {'Motor':<10} {'KV':>5} {'S':>2}  {'Prop':<14} {'pts':>3}")
    for m in TestedMotorData.available_motors():
        print(f"  {m['brand']:<14} {m['motor']:<10} {m['kv']:>5} {m['batt_s']:>2}  "
              f"{m['prop']:<14} {m['points']:>3}")

    motor = TestedMotorData(
        brand="Uangel",
        motor="A2807",
        kv=1300,
        batt_s=6,
    )

    props = motor.available_props()
    print(f"\n  Available props: {props}")

    PROP = props[0]
    info = motor.prop_info(PROP)
    print(f"  Using: {PROP}  ({info['diameter_in']}x{info['pitch_in']})")
    print(f"  k = {info['k_corr']:.3f},  {info['points']} data points")
    tip_lim = motor.tip_limited_rpm(PROP)
    print(f"  Tip limit: {tip_lim:.0f} RPM (Mach 0.9)")
    print(f"  Max measured: {info['rpm_range'][1]:.0f} RPM, "
          f"{info['thrust_range_g'][1]:.0f}g, "
          f"{info['max_current_a']:.1f}A, {info['max_watts']:.0f}W")

    # -- from_throttle (static) --------------------------------------------
    hdr = f"  {'thr%':>5}  {'RPM':>7}  {'thrust':>7}  {'amps':>6}  {'watts':>6}  {'g/W':>5}  {'Mach':>5}"
    print(f"\n--- from_throttle (static) ---\n{hdr}")
    for t in [40, 50, 60, 70, 80, 90, 100]:
        print(f"  {motor.from_throttle(PROP, t).summary()}")

    # -- from_thrust (static) ----------------------------------------------
    print(f"\n--- from_thrust (static) ---\n{hdr}")
    for tg in [400, 600, 800, 1000, 1200, 1500, 1800]:
        print(f"  {motor.from_thrust(PROP, tg).summary()}")

    # -- from_rpm (static) -------------------------------------------------
    print(f"\n--- from_rpm (static) ---\n{hdr}")
    for rpm in [10000, 12000, 14000, 16000, 18000, 20000]:
        print(f"  {motor.from_rpm(PROP, rpm).summary()}")

    # -- Airspeed sweep at 100% throttle -----------------------------------
    print("\n--- Airspeed sweep @ 100% throttle ---")
    print(f"  {'m/s':>5}  {'km/h':>6}  {'thrust':>7}  {'amps':>6}  {'watts':>6}  "
          f"{'Vpitch':>7}")
    for op in motor.sweep_airspeed(PROP, 100):
        print(
            f"  {op.airspeed_mps:5.1f}  {op.airspeed_mps * 3.6:6.1f}  "
            f"{op.thrust_g:6.0f}g  {op.current_a:5.1f}A  {op.power_w:5.0f}W  "
            f"{op.pitch_speed_mps:6.1f}"
        )

    # -- Endurance estimate ------------------------------------------------
    print("\n--- Endurance estimate ---")
    AUW = 2000
    hover_g = AUW / 4
    print(f"  4-motor quad, AUW = {AUW}g, hover thrust = {hover_g:.0f}g/motor")

    h = motor.from_thrust(PROP, hover_g)
    print(f"  Hover:  {h.throttle_pct:.0f}% thr, {h.current_a:.1f}A, "
          f"{h.power_w:.0f}W/motor")

    batt_mah = 3000
    batt_wh = batt_mah / 1000 * 3.7 * motor.batt_s
    total_w = h.power_w * 4
    endurance = batt_wh / total_w * 60 * 0.80
    print(f"  Pack: {motor.batt_s}S {batt_mah}mAh ~ {batt_wh:.0f}Wh")
    print(f"  Hover endurance: ~{endurance:.1f} min ({total_w:.0f}W total)")

    cruise_v = 15.0
    c = motor.from_thrust(PROP, hover_g * 1.05, airspeed_mps=cruise_v)
    total_wc = c.power_w * 4
    endurance_c = batt_wh / total_wc * 60 * 0.80
    print(f"  Cruise {cruise_v:.0f} m/s: {c.throttle_pct:.0f}% thr, "
          f"{c.current_a:.1f}A, {c.power_w:.0f}W/motor")
    print(f"  Cruise endurance: ~{endurance_c:.1f} min ({total_wc:.0f}W total)")

    # -- Compare all tested props at 100% throttle -------------------------
    print("\n--- All tested props @ 100% throttle (static, REAL DATA) ---")
    print(f"  {'prop':<12}  {'d×p':>7}  {'RPM':>7}  {'thrust':>7}  "
          f"{'amps':>6}  {'watts':>6}  {'g/W':>5}  {'Vpitch':>7}  {'Mach':>5}")
    for pn in props:
        op = motor.from_throttle(pn, 100)
        pi = motor.prop_info(pn)
        dp = f"{pi['diameter_in']:.0f}x{pi['pitch_in']}"
        print(f"  {pn:<12}  {dp:>7}  {op.rpm:7.0f}  {op.thrust_g:6.0f}g  "
              f"{op.current_a:5.1f}A  {op.power_w:5.0f}W  "
              f"{op.efficiency_gw:.2f}  {op.pitch_speed_mps:6.1f}  "
              f"M{op.tip_mach:.2f}")

    # -- Compare all tested props at hover thrust --------------------------
    print(f"\n--- All tested props @ {hover_g:.0f}g hover thrust (static, REAL DATA) ---")
    print(f"  {'prop':<12}  {'thr%':>5}  {'RPM':>7}  {'amps':>6}  {'watts':>6}  "
          f"{'g/W':>5}  {'Mach':>5}")
    for pn in props:
        op = motor.from_thrust(pn, hover_g)
        print(f"  {pn:<12}  {op.throttle_pct:5.1f}%  {op.rpm:7.0f}  "
              f"{op.current_a:5.1f}A  {op.power_w:5.0f}W  "
              f"{op.efficiency_gw:.2f}  M{op.tip_mach:.2f}")

    # -- Untested prop error -----------------------------------------------
    print("\n--- Untested prop error ---")
    try:
        motor.from_throttle("FANTASY_PROP_9999", 100)
    except ValueError as e:
        print(f"  {e}")

    print(f"\n{HR}")


if __name__ == "__main__":
    main()
