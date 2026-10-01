"""
Usage:
python aero_check.py
python aero_check.py --config m62v2.json quadlancet62.json --aoa 2 5 --airspeed 20

Cross-checks the configuration analysis (AeroBuildup) against AeroSandbox VLM, per lifting-surface set.

VLM ignores the fuselage, so exposed panels lose the lift carried across the body and read low.
Single horizontal pairs are therefore also solved with the panels carried through the body to the
centerline, which is the reference they are checked against. Other layouts (X, +, fins) cannot be
carried through without panels meeting on the axis; they are reported against exposed-panel VLM
as a lower bound only.

Exit code 1 if any carried-through pair differs from AeroBuildup by more than --tolerance.
"""

import argparse
import copy
import json
from pathlib import Path

import aerosandbox as asb
import numpy as np

from libs.analysis import build_asb_airplane, load_builder_data, recompute_analysis

VLM_SPANWISE_RESOLUTION = 12
VLM_CHORDWISE_RESOLUTION = 8


def vlm_lift_n(airplane: asb.Airplane, aoa_deg: float, airspeed_m_s: float) -> float:
    vlm = asb.VortexLatticeMethod(
        airplane=airplane,
        op_point=asb.OperatingPoint(velocity=airspeed_m_s, alpha=aoa_deg),
        spanwise_resolution=VLM_SPANWISE_RESOLUTION,
        chordwise_resolution=VLM_CHORDWISE_RESOLUTION,
    )
    return float(vlm.run()["L"])


def carried_through(wing: asb.Wing) -> asb.Wing:
    """Copy of a symmetric pair wing with an extra root section on the centerline, along the panel direction."""
    root, tip = wing.xsecs[0], wing.xsecs[-1]
    dir_y = tip.xyz_le[1] - root.xyz_le[1]
    dir_z = tip.xyz_le[2] - root.xyz_le[2]
    k = root.xyz_le[1] / dir_y
    inner = copy.deepcopy(root)
    inner.xyz_le = np.array([root.xyz_le[0], 0.0, root.xyz_le[2] - k * dir_z])
    return asb.Wing(name=wing.name, symmetric=True, xsecs=[inner, *wing.xsecs])


def main() -> None:
    parser = argparse.ArgumentParser(description="Cross-check AeroBuildup surface lift against VLM.")
    parser.add_argument("--config", type=Path, nargs="*", default=[], help="Saved aircraft JSON files. data/default.json is always included.")
    parser.add_argument("--aoa", type=float, nargs="+", default=[2.0, 5.0], help="AoA values in degrees.")
    parser.add_argument("--airspeed", type=float, default=20.0, help="Airspeed in m/s.")
    parser.add_argument("--tolerance", type=float, default=0.15, help="Allowed relative lift difference for carried-through pairs.")
    args = parser.parse_args()

    modules_data, parts_data, default_config, standards_data, motor_data_df, _motor_names, _propeller_names = load_builder_data()
    configs = [("default", default_config)]
    for path in args.config:
        with open(path, "r") as file:
            configs.append((path.stem, json.load(file)["config"]))

    failures = []
    header = f"{'config':<14} {'AoA':>4}  {'surface':<18} {'layout':<6} {'AeroBuildup':>11} {'VLM thru':>9} {'VLM exposed':>11}  status"
    print(header)
    print("-" * len(header))
    for name, config in configs:
        for aoa_deg in args.aoa:
            analysis = recompute_analysis(name, config, modules_data, parts_data, standards_data, motor_data_df, aoa_deg, args.airspeed, 0.0)
            for surface, row in zip(analysis["surface_sets"], analysis["surface_force_rows"]):
                single = dict(analysis, surface_sets=[surface], motor_draw_rows=[])
                airplane = build_asb_airplane(single)
                airplane.fuselages = []
                exposed_n = vlm_lift_n(airplane, aoa_deg, args.airspeed)
                layout = "".join(sorted({"H" if abs(panel["dir_z"]) < 1e-9 else "V" if abs(panel["dir_y"]) < 1e-9 else "X" for panel in surface["panels"]}))
                if len(surface["panels"]) == 2 and surface["panels"][0]["xz_mirror"] == "primary":
                    airplane.wings = [carried_through(wing) for wing in airplane.wings]
                    through_n = vlm_lift_n(airplane, aoa_deg, args.airspeed)
                    error = abs(row["lift_n"] - through_n) / abs(through_n)
                    status = f"{'FAIL' if error > args.tolerance else 'ok'} ({error * 100.0:.0f} %)"
                    if error > args.tolerance:
                        failures.append((name, aoa_deg, surface["name"]))
                    through_text = f"{through_n:9.3f}"
                else:
                    status = f"info (x{row['lift_n'] / exposed_n:.2f} of exposed)" if abs(exposed_n) > 1e-9 else "info"
                    through_text = f"{'n/a':>9}"
                print(f"{name:<14} {aoa_deg:4.1f}  {surface['name']:<18} {layout:<6} {row['lift_n']:11.3f} {through_text} {exposed_n:11.3f}  {status}")
            print(
                f"{name:<14} {aoa_deg:4.1f}  {'(aircraft)':<18} {'':<6} {analysis['aero_lift_n']:11.3f} {'':>9} {'':>11}  "
                f"x_np={analysis['x_np']:.4f} m  cg={analysis['cg_x_m']:.4f} m  {analysis['stability_verdict']}"
            )

    if failures:
        print(f"\n{len(failures)} carried-through pair(s) outside {args.tolerance * 100.0:.0f} %: {failures}")
        raise SystemExit(1)
    print("\nAll carried-through pairs within tolerance.")


if __name__ == "__main__":
    main()
