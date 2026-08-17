"""
Usage:
python analyze_headless.py
python analyze_headless.py --config test.json
python analyze_headless.py --config test.json --aoa 5
python analyze_headless.py --config test.json --export-step out.step
python analyze_headless.py --config test.json --export-step out.step --step-mode fused --export-stl out.stl
"""

import argparse
import json
from pathlib import Path

from libs.analysis import json_safe_data, load_builder_data, recompute_analysis

SCRIPT_DIR = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(description="Run guibuilder analysis without Tk.")
    parser.add_argument("--config", type=Path, default=None, help="Path to saved aircraft JSON. Defaults to data/default.json.")
    parser.add_argument("--aoa", type=float, default=0.0, help="AoA in degrees for the linear aero pass.")
    parser.add_argument("--airspeed", type=float, default=10.0, help="Operating airspeed in m/s for dimensional surface-force reporting.")
    parser.add_argument("--altitude", type=float, default=0.0, help="Operating altitude in meters for dimensional surface-force reporting.")
    parser.add_argument("--dump-analysis", type=Path, default=None, help="Optional path to write the full computed analysis JSON.")
    parser.add_argument("--export-step", type=Path, default=None, help="Optional path to write the airframe as a STEP file.")
    parser.add_argument("--step-mode", choices=["assembly", "fused"], default="assembly", help="STEP export mode: named component assembly or fused single OML.")
    parser.add_argument("--export-stl", type=Path, default=None, help="Optional path to write the airframe as an STL mesh.")
    parser.add_argument("--export-prop-discs", action="store_true", help="Include thin propeller discs in CAD exports (for actuator-disc CFD setups).")
    args = parser.parse_args()

    modules_data, parts_data, default_config, standards_data, motor_data_df, _motor_names, _propeller_names = load_builder_data()

    if args.config is None:
        assembly_name = "default"
        config = default_config
    else:
        with open(args.config, "r") as file:
            data = json.load(file)
        if "parts" in data:
            assembly_name = args.config.stem
            config = data
        elif "assembly_name" in data and "config" in data:
            assembly_name = data["assembly_name"]
            config = data["config"]
        else:
            assembly_name = next(iter(data.keys()))
            config = data[assembly_name]

    analysis = recompute_analysis(
        assembly_name=assembly_name,
        config=config,
        modules_data=modules_data,
        parts_data=parts_data,
        standards_data=standards_data,
        motor_data_df=motor_data_df,
        aoa_eval_deg=args.aoa,
        force_eval_airspeed_m_s=args.airspeed,
        force_eval_altitude_m=args.altitude,
    )
    print(analysis["report_text"])

    if args.dump_analysis is not None:
        analysis_dump = {
            key: value
            for key, value in analysis.items()
            if key != "report_text"
        }
        with open(args.dump_analysis, "w") as file:
            json.dump(json_safe_data(analysis_dump), file, indent=4, allow_nan=False)

    if args.export_step is not None:
        from libs.cad import export_step

        written_mode = export_step(analysis, args.export_step, mode=args.step_mode, include_prop_discs=args.export_prop_discs)
        print(f"STEP written ({written_mode}): {args.export_step}")

    if args.export_stl is not None:
        from libs.cad import export_stl

        written_mode = export_stl(analysis, args.export_stl, include_prop_discs=args.export_prop_discs)
        print(f"STL written ({written_mode}): {args.export_stl}")


if __name__ == "__main__":
    main()
