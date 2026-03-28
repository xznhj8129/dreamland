"""
Usage:
python analyze_headless.py
python analyze_headless.py --config test.json
python analyze_headless.py --config test.json --aoa 5
"""

import argparse
import json
from pathlib import Path

from analysis import json_safe_data, load_builder_data, recompute_analysis

SCRIPT_DIR = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(description="Run guibuilder analysis without Tk.")
    parser.add_argument("--config", type=Path, default=None, help="Path to saved aircraft JSON. Defaults to libs/default.json.")
    parser.add_argument("--aoa", type=float, default=0.0, help="AoA in degrees for the linear aero pass.")
    parser.add_argument("--dump-analysis", type=Path, default=None, help="Optional path to write the full computed analysis JSON.")
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
    )
    print(analysis["report_text"])

    if args.dump_analysis is not None:
        with open(args.dump_analysis, "w") as file:
            json.dump(json_safe_data(analysis), file, indent=4, allow_nan=False)


if __name__ == "__main__":
    main()
