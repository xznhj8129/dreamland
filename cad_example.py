"""
Usage:
python cad_example.py
"""

import json

from analysis import load_builder_data, recompute_analysis
from cad import build_freecad_exports


def main() -> None:
    modules_data, parts_data, default_config, standards_data, motor_data_df, _motor_names, _propeller_names = load_builder_data()
    analysis = recompute_analysis(
        assembly_name="default",
        config=default_config,
        modules_data=modules_data,
        parts_data=parts_data,
        standards_data=standards_data,
        motor_data_df=motor_data_df,
        aoa_eval_deg=0.0,
    )
    exports = build_freecad_exports(default_config, analysis, standards_data)
    print(json.dumps(exports, indent=2))


if __name__ == "__main__":
    main()
