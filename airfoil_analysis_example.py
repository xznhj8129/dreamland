"""
Usage:
python airfoil_analysis_example.py
"""

from __future__ import annotations

from libs.airfoil_analysis import (
    DEFAULT_AIRFOIL_OPERATING_POINT,
    DEFAULT_NEURALFOIL_SETTINGS,
    analyze_airfoils,
    load_airfoil_library,
)


def main() -> None:
    airfoil_library = load_airfoil_library()
    records = [
        airfoil_library["naca2412"],
        airfoil_library["naca0008"],
    ]
    results = analyze_airfoils(records, DEFAULT_AIRFOIL_OPERATING_POINT, DEFAULT_NEURALFOIL_SETTINGS)
    print(results["summary_text"])


if __name__ == "__main__":
    main()
