"""
One line of a text report with how much it matters; the GUI colours lines by level, plain text ignores it.
"""

from dataclasses import dataclass

LEVELS = ("info", "warning", "problem")


@dataclass(frozen=True)
class ReportLine:
    text: str
    level: str = "info"  # LEVELS
