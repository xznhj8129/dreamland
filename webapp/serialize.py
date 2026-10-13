"""JSON-safe snapshots of AppState for the HTTP API and the websocket state pushes."""

from __future__ import annotations

import dataclasses
from typing import Any

from libs.presets import preset_carry_through, preset_spars, preset_status
from libs.propulsion import summary_lines
from libs.reportline import ReportLine
from webapp.report import build_report_pages
from webapp.state import AppState


def lines_payload(lines: list[ReportLine]) -> list[dict[str, str]]:
    return [{"text": line.text, "level": line.level} for line in lines]


def chart_payload(summary) -> dict[str, Any]:
    """The Motor info chart of the propulsion summary (editor_view.build_motor_info 1069-1098): one point per duty
    (0..1) for each series, and the hover duty/index when the aircraft can hover. The summary's numpy arrays become
    plain lists; nothing else of the summary is shipped."""
    curve = summary.curve
    hover = summary.hover
    hover_duty = None if hover is None else float(hover.duty)
    hover_index = None
    if hover_duty is not None:
        hover_index = min(range(len(curve.duty)), key=lambda index: abs(float(curve.duty[index]) - hover_duty))
    return {
        "duty": curve.duty.tolist(),
        "thrust_n": curve.thrust_n.tolist(),
        "rpm": curve.rpm.tolist(),
        "power_w": curve.power_w.tolist(),
        "hover_duty": hover_duty,
        "hover_index": hover_index,
    }


def pack_payload(state: AppState) -> dict[str, Any] | None:
    """The resolved pack summary (editor_view.build_battery_group 1000-1005): label, size, mass and capacity."""
    if state.propulsion is None or state.resolved is None or state.resolved.airframe.propulsion is None or state.resolved.airframe.propulsion.electric is None:
        return None
    pack = state.resolved.airframe.propulsion.electric.battery
    return {
        "label": pack.label,
        "size_mm": None if pack.placed.size_m is None else [float(value) * 1000.0 for value in pack.placed.size_m],
        "mass_g": float(pack.placed.mass_kg[0]) * 1000.0,
        "capacity_mah": float(pack.capacity_mah),
    }


def snapshot(state: AppState) -> dict[str, Any]:
    """The whole state as plain JSON types; json.dumps needs no custom encoder."""
    return {
        "project": state.project.name,
        "title": state.title,
        "session_id": state.session_id,
        "definition": state.definition,
        "definition_revision": state.definition_revision,
        "library_revision": state.library_revision,
        "definition_valid": state.definition_valid,
        "dirty": state.dirty,
        "preset_status": preset_status(state.definition),
        # The current preset label's spars (libs.presets.preset_spars 34-37): the surfaces form prefills a new spars
        # block from these, and its carry-through (40-42) for the nested block; null without either.
        "preset_spars": preset_spars(state.definition),
        "preset_carry_through": preset_carry_through(state.definition),
        "resolved": None if state.resolved is None else dataclasses.asdict(state.resolved),
        "problems": list(state.problems),
        "propulsion": [] if state.propulsion is None else lines_payload(summary_lines(state.propulsion)),
        "propulsion_chart": None if state.propulsion is None else chart_payload(state.propulsion),
        "pack_summary": pack_payload(state),
        "report": lines_payload(state.report),
        "report_pages": None if state.resolved is None else dataclasses.asdict(build_report_pages(state.resolved, state.aero, state.problems, state.propulsion, state.status)),
        "log": [dataclasses.asdict(entry) for entry in state.log],
        "status": state.status,
        "aero_pending": state.aero is None,
        "geometry": state.geometry_status,  # Compute Geometry: none | stale | exact | current | computing (AppState.geometry_status)
    }
