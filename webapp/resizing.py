"""Longitudinal editor resizing: section lengths and positions share one station map."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from itertools import accumulate


@dataclass(frozen=True)
class StationMap:
    before: tuple[float, ...]
    after: tuple[float, ...]

    def __call__(self, x: float) -> float:
        # Overhanging placements keep their distance from the nearest end.
        if x <= 0:
            return x
        if x >= self.before[-1]:
            return self.after[-1] + x - self.before[-1]
        for low, high, new_low, new_high in zip(self.before, self.before[1:], self.after, self.after[1:]):
            if x <= high:
                return new_low + (x - low) * (new_high - new_low) / (high - low)
        raise AssertionError(f"unmapped station {x}")


def _positive_length(value: object) -> float:
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"length must be a positive finite number of metres, got {value!r}")
    return float(value)


def _stations(definition: dict) -> tuple[float, ...]:
    stations = (0.0, *definition["sections"]["cuts_m"], definition["fuselage"]["length_m"])
    if any(not math.isfinite(x) for x in stations) or any(a >= b for a, b in zip(stations, stations[1:])):
        raise ValueError("Fix section boundaries before resizing the airframe")
    if len(definition["sections"]["per_section"]) != len(stations) - 1:
        raise ValueError("Fix the section count before resizing the airframe")
    return stations


def _distribute(lengths: list[float], indices: list[int], available: float) -> None:
    total = sum(lengths[index] for index in indices)
    if not indices:
        if not math.isclose(available, 0.0, abs_tol=1e-12):
            raise ValueError("No unfixed sections available to absorb this length change")
        return
    if available <= 0:
        raise ValueError("Requested length leaves no room for the unfixed sections")
    factor = available / total
    for index in indices:
        lengths[index] *= factor


def _apply(definition: dict, before: tuple[float, ...], lengths: list[float]) -> None:
    target = definition
    definition = copy.deepcopy(target)
    station = StationMap(before, (0.0, *accumulate(lengths)))
    fuselage = definition["fuselage"]
    old_length = before[-1]
    # Profile lengths represent stations on the body; hardware and planform sizes do not.
    fuselage["nose"]["length_m"] = station(fuselage["nose"]["length_m"])
    fuselage["tail"]["length_m"] = station.after[-1] - station(old_length - fuselage["tail"]["length_m"])
    extents = [*fuselage.get("channels", [])]
    if fuselage.get("reinforcement") is not None:
        extents.append(fuselage["reinforcement"])
    for extent in extents:
        for key in ("from_x_m", "to_x_m"):
            extent[key] = station(extent[key])

    volume_x = {volume["name"]: volume["x_m"] for volume in definition["volumes"]}
    placements = [*definition["parts"]]
    if definition.get("propulsion") is not None:
        placements.append(definition["propulsion"]["battery"])
    for item in placements:
        if "volume" in item:
            center = volume_x[item["volume"]]
            if "x_offset_m" in item:
                item["x_offset_m"] = station(center + item["x_offset_m"]) - station(center)
        else:
            item["x_m"] = station(item["x_m"])
    for volume in definition["volumes"]:
        volume["x_m"] = station(volume["x_m"])
    for surface in definition["surfaces"]:
        surface["x_root_le_m"] = station(surface["x_root_le_m"])
    # Surface mounts and spars follow their surface, nose/tail mounts follow the body ends.
    definition["sections"]["cuts_m"] = list(station.after[1:-1])
    fuselage["length_m"] = station.after[-1]
    target.update(definition)


def resize_airframe(definition: dict, value: object) -> None:
    length = _positive_length(value)
    before = _stations(definition)
    if length == before[-1]:
        return
    lengths = [b - a for a, b in zip(before, before[1:])]
    rows = definition["sections"]["per_section"]
    free = [index for index, row in enumerate(rows) if not row.get("fixed_length", False)]
    fixed = sum(size for size, row in zip(lengths, rows) if row.get("fixed_length", False))
    _distribute(lengths, free, length - fixed)
    _apply(definition, before, lengths)


def resize_section(definition: dict, index: object, value: object) -> None:
    length = _positive_length(value)
    before = _stations(definition)
    lengths = [b - a for a, b in zip(before, before[1:])]
    if type(index) is not int or not 0 <= index < len(lengths):
        raise ValueError(f"section index {index!r} is out of range")
    if math.isclose(length, lengths[index], abs_tol=1e-12):
        return
    rows = definition["sections"]["per_section"]
    free = [i for i in range(len(lengths)) if i != index and not rows[i].get("fixed_length", False)]
    fixed = sum(lengths[i] for i in range(len(lengths)) if i != index and rows[i].get("fixed_length", False))
    _distribute(lengths, free, before[-1] - length - fixed)
    lengths[index] = length
    _apply(definition, before, lengths)
