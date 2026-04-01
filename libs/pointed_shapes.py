"""
Usage:
from libs.pointed_shapes import sample_pointed_profile
"""

from __future__ import annotations

import math

POINTED_SHAPE_ALIASES = {
    "ogive": "tangent_ogive",
    "ogive_blunt": "spherically_blunted_tangent_ogive",
    "line": "conic",
    "para": "parabolic",
    "elli": "elliptical",
}
POINTED_SHAPE_PARAM_KEYS = {
    "conic": [],
    "elliptical": [],
    "parabolic": ["k_prime"],
    "spherically_blunted_conic": ["nose_radius"],
    "bi_conic": ["break_fraction", "break_radius_fraction"],
    "tangent_ogive": [],
    "spherically_blunted_tangent_ogive": ["nose_radius"],
    "secant_ogive": ["rho_factor"],
    "power_series": ["exponent"],
    "haack_series": ["c"],
}


def normalize_shape_name(shape_name: str) -> str:
    lowered = shape_name.strip().lower().replace("-", "_").replace(" ", "_")
    return POINTED_SHAPE_ALIASES.get(lowered, lowered)


def ogive_radius(radius_value: float, length_value: float) -> float:
    return (radius_value * radius_value + length_value * length_value) / (2.0 * radius_value)


def pointed_shape_param_values(pointed_shape: dict) -> dict:
    shape_key = normalize_shape_name(pointed_shape["shape"])
    return {key: pointed_shape[key] for key in POINTED_SHAPE_PARAM_KEYS[shape_key]}


def pointed_shape_radius(shape_name: str, x_value: float, length_value: float, radius_value: float, shape_params: dict | None = None) -> float:
    params = shape_params or {}
    shape_key = normalize_shape_name(shape_name)
    if length_value <= 0.0:
        return radius_value
    u_value = min(max(x_value / length_value, 0.0), 1.0)
    if shape_key == "conic":
        return radius_value * u_value
    if shape_key == "elliptical":
        return radius_value * math.sqrt(max(1.0 - (1.0 - u_value) ** 2, 0.0))
    if shape_key == "parabolic":
        k_prime = params["k_prime"]
        return radius_value * (2.0 * u_value - k_prime * u_value * u_value) / (2.0 - k_prime)
    if shape_key == "power_series":
        exponent = params["exponent"]
        return radius_value * (u_value ** exponent)
    if shape_key == "haack_series":
        haack_c = params["c"]
        theta = math.acos(1.0 - 2.0 * u_value)
        return radius_value * math.sqrt(max((theta - math.sin(2.0 * theta) / 2.0 + haack_c * math.sin(theta) ** 3) / math.pi, 0.0))
    if shape_key == "bi_conic":
        break_length = params["break_fraction"] * length_value
        break_radius = params["break_radius_fraction"] * radius_value
        if x_value <= break_length:
            return x_value * break_radius / break_length
        return break_radius + (x_value - break_length) * (radius_value - break_radius) / (length_value - break_length)
    if shape_key == "tangent_ogive":
        rho_value = ogive_radius(radius_value, length_value)
        term_value = rho_value * rho_value - (length_value - x_value) * (length_value - x_value)
        return math.sqrt(max(term_value, 0.0)) + radius_value - rho_value
    if shape_key == "secant_ogive":
        rho_value = params["rho_factor"] * ogive_radius(radius_value, length_value)
        alpha_value = math.atan(radius_value / length_value) - math.acos(math.sqrt(radius_value * radius_value + length_value * length_value) / (2.0 * rho_value))
        term_value = rho_value * rho_value - (x_value - rho_value * math.cos(alpha_value)) ** 2
        return math.sqrt(max(term_value, 0.0)) + rho_value * math.sin(alpha_value)
    raise RuntimeError(f"Unsupported pointed shape: {shape_name}")


def sample_pointed_profile(shape_name: str, length_value: float, radius_value: float, sample_count: int, shape_params: dict | None = None) -> list[tuple[float, float]]:
    shape_key = normalize_shape_name(shape_name)
    if sample_count < 2:
        raise RuntimeError("sample_count must be at least 2")
    if shape_key in {"spherically_blunted_conic", "spherically_blunted_tangent_ogive"}:
        return sample_spherical_blunted_profile(shape_key, length_value, radius_value, sample_count, shape_params)
    stations = []
    for index in range(sample_count):
        x_value = length_value * index / (sample_count - 1)
        stations.append((x_value, pointed_shape_radius(shape_key, x_value, length_value, radius_value, shape_params)))
    return stations


def sample_spherical_blunted_profile(shape_name: str, length_value: float, radius_value: float, sample_count: int, shape_params: dict | None = None) -> list[tuple[float, float]]:
    params = dict(shape_params or {})
    cap_radius = params["nose_radius"]
    base_shape = "conic" if shape_name == "spherically_blunted_conic" else "tangent_ogive"
    underlying_length = length_value
    if cap_radius > 0.0:
        lo_min = length_value
        lo_max = max(length_value * 2.0, length_value + radius_value + cap_radius)
        for _ in range(64):
            if shape_name == "spherically_blunted_conic":
                x_t = cap_radius * lo_max * lo_max / (radius_value * math.sqrt(radius_value * radius_value + lo_max * lo_max))
                y_t = cap_radius * lo_max / math.sqrt(radius_value * radius_value + lo_max * lo_max)
                x_o = x_t + math.sqrt(cap_radius * cap_radius - y_t * y_t)
            else:
                x_o = lo_max - math.sqrt((radius_value - cap_radius) * ((lo_max * lo_max / radius_value) - cap_radius))
            x_a = x_o - cap_radius
            if lo_max - x_a >= length_value:
                break
            lo_max *= 2.0
        for _ in range(80):
            underlying_length = 0.5 * (lo_min + lo_max)
            if shape_name == "spherically_blunted_conic":
                x_t = cap_radius * underlying_length * underlying_length / (radius_value * math.sqrt(radius_value * radius_value + underlying_length * underlying_length))
                y_t = cap_radius * underlying_length / math.sqrt(radius_value * radius_value + underlying_length * underlying_length)
                x_o = x_t + math.sqrt(cap_radius * cap_radius - y_t * y_t)
            else:
                x_o = underlying_length - math.sqrt((radius_value - cap_radius) * ((underlying_length * underlying_length / radius_value) - cap_radius))
            x_a = x_o - cap_radius
            final_length = underlying_length - x_a
            if abs(final_length - length_value) < 1e-12:
                break
            if final_length > length_value:
                lo_max = underlying_length
            else:
                lo_min = underlying_length
    if shape_name == "spherically_blunted_conic":
        x_t = cap_radius * underlying_length * underlying_length / (radius_value * math.sqrt(radius_value * radius_value + underlying_length * underlying_length))
        y_t = cap_radius * underlying_length / math.sqrt(radius_value * radius_value + underlying_length * underlying_length)
        x_o = x_t + math.sqrt(cap_radius * cap_radius - y_t * y_t)
    else:
        x_o = underlying_length - math.sqrt((radius_value - cap_radius) * ((underlying_length * underlying_length / radius_value) - cap_radius))
        y_t = (
            cap_radius * (underlying_length * underlying_length - radius_value * radius_value)
            / (underlying_length * underlying_length + radius_value * radius_value - 2.0 * radius_value * cap_radius)
        )
        x_t = x_o - cap_radius * math.sqrt(
            1.0
            - (
                (underlying_length * underlying_length - radius_value * radius_value)
                / (underlying_length * underlying_length + radius_value * radius_value - 2.0 * radius_value * cap_radius)
            )
            ** 2
        )
    x_a = x_o - cap_radius
    x_t_final = x_t - x_a
    stations = []
    for index in range(sample_count):
        x_value = length_value * index / (sample_count - 1)
        if x_value <= x_t_final:
            radius_out = math.sqrt(max(cap_radius * cap_radius - (x_value - cap_radius) * (x_value - cap_radius), 0.0))
        else:
            source_x = x_value + x_a
            radius_out = pointed_shape_radius(base_shape, source_x, underlying_length, radius_value, params)
        stations.append((x_value, radius_out))
    return stations
