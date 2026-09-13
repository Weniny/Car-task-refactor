"""Pure Ranger Mini V3 Twist adaptation helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RangerMiniV3Geometry:
    wheelbase_m: float = 0.494
    track_width_m: float = 0.364
    driver_min_turn_radius_m: float = 0.47644


def adapt_yaw_rate_for_ranger_driver(
    *,
    linear_x_mps: float,
    desired_yaw_rate_radps: float,
    geometry: RangerMiniV3Geometry = RangerMiniV3Geometry(),
) -> float:
    """Return the yaw-rate field that makes Ranger's driver realize the desired yaw."""

    speed = float(linear_x_mps)
    desired_yaw_rate = float(desired_yaw_rate_radps)
    if abs(speed) < 1e-9 or abs(desired_yaw_rate) < 1e-9:
        return desired_yaw_rate

    desired_curvature = desired_yaw_rate / speed
    command_radius = command_radius_for_actual_curvature(
        abs(desired_curvature), geometry=geometry
    )
    return math.copysign(abs(speed) / command_radius, desired_yaw_rate)


def command_radius_for_actual_curvature(
    curvature_1pm: float, *, geometry: RangerMiniV3Geometry
) -> float:
    if geometry.wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    if geometry.track_width_m < 0.0:
        raise ValueError("track_width_m must be non-negative")
    if geometry.driver_min_turn_radius_m <= 0.0:
        raise ValueError("driver_min_turn_radius_m must be positive")
    if curvature_1pm < 0.0:
        raise ValueError("curvature_1pm must be non-negative")
    if curvature_1pm < 1e-9:
        return math.inf

    central_sin = curvature_1pm * geometry.wheelbase_m / 2.0
    if central_sin >= 1.0:
        command_radius = geometry.driver_min_turn_radius_m
    else:
        central_angle = math.asin(central_sin)
        central_tan = math.tan(central_angle)
        denominator = geometry.wheelbase_m - central_tan * geometry.track_width_m
        if denominator <= 0.0:
            command_radius = geometry.driver_min_turn_radius_m
        else:
            inner_tan = central_tan * geometry.wheelbase_m / denominator
            command_radius = (geometry.wheelbase_m / 2.0) / inner_tan
    return max(command_radius, geometry.driver_min_turn_radius_m)
