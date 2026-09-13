"""Near-field point-cloud stop gate for supervised low-speed runs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class ProximityStopConfig:
    x_min_m: float = 0.25
    stop_distance_m: float = 0.55
    front_half_angle_rad: float = 0.4363
    lateral_half_width_m: float = 0.45
    z_min_m: float = -0.25
    z_max_m: float = 0.80
    min_points: int = 3

    def __post_init__(self) -> None:
        if self.x_min_m < 0.0:
            raise ValueError("x_min_m must be non-negative")
        if self.stop_distance_m < 0.0:
            raise ValueError("stop_distance_m must be non-negative")
        if 0.0 < self.stop_distance_m <= self.x_min_m:
            raise ValueError(
                "enabled stop_distance_m must be greater than x_min_m"
            )
        if not (0.0 < self.front_half_angle_rad < math.pi / 2.0):
            raise ValueError("front_half_angle_rad must be in (0, pi/2)")
        if self.lateral_half_width_m <= 0.0:
            raise ValueError("lateral_half_width_m must be positive")
        if self.z_max_m <= self.z_min_m:
            raise ValueError("z_max_m must be greater than z_min_m")
        if self.min_points < 1:
            raise ValueError("min_points must be at least 1")


@dataclass(frozen=True)
class LocalGridConfig:
    resolution_m: float = 0.05
    x_min_m: float = -0.50
    x_max_m: float = 3.00
    y_min_m: float = -1.50
    y_max_m: float = 1.50
    inflation_radius_m: float = 0.20
    scan_bin_count: int = 360
    scan_range_min_m: float = 0.10
    scan_range_max_m: float = 6.00

    def __post_init__(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.x_max_m <= self.x_min_m or self.y_max_m <= self.y_min_m:
            raise ValueError("grid maximum bounds must exceed minimum bounds")
        if self.inflation_radius_m < 0.0:
            raise ValueError("inflation_radius_m must be non-negative")
        if self.scan_bin_count < 1:
            raise ValueError("scan_bin_count must be at least 1")
        if self.scan_range_min_m < 0.0:
            raise ValueError("scan_range_min_m must be non-negative")
        if self.scan_range_max_m <= self.scan_range_min_m:
            raise ValueError("scan_range_max_m must exceed scan_range_min_m")


@dataclass(frozen=True)
class LocalCostmap:
    resolution_m: float
    width: int
    height: int
    origin_x_m: float
    origin_y_m: float
    data: tuple[int, ...]


@dataclass(frozen=True)
class LocalClearanceResult:
    stop: bool
    point_count: int
    nearest_obstacle_distance_m: float | None
    costmap: LocalCostmap
    scan_angle_min_rad: float
    scan_angle_increment_rad: float
    scan_ranges_m: tuple[float, ...]


def laser_scan_points(
    ranges_m: Iterable[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
) -> tuple[tuple[float, float, float], ...]:
    """Convert valid planar scan bins into body-frame points."""

    if angle_increment_rad <= 0.0:
        raise ValueError("angle_increment_rad must be positive")
    if range_min_m < 0.0 or range_max_m <= range_min_m:
        raise ValueError("scan range limits are invalid")
    points: list[tuple[float, float, float]] = []
    for index, value in enumerate(ranges_m):
        distance = float(value)
        if not math.isfinite(distance):
            continue
        if not range_min_m <= distance <= range_max_m:
            continue
        angle = angle_min_rad + index * angle_increment_rad
        points.append(
            (
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.0,
            )
        )
    return tuple(points)


def count_points_in_stop_box(
    points: Iterable[tuple[float, float, float]],
    config: ProximityStopConfig,
) -> int:
    count = 0
    for x, y, z in points:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        range_xy = math.hypot(x, y)
        if not (config.z_min_m <= z <= config.z_max_m):
            continue
        in_front_sector = (
            config.x_min_m <= range_xy <= config.stop_distance_m
            and x > 0.0
            and abs(math.atan2(y, x)) <= config.front_half_angle_rad
        )
        in_body_corridor = (
            config.x_min_m <= x <= config.stop_distance_m
            and abs(y) <= config.lateral_half_width_m
        )
        if not (in_front_sector or in_body_corridor):
            continue
        count += 1
    return count


def should_stop_for_points(
    points: Iterable[tuple[float, float, float]],
    config: ProximityStopConfig,
) -> tuple[bool, int]:
    count = count_points_in_stop_box(points, config)
    return count >= config.min_points, count


def evaluate_local_clearance(
    points: Iterable[tuple[float, float, float]],
    stop_config: ProximityStopConfig,
    grid_config: LocalGridConfig,
) -> LocalClearanceResult:
    filtered_points = [
        (float(x), float(y), float(z))
        for x, y, z in points
        if math.isfinite(x)
        and math.isfinite(y)
        and math.isfinite(z)
        and stop_config.z_min_m <= z <= stop_config.z_max_m
    ]

    stop_distances = [
        math.hypot(x, y)
        for x, y, _ in filtered_points
        if _point_is_in_stop_box(x, y, stop_config)
    ]
    point_count = len(stop_distances)
    costmap = _build_local_costmap(filtered_points, grid_config)
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / grid_config.scan_bin_count
    scan_ranges = [math.inf] * grid_config.scan_bin_count
    for x, y, _ in filtered_points:
        distance = math.hypot(x, y)
        if not (
            grid_config.scan_range_min_m
            <= distance
            <= grid_config.scan_range_max_m
        ):
            continue
        index = int((math.atan2(y, x) - angle_min) / angle_increment)
        index = min(max(index, 0), grid_config.scan_bin_count - 1)
        scan_ranges[index] = min(scan_ranges[index], distance)

    return LocalClearanceResult(
        stop=point_count >= stop_config.min_points,
        point_count=point_count,
        nearest_obstacle_distance_m=min(stop_distances) if stop_distances else None,
        costmap=costmap,
        scan_angle_min_rad=angle_min,
        scan_angle_increment_rad=angle_increment,
        scan_ranges_m=tuple(scan_ranges),
    )


def evaluate_fused_local_clearance(
    point_frames: Iterable[Iterable[tuple[float, float, float]]],
    stop_config: ProximityStopConfig,
    grid_config: LocalGridConfig,
) -> LocalClearanceResult:
    """Build one local obstacle layer from consecutive planar scan frames."""

    return evaluate_local_clearance(
        (
            point
            for frame_points in point_frames
            for point in frame_points
        ),
        stop_config,
        grid_config,
    )


def advance_periodic_deadline(
    *,
    now_s: float,
    next_deadline_s: float | None,
    period_s: float,
) -> tuple[bool, float]:
    """Keep a fixed-rate phase when input callbacks arrive slightly early."""

    if period_s <= 0.0:
        raise ValueError("period_s must be positive")
    if next_deadline_s is None:
        return True, now_s + period_s
    if now_s < next_deadline_s:
        return False, next_deadline_s
    elapsed_periods = math.floor((now_s - next_deadline_s) / period_s)
    return True, next_deadline_s + (elapsed_periods + 1) * period_s


def _point_is_in_stop_box(
    x: float,
    y: float,
    config: ProximityStopConfig,
) -> bool:
    if config.stop_distance_m == 0.0:
        return False
    range_xy = math.hypot(x, y)
    in_front_sector = (
        config.x_min_m <= range_xy <= config.stop_distance_m
        and x > 0.0
        and abs(math.atan2(y, x)) <= config.front_half_angle_rad
    )
    in_body_corridor = (
        config.x_min_m <= x <= config.stop_distance_m
        and abs(y) <= config.lateral_half_width_m
    )
    return in_front_sector or in_body_corridor


def _build_local_costmap(
    points: Iterable[tuple[float, float, float]],
    config: LocalGridConfig,
) -> LocalCostmap:
    width = math.ceil((config.x_max_m - config.x_min_m) / config.resolution_m)
    height = math.ceil((config.y_max_m - config.y_min_m) / config.resolution_m)
    data = [-1] * (width * height)
    occupied_cells = {
        (
            int((x - config.x_min_m) / config.resolution_m),
            int((y - config.y_min_m) / config.resolution_m),
        )
        for x, y, _ in points
        if config.x_min_m <= x < config.x_max_m
        and config.y_min_m <= y < config.y_max_m
    }
    radius_cells = math.ceil(config.inflation_radius_m / config.resolution_m)
    inflation_offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * config.resolution_m
        <= config.inflation_radius_m + 1e-9
    ]
    for x_index, y_index in occupied_cells:
        for dx, dy in inflation_offsets:
            inflated_x = x_index + dx
            inflated_y = y_index + dy
            if not (0 <= inflated_x < width and 0 <= inflated_y < height):
                continue
            index = inflated_y * width + inflated_x
            data[index] = max(data[index], 50)
        data[y_index * width + x_index] = 100

    return LocalCostmap(
        resolution_m=config.resolution_m,
        width=width,
        height=height,
        origin_x_m=config.x_min_m,
        origin_y_m=config.y_min_m,
        data=tuple(data),
    )
