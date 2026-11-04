"""Robust body-frame shelf observation for map-independent final docking."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class ShelfAlignmentConfig:
    side: str = "RIGHT"
    min_range_m: float = 0.10
    max_range_m: float = 1.50
    min_longitudinal_m: float = -0.60
    max_longitudinal_m: float = 0.80
    min_side_distance_m: float = 0.20
    max_side_distance_m: float = 1.00
    min_points: int = 12
    min_span_m: float = 0.30
    max_residual_m: float = 0.025
    max_heading_error_rad: float = math.radians(15.0)
    max_candidate_points: int = 40

    def __post_init__(self) -> None:
        if self.side not in {"LEFT", "RIGHT"}:
            raise ValueError("shelf side must be LEFT or RIGHT")
        if not (0.0 <= self.min_range_m < self.max_range_m):
            raise ValueError("shelf scan ranges must satisfy 0 <= min < max")
        if self.min_longitudinal_m >= self.max_longitudinal_m:
            raise ValueError("shelf longitudinal window must satisfy min < max")
        if not (0.0 < self.min_side_distance_m < self.max_side_distance_m):
            raise ValueError("shelf side distance window must satisfy 0 < min < max")
        if self.min_points < 2:
            raise ValueError("shelf fit needs at least two points")
        if self.min_span_m <= 0.0 or self.max_residual_m <= 0.0:
            raise ValueError("shelf fit span and residual limits must be positive")
        if not (0.0 < self.max_heading_error_rad < math.pi / 2.0):
            raise ValueError("shelf heading limit must be between 0 and pi/2")
        if self.max_candidate_points < self.min_points:
            raise ValueError("shelf candidate cap must cover the minimum point count")


@dataclass(frozen=True)
class ShelfObservation:
    side_distance_m: float
    heading_error_rad: float
    point_count: int
    residual_rms_m: float
    span_m: float


@dataclass(frozen=True)
class ShelfScan:
    ranges: tuple[float, ...]
    angle_min_rad: float
    angle_increment_rad: float


def estimate_shelf_from_scan(
    ranges: Sequence[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    config: ShelfAlignmentConfig,
) -> ShelfObservation | None:
    """Fit the selected shelf side without trusting the global map pose."""

    return estimate_shelf_from_scans(
        (
            ShelfScan(
                ranges=tuple(float(value) for value in ranges),
                angle_min_rad=angle_min_rad,
                angle_increment_rad=angle_increment_rad,
            ),
        ),
        config=config,
    )


def estimate_shelf_from_scans(
    scans: Sequence[ShelfScan],
    *,
    config: ShelfAlignmentConfig,
) -> ShelfObservation | None:
    """Fit a shelf from a short, bounded scan window used only while stopped."""

    side_sign = 1.0 if config.side == "LEFT" else -1.0
    points: list[tuple[float, float]] = []
    for scan in scans:
        if (
            not math.isfinite(scan.angle_min_rad)
            or not math.isfinite(scan.angle_increment_rad)
            or scan.angle_increment_rad == 0.0
        ):
            continue
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if not (config.min_range_m <= distance <= config.max_range_m):
                continue
            angle = scan.angle_min_rad + index * scan.angle_increment_rad
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            side_distance = side_sign * y
            if not (config.min_longitudinal_m <= x <= config.max_longitudinal_m):
                continue
            if not (
                config.min_side_distance_m
                <= side_distance
                <= config.max_side_distance_m
            ):
                continue
            points.append((x, y))
    if len(points) < config.min_points:
        return None

    fit_points = _uniform_sample(points, config.max_candidate_points * 2)
    candidates = _uniform_sample(fit_points, config.max_candidate_points)
    inlier_threshold = config.max_residual_m * 1.5
    best: tuple[int, float, float, list[tuple[float, float]]] | None = None
    for left_index, left in enumerate(candidates[:-1]):
        for right in candidates[left_index + 1 :]:
            dx = right[0] - left[0]
            dy = right[1] - left[1]
            pair_span = math.hypot(dx, dy)
            if pair_span < config.min_span_m * 0.5:
                continue
            tangent_x = dx / pair_span
            tangent_y = dy / pair_span
            if tangent_x < 0.0:
                tangent_x = -tangent_x
                tangent_y = -tangent_y
            heading = math.atan2(tangent_y, tangent_x)
            if abs(heading) > config.max_heading_error_rad:
                continue
            normal_x = -tangent_y
            normal_y = tangent_x
            candidate_side_distance = side_sign * (
                left[1] - tangent_y * left[0] / tangent_x
            )
            if not (
                config.min_side_distance_m
                <= candidate_side_distance
                <= config.max_side_distance_m
            ):
                continue
            inliers = [
                point
                for point in fit_points
                if abs(
                    normal_x * (point[0] - left[0])
                    + normal_y * (point[1] - left[1])
                )
                <= inlier_threshold
            ]
            if len(inliers) < config.min_points:
                continue
            projections = [
                tangent_x * point[0] + tangent_y * point[1]
                for point in inliers
            ]
            span = max(projections) - min(projections)
            # The working shelf is the nearest shelf-like parallel surface.
            # Prioritizing raw inlier count can select a longer background wall
            # after the vehicle passes the end of the shelf.
            score = (-candidate_side_distance, len(inliers), span)
            if best is None or score > best[:3]:
                best = (score[0], score[1], score[2], inliers)
    if best is None:
        return None

    fitted = _fit_line(best[3])
    if fitted is None:
        return None
    center_x, center_y, tangent_x, tangent_y = fitted
    normal_x = -tangent_y
    normal_y = tangent_x
    trimmed = [
        point
        for point in best[3]
        if abs(
            normal_x * (point[0] - center_x)
            + normal_y * (point[1] - center_y)
        )
        <= config.max_residual_m * 1.5
    ]
    if len(trimmed) < config.min_points:
        return None
    fitted = _fit_line(trimmed)
    if fitted is None:
        return None
    center_x, center_y, tangent_x, tangent_y = fitted
    heading = math.atan2(tangent_y, tangent_x)
    if abs(heading) > config.max_heading_error_rad:
        return None
    normal_x = -tangent_y
    normal_y = tangent_x
    residuals = [
        normal_x * (point[0] - center_x)
        + normal_y * (point[1] - center_y)
        for point in trimmed
    ]
    residual_rms = math.sqrt(
        sum(residual * residual for residual in residuals) / len(residuals)
    )
    projections = [
        tangent_x * point[0] + tangent_y * point[1] for point in trimmed
    ]
    span = max(projections) - min(projections)
    if residual_rms > config.max_residual_m or span < config.min_span_m:
        return None
    if abs(tangent_x) < 1e-6:
        return None
    shelf_y_at_vehicle_center = center_y - tangent_y * center_x / tangent_x
    side_distance = side_sign * shelf_y_at_vehicle_center
    if not (
        config.min_side_distance_m
        <= side_distance
        <= config.max_side_distance_m
    ):
        return None
    return ShelfObservation(
        side_distance_m=side_distance,
        heading_error_rad=heading,
        point_count=len(trimmed),
        residual_rms_m=residual_rms,
        span_m=span,
    )


def _uniform_sample(
    points: list[tuple[float, float]],
    maximum: int,
) -> list[tuple[float, float]]:
    if len(points) <= maximum:
        return points
    return [
        points[round(index * (len(points) - 1) / (maximum - 1))]
        for index in range(maximum)
    ]


def _fit_line(
    points: list[tuple[float, float]],
) -> tuple[float, float, float, float] | None:
    if len(points) < 2:
        return None
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    covariance_xx = sum((point[0] - center_x) ** 2 for point in points)
    covariance_xy = sum(
        (point[0] - center_x) * (point[1] - center_y) for point in points
    )
    covariance_yy = sum((point[1] - center_y) ** 2 for point in points)
    if covariance_xx + covariance_yy <= 1e-12:
        return None
    heading = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    tangent_x = math.cos(heading)
    tangent_y = math.sin(heading)
    if tangent_x < 0.0:
        tangent_x = -tangent_x
        tangent_y = -tangent_y
    return center_x, center_y, tangent_x, tangent_y
