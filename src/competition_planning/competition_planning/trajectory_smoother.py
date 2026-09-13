"""Path smoothing utilities for global trajectory handoff."""

from __future__ import annotations

import math
from typing import Sequence

from competition_planning.semantic_planner import PathPoint


class CubicBezierSmoother:
    """Smooth semantic anchor paths with piecewise cubic Bezier curves.

    Points with ``ref_id`` are treated as hard anchors. The curve passes through
    every anchor exactly, uses each anchor yaw as the local tangent direction
    when it is compatible with the route direction, and samples intermediate
    points at a stable spacing.
    """

    def __init__(self, *, sample_spacing_m: float, tangent_scale: float) -> None:
        self._sample_spacing_m = max(0.05, sample_spacing_m)
        self._tangent_scale = max(0.0, tangent_scale)

    def smooth(self, path: Sequence[PathPoint]) -> tuple[PathPoint, ...]:
        anchors = [point for point in path if point.ref_id]
        if len(anchors) < 3:
            return tuple(path)

        tangents = _anchor_tangents(anchors, self._tangent_scale)
        smoothed: list[PathPoint] = []
        for index, (start, end) in enumerate(zip(anchors, anchors[1:])):
            start_tangent = tangents[index]
            end_tangent = tangents[index + 1]
            controls = (
                (start.x, start.y),
                (start.x + start_tangent[0] / 3.0, start.y + start_tangent[1] / 3.0),
                (end.x - end_tangent[0] / 3.0, end.y - end_tangent[1] / 3.0),
                (end.x, end.y),
            )
            samples = max(
                1,
                math.ceil(_control_polygon_length(controls) / self._sample_spacing_m),
            )
            for sample_index in range(samples + 1):
                if smoothed and sample_index == 0:
                    continue
                t = sample_index / samples
                x, y = _bezier_xy(controls, t)
                dx, dy = _bezier_derivative_xy(controls, t)
                yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-12 else start.yaw
                ref_id = (
                    start.ref_id
                    if sample_index == 0
                    else end.ref_id
                    if sample_index == samples
                    else None
                )
                smoothed.append(PathPoint(x=x, y=y, yaw=yaw, ref_id=ref_id))
        return tuple(smoothed)


def _anchor_tangents(
    anchors: Sequence[PathPoint],
    tangent_scale: float,
) -> list[tuple[float, float]]:
    tangents: list[tuple[float, float]] = []
    for index, point in enumerate(anchors):
        if index == 0:
            next_point = anchors[index + 1]
            route_tangent = (next_point.x - point.x, next_point.y - point.y)
            local_spacing_m = math.hypot(*route_tangent)
        elif index == len(anchors) - 1:
            previous = anchors[index - 1]
            route_tangent = (point.x - previous.x, point.y - previous.y)
            local_spacing_m = math.hypot(*route_tangent)
        else:
            previous = anchors[index - 1]
            next_point = anchors[index + 1]
            route_tangent = (
                (next_point.x - previous.x) * 0.5,
                (next_point.y - previous.y) * 0.5,
            )
            local_spacing_m = min(
                math.hypot(point.x - previous.x, point.y - previous.y),
                math.hypot(next_point.x - point.x, next_point.y - point.y),
            )

        tangent = _compatible_yaw_tangent(point, route_tangent, tangent_scale)
        tangent_length_m = math.hypot(*tangent)
        max_length_m = local_spacing_m * tangent_scale
        if tangent_length_m > max_length_m > 1e-9:
            ratio = max_length_m / tangent_length_m
            tangent = (tangent[0] * ratio, tangent[1] * ratio)
        tangents.append(tangent)
    return tangents


def _compatible_yaw_tangent(
    point: PathPoint,
    route_tangent: tuple[float, float],
    tangent_scale: float,
) -> tuple[float, float]:
    length = math.hypot(route_tangent[0], route_tangent[1])
    if length <= 1e-9:
        return (0.0, 0.0)

    route_yaw = math.atan2(route_tangent[1], route_tangent[0])
    if _angle_delta(point.yaw, route_yaw) <= math.radians(120.0):
        return (
            math.cos(point.yaw) * length * tangent_scale,
            math.sin(point.yaw) * length * tangent_scale,
        )
    return (route_tangent[0] * tangent_scale, route_tangent[1] * tangent_scale)


def _angle_delta(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + math.pi) % (2.0 * math.pi) - math.pi)


def _control_polygon_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


def _bezier_xy(
    controls: Sequence[tuple[float, float]],
    t: float,
) -> tuple[float, float]:
    p0, p1, p2, p3 = controls
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def _bezier_derivative_xy(
    controls: Sequence[tuple[float, float]],
    t: float,
) -> tuple[float, float]:
    p0, p1, p2, p3 = controls
    u = 1.0 - t
    return (
        3.0 * u * u * (p1[0] - p0[0])
        + 6.0 * u * t * (p2[0] - p1[0])
        + 3.0 * t * t * (p3[0] - p2[0]),
        3.0 * u * u * (p1[1] - p0[1])
        + 6.0 * u * t * (p2[1] - p1[1])
        + 3.0 * t * t * (p3[1] - p2[1]),
    )


class CircularArcSmoother:
    """Sample exact circular arcs encoded as entry/turn/exit anchor triples."""

    def __init__(self, *, sample_spacing_m: float) -> None:
        self._sample_spacing_m = max(0.05, sample_spacing_m)

    def smooth(self, path: Sequence[PathPoint]) -> tuple[PathPoint, ...]:
        anchors = [point for point in path if point.ref_id]
        if len(anchors) < 4:
            return tuple(path)

        output = [anchors[0]]
        index = 0
        while index < len(anchors) - 1:
            if index + 3 < len(anchors) and self._is_arc(
                anchors[index + 1], anchors[index + 2], anchors[index + 3]
            ):
                self._append_line(output, anchors[index], anchors[index + 1])
                self._append_arc(
                    output,
                    anchors[index + 1],
                    anchors[index + 2],
                    anchors[index + 3],
                )
                index += 3
            else:
                self._append_line(output, anchors[index], anchors[index + 1])
                index += 1
        return tuple(output)

    @staticmethod
    def _is_arc(entry: PathPoint, turn: PathPoint, exit: PathPoint) -> bool:
        if not entry.ref_id or not entry.ref_id.endswith("_entry"):
            return False
        prefix = entry.ref_id[:-len("_entry")]
        return turn.ref_id == f"{prefix}_turn" and exit.ref_id == f"{prefix}_exit"

    def _append_line(self, output, start: PathPoint, end: PathPoint) -> None:
        dx, dy = end.x - start.x, end.y - start.y
        length_m = math.hypot(dx, dy)
        if length_m <= 1e-9:
            return
        yaw = math.atan2(dy, dx)
        count = max(1, math.ceil(length_m / self._sample_spacing_m))
        for index in range(1, count + 1):
            ratio = index / count
            output.append(PathPoint(
                x=start.x + dx * ratio,
                y=start.y + dy * ratio,
                yaw=yaw,
                ref_id=end.ref_id if index == count else None,
            ))

    def _append_arc(self, output, start: PathPoint, mid: PathPoint, end: PathPoint) -> None:
        center = self._circumcenter(start, mid, end)
        if center is None:
            self._append_line(output, start, mid)
            self._append_line(output, mid, end)
            return

        cx, cy = center
        start_angle = math.atan2(start.y - cy, start.x - cx)
        mid_angle = math.atan2(mid.y - cy, mid.x - cx)
        end_angle = math.atan2(end.y - cy, end.x - cx)
        ccw_end = (end_angle - start_angle) % (2.0 * math.pi)
        ccw_mid = (mid_angle - start_angle) % (2.0 * math.pi)
        direction = 1.0 if ccw_mid <= ccw_end else -1.0
        radius_m = math.hypot(start.x - cx, start.y - cy)

        self._append_arc_segment(output, start, mid, cx, cy, radius_m, direction)
        self._append_arc_segment(output, mid, end, cx, cy, radius_m, direction)

    def _append_arc_segment(self, output, start, end, cx, cy, radius_m, direction) -> None:
        start_angle = math.atan2(start.y - cy, start.x - cx)
        end_angle = math.atan2(end.y - cy, end.x - cx)
        ccw = (end_angle - start_angle) % (2.0 * math.pi)
        sweep = ccw if direction > 0.0 else ccw - 2.0 * math.pi
        count = max(1, math.ceil(abs(sweep) * radius_m / self._sample_spacing_m))
        for index in range(1, count + 1):
            angle = start_angle + sweep * index / count
            output.append(PathPoint(
                x=cx + radius_m * math.cos(angle),
                y=cy + radius_m * math.sin(angle),
                yaw=angle + direction * math.pi / 2.0,
                ref_id=end.ref_id if index == count else None,
            ))

    @staticmethod
    def _circumcenter(first, second, third):
        ax, ay = first.x, first.y
        bx, by = second.x, second.y
        cx, cy = third.x, third.y
        denominator = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(denominator) <= 1e-9:
            return None
        aa, bb, cc = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
        return (
            (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by)) / denominator,
            (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax)) / denominator,
        )
