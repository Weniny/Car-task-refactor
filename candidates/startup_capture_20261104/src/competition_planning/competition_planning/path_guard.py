"""Collision-checked path prefixes and expiring, path-specific speed limits.

All distances use the planner's existing inflated collision model. This does
not add object tracking or claim a full rectangular vehicle footprint model.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Callable, Sequence

from competition_planning.semantic_planner import PathPoint


@dataclass(frozen=True)
class PathProjection:
    segment: int
    distance_m: float
    heading_error_rad: float
    remaining_m: float


def project_path(path: Sequence[PathPoint], pose: PathPoint) -> PathProjection:
    if len(path) < 2:
        raise ValueError("path needs two points")
    lengths = [math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(path, path[1:])]
    best = None
    remaining = sum(lengths)
    for i, (a, b, length) in enumerate(zip(path, path[1:], lengths)):
        if length <= 1e-9:
            continue
        f = max(0.0, min(1.0, ((pose.x-a.x)*(b.x-a.x)+(pose.y-a.y)*(b.y-a.y))/length**2))
        distance = math.hypot(pose.x-a.x-f*(b.x-a.x), pose.y-a.y-f*(b.y-a.y))
        delta = math.atan2(math.sin(b.yaw-a.yaw), math.cos(b.yaw-a.yaw))
        heading = math.atan2(math.sin(pose.yaw-a.yaw-f*delta), math.cos(pose.yaw-a.yaw-f*delta))
        if best is None or distance < best.distance_m:
            best = PathProjection(i, distance, heading, remaining-f*length)
        remaining -= length
    if best is None:
        raise ValueError("degenerate path")
    return best


def stopping_distance(speed_mps: float, latency_s: float, deceleration_mps2: float) -> float:
    values = (speed_mps, latency_s, deceleration_mps2)
    if not all(math.isfinite(v) for v in values) or speed_mps < 0 or latency_s < 0 or deceleration_mps2 <= 0:
        raise ValueError("invalid braking budget")
    return speed_mps*latency_s + speed_mps**2/(2.0*deceleration_mps2)


def speed_for_distance(distance_m: float, latency_s: float, deceleration_mps2: float) -> float:
    stopping_distance(0.0, latency_s, deceleration_mps2)
    if not math.isfinite(distance_m):
        raise ValueError("invalid distance")
    a = deceleration_mps2
    return max(0.0, math.sqrt((a*latency_s)**2+2*a*max(0.0, distance_m))-a*latency_s)


def validated_prefix(
    path: Sequence[PathPoint], pose: PathPoint, *,
    is_clear: Callable, is_observed: Callable,
    capture_distance_m: float, capture_heading_rad: float,
) -> tuple[PathPoint, ...]:
    """Retain original geometry only to the first invalid segment, never bridge it."""
    if len(path) < 2:
        return ()
    projection = project_path(path, pose)
    if projection.distance_m > capture_distance_m or abs(projection.heading_error_rad) > capture_heading_rad:
        return ()
    tail = tuple(path[projection.segment:])
    if not is_observed(pose) or not is_observed(tail[0]) or not is_clear((pose, tail[0])):
        return ()
    kept = [tail[0]]
    for point in tail[1:]:
        if not is_observed(point) or not is_clear((kept[-1], point)):
            break
        kept.append(point)
    return tuple(kept) if len(kept) >= 2 else ()


@dataclass(frozen=True)
class PathGuard:
    path_stamp_ns: int
    valid_until_ns: int
    speed_limit_mps: float
    continuation: bool
    latency_s: float
    deceleration_mps2: float
    stop_margin_m: float
    capture_distance_m: float
    capture_heading_rad: float

    @classmethod
    def parse(cls, payload: str) -> "PathGuard":
        row = json.loads(payload)
        guard = cls(**row)
        if type(guard.path_stamp_ns) is not int or type(guard.valid_until_ns) is not int or type(guard.continuation) is not bool:
            raise ValueError("invalid guard identity")
        if guard.path_stamp_ns <= 0 or not guard.path_stamp_ns < guard.valid_until_ns <= guard.path_stamp_ns+2_000_000_000:
            raise ValueError("invalid guard expiry")
        values = (guard.speed_limit_mps, guard.latency_s, guard.deceleration_mps2,
                  guard.stop_margin_m, guard.capture_distance_m, guard.capture_heading_rad)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("invalid guard bounds")
        if guard.deceleration_mps2 <= 0 or guard.latency_s <= 0:
            raise ValueError("invalid guard braking budget")
        return guard

    def speed_limit(self, path, pose, *, now_ns: int, path_stamp_ns: int, speed_mps: float) -> float | None:
        if path_stamp_ns != self.path_stamp_ns or not self.path_stamp_ns <= now_ns < self.valid_until_ns:
            return None
        if not self.continuation:
            return self.speed_limit_mps
        projection = project_path(path, pose)
        if projection.distance_m > self.capture_distance_m or abs(projection.heading_error_rad) > self.capture_heading_rad:
            return None
        age_s = (now_ns-self.path_stamp_ns)/1e9
        usable = projection.remaining_m-self.stop_margin_m
        latency = self.latency_s+age_s
        if stopping_distance(abs(speed_mps), latency, self.deceleration_mps2) > usable:
            return None
        limit = min(self.speed_limit_mps, speed_for_distance(usable, latency, self.deceleration_mps2))
        return limit if limit > 0 else None
