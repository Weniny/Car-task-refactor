"""Task-stop checkpoints resolved independently from motion segmentation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from competition_control.mppi_controller import ControlTrajectory


@dataclass(frozen=True)
class MissionCheckpoint:
    ref_id: str
    x: float
    y: float
    yaw: float
    trajectory_index: int = 0


def fast_stop_checkpoint_refs_from_route(
    route: dict[str, Any],
    checkpoint_refs: set[str],
) -> frozenset[str]:
    """Return checkpoints that brake on the trajectory without dock crawling."""

    raw_refs = route.get("fast_stop_checkpoints", [])
    if not isinstance(raw_refs, list):
        raise ValueError("fast_stop_checkpoints must be a list")
    refs: list[str] = []
    for raw_ref in raw_refs:
        if not isinstance(raw_ref, str) or not raw_ref:
            raise ValueError(
                "fast_stop_checkpoints must contain non-empty strings"
            )
        if raw_ref not in checkpoint_refs:
            raise ValueError(
                f"fast-stop ref is not a mission checkpoint: {raw_ref}"
            )
        if raw_ref not in refs:
            refs.append(raw_ref)
    return frozenset(refs)


def next_checkpoint_index(
    checkpoints: tuple[MissionCheckpoint, ...],
    current_index: int,
    checkpoint_ref: str,
) -> int | None:
    """Find the next future occurrence of a possibly repeated checkpoint."""
    return next(
        (
            index
            for index in range(current_index + 1, len(checkpoints))
            if checkpoints[index].ref_id == checkpoint_ref
        ),
        None,
    )


def mission_checkpoints_from_route(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    trajectory: ControlTrajectory,
) -> tuple[MissionCheckpoint, ...]:
    """Resolve ordered task stops while verifying that the path visits them."""

    raw_refs = route.get("mission_checkpoints", [])
    if raw_refs is None:
        return ()
    if not isinstance(raw_refs, list):
        raise ValueError("mission_checkpoints must be a list")
    points = semantic_map.get("points", {})
    if not isinstance(points, dict):
        raise ValueError("semantic map points must be a mapping")

    checkpoints: list[MissionCheckpoint] = []
    trajectory_index = 0
    for raw_ref in raw_refs:
        if not isinstance(raw_ref, str) or not raw_ref:
            raise ValueError("mission checkpoint refs must be non-empty strings")
        matching_index = next(
            (
                index
                for index in range(trajectory_index, len(trajectory.points))
                if trajectory.points[index].ref_id == raw_ref
            ),
            None,
        )
        if matching_index is None:
            raise ValueError(
                f"continuous trajectory does not visit mission checkpoint {raw_ref}"
            )
        raw_point = points.get(raw_ref)
        if not isinstance(raw_point, dict):
            raise ValueError(f"semantic map has no point for checkpoint {raw_ref}")
        try:
            values = (
                float(raw_point["x"]),
                float(raw_point["y"]),
                float(raw_point.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid semantic checkpoint {raw_ref}") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"semantic checkpoint {raw_ref} is non-finite")
        checkpoints.append(
            MissionCheckpoint(
                ref_id=raw_ref,
                x=values[0],
                y=values[1],
                yaw=values[2],
                trajectory_index=matching_index,
            )
        )
        trajectory_index = matching_index + 1
    return tuple(checkpoints)
