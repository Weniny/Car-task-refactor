"""Non-stopping semantic events attached to upcoming checkpoints."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence


@dataclass(frozen=True)
class MissionMarker:
    ref_id: str
    before_checkpoint_ref: str
    trigger_distance_m: float
    checkpoint_occurrence: int | None = None


def mission_markers_from_route(
    route: dict[str, Any],
    *,
    checkpoint_refs: Sequence[str],
) -> tuple[MissionMarker, ...]:
    raw_markers = route.get("mission_markers", [])
    if raw_markers is None:
        return ()
    if not isinstance(raw_markers, list):
        raise ValueError("mission_markers must be a list")
    known_checkpoints = set(checkpoint_refs)
    markers: list[MissionMarker] = []
    seen_refs: set[str] = set()
    for raw_marker in raw_markers:
        if not isinstance(raw_marker, dict):
            raise ValueError("mission marker entries must be mappings")
        ref_id = str(raw_marker.get("id", "")).strip()
        checkpoint_ref = str(
            raw_marker.get("before_checkpoint_ref", "")
        ).strip()
        try:
            trigger_distance_m = float(raw_marker["trigger_distance_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("mission marker trigger distance is invalid") from exc
        if not ref_id or not checkpoint_ref:
            raise ValueError("mission marker refs must be non-empty")
        if ref_id in seen_refs:
            raise ValueError(f"duplicate mission marker {ref_id}")
        if checkpoint_ref not in known_checkpoints:
            raise ValueError(
                f"mission marker {ref_id} references unknown checkpoint "
                f"{checkpoint_ref}"
            )
        if not math.isfinite(trigger_distance_m) or trigger_distance_m <= 0.0:
            raise ValueError("mission marker trigger distance must be positive")
        occurrence_value = raw_marker.get("checkpoint_occurrence")
        checkpoint_occurrence = None
        if occurrence_value is not None:
            try:
                checkpoint_occurrence = int(occurrence_value)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "mission marker checkpoint occurrence must be an integer"
                ) from exc
            if checkpoint_occurrence <= 0:
                raise ValueError(
                    "mission marker checkpoint occurrence must be positive"
                )
        markers.append(
            MissionMarker(
                ref_id=ref_id,
                before_checkpoint_ref=checkpoint_ref,
                trigger_distance_m=trigger_distance_m,
                checkpoint_occurrence=checkpoint_occurrence,
            )
        )
        seen_refs.add(ref_id)
    return tuple(markers)


class MissionMarkerTracker:
    """Emit each route marker once without affecting motion control."""

    def __init__(self, markers: Sequence[MissionMarker]) -> None:
        self._markers = tuple(markers)
        self.reset()

    def reset(self) -> None:
        self._emitted: set[str] = set()

    def update(
        self,
        *,
        active_checkpoint_ref: str,
        distance_to_checkpoint_m: float,
        active_checkpoint_occurrence: int | None = None,
    ) -> tuple[str, ...]:
        if not math.isfinite(distance_to_checkpoint_m):
            return ()
        emitted: list[str] = []
        for marker in self._markers:
            if (
                marker.ref_id not in self._emitted
                and marker.before_checkpoint_ref == active_checkpoint_ref
                and (
                    marker.checkpoint_occurrence is None
                    or marker.checkpoint_occurrence
                    == active_checkpoint_occurrence
                )
                and distance_to_checkpoint_m <= marker.trigger_distance_m
            ):
                self._emitted.add(marker.ref_id)
                emitted.append(marker.ref_id)
        return tuple(emitted)
