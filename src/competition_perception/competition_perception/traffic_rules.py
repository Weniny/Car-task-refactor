"""Pure state machines for the start flag and traffic-light rules."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from math import hypot
from statistics import mean
from typing import Any


class WaveState(str, Enum):
    IDLE = "idle"
    TRACKING = "tracking"
    TRIGGERED = "triggered"


@dataclass(frozen=True)
class WaveConfig:
    trajectory_window: int = 20
    min_displacement_px: float = 30.0
    cooldown_s: float = 2.0
    max_lost_frames: int = 8


class FlagWaveDetector:
    """Detect visible red-flag motion in any image direction."""

    def __init__(self, config: WaveConfig | None = None) -> None:
        self.config = config or WaveConfig()
        if self.config.trajectory_window < 6:
            raise ValueError("trajectory_window must be at least 6")
        self.state = WaveState.IDLE
        self.trajectory: deque[tuple[float, float, float]] = deque(
            maxlen=self.config.trajectory_window
        )
        self.reset()

    def reset(self) -> None:
        self.state = WaveState.IDLE
        self.trajectory.clear()
        self.last_trigger_s = float("-inf")
        self.lost_frames = 0

    def update(
        self,
        *,
        centroid_x: float | None,
        centroid_y: float | None,
        timestamp_s: float,
    ) -> bool:
        if centroid_x is None or centroid_y is None:
            self.lost_frames += 1
            if self.lost_frames > self.config.max_lost_frames:
                self._clear_tracking(WaveState.IDLE)
            return False

        self.lost_frames = 0
        self.trajectory.append(
            (timestamp_s, float(centroid_x), float(centroid_y))
        )

        if len(self.trajectory) < 6:
            if self.state == WaveState.IDLE:
                self.state = WaveState.TRACKING
            return False
        if timestamp_s - self.last_trigger_s < self.config.cooldown_s:
            return False

        samples = list(self.trajectory)
        early_x = mean(point[1] for point in samples[:3])
        early_y = mean(point[2] for point in samples[:3])
        recent_x = mean(point[1] for point in samples[-3:])
        recent_y = mean(point[2] for point in samples[-3:])
        displacement = hypot(recent_x - early_x, recent_y - early_y)
        if displacement >= self.config.min_displacement_px:
            return self._trigger(timestamp_s)
        return False

    def _trigger(self, timestamp_s: float) -> bool:
        self.state = WaveState.TRIGGERED
        self.last_trigger_s = timestamp_s
        self.trajectory.clear()
        return True

    def _clear_tracking(self, state: WaveState) -> None:
        self.state = state
        self.trajectory.clear()


class LightState(str, Enum):
    UNKNOWN = "unknown"
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    OFF = "off"


@dataclass(frozen=True)
class TrafficDecision:
    started: bool
    light: LightState
    stop_required: bool
    reason: str


class TrafficRuleController:
    """Latch the flag start, then enforce stable traffic-light observations."""

    def __init__(self, confirm_frames: int = 3) -> None:
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be positive")
        self._confirm_frames = confirm_frames
        self.reset()

    def reset(self) -> None:
        self._started = False
        self._light = LightState.UNKNOWN
        self._candidate = LightState.UNKNOWN
        self._candidate_count = 0
        self._traffic_active = False
        self._traffic_stop_enabled = False
        self._stop_required = True
        self._reason = "waiting_for_flag"

    @property
    def decision(self) -> TrafficDecision:
        return TrafficDecision(
            started=self._started,
            light=self._light,
            stop_required=self._stop_required,
            reason=self._reason,
        )

    def observe_flag_wave(self) -> TrafficDecision:
        self._started = True
        if self._traffic_stop_enabled and self._light in (
            LightState.RED,
            LightState.YELLOW,
            LightState.OFF,
        ):
            self._stop_required = True
            self._reason = f"traffic_{self._light.value}"
        elif self._traffic_stop_enabled:
            self._stop_required = True
            self._reason = "traffic_pending"
        elif self._traffic_active:
            self._stop_required = False
            self._reason = "traffic_preheating"
        else:
            self._stop_required = False
            self._reason = "flag_start"
        return self.decision

    def set_traffic_active(self, active: bool) -> TrafficDecision:
        self._traffic_active = bool(active)
        self._light = LightState.UNKNOWN
        self._candidate = LightState.UNKNOWN
        self._candidate_count = 0
        if not self._traffic_active:
            self._traffic_stop_enabled = False
        if not self._started:
            self._stop_required = True
            self._reason = "waiting_for_flag"
        elif self._traffic_stop_enabled:
            self._stop_required = True
            self._reason = "traffic_pending"
        elif self._traffic_active:
            self._stop_required = False
            self._reason = "traffic_preheating"
        else:
            self._stop_required = False
            self._reason = "traffic_inactive"
        return self.decision

    def set_traffic_stop_enabled(self, enabled: bool) -> TrafficDecision:
        self._traffic_stop_enabled = bool(enabled)
        if not self._started:
            self._stop_required = True
            self._reason = "waiting_for_flag"
        elif not self._traffic_active:
            self._stop_required = False
            self._reason = "traffic_inactive"
        elif not self._traffic_stop_enabled:
            self._stop_required = False
            self._reason = "traffic_preheating"
        elif self._light == LightState.GREEN:
            self._stop_required = False
            self._reason = "traffic_green"
        elif self._light in (LightState.RED, LightState.YELLOW, LightState.OFF):
            self._stop_required = True
            self._reason = f"traffic_{self._light.value}"
        else:
            self._stop_required = True
            self._reason = "traffic_pending"
        return self.decision

    def observe_light(self, class_name: str | None) -> TrafficDecision:
        if not self._traffic_active:
            return self.decision
        observation = _light_state(class_name)
        if observation == LightState.UNKNOWN:
            self._candidate = LightState.UNKNOWN
            self._candidate_count = 0
            return self.decision
        if observation == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = observation
            self._candidate_count = 1
        if self._candidate_count < self._confirm_frames:
            return self.decision

        self._light = observation
        if not self._started:
            self._stop_required = True
            self._reason = "waiting_for_flag"
        elif not self._traffic_stop_enabled:
            self._stop_required = False
            self._reason = "traffic_preheating"
        elif observation == LightState.GREEN:
            self._stop_required = False
            self._reason = "traffic_green"
        else:
            self._stop_required = True
            self._reason = f"traffic_{observation.value}"
        return self.decision


def filter_light_detection(
    payload: dict[str, Any],
    *,
    max_distance_m: float,
) -> tuple[str | None, float | None]:
    """Accept recognized lights unless measured depth is out of range.

    Illuminated lamp faces commonly contain no usable RealSense depth pixels.
    The updated field detector falls back to the largest circular spot in that
    case, so missing depth must not erase an otherwise valid color result.
    """

    if not math.isfinite(max_distance_m) or max_distance_m <= 0.0:
        raise ValueError("max_distance_m must be finite and positive")
    observation = str(payload.get("class_name") or "").strip().lower()
    if observation not in {"red", "green", "yellow", "off"}:
        return None, None
    raw_depth = payload.get("depth_m")
    if raw_depth is None:
        return observation, None
    try:
        depth_m = float(raw_depth)
    except (TypeError, ValueError):
        return None, None
    if not math.isfinite(depth_m) or depth_m <= 0.0 or depth_m > max_distance_m:
        return None, None
    return observation, depth_m


def _light_state(class_name: str | None) -> LightState:
    normalized = str(class_name or "").strip().lower()
    try:
        return LightState(normalized)
    except ValueError:
        return LightState.UNKNOWN
