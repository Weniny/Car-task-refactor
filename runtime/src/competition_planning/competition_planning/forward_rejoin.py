"""Select a rejoin after observed blockage without changing the search model."""
from dataclasses import dataclass
import math
from typing import Callable, Sequence


@dataclass(frozen=True)
class ForwardRejoin:
    blocked_index: int | None
    goal_index: int | None


@dataclass(frozen=True)
class ObservationWindow:
    x: float
    y: float
    yaw: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def contains(self, point, margin_m=0.0):
        dx,dy=point.x-self.x,point.y-self.y
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        x,y=c*dx+s*dy,-s*dx+c*dy
        return (self.x_min+margin_m <= x <= self.x_max-margin_m and
                self.y_min+margin_m <= y <= self.y_max-margin_m)


def path_capture_error(path: Sequence, pose) -> tuple[float, float]:
    """Distance and heading error to the closest segment of retained geometry."""
    best=(math.inf,math.inf)
    for a,b in zip(path,path[1:]):
        dx,dy=b.x-a.x,b.y-a.y
        f=max(0.,min(1.,((pose.x-a.x)*dx+(pose.y-a.y)*dy)/max(dx*dx+dy*dy,1e-12)))
        distance=math.hypot(pose.x-a.x-f*dx,pose.y-a.y-f*dy)
        delta=math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw))
        heading=math.atan2(math.sin(pose.yaw-a.yaw-f*delta),math.cos(pose.yaw-a.yaw-f*delta))
        if distance<best[0]:best=(distance,heading)
    return best


def select_forward_rejoin(
    reference: Sequence, *, start_index: int, nominal_index: int,
    maximum_index: int, clear_run_m: float, is_clear: Callable,
    is_goal_allowed: Callable | None = None,
) -> ForwardRejoin:
    """Prefer the furthest valid goal with a clear run after a blocked prefix.

    Only blockage in the nominal horizon activates detouring. The callback
    includes map bounds; outside-map points cannot be used as free goals.
    No shorter goal preceding the obstruction is accepted.
    """
    if not 0 <= start_index < nominal_index <= maximum_index < len(reference):
        raise ValueError("invalid forward reference interval")
    if not math.isfinite(clear_run_m) or clear_run_m <= 0:
        raise ValueError("clear run must be finite and positive")
    first = next((i for i in range(start_index + 1, nominal_index + 1)
                  if not is_clear(reference[i])), None)
    if first is None:
        return ForwardRejoin(None, nominal_index)
    run = 0.0
    goal = None
    for i in range(first + 1, maximum_index + 1):
        if not is_clear(reference[i]) or (is_goal_allowed is not None and not is_goal_allowed(reference[i])):
            run = 0.0
            continue
        # Do not count the segment leaving an occupied point as free run.
        if is_clear(reference[i - 1]):
            run += math.hypot(reference[i].x-reference[i-1].x,
                              reference[i].y-reference[i-1].y)
        if run + 1e-9 >= clear_run_m:
            goal = i
    return ForwardRejoin(first, goal)
