"""Best-effort safety output during ROS process shutdown."""

from __future__ import annotations

from collections.abc import Callable


def publish_shutdown_zero_if_ready(
    *,
    context_is_valid: Callable[[], bool],
    publish_zero: Callable[[], None],
) -> bool:
    """Publish a final zero only while the ROS context still accepts writes."""

    if not context_is_valid():
        return False
    publish_zero()
    return True
