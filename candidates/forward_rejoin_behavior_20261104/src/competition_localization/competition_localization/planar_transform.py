from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class PlanarTransform:
    x: float
    y: float
    yaw: float

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.yaw)

    def compose(self, other: "PlanarTransform") -> "PlanarTransform":
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return PlanarTransform(
            x=self.x + cos_yaw * other.x - sin_yaw * other.y,
            y=self.y + sin_yaw * other.x + cos_yaw * other.y,
            yaw=wrap_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "PlanarTransform":
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return PlanarTransform(
            x=-cos_yaw * self.x - sin_yaw * self.y,
            y=sin_yaw * self.x - cos_yaw * self.y,
            yaw=wrap_angle(-self.yaw),
        )
