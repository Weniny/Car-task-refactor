"""HSV circular-light detector adapted from the field reference script."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class LightSpotConfig:
    brightness_threshold: int = 110
    min_area_px: float = 30.0
    max_area_px: float = 20000.0
    min_circularity: float = 0.70
    morphology_kernel_size: int = 9
    opening_kernel_size: int = 3
    roi_max_x_ratio: float = 1.0
    roi_max_y_ratio: float = 1.0

    def __post_init__(self) -> None:
        if not 0 <= self.brightness_threshold <= 255:
            raise ValueError("brightness_threshold must be in [0, 255]")
        if self.min_area_px < 0.0 or self.max_area_px <= self.min_area_px:
            raise ValueError("spot area limits are invalid")
        if not 0.0 <= self.min_circularity <= 1.0:
            raise ValueError("min_circularity must be in [0, 1]")
        if not math.isfinite(self.roi_max_x_ratio) or not (
            0.0 < self.roi_max_x_ratio <= 1.0
        ):
            raise ValueError("roi_max_x_ratio must be in (0, 1]")
        if not math.isfinite(self.roi_max_y_ratio) or not (
            0.0 < self.roi_max_y_ratio <= 1.0
        ):
            raise ValueError("roi_max_y_ratio must be in (0, 1]")
        if (
            self.morphology_kernel_size <= 0
            or self.morphology_kernel_size % 2 == 0
        ):
            raise ValueError("morphology_kernel_size must be a positive odd integer")
        if self.opening_kernel_size <= 0 or self.opening_kernel_size % 2 == 0:
            raise ValueError("opening_kernel_size must be a positive odd integer")


@dataclass(frozen=True)
class LightSpotDetection:
    color_name: str
    center_x: int
    center_y: int
    radius_px: int
    bbox: tuple[int, int, int, int]
    area_px: float
    circularity: float
    center_brightness: float
    confidence: float
    depth_m: float | None


# These ranges intentionally match the supplied field script. In particular,
# the wider/lower-saturation green range retains overexposed green lamps.
_COLOR_RANGES = {
    "red": (
        ((0, 150, 100), (10, 255, 255)),
        ((160, 150, 100), (180, 255, 255)),
    ),
    "yellow": (((18, 160, 120), (35, 255, 255)),),
    "green": (((35, 60, 100), (90, 255, 255)),),
}


class LightSpotDetector:
    """Select the nearest circular colored spot, or the largest without depth."""

    def __init__(self, config: LightSpotConfig | None = None) -> None:
        self.config = config or LightSpotConfig()
        size = self.config.morphology_kernel_size
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        opening_size = self.config.opening_kernel_size
        self._opening_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (opening_size, opening_size),
        )

    def detect(
        self,
        frame,
        depth_image=None,
        *,
        depth_scale_m: float = 0.001,
    ) -> LightSpotDetection | None:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a BGR image")
        if depth_scale_m <= 0.0:
            raise ValueError("depth_scale_m must be positive")
        if depth_image is not None and (
            depth_image.ndim != 2 or depth_image.shape != frame.shape[:2]
        ):
            raise ValueError("depth_image must be aligned with the BGR image")

        roi_width = max(
            1,
            min(
                frame.shape[1],
                int(frame.shape[1] * self.config.roi_max_x_ratio),
            ),
        )
        roi_height = max(
            1,
            min(
                frame.shape[0],
                int(frame.shape[0] * self.config.roi_max_y_ratio),
            ),
        )
        frame = frame[:roi_height, :roi_width]
        if depth_image is not None:
            depth_image = depth_image[:roi_height, :roi_width]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        candidates: list[LightSpotDetection] = []

        for color_name, ranges in _COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                dynamic_lower = np.array(lower, dtype=np.uint8)
                dynamic_lower[2] = self.config.brightness_threshold
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(
                        hsv,
                        dynamic_lower,
                        np.array(upper, dtype=np.uint8),
                    ),
                )
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self._kernel, iterations=2
            )
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self._opening_kernel, iterations=1
            )
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                candidate = self._candidate_from_contour(
                    contour,
                    color_name=color_name,
                    v_channel=v_channel,
                    depth_image=depth_image,
                    depth_scale_m=depth_scale_m,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                )
                if candidate is not None:
                    candidates.append(candidate)

        if not candidates:
            return None
        candidates_with_depth = [
            candidate
            for candidate in candidates
            if candidate.depth_m is not None
        ]
        if candidates_with_depth:
            return min(
                candidates_with_depth,
                key=lambda item: (float(item.depth_m), -item.area_px),
            )
        return max(candidates, key=lambda item: item.area_px)

    def _candidate_from_contour(
        self,
        contour,
        *,
        color_name: str,
        v_channel,
        depth_image,
        depth_scale_m: float,
        image_width: int,
        image_height: int,
    ) -> LightSpotDetection | None:
        # Fill the colored ring of an overexposed lamp before evaluating its
        # geometry, as in the updated field detector.
        hull = cv2.convexHull(contour)
        area = float(cv2.contourArea(hull))
        if area < self.config.min_area_px or area > self.config.max_area_px:
            return None
        perimeter = float(cv2.arcLength(hull, True))
        if perimeter <= 0.0:
            return None
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < self.config.min_circularity:
            return None

        (center_x, center_y), radius = cv2.minEnclosingCircle(hull)
        cx, cy, radius_px = int(center_x), int(center_y), int(radius)
        brightness = _center_brightness(v_channel, cx, cy, radius_px)
        confidence = min(1.0, (brightness / 255.0) * min(1.0, circularity))
        return LightSpotDetection(
            color_name=color_name,
            center_x=cx,
            center_y=cy,
            radius_px=radius_px,
            bbox=(
                max(0, cx - radius_px),
                max(0, cy - radius_px),
                min(image_width - 1, cx + radius_px),
                min(image_height - 1, cy + radius_px),
            ),
            area_px=area,
            circularity=circularity,
            center_brightness=brightness,
            confidence=confidence,
            depth_m=_depth_at(
                depth_image,
                cx,
                cy,
                radius_px,
                depth_scale_m=depth_scale_m,
            ),
        )


def _center_brightness(v_channel, cx: int, cy: int, radius: int) -> float:
    sample_radius = max(3, radius // 3)
    y1, y2 = max(0, cy - sample_radius), min(
        v_channel.shape[0], cy + sample_radius
    )
    x1, x2 = max(0, cx - sample_radius), min(
        v_channel.shape[1], cx + sample_radius
    )
    region = v_channel[y1:y2, x1:x2]
    return 0.0 if region.size == 0 else float(region.mean())


def _depth_at(
    depth_image,
    cx: int,
    cy: int,
    radius: int,
    *,
    depth_scale_m: float,
) -> float | None:
    if depth_image is None:
        return None
    sample_radius = max(2, radius // 3)
    y1, y2 = max(0, cy - sample_radius), min(
        depth_image.shape[0], cy + sample_radius
    )
    x1, x2 = max(0, cx - sample_radius), min(
        depth_image.shape[1], cx + sample_radius
    )
    region = np.asarray(depth_image[y1:y2, x1:x2], dtype=np.float64)
    valid = region[np.isfinite(region) & (region > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale_m
