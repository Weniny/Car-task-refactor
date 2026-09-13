#!/usr/bin/env python3

"""
Simple red flag wave detector.

Used by wrist_traffic_node:
    Image
      |
      v
 HSV red detection
      |
      v
 downward movement detection
      |
      v
 trigger start
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


# ==============================
# Config
# ==============================

@dataclass
class WaveConfig:
    window: int = 8

    # downward pixel movement
    min_down_px: float = 15.0

    # speed threshold
    min_speed: float = 10.0

    cooldown: float = 2.0


# ==============================
# Wave detector
# ==============================

class WaveDetector:

    def __init__(self, config=None):

        self.config = config or WaveConfig()

        self.history = deque(
            maxlen=self.config.window
        )

        self.last_trigger = -999

        self.state = "idle"


    def update(
        self,
        cy,
        timestamp,
        flag_detected=True,
    ):

        if (
            not flag_detected
            or cy is None
        ):
            self.state = "idle"
            return False, self.state


        self.history.append(
            (
                timestamp,
                float(cy)
            )
        )

        if len(self.history) < 4:
            self.state = "tracking"
            return False, self.state


        if (
            timestamp
            -
            self.last_trigger
            <
            self.config.cooldown
        ):
            return False, self.state


        old_t, old_y = self.history[0]
        new_t, new_y = self.history[-1]


        dt = new_t - old_t

        if dt <= 0:
            return False, self.state


        dy = new_y - old_y

        speed = dy / dt


        self.state = "tracking"


        # y增加 = 向下
        if (
            dy > self.config.min_down_px
            and speed > self.config.min_speed
        ):

            self.last_trigger = timestamp

            self.history.clear()

            self.state = "triggered"

            return True, self.state


        return False, self.state



# ==============================
# Red color detector
# ==============================

class RedFlagDetector:


    def __init__(
        self,
        saturation_threshold=100,
        value_threshold=100,
        min_area_px=1000,
    ):

        self.saturation_threshold = saturation_threshold
        self.value_threshold = value_threshold
        self.min_area_px = min_area_px

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (5,5)
        )


    def detect(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )


        lower1 = np.array(
            [
                0,
                self.saturation_threshold,
                self.value_threshold
            ],
            dtype=np.uint8
        )

        upper1 = np.array(
            [
                10,
                255,
                255
            ],
            dtype=np.uint8
        )


        lower2 = np.array(
            [
                160,
                self.saturation_threshold,
                self.value_threshold
            ],
            dtype=np.uint8
        )

        upper2 = np.array(
            [
                180,
                255,
                255
            ],
            dtype=np.uint8
        )


        mask = (
            cv2.inRange(
                hsv,
                lower1,
                upper1
            )
            |
            cv2.inRange(
                hsv,
                lower2,
                upper2
            )
        )


        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel
        )


        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        if not contours:
            return None


        contour = max(
            contours,
            key=cv2.contourArea
        )


        area = cv2.contourArea(
            contour
        )


        if area < self.min_area_px:
            return None


        x,y,w,h = cv2.boundingRect(
            contour
        )


        cx = x + w/2
        cy = y + h/2


        return (
            cx,
            cy,
            x,
            y,
            w,
            h,
            mask,
            area
        )