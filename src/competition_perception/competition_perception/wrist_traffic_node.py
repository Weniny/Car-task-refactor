#!/usr/bin/env python3
"""Run lightweight flag, traffic-rule, and visualization processing."""

from __future__ import annotations

import json
import math

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from competition_perception.red_flag import RedFlagColorDetector
from competition_perception.traffic_rules import (
    FlagWaveDetector,
    TrafficRuleController,
    WaveConfig,
    filter_light_detection,
)


def _event_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class WristTrafficPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("wrist_traffic_perception")
        self._camera_timeout_s = float(
            self.declare_parameter("camera_timeout_s", 1.0).value
        )
        self._last_frame_s: float | None = None
        self._last_light_confidence = 0.0
        self._last_light_observation: str | None = None
        self._last_light_bbox: tuple[int, int, int, int] | None = None
        self._traffic_active = False
        self._traffic_stop_enabled = False
        self._max_light_distance_m = float(
            self.declare_parameter("max_light_distance_m", 10.0).value
        )
        if (
            not math.isfinite(self._max_light_distance_m)
            or self._max_light_distance_m <= 0.0
        ):
            raise ValueError("max_light_distance_m must be finite and positive")
        self._bridge = CvBridge()

        self._flag_color = RedFlagColorDetector(
            saturation_threshold=int(
                self.declare_parameter("flag_saturation_threshold", 100).value
            ),
            value_threshold=int(
                self.declare_parameter("flag_value_threshold", 100).value
            ),
            min_area_px=float(self.declare_parameter("flag_min_area_px", 800.0).value),
            border_margin_px=int(
                self.declare_parameter("flag_border_margin_px", 8).value
            ),
        )
        self._flag_wave = FlagWaveDetector(
            WaveConfig(
                trajectory_window=int(
                    self.declare_parameter("flag_trajectory_window", 20).value
                ),
                min_displacement_px=float(
                    self.declare_parameter("flag_min_motion_px", 30.0).value
                ),
                cooldown_s=float(
                    self.declare_parameter("flag_cooldown_s", 2.0).value
                ),
                max_lost_frames=int(
                    self.declare_parameter("flag_max_lost_frames", 8).value
                ),
            )
        )
        self._rules = TrafficRuleController(
            confirm_frames=int(
                self.declare_parameter("light_confirm_frames", 3).value
            )
        )

        stop_topic = str(
            self.declare_parameter(
                "stop_request_topic", "/perception/traffic_stop_request"
            ).value
        )
        flag_topic = str(
            self.declare_parameter(
                "flag_event_topic", "/perception/flag_wave_detected"
            ).value
        )
        detection_topic = str(
            self.declare_parameter(
                "light_detection_topic", "/perception/traffic_light_detection"
            ).value
        )
        active_topic = str(
            self.declare_parameter(
                "light_active_topic", "/perception/traffic_light_active"
            ).value
        )
        stop_enable_topic = str(
            self.declare_parameter(
                "traffic_stop_enable_topic", "/perception/traffic_stop_enable"
            ).value
        )
        reset_topic = str(
            self.declare_parameter(
                "flag_reset_topic", "/perception/flag_reset"
            ).value
        )
        self._stop_publisher = self.create_publisher(Bool, stop_topic, 10)
        self._flag_publisher = self.create_publisher(Bool, flag_topic, _event_qos())
        self._light_publisher = self.create_publisher(
            String, "/perception/traffic_light_state", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/perception/traffic_rules_status", 10
        )
        debug_image_topic = str(
            self.declare_parameter(
                "debug_image_topic", "/perception/wrist_traffic_annotated"
            ).value
        )
        self._debug_publisher = self.create_publisher(
            Image, debug_image_topic, qos_profile_sensor_data
        )
        image_topic = str(
            self.declare_parameter(
                "image_topic", "/left_wrist_camera/camera/color/image_raw"
            ).value
        )
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            detection_topic,
            self._light_detection_callback,
            10,
        )
        self.create_subscription(
            Bool,
            active_topic,
            self._light_active_callback,
            _event_qos(),
        )
        self.create_subscription(
            Bool,
            stop_enable_topic,
            self._traffic_stop_enable_callback,
            _event_qos(),
        )
        self.create_subscription(Bool, reset_topic, self._flag_reset_callback, 10)
        self.create_timer(0.10, self._publish_state)
        self._flag_publisher.publish(Bool(data=False))
        self.get_logger().info(
            f"Wrist flag/rules/view ready; image={image_topic}; "
            f"light_detection={detection_topic}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _image_callback(self, message: Image) -> None:
        now_s = self._now_s()
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        flag = None
        if not self._rules.decision.started:
            flag = self._flag_color.detect(frame)
            triggered = self._flag_wave.update(
                centroid_x=None if flag is None else float(flag.centroid_x),
                centroid_y=None if flag is None else float(flag.centroid_y),
                timestamp_s=now_s,
            )
            if triggered:
                self._rules.observe_flag_wave()
                self._flag_publisher.publish(Bool(data=True))
                self.get_logger().info(
                    "Start red motion confirmed; flag detector stopped"
                )

        self._publish_debug_image(frame, message, flag)
        self._last_frame_s = self._now_s()

    def _light_active_callback(self, message: Bool) -> None:
        active = bool(message.data)
        if active == self._traffic_active:
            return
        self._traffic_active = active
        self._rules.set_traffic_active(active)
        self._last_light_observation = None
        self._last_light_confidence = 0.0
        self._last_light_bbox = None

    def _traffic_stop_enable_callback(self, message: Bool) -> None:
        self._traffic_stop_enabled = bool(message.data)
        self._rules.set_traffic_stop_enabled(self._traffic_stop_enabled)

    def _flag_reset_callback(self, message: Bool) -> None:
        if not message.data:
            return
        self._flag_wave.reset()
        self._rules.reset()
        self._traffic_active = False
        self._traffic_stop_enabled = False
        self._last_light_observation = None
        self._last_light_confidence = 0.0
        self._last_light_bbox = None
        self._flag_publisher.publish(Bool(data=False))
        self.get_logger().info("Flag test reset; detector active")

    def _light_detection_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warning("Ignored malformed traffic-light detection")
            return
        if not self._traffic_active:
            return
        observation, _depth_m = filter_light_detection(
            payload,
            max_distance_m=self._max_light_distance_m,
        )
        bbox = payload.get("bbox")
        if observation is not None and (
            isinstance(bbox, list)
            and len(bbox) == 4
            and all(isinstance(value, (int, float)) for value in bbox)
        ):
            self._last_light_bbox = tuple(int(value) for value in bbox)
        else:
            self._last_light_bbox = None
        self._last_light_observation = observation
        self._last_light_confidence = float(payload.get("confidence", 0.0))
        self._rules.observe_light(observation)

    def _publish_debug_image(self, frame, source_message: Image, flag) -> None:
        if self._debug_publisher.get_subscription_count() == 0:
            return

        annotated = frame.copy()
        if flag is not None:
            cv2.rectangle(
                annotated,
                (flag.x, flag.y),
                (flag.x + flag.width, flag.y + flag.height),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                annotated,
                (flag.centroid_x, flag.centroid_y),
                4,
                (0, 255, 0),
                -1,
            )
            cv2.putText(
                annotated,
                "RED FLAG COLOR",
                (flag.x, max(20, flag.y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        if self._last_light_bbox is not None:
            x1, y1, x2, y2 = self._last_light_bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 180, 0), 2)
            label = (
                f"LIGHT {str(self._last_light_observation).upper()} "
                f"{self._last_light_confidence:.2f}"
            )
            cv2.putText(
                annotated,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 180, 0),
                2,
            )

        decision = self._rules.decision
        started_text = "CONFIRMED" if decision.started else "WAITING"
        flag_text = "DONE" if decision.started else self._flag_wave.state.value.upper()
        light_text = (
            decision.light.value.upper() if self._traffic_active else "DISABLED"
        )
        action_text = "STOP" if decision.stop_required else "GO"
        action_color = (0, 0, 255) if decision.stop_required else (0, 255, 0)
        cv2.rectangle(
            annotated,
            (0, 0),
            (annotated.shape[1], 104),
            (20, 20, 20),
            -1,
        )
        lines = (
            f"FLAG: {flag_text}",
            f"START: {started_text}",
            f"LIGHT: {light_text}  CONF: {self._last_light_confidence:.2f}",
        )
        for index, text in enumerate(lines):
            cv2.putText(
                annotated,
                text,
                (12, 24 + index * 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                2,
            )
        cv2.putText(
            annotated,
            action_text,
            (annotated.shape[1] - 100, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            action_color,
            3,
        )

        debug_message = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_message.header = source_message.header
        self._debug_publisher.publish(debug_message)

    def _publish_state(self) -> None:
        now_s = self._now_s()
        camera_stale = bool(
            self._last_frame_s is None
            or now_s - self._last_frame_s > self._camera_timeout_s
        )
        decision = self._rules.decision
        stop_required = bool(camera_stale or decision.stop_required)
        reason = "wrist_camera_stale" if camera_stale else decision.reason
        self._stop_publisher.publish(Bool(data=stop_required))
        self._light_publisher.publish(String(data=decision.light.value))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "started": decision.started,
                        "flag_active": not decision.started,
                        "traffic_active": self._traffic_active,
                        "traffic_stop_enabled": self._traffic_stop_enabled,
                        "light": decision.light.value,
                        "light_confidence": round(self._last_light_confidence, 4),
                        "flag_state": self._flag_wave.state.value,
                        "camera_stale": camera_stale,
                        "stop_required": stop_required,
                        "reason": reason,
                    },
                    separators=(",", ":"),
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = WristTrafficPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop_publisher.publish(Bool(data=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
