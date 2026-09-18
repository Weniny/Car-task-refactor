#!/usr/bin/env python3
"""Synthetic ROS integration checks in domain 84; never starts chassis nodes."""

import math
import os
import time
import json

if os.environ.get("ROS_DOMAIN_ID") != "84":
    raise SystemExit("Run only in isolated domain: ROS_DOMAIN_ID=84 python3 scripts/test_path_aware_ros.py")

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from ranger_msgs.msg import MotionState, SystemState
from tf2_ros import TransformBroadcaster

from competition_safety.proximity_stop_node import ProximityStopNode
from competition_safety.safety_node import SafetyNode


def main():
    rclpy.init(args=[
        "--ros-args", "-p", "path_aware_stop_enabled:=true",
        "-p", "stop_distance_m:=1.8", "-p", "fusion_frame_count:=1",
        "-p", "require_avoidance_speed_limit:=true", "-p", "max_speed_mps:=0.5",
    ])
    proximity = None
    harness = None
    safety = None
    executor = SingleThreadedExecutor()
    try:
        proximity = ProximityStopNode()
        safety = SafetyNode()
        harness = Node("path_aware_synthetic_test")
        executor.add_node(proximity)
        executor.add_node(safety)
        executor.add_node(harness)
        broadcaster = TransformBroadcaster(harness)
        scan_pub = harness.create_publisher(LaserScan, "/scan", 10)
        odom_pub = harness.create_publisher(Odometry, "/odom", 10)
        path_pub = harness.create_publisher(Path, "/planning/local_trajectory", 10)
        statuses = []
        safe_commands = []
        safety_events = []
        harness.create_subscription(String, "/avoidance/proximity_status", lambda m: statuses.append(json.loads(m.data)), 10)
        harness.create_subscription(Twist, "/cmd_vel_safe", lambda m: safe_commands.append(m), 10)
        harness.create_subscription(String, "/safety/event", lambda m: safety_events.append(json.loads(m.data)), 10)
        command_pub = harness.create_publisher(TwistStamped, "/control/body_cmd", 10)
        error_pub = harness.create_publisher(Vector3Stamped, "/control/tracking_error", 10)
        valid_pub = harness.create_publisher(Bool, "/control/state_valid", 10)
        status_pub = harness.create_publisher(String, "/control/status", 10)
        system_pub = harness.create_publisher(SystemState, "/system_state", 10)
        motion_pub = harness.create_publisher(MotionState, "/motion_state", 10)
        stop_pub = harness.create_publisher(Bool, "/avoidance/stop_request", 10)
        around = [
            (0, 0), (0.2, 0), (0.4, -0.1), (0.6, -0.3),
            (0.7, -0.45), (0.8, -0.6), (1.0, -0.8),
            (1.2, -0.8), (1.4, -0.8), (1.6, -0.8), (1.8, -0.8),
        ]
        straight = [(0.2 * i, 0) for i in range(16)]

        def drive_synthetic(distance, path, speed=0.25, bad_stamp=False, duration=1.5, publish_scan=True):
            statuses.clear()
            safe_commands.clear()
            safety_events.clear()
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if harness.count_publishers("/cmd_vel"):
                    raise RuntimeError("Unexpected chassis command publisher in isolated domain")
                now = harness.get_clock().now()
                transform = TransformStamped()
                transform.header.stamp = now.to_msg()
                transform.header.frame_id = "map"
                transform.child_frame_id = "body"
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "body"
                odom.twist.twist.linear.x = speed
                odom_pub.publish(odom)
                command = TwistStamped()
                command.header.stamp = now.to_msg()
                command.twist.linear.x = 0.5
                command.twist.angular.z = 0.2
                command_pub.publish(command)
                error = Vector3Stamped()
                error.header.stamp = now.to_msg()
                error_pub.publish(error)
                valid_pub.publish(Bool(data=True))
                status_pub.publish(String(data='{"status":"TRACKING"}'))
                system = SystemState()
                system.control_mode = SystemState.CONTROL_MODE_CAN
                system_pub.publish(system)
                motion = MotionState()
                motion.motion_mode = MotionState.MOTION_MODE_DUAL_ACKERMAN
                motion_pub.publish(motion)
                if path is not None:
                    message = Path()
                    message.header.stamp = now.to_msg()
                    message.header.frame_id = "map"
                    for x, y in path:
                        pose = PoseStamped()
                        pose.header = message.header
                        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                        pose.pose.orientation.w = 1.0
                        message.poses.append(pose)
                    path_pub.publish(message)
                scan = LaserScan()
                scan.header.frame_id = "body"
                if not bad_stamp:
                    scan.header.stamp = rclpy.time.Time(nanoseconds=now.nanoseconds - 50_000_000).to_msg()
                scan.angle_min, scan.angle_max = -0.01, 0.01
                scan.angle_increment = 0.01
                scan.range_min, scan.range_max = 0.1, 6.0
                scan.ranges = [float(distance)] * 3 if distance else [math.inf] * 3
                if publish_scan:
                    scan_pub.publish(scan)
                else:
                    stop_pub.publish(Bool(data=False))
                cycle_end = time.monotonic() + 0.08
                while time.monotonic() < cycle_end:
                    executor.spin_once(timeout_sec=0.002)
            if not publish_scan:
                return None
            if not statuses:
                raise AssertionError("No proximity status messages received")
            return statuses[-1]

        result = drive_synthetic(1.45, around)
        assert result["path_valid"] and not result["stop"], result
        assert result["speed_limit_mps"] == 0.25, result
        assert safe_commands and max(m.linear.x for m in safe_commands) <= 0.25 + 1e-6
        assert safe_commands[-1].linear.x > 0.0
        print("PASS: valid bypass slows without inheriting fixed-box stop")
        print("PASS: Safety independently caps actual output to 0.25 m/s")
        result = drive_synthetic(1.45, straight)
        assert result["stop"] and result["reason"] == "obstacle_on_local_path", result
        print("PASS: swept-path conflict stops")
        result = drive_synthetic(0.8, around)
        assert result["stop"] and result["reason"] == "emergency_corridor", result
        print("PASS: independent near-body emergency corridor stops")
        result = drive_synthetic(1.45, around, speed=0.5)
        assert result["stop"] and result["emergency_distance_m"] > 1.4, result
        print("PASS: measured high speed keeps braking space despite slow request")
        result = drive_synthetic(1.45, None, duration=2.0)
        assert result["stop"] and result["reason"] == "fixed_fallback", result
        print("PASS: stale path falls back to fixed 1.80 m stop")
        result = drive_synthetic(1.45, around, bad_stamp=True)
        assert result["stop"] and result["reason"] == "invalid_scan_stamp", result
        print("PASS: invalid scan timestamp holds")
        result = drive_synthetic(None, straight)
        assert not result["stop"] and result["speed_limit_mps"] is None, result
        print("PASS: clear scans release stop and speed cap after hysteresis")
        drive_synthetic(None, straight, publish_scan=False, duration=1.0)
        assert safety_events and "avoidance_speed_limit_stale" in safety_events[-1]["reasons"], safety_events
        assert safe_commands[-1].linear.x == 0.0
        print("PASS: missing speed-cap heartbeat holds even with fresh stop=False messages")
        print("All synthetic checks passed; no chassis nodes or motion commands used.")
    finally:
        if proximity is not None:
            executor.remove_node(proximity)
            proximity.destroy_node()
        if harness is not None:
            executor.remove_node(harness)
            harness.destroy_node()
        if safety is not None:
            executor.remove_node(safety)
            safety.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
