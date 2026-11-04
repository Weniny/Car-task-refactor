#!/usr/bin/env python3
"""Isolated ROS test. Requires ROS_DOMAIN_ID=93; no chassis/controller is started."""
import math
import os
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from competition_safety.proximity_stop_node import ProximityStopNode


def main():
    assert os.environ.get("ROS_DOMAIN_ID") == "93", "use isolated ROS_DOMAIN_ID=93"
    rclpy.init(args=["--ros-args", "-p", "obstacle_slowdown_enabled:=true",
                      "-p", "input_type:=laser_scan", "-p", "stop_distance_m:=0.40",
                      "-p", "max_scan_age_s:=0.30"])
    gate = ProximityStopNode()
    probe = Node("obstacle_slowdown_test")
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(probe)
    seen = {}
    publisher = probe.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    subscriptions = [probe.create_subscription(cls, topic,
        lambda m, t=topic: seen.__setitem__(t, m.data), 10)
        for topic, cls in [("/avoidance/stop_request", Bool),
                           ("/avoidance/speed_limit", Float32)]]

    def send(distance, seconds=1.0, age=0.0, frame="body"):
        seen.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            message = LaserScan()
            stamp = probe.get_clock().now().nanoseconds - int(age * 1e9)
            message.header.stamp.sec = stamp // 1_000_000_000
            message.header.stamp.nanosec = stamp % 1_000_000_000
            message.header.frame_id = frame
            message.angle_min = -0.02
            message.angle_max = 0.02
            message.angle_increment = 0.02
            message.range_min = 0.1
            message.range_max = 6.0
            message.ranges = [distance] * 3
            publisher.publish(message)
            for _ in range(8):
                executor.spin_once(timeout_sec=0.01)
        assert set(seen) == {"/avoidance/stop_request", "/avoidance/speed_limit"}, seen

    try:
        send(2.0)
        assert not seen["/avoidance/stop_request"] and abs(seen["/avoidance/speed_limit"]-.2)<1e-6, seen
        print("PASS obstacle at 2m slows without fixed-stop override")
        send(.30)
        assert seen["/avoidance/stop_request"], seen
        print("PASS original close obstacle stop preserved")
        send(math.inf, seconds=.2)
        assert abs(seen["/avoidance/speed_limit"]-.2)<1e-6, seen
        send(math.inf)
        assert not seen["/avoidance/stop_request"] and seen["/avoidance/speed_limit"] == 0., seen
        print("PASS slowdown release hysteresis and clear heartbeat")
        for kwargs in ({"age": 2.0}, {"age": -2.0}, {"frame": "wrong_frame"}):
            send(math.inf, **kwargs)
            assert seen["/avoidance/stop_request"] and abs(seen["/avoidance/speed_limit"]-.2)<1e-6, seen
        print("PASS stale/future/wrong-frame scans stop")
    finally:
        executor.shutdown()
        gate.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
