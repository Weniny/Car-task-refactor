#!/usr/bin/env python3
"""ROS integration tests using isolated topics and a simulated controller only."""

import json
import threading
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from route_command import RouteCommand


class FakeController(Node):
    def __init__(self, prefix, *, name="test_route_controller", ignore_start=False,
                 initial_enabled=False, stop_delay=0.0):
        super().__init__(name)
        self.enabled = initial_enabled
        self.ignore_start = ignore_start
        self.stop_delay = stop_delay
        self.stop_at = None
        self.received = []
        self.create_subscription(Bool, prefix + "/enable", self.command, 10)
        self.status_pub = self.create_publisher(String, prefix + "/status", 10)
        self.safe_pub = self.create_publisher(Twist, prefix + "/safe", 10)
        self.chassis_pub = self.create_publisher(Twist, prefix + "/chassis", 10)
        self.odom_pub = self.create_publisher(Odometry, prefix + "/odom", 10)
        self.create_timer(0.05, self.tick)

    def command(self, message):
        self.received.append(message.data)
        if message.data and self.ignore_start:
            return
        self.enabled = message.data
        if not message.data:
            self.stop_at = time.monotonic()

    def tick(self):
        self.status_pub.publish(String(data=json.dumps({
            "route_enabled": self.enabled,
            "status": "TRACKING" if self.enabled else "ROUTE_DISABLED",
        })))
        self.safe_pub.publish(Twist())
        self.chassis_pub.publish(Twist())
        odom = Odometry()
        if self.stop_at is not None and time.monotonic() - self.stop_at < self.stop_delay:
            odom.twist.twist.linear.x = 0.1
        self.odom_pub.publish(odom)


class RouteCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def make_nodes(self, *, expected="/test_route_controller", **kwargs):
        prefix = "/route_command_test_" + uuid.uuid4().hex
        fake = FakeController(prefix, **kwargs)
        executor = SingleThreadedExecutor()
        executor.add_node(fake)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        command = RouteCommand(
            route_topic=prefix + "/enable", status_topic=prefix + "/status",
            safe_topic=prefix + "/safe", chassis_topic=prefix + "/chassis",
            odom_topic=prefix + "/odom", controller_node=expected,
        )

        def cleanup():
            command.cancel_unconfirmed_start()
            command.destroy_node()
            executor.shutdown()
            worker.join(timeout=3)
            fake.destroy_node()

        self.addCleanup(cleanup)
        return command, fake

    def test_controller_alone_does_not_require_recorder(self):
        command, fake = self.make_nodes()
        result = command.command(True, 8)
        self.assertTrue(result["route_enabled"])
        self.assertEqual(result["control_status"], "TRACKING")
        self.assertEqual(fake.received, [True])
        stopped = command.command(False, 8)
        self.assertTrue(stopped["zero_commands_and_odom_confirmed"])
        self.assertFalse(fake.enabled)

    def test_unrelated_subscriber_cannot_satisfy_controller_gate(self):
        command, fake = self.make_nodes(expected="/missing_controller", name="recorder_only")
        with self.assertRaisesRegex(RuntimeError, "subscription did not become ready"):
            command.command(True, 1.5)
        self.assertEqual(fake.received, [])

    def test_missing_ack_disarms_and_does_not_repeat_true(self):
        command, fake = self.make_nodes(ignore_start=True)
        with self.assertRaisesRegex(RuntimeError, "did not confirm"):
            command.command(True, 4)
        command.cancel_unconfirmed_start()
        self.assertEqual(fake.received.count(True), 1)
        self.assertGreaterEqual(fake.received.count(False), 1)
        self.assertFalse(fake.enabled)

    def test_stop_waits_for_new_stationary_feedback(self):
        command, fake = self.make_nodes(initial_enabled=True, stop_delay=0.5)
        result = command.command(False, 8)
        self.assertTrue(result["zero_commands_and_odom_confirmed"])
        self.assertGreaterEqual(time.monotonic() - fake.stop_at, 0.8)

    def test_status_observation_sends_no_command(self):
        command, fake = self.make_nodes()
        self.assertTrue(command.wait(lambda: command.status is not None, 8))
        self.assertFalse(command.status["route_enabled"])
        self.assertEqual(fake.received, [])


if __name__ == "__main__":
    unittest.main()
