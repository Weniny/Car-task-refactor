from dataclasses import replace
import unittest

from competition_control.mppi_controller import BodyCommand
from competition_safety.supervisor import SafetyContext, SafetyLimits, SafetySupervisor


class AvoidanceSpeedLimitTests(unittest.TestCase):
    def setUp(self):
        self.supervisor = SafetySupervisor(SafetyLimits(max_speed_mps=0.5))
        self.command = BodyCommand(
            linear_x_mps=0.5,
            yaw_rate_radps=0.2,
            curvature_1pm=0.4,
            target_index=1,
            lateral_error_m=0.0,
            heading_error_rad=0.0,
            status="TRACKING",
        )
        self.context = SafetyContext(
            now_s=1.0,
            command_stamp_s=1.0,
            state_stamp_s=1.0,
            measured_speed_mps=0.5,
            estop_ready=True,
            remote_ready=True,
            state_valid=True,
            avoidance_ready=True,
            avoidance_stop=False,
            chassis_fault=False,
        )

    def test_external_cap_preserves_curvature_and_bounds_output(self):
        output = self.supervisor.filter_command(
            self.command, replace(self.context, avoidance_speed_limit_mps=0.25)
        )
        self.assertAlmostEqual(output.linear_x_mps, 0.25)
        self.assertAlmostEqual(output.yaw_rate_radps, 0.1)
        self.assertIn("avoidance_speed_limited", output.reasons)

    def test_missing_speed_limit_heartbeat_holds(self):
        output = self.supervisor.filter_command(
            self.command, replace(self.context, avoidance_speed_limit_ready=False)
        )
        self.assertEqual(output.status, "SAFE_HOLD")
        self.assertEqual(output.linear_x_mps, 0.0)
        self.assertIn("avoidance_speed_limit_stale", output.reasons)

    def test_acceleration_ramp_with_cap_preserves_curvature(self):
        output = self.supervisor.filter_command(
            replace(self.command, linear_x_mps=0.25, yaw_rate_radps=0.1),
            replace(self.context, measured_speed_mps=0.0, avoidance_speed_limit_mps=0.25),
        )
        self.assertGreater(output.linear_x_mps, 0.0)
        self.assertAlmostEqual(output.yaw_rate_radps / output.linear_x_mps, 0.4)

    def test_invalid_caps_hold(self):
        for cap in (-0.1, float("nan"), float("inf")):
            with self.subTest(cap=cap):
                output = self.supervisor.filter_command(
                    self.command, replace(self.context, avoidance_speed_limit_mps=cap)
                )
                self.assertEqual(output.status, "SAFE_HOLD")

    def test_fixed_stop_overrides_external_cap(self):
        output = self.supervisor.filter_command(
            self.command,
            replace(self.context, avoidance_stop=True, avoidance_speed_limit_mps=0.25),
        )
        self.assertEqual(output.linear_x_mps, 0.0)
        self.assertEqual(output.yaw_rate_radps, 0.0)

    def test_disabled_feature_leaves_existing_behavior(self):
        output = self.supervisor.filter_command(self.command, self.context)
        self.assertAlmostEqual(output.linear_x_mps, 0.5)
        self.assertAlmostEqual(output.yaw_rate_radps, 0.2)

    def test_goal_deceleration_cannot_exceed_cap(self):
        output = self.supervisor.filter_command(
            replace(self.command, status="GOAL_REACHED"),
            replace(self.context, avoidance_speed_limit_mps=0.25),
        )
        self.assertLessEqual(output.linear_x_mps, 0.25)


if __name__ == "__main__":
    unittest.main()
