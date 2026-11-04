"""Regression for tight turns introduced by recovery speed reductions."""

from dataclasses import replace
import unittest

from competition_control.mppi_controller import BodyCommand
from competition_safety.supervisor import SafetyContext, SafetyLimits, SafetySupervisor


class RecoveryCurvatureTests(unittest.TestCase):
    def setUp(self):
        self.limits = SafetyLimits(
            max_speed_mps=0.5, min_turning_radius_m=0.6,
            recovery_lateral_error_m=0.2,
            recovery_clear_lateral_error_m=0.1, recovery_speed_mps=0.15,
        )
        self.command = BodyCommand(0.3, -0.24, -0.8, 470, -0.23, 0.0, "TRACKING")
        self.context = SafetyContext(
            now_s=1.0, command_stamp_s=1.0, state_stamp_s=1.0,
            measured_speed_mps=0.3, estop_ready=True, remote_ready=True,
            state_valid=True, avoidance_ready=True, avoidance_stop=False,
            chassis_fault=False,
        )

    def test_recovery_ramp_does_not_tighten_either_turn_direction(self):
        for sign in (-1, 1):
            supervisor = SafetySupervisor(self.limits)
            command = replace(self.command, yaw_rate_radps=sign * 0.24,
                              curvature_1pm=sign * 0.8)
            for step in range(25):
                now = 1.0 + step * 0.05
                output = supervisor.filter_command(command, replace(
                    self.context, now_s=now, command_stamp_s=now, state_stamp_s=now))
                self.assertEqual(output.status, "SAFE_RECOVERY")
                self.assertAlmostEqual(output.yaw_rate_radps / output.linear_x_mps,
                                       sign * 0.8)
            self.assertAlmostEqual(output.linear_x_mps, 0.15)
            self.assertAlmostEqual(abs(output.yaw_rate_radps), 0.12)

    def test_startup_ramp_preserves_curvature_without_avoidance_cap(self):
        output = SafetySupervisor(self.limits).filter_command(
            replace(self.command, lateral_error_m=0.0),
            replace(self.context, measured_speed_mps=0.0))
        self.assertGreater(output.linear_x_mps, 0.0)
        self.assertAlmostEqual(output.yaw_rate_radps / output.linear_x_mps, -0.8)

    def test_normal_speed_cap_preserves_curvature(self):
        output = SafetySupervisor(replace(self.limits, max_speed_mps=0.2)).filter_command(
            replace(self.command, lateral_error_m=0.0),
            replace(self.context, measured_speed_mps=0.2))
        self.assertAlmostEqual(output.linear_x_mps, 0.2)
        self.assertAlmostEqual(output.yaw_rate_radps, -0.16)

    def test_persistent_error_still_stops(self):
        supervisor = SafetySupervisor(self.limits)
        command = replace(self.command, lateral_error_m=0.45)
        supervisor.filter_command(command, self.context)
        output = supervisor.filter_command(command, replace(
            self.context, now_s=2.1, command_stamp_s=2.1, state_stamp_s=2.1))
        self.assertEqual(output.status, "SAFE_HOLD")
        self.assertIn("lateral_error_persistent", output.reasons)
        self.assertEqual((output.linear_x_mps, output.yaw_rate_radps), (0.0, 0.0))

    def test_minimum_radius_still_bounds_infeasible_turn(self):
        output = SafetySupervisor(self.limits).filter_command(
            replace(self.command, yaw_rate_radps=-0.9, curvature_1pm=-3.0),
            self.context)
        self.assertLessEqual(abs(output.yaw_rate_radps),
                             output.linear_x_mps / self.limits.min_turning_radius_m)
        self.assertIn("curvature_limited", output.reasons)


if __name__ == "__main__":
    unittest.main()
