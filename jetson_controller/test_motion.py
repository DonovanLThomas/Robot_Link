import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import config
from joint_mapper import JointMapper
import measure_joint_limits
from robot_controller import RobotController, make_follower
from safety import SafetyLimiter
import run_teleop


class LimitMeasurementTests(unittest.TestCase):
    def test_inward_margins_and_gripper_units(self):
        ranges = dict.fromkeys(config.HUMAN_JOINTS, (-40.126, 50.128))
        ranges["gripper"] = (-1, 101)
        limits = measure_joint_limits.padded_limits(ranges, 5, 5)
        self.assertEqual(limits["elbow_flex"], (-35.12, 45.12))
        self.assertEqual(limits["gripper"], (5, 95))

    def test_unmoved_joint_cannot_be_certified(self):
        ranges = dict.fromkeys(config.ROBOT_JOINTS, (0, 50))
        ranges["wrist_roll"] = (10, 10)
        with self.assertRaisesRegex(ValueError, "wrist_roll"):
            measure_joint_limits.padded_limits(ranges, 5, 5)

    def test_recording_reads_extrema_without_commanding_motors(self):
        bus = Mock()
        bus.sync_read.side_effect = [dict.fromkeys(config.ROBOT_JOINTS, 10),
                                     dict.fromkeys(config.ROBOT_JOINTS, -10)]
        with patch("measure_joint_limits.select.select", side_effect=[([], [], []), ([True], [], [])]), \
             patch("measure_joint_limits.sys.stdin.readline", return_value="\n"), \
             patch("measure_joint_limits.time.sleep"), contextlib.redirect_stdout(io.StringIO()):
            result = measure_joint_limits.record_ranges(bus)
        self.assertEqual(result["shoulder_lift"], [-10, 10])
        bus.sync_write.assert_not_called()
        bus.enable_torque.assert_not_called()

    def test_recording_rejects_encoder_wrap(self):
        bus = Mock()
        bus.sync_read.side_effect = [dict.fromkeys(config.ROBOT_JOINTS, 179),
                                     dict.fromkeys(config.ROBOT_JOINTS, -179)]
        with patch("measure_joint_limits.select.select", return_value=([], [], [])), \
             patch("measure_joint_limits.time.sleep"), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "wrap"):
                measure_joint_limits.record_ranges(bus)

    def test_measurement_disables_torque_and_saves_only_completed_ranges(self):
        for failure in (False, True):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                output_path = Path(directory) / "limits.json"
                with patch("sys.argv", ["measure_joint_limits.py", "--port", "/dev/robot",
                                        "--output", str(output_path)]), \
                     patch("measure_joint_limits.sys.stdin.isatty", return_value=True), \
                     patch("builtins.input", return_value=""), \
                     patch("measure_joint_limits.make_follower") as factory, \
                     patch("measure_joint_limits.record_ranges") as record, \
                     contextlib.redirect_stdout(io.StringIO()):
                    robot = factory.return_value
                    robot.calibration = {"saved": True}
                    robot.is_calibrated = True
                    robot.bus.is_connected = True
                    if failure:
                        record.side_effect = OSError("read failed")
                        with self.assertRaises(OSError):
                            measure_joint_limits.main()
                        self.assertFalse(output_path.exists())
                    else:
                        record.return_value = dict.fromkeys(config.ROBOT_JOINTS, (0, 60))
                        self.assertEqual(measure_joint_limits.main(), 0)
                        report = json.loads(output_path.read_text())
                        self.assertEqual(report["suggested_limits"]["elbow_flex"], [5, 55])
                    robot.bus.disable_torque.assert_called_once()
                    robot.bus.disconnect.assert_called_once_with(disable_torque=True)
                    robot.connect.assert_not_called()
                    robot.send_action.assert_not_called()
                    robot.bus.enable_torque.assert_not_called()


class MotionTests(unittest.TestCase):
    def test_start_pose_is_not_clamped_or_gripper_jumped(self):
        limiter = SafetyLimiter()
        start = dict(config.DRY_RUN_ROBOT_START_POSE, gripper=42)
        limiter.initialize(start)
        self.assertEqual(limiter.hold_position(), start)
        with self.assertRaisesRegex(ValueError, "outside"):
            limiter.initialize(dict(start, elbow_flex=999))

    def test_limiter_rejects_nan_and_limits_steps(self):
        limiter = SafetyLimiter()
        start = dict(config.DRY_RUN_ROBOT_START_POSE)
        limiter.initialize(start)
        with self.assertRaises(ValueError):
            limiter.apply(dict(start, elbow_flex=float("nan")))
        result = limiter.apply(dict(start, elbow_flex=999, gripper=999))
        self.assertEqual(result["elbow_flex"], config.MAX_STEP_DEG)
        self.assertEqual(result["gripper"], start["gripper"] + config.MAX_STEP_GRIPPER)

    def test_only_selected_joint_moves_and_gripper_holds(self):
        mapper = JointMapper()
        human = dict.fromkeys(config.HUMAN_JOINTS, 0)
        start = dict(config.DRY_RUN_ROBOT_START_POSE)
        mapper.calibrate(human, start)
        with patch.object(config, "ACTIVE_JOINTS", ("shoulder_lift",)), \
             patch.object(config, "FIXED_GRIPPER_POSITION", None):
            _, target = mapper.update(dict.fromkeys(config.HUMAN_JOINTS, 40))
        self.assertGreater(target["shoulder_lift"], start["shoulder_lift"])
        for joint in config.ROBOT_JOINTS:
            if joint != "shoulder_lift":
                self.assertEqual(target[joint], start[joint])
        mapper.reset()
        self.assertFalse(mapper.is_calibrated())

    def test_follower_uses_explicit_units_and_hardware_relative_limit(self):
        module = Mock()
        with patch.dict("sys.modules", {"lerobot": Mock(), "lerobot.robots": Mock(),
                                        "lerobot.robots.so_follower": module}):
            make_follower("/dev/robot", "ladon", 0.25)
        module.SO101FollowerConfig.assert_called_once_with(
            port="/dev/robot", id="ladon", use_degrees=True,
            max_relative_target=0.25, disable_torque_on_disconnect=True)

    def test_controller_returns_applied_action_and_rejects_bad_targets(self):
        controller = RobotController(dry_run=False)
        controller.robot = Mock()
        start = dict(config.DRY_RUN_ROBOT_START_POSE)
        applied = dict(start, elbow_flex=0.1)
        controller.robot.send_action.return_value = {f"{joint}.pos": value for joint, value in applied.items()}
        self.assertEqual(controller.send_action(start), applied)
        controller.robot.send_action.reset_mock()
        with self.assertRaises(ValueError):
            controller.send_action(dict(start, elbow_flex=float("nan")))
        controller.robot.send_action.assert_not_called()

    def test_live_start_holds_measured_pose_before_enabling_torque(self):
        start = dict(config.DRY_RUN_ROBOT_START_POSE)
        with patch.object(config, "ROBOT_PORT", "/dev/robot"), \
             patch("robot_controller.make_follower") as factory, \
             contextlib.redirect_stdout(io.StringIO()):
            robot = factory.return_value
            robot.calibration = {"saved": True}
            robot.is_calibrated = True
            robot.get_observation.return_value = {f"{joint}.pos": value for joint, value in start.items()}
            controller = RobotController(dry_run=False)
            controller.connect()
            calls = [call[0] for call in robot.mock_calls]
            self.assertLess(calls.index("bus.disable_torque"), calls.index("get_observation"))
            self.assertLess(calls.index("bus.sync_write"), calls.index("configure"))
            robot.bus.sync_write.assert_called_once_with("Goal_Position", start)
            controller.disconnect()
            robot.bus.disconnect.assert_called_once_with(disable_torque=True)

    def test_bad_start_or_calibration_cannot_enable_torque(self):
        for calibrated in (False, True):
            with self.subTest(calibrated=calibrated), \
                 patch.object(config, "ROBOT_PORT", "/dev/robot"), \
                 patch("robot_controller.make_follower") as factory:
                robot = factory.return_value
                robot.calibration = {"saved": True}
                robot.is_calibrated = calibrated
                robot.get_observation.return_value = {f"{joint}.pos": 999 for joint in config.ROBOT_JOINTS}
                controller = RobotController(dry_run=False)
                with self.assertRaises((RuntimeError, ValueError)):
                    controller.connect()
                robot.configure.assert_not_called()
                robot.bus.sync_write.assert_not_called()
                controller.disconnect()

    def test_live_mode_refuses_unverified_limits(self):
        with patch.object(config, "MODE", 3), patch.object(config, "DRY_RUN", False), \
             patch.object(config, "JOINT_LIMITS_VERIFIED", False):
            with self.assertRaisesRegex(SystemExit, "JOINT_LIMITS_VERIFIED"):
                run_teleop.validate_config()

    def test_timeout_requires_new_neutral_calibration(self):
        with patch.object(config, "MODE", 2), patch.object(config, "DRY_RUN", True), \
             patch.object(config, "IMU_TRANSPORT", "serial"), \
             patch("sys.argv", ["run_teleop.py"]), \
             patch("run_teleop.SerialImuReceiver") as receiver_factory, \
             patch("run_teleop.RobotController") as robot_factory, \
             patch("run_teleop.Keyboard") as keyboard_factory, \
             patch("run_teleop.print_diagnostics"), \
             patch("run_teleop.time.sleep"), contextlib.redirect_stdout(io.StringIO()):
            receiver = receiver_factory.return_value
            receiver.read_latest.return_value = Mock(human=dict.fromkeys(config.HUMAN_JOINTS, 0))
            receiver.timed_out.side_effect = [False, True]
            robot_factory.return_value.get_current_pose.return_value = config.DRY_RUN_ROBOT_START_POSE
            keyboard_factory.return_value.__enter__.return_value.read_key.side_effect = ["c", None, None, "q"]
            self.assertEqual(run_teleop.main(), 0)
            robot_factory.return_value.send_action.assert_not_called()
            robot_factory.return_value.disconnect.assert_called_once()


if __name__ == "__main__":
    unittest.main()
