from dataclasses import dataclass
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import config
from robot_controller import apply_motor_mapping


@dataclass
class Motor:
    id: int
    model: str
    norm_mode: str


@dataclass
class Calibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


class Bus:
    def __init__(self, port, motors, calibration, protocol_version=0):
        self.port = port
        self.motors = motors
        self.calibration = calibration
        self.protocol_version = protocol_version
        self.id_to_name = {motor.id: joint for joint, motor in motors.items()}


def follower():
    motors = {joint: Motor(index, "sts3215", "0-100" if joint == "gripper" else "degrees")
              for index, joint in enumerate(config.ROBOT_JOINTS, 1)}
    calibration = {joint: Calibration(motor.id, 0, motor.id * 10, motor.id * 100, motor.id * 100 + 1000)
                   for joint, motor in motors.items()}
    return SimpleNamespace(bus=Bus("/dev/robot", motors, calibration), calibration=calibration)


class MotorMappingTests(unittest.TestCase):
    def test_swapped_ids_preserve_physical_units_and_calibration(self):
        robot = follower()
        original_bus = robot.bus
        original_calibration = robot.calibration
        apply_motor_mapping(robot)
        self.assertEqual(robot.bus.motors["wrist_roll"].id, 6)
        self.assertEqual(robot.bus.motors["gripper"].id, 5)
        self.assertEqual(robot.bus.motors["wrist_roll"].norm_mode, "degrees")
        self.assertEqual(robot.bus.motors["gripper"].norm_mode, "0-100")
        self.assertEqual(robot.bus.id_to_name[6], "wrist_roll")
        self.assertEqual(robot.bus.id_to_name[5], "gripper")
        self.assertEqual(robot.calibration["wrist_roll"], original_calibration["gripper"])
        self.assertEqual(robot.calibration["gripper"], original_calibration["wrist_roll"])
        self.assertIsNot(robot.calibration["gripper"], original_calibration["wrist_roll"])
        self.assertIs(robot.bus.calibration, robot.calibration)
        self.assertEqual(original_bus.motors["wrist_roll"].id, 5)
        self.assertEqual(original_calibration["wrist_roll"].id, 5)
        for joint in config.ROBOT_JOINTS[:4]:
            self.assertEqual(robot.bus.motors[joint], original_bus.motors[joint])
            self.assertEqual(robot.calibration[joint], original_calibration[joint])

    def test_already_remapped_calibration_is_not_swapped_twice(self):
        robot = apply_motor_mapping(follower())
        calibration = dict(robot.calibration)
        apply_motor_mapping(robot)
        self.assertEqual(robot.calibration, calibration)
        self.assertEqual(robot.bus.motors["wrist_roll"].id, 6)

    def test_standard_motor_ids_remain_supported(self):
        robot = follower()
        original_calibration = dict(robot.calibration)
        ids = {joint: motor.id for joint, motor in robot.bus.motors.items()}
        with patch.object(config, "ROBOT_MOTOR_IDS", ids):
            apply_motor_mapping(robot)
        self.assertEqual(robot.calibration, original_calibration)

    def test_no_calibration_is_not_fabricated(self):
        robot = follower()
        robot.calibration = {}
        apply_motor_mapping(robot)
        self.assertEqual(robot.calibration, {})
        self.assertEqual(robot.bus.calibration, {})

    def test_bad_configuration_and_missing_calibration_fail(self):
        for ids in ({}, dict(config.ROBOT_MOTOR_IDS, wrist_roll=5),
                    dict(config.ROBOT_MOTOR_IDS, wrist_roll=True)):
            with self.subTest(ids=ids), patch.object(config, "ROBOT_MOTOR_IDS", ids):
                with self.assertRaises(ValueError):
                    apply_motor_mapping(follower())
        robot = follower()
        del robot.calibration["gripper"]
        with self.assertRaisesRegex(ValueError, "calibration"):
            apply_motor_mapping(robot)


if __name__ == "__main__":
    unittest.main()
