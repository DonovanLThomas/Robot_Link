"""Thin LeRobot SO-101 wrapper.

The import happens only for live mode so networking and dry-run mapping can be
tested on machines that do not have LeRobot installed.
"""

from __future__ import annotations

from typing import Any
import math

import config


def make_follower(port: str, robot_id: str, max_relative_target=None):
    try:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    except ImportError:
        try:
            from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        except ImportError as exc:
            raise RuntimeError(
                "Activate your LeRobot environment with SO101Follower and Feetech support."
            ) from exc
    return SO101Follower(SO101FollowerConfig(
        port=port, id=robot_id, use_degrees=True,
        max_relative_target=max_relative_target, disable_torque_on_disconnect=True,
    ))


class RobotController:
    def __init__(self, dry_run: bool = config.DRY_RUN) -> None:
        self.dry_run = dry_run
        self.robot: Any | None = None
        self.action_features: Any | None = None

    def connect(self) -> None:
        if self.dry_run:
            print("DRY RUN: not connecting to LeRobot or SO-101.")
            return
        if not config.ROBOT_PORT:
            raise RuntimeError("Set ROBOT_PORT in jetson_controller/config.py before live mode.")

        self.robot = make_follower(config.ROBOT_PORT, config.ROBOT_ID, {
            joint: config.MAX_STEP_GRIPPER if joint == "gripper" else config.MAX_STEP_DEG
            for joint in config.ROBOT_JOINTS
        })
        if not self.robot.calibration:
            raise RuntimeError("Run lerobot-calibrate for this ROBOT_ID before live mode.")
        self.robot.bus.connect()
        self.robot.bus.disable_torque()
        if not self.robot.is_calibrated:
            raise RuntimeError("Robot calibration does not match the motors. Run lerobot-calibrate first.")
        start_pose = self.get_current_pose()
        for joint, value in start_pose.items():
            low, high = config.JOINT_LIMITS[joint]
            if not low <= value <= high:
                raise ValueError(f"Place {joint} inside [{low}, {high}] before live mode; measured {value}")
        self.robot.bus.sync_write("Goal_Position", start_pose)
        self.robot.configure()
        self.action_features = getattr(self.robot, "action_features", None)
        print("Connected to SO-101 follower.")
        self.print_feature_info()

    def print_feature_info(self) -> None:
        if self.robot is None:
            return
        action_features = getattr(self.robot, "action_features", None)
        observation_features = getattr(self.robot, "observation_features", None)
        features = getattr(self.robot, "features", None)
        print(f"LeRobot action_features: {action_features}")
        print(f"LeRobot observation_features: {observation_features}")
        if features is not None:
            print(f"LeRobot features: {features}")

    def get_current_pose(self) -> dict[str, float]:
        if self.dry_run:
            return dict(config.DRY_RUN_ROBOT_START_POSE)
        if self.robot is None:
            raise RuntimeError("Robot is not connected")

        observation = self._read_observation()
        pose = {}
        for joint in config.ROBOT_JOINTS:
            value = self._extract_joint_value(observation, joint)
            if value is None or not math.isfinite(value):
                raise RuntimeError(
                    f"Could not find current position for {joint!r} in observation: {observation}"
                )
            pose[joint] = value
        return pose

    def send_action(self, command: dict[str, float]) -> dict[str, float]:
        if self.dry_run:
            return dict(command)
        if self.robot is None:
            raise RuntimeError("Robot is not connected")

        action = {}
        for joint in config.ROBOT_JOINTS:
            value = float(command[joint])
            low, high = config.JOINT_LIMITS[joint]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"Unsafe command for {joint}: {value}")
            action[self._action_key(joint)] = value
        applied = self.robot.send_action(action)
        result = {}
        for joint in config.ROBOT_JOINTS:
            value = self._extract_joint_value(applied, joint)
            if value is None or not math.isfinite(value):
                raise RuntimeError(f"LeRobot did not return a valid applied position for {joint}")
            low, high = config.JOINT_LIMITS[joint]
            if not low <= value <= high:
                raise ValueError(f"Applied position for {joint} is outside configured limits")
            result[joint] = value
        return result

    def disconnect(self) -> None:
        if self.robot is not None:
            if self.robot.bus.is_connected:
                self.robot.bus.disconnect(disable_torque=True)
            self.robot = None

    def _read_observation(self) -> Any:
        assert self.robot is not None
        if hasattr(self.robot, "get_observation"):
            return self.robot.get_observation()
        if hasattr(self.robot, "read"):
            return self.robot.read()
        raise RuntimeError("Installed LeRobot robot object has no known observation method")

    def _extract_joint_value(self, observation: Any, joint: str) -> float | None:
        keys = (f"{joint}.pos", joint)
        if isinstance(observation, dict):
            for key in keys:
                if key in observation:
                    return float(observation[key])
            for key, value in observation.items():
                if isinstance(key, str) and key.endswith(f"{joint}.pos"):
                    return float(value)
        return None

    def _action_key(self, joint: str) -> str:
        default = f"{joint}.pos"
        if isinstance(self.action_features, dict):
            if default in self.action_features:
                return default
            for key in self.action_features:
                if isinstance(key, str) and key.endswith(default):
                    return key
        return default
