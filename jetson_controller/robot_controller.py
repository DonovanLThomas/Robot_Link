"""Thin LeRobot SO-101 wrapper.

The import happens only for live mode so networking and dry-run mapping can be
tested on machines that do not have LeRobot installed.
"""

from __future__ import annotations

from typing import Any

import config


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

        try:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        except ImportError as exc:
            raise RuntimeError(
                "Could not import the current LeRobot SO101Follower API. "
                "Activate the Jetson environment where LeRobot is installed."
            ) from exc

        robot_config = SO101FollowerConfig(port=config.ROBOT_PORT, id=config.ROBOT_ID)
        self.robot = SO101Follower(robot_config)
        self.robot.connect()
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
            if value is None:
                raise RuntimeError(
                    f"Could not find current position for {joint!r} in observation: {observation}"
                )
            pose[joint] = value
        return pose

    def send_action(self, command: dict[str, float]) -> None:
        if self.dry_run:
            return
        if self.robot is None:
            raise RuntimeError("Robot is not connected")

        action = {}
        for joint in config.ROBOT_JOINTS:
            action[self._action_key(joint)] = float(command[joint])
        self.robot.send_action(action)

    def disconnect(self) -> None:
        if self.robot is not None:
            self.robot.disconnect()
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
