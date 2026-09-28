"""Safety filters applied after mapping and before any robot command."""

from __future__ import annotations

import config


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SafetyLimiter:
    def __init__(self) -> None:
        self.current_command: dict[str, float] | None = None

    def initialize(self, start_pose: dict[str, float]) -> None:
        self.current_command = {joint: start_pose[joint] for joint in config.ROBOT_JOINTS}
        self.current_command["gripper"] = config.FIXED_GRIPPER_POSITION
        self.current_command = self.apply_joint_limits(self.current_command)

    def hold_position(self) -> dict[str, float] | None:
        if self.current_command is None:
            return None
        return dict(self.current_command)

    def apply(self, target: dict[str, float]) -> dict[str, float]:
        if self.current_command is None:
            self.initialize(target)

        limited = self.apply_joint_limits(target)
        safe = {}
        assert self.current_command is not None
        for joint in config.ROBOT_JOINTS:
            delta = limited[joint] - self.current_command[joint]
            step = clamp(delta, -config.MAX_STEP_DEG, config.MAX_STEP_DEG)
            safe[joint] = self.current_command[joint] + step

        safe = self.apply_joint_limits(safe)
        self.current_command = safe
        return dict(safe)

    @staticmethod
    def apply_joint_limits(target: dict[str, float]) -> dict[str, float]:
        limited = {}
        for joint in config.ROBOT_JOINTS:
            low, high = config.JOINT_LIMITS[joint]
            limited[joint] = clamp(target[joint], low, high)
        return limited
