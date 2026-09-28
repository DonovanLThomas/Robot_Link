"""Neutral calibration and human-angle to SO-101 target mapping."""

from __future__ import annotations

from dataclasses import dataclass
import copy

import config


@dataclass
class Calibration:
    human_zero: dict[str, float]
    robot_start: dict[str, float]


class JointMapper:
    def __init__(self) -> None:
        self.calibration: Calibration | None = None
        self.filtered_human: dict[str, float] | None = None

    def calibrate(self, human_angles: dict[str, float],
                  robot_start: dict[str, float]) -> None:
        self.calibration = Calibration(
            human_zero={joint: human_angles[joint] for joint in config.HUMAN_JOINTS},
            robot_start={joint: robot_start[joint] for joint in config.ROBOT_JOINTS},
        )
        self.filtered_human = copy.deepcopy(self.calibration.human_zero)

    def is_calibrated(self) -> bool:
        return self.calibration is not None

    def reset(self) -> None:
        self.calibration = None
        self.filtered_human = None

    def update(self, human_angles: dict[str, float]) -> tuple[dict[str, float], dict[str, float]]:
        """Return filtered human angles and mapped robot targets."""
        if self.calibration is None:
            raise RuntimeError("JointMapper must be calibrated before mapping")

        filtered = self._filter_human_angles(human_angles)
        target = {}
        for joint in config.HUMAN_JOINTS:
            if joint not in config.ACTIVE_JOINTS:
                target[joint] = self.calibration.robot_start[joint]
                continue
            source = config.JOINT_SOURCES[joint]
            human_value = self._resolve_source(filtered, source)
            human_zero = self._resolve_source(self.calibration.human_zero, source)
            delta = self._apply_deadband(joint, human_value - human_zero)
            target[joint] = (
                self.calibration.robot_start[joint]
                + config.SIGNS[joint] * config.GAINS[joint] * delta
            )

        target["gripper"] = (self.calibration.robot_start["gripper"]
                             if config.FIXED_GRIPPER_POSITION is None
                             else config.FIXED_GRIPPER_POSITION)
        return filtered, target

    def _filter_human_angles(self, human_angles: dict[str, float]) -> dict[str, float]:
        if self.filtered_human is None:
            self.filtered_human = {joint: human_angles[joint] for joint in config.HUMAN_JOINTS}
            return dict(self.filtered_human)

        alpha = config.LOW_PASS_ALPHA
        for joint in config.HUMAN_JOINTS:
            previous = self.filtered_human[joint]
            self.filtered_human[joint] = alpha * human_angles[joint] + (1.0 - alpha) * previous
        return dict(self.filtered_human)

    @staticmethod
    def _resolve_source(values: dict[str, float], source: str) -> float:
        if source not in values:
            raise KeyError(f"Configured source {source!r} is not available in packet")
        return values[source]

    @staticmethod
    def _apply_deadband(joint: str, delta: float) -> float:
        if abs(delta) < config.DEADBAND_DEG[joint]:
            return 0.0
        return delta
