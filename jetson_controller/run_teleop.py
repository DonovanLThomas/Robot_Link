#!/usr/bin/env python3
"""Run staged IMU teleoperation on the Jetson.

Modes:
  1: UDP/network only. Print valid Pico packets. No robot connection.
  2: Mapping dry run. Calibrate, map, filter, limit, and print. No movement.
  3: Live teleoperation. Requires DRY_RUN = False and verified joint limits.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

import config
from imu_receiver import UdpImuReceiver
from joint_mapper import JointMapper
from robot_controller import RobotController
from safety import SafetyLimiter


class Keyboard:
    def __enter__(self) -> "Keyboard":
        self.fd = sys.stdin.fileno()
        self.enabled = sys.stdin.isatty()
        self.old_settings = None
        if self.enabled:
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self) -> str | None:
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return os.read(self.fd, 1).decode("utf-8", errors="ignore")


def main() -> int:
    validate_config()
    live = config.MODE == 3 and not config.DRY_RUN

    print_startup_banner(live)
    receiver = UdpImuReceiver()
    mapper = JointMapper()
    limiter = SafetyLimiter()
    robot = RobotController(dry_run=not live)

    latest_human: dict[str, float] | None = None
    filtered_human: dict[str, float] | None = None
    robot_target: dict[str, float] | None = None
    safe_command: dict[str, float] | None = None
    last_timeout_warning = 0.0
    last_diag = 0.0
    control_period = 1.0 / config.CONTROL_HZ
    diag_period = 1.0 / config.DIAGNOSTICS_HZ

    try:
        robot.connect()
        with Keyboard() as keyboard:
            while True:
                loop_start = time.monotonic()
                key = keyboard.read_key()
                if key in (" ", "q", "Q"):
                    print("Emergency stop requested. Holding/disconnecting.")
                    break

                packet = receiver.read_latest()
                if packet is not None:
                    latest_human = packet.human

                if config.MODE == 1:
                    safe_command = None
                elif latest_human is not None and key in ("\n", "\r"):
                    robot_start = robot.get_current_pose()
                    mapper.calibrate(latest_human, robot_start)
                    limiter.initialize(robot_start)
                    print("Neutral pose calibrated.")
                elif mapper.is_calibrated():
                    if receiver.timed_out():
                        now = time.monotonic()
                        safe_command = limiter.hold_position()
                        if now - last_timeout_warning > 1.0:
                            print("IMU DATA TIMEOUT - HOLDING POSITION")
                            last_timeout_warning = now
                    elif latest_human is not None:
                        filtered_human, robot_target = mapper.update(latest_human)
                        safe_command = limiter.apply(robot_target)
                        robot.send_action(safe_command)

                now = time.monotonic()
                if now - last_diag >= diag_period:
                    print_diagnostics(receiver, latest_human, filtered_human,
                                      robot_target, safe_command, mapper.is_calibrated())
                    last_diag = now

                sleep_for = control_period - (time.monotonic() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("Interrupted. Holding/disconnecting.")
    finally:
        receiver.close()
        robot.disconnect()
    return 0


def validate_config() -> None:
    if config.MODE not in (1, 2, 3):
        raise SystemExit("MODE must be 1, 2, or 3")
    if config.CONTROL_HZ <= 0 or config.DIAGNOSTICS_HZ <= 0:
        raise SystemExit("CONTROL_HZ and DIAGNOSTICS_HZ must be positive")
    if not 0.0 < config.LOW_PASS_ALPHA <= 1.0:
        raise SystemExit("LOW_PASS_ALPHA must be in (0, 1]")
    if config.MAX_STEP_DEG <= 0:
        raise SystemExit("MAX_STEP_DEG must be positive")
    for joint, sign in config.SIGNS.items():
        if sign not in (-1, 1):
            raise SystemExit(f"{joint} sign must be 1 or -1")
    if config.MODE == 3:
        if config.DRY_RUN:
            raise SystemExit("MODE 3 requires DRY_RUN = False. Use MODE 2 for dry-run mapping.")
        if not config.JOINT_LIMITS_VERIFIED:
            raise SystemExit("Set JOINT_LIMITS_VERIFIED = True only after verifying safe limits.")
        if not config.ROBOT_PORT:
            raise SystemExit("Set ROBOT_PORT before MODE 3 live teleoperation.")


def print_startup_banner(live: bool) -> None:
    mode_names = {
        1: "MODE 1 - IMU/network only",
        2: "MODE 2 - robot mapping dry run",
        3: "MODE 3 - live teleoperation",
    }
    print("=" * 72)
    print(mode_names[config.MODE])
    print(f"UDP listen: {config.UDP_BIND_IP}:{config.UDP_PORT}")
    print(f"DRY_RUN: {config.DRY_RUN}")
    print(f"Live robot commands enabled: {live}")
    print("Keys: Enter = calibrate neutral pose, Space/q = emergency stop")
    print("=" * 72)


def print_diagnostics(receiver: UdpImuReceiver,
                      raw_human: dict[str, float] | None,
                      filtered_human: dict[str, float] | None,
                      robot_target: dict[str, float] | None,
                      safe_command: dict[str, float] | None,
                      calibrated: bool) -> None:
    age = receiver.last_packet_age()
    age_text = "--" if age is None else f"{age * 1000.0:.0f} ms"
    print("\n--- diagnostics ---")
    print(f"packets/sec: {receiver.packets_per_second():.0f}  last age: {age_text}  calibrated: {calibrated}")
    print_joint_block("Human raw", raw_human, config.HUMAN_JOINTS)
    print_joint_block("Human filtered", filtered_human, config.HUMAN_JOINTS)
    print_joint_block("Robot target", robot_target, config.ROBOT_JOINTS)
    print_joint_block("Safety command", safe_command, config.ROBOT_JOINTS)
    if config.MODE in (2, 3) and not calibrated:
        print("Waiting: receive a valid packet, hold neutral pose, then press Enter.")


def print_joint_block(title: str, values: dict[str, float] | None,
                      joints: tuple[str, ...]) -> None:
    print(f"{title}:")
    if values is None:
        print("  --")
        return
    for joint in joints:
        print(f"  {joint:13s}: {values[joint]:+7.2f}")


if __name__ == "__main__":
    raise SystemExit(main())
