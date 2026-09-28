#!/usr/bin/env python3
"""Run staged IMU teleoperation on the Jetson.

Modes:
  1: IMU input only. Print valid Pico packets. No robot connection.
  2: Mapping dry run. Calibrate, map, filter, limit, and print. No movement.
  3: Live teleoperation. Requires DRY_RUN = False and verified joint limits.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import config
from imu_receiver import ImuReceiver, SerialImuReceiver, UdpImuReceiver
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", type=int, choices=(1, 2, 3), default=config.MODE)
    parser.add_argument("--transport", choices=("serial", "udp"), default=config.IMU_TRANSPORT)
    parser.add_argument("--serial-port", default=config.IMU_SERIAL_PORT)
    parser.add_argument("--robot-port", default=config.ROBOT_PORT)
    parser.add_argument("--joint", action="append", choices=config.HUMAN_JOINTS,
                        help="Control only this joint; repeat for multiple joints")
    args = parser.parse_args()
    config.MODE = args.mode
    config.IMU_TRANSPORT = args.transport
    config.IMU_SERIAL_PORT = args.serial_port
    config.ROBOT_PORT = args.robot_port
    if args.joint:
        config.ACTIVE_JOINTS = tuple(args.joint)
    validate_config()
    live = config.MODE == 3 and not config.DRY_RUN

    print_startup_banner(live)
    try:
        receiver = (SerialImuReceiver(config.IMU_SERIAL_PORT, config.IMU_SERIAL_BAUD)
                    if config.IMU_TRANSPORT == "serial"
                    else UdpImuReceiver())
    except ImportError as exc:
        raise SystemExit("Install serial support: python3 -m pip install pyserial") from exc
    except OSError as exc:
        raise SystemExit(f"Cannot open IMU input: {exc}") from exc
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
        if live and not sys.stdin.isatty():
            raise RuntimeError("Live mode requires an interactive terminal (use ssh -t).")
        robot.connect()
        with Keyboard() as keyboard:
            if not keyboard.enabled:
                print("Keyboard input unavailable. Run in an interactive terminal; use ssh -t for remote commands.")
            while True:
                loop_start = time.monotonic()
                key = keyboard.read_key()
                if key in (" ", "q", "Q"):
                    print("Stop requested. Disconnecting; live motor torque will be disabled.")
                    break

                packet = receiver.read_latest()
                if packet is not None:
                    latest_human = packet.human

                if key in ("\n", "\r", "c", "C"):
                    if config.MODE == 1:
                        print("Mode 1 only displays input. Restart with --mode 2 to calibrate.")
                    elif latest_human is None or receiver.timed_out():
                        print("Calibration not ready: waiting for fresh IMU packets.")
                    else:
                        robot_start = robot.get_current_pose()
                        limiter.initialize(robot_start)
                        mapper.calibrate(latest_human, robot_start)
                        print("Neutral pose calibrated. Tilt an IMU to change the mapped targets.")
                elif config.MODE == 1:
                    safe_command = None
                elif mapper.is_calibrated():
                    if receiver.timed_out():
                        now = time.monotonic()
                        safe_command = limiter.hold_position()
                        if now - last_timeout_warning > 1.0:
                            print("IMU DATA TIMEOUT - HOLDING LAST TARGET. Press c with fresh data to resume.")
                            last_timeout_warning = now
                        mapper.reset()
                    elif latest_human is not None:
                        filtered_human, robot_target = mapper.update(latest_human)
                        safe_command = limiter.apply(robot_target)
                        safe_command = robot.send_action(safe_command)
                        limiter.current_command = dict(safe_command)

                now = time.monotonic()
                if now - last_diag >= diag_period:
                    print_diagnostics(receiver, latest_human, filtered_human,
                                      robot_target, safe_command, mapper.is_calibrated())
                    last_diag = now

                sleep_for = control_period - (time.monotonic() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("Interrupted. Disconnecting; live motor torque will be disabled.")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Controller stopped: {exc}. Disconnecting; live motor torque will be disabled.")
        return 1
    finally:
        try:
            receiver.close()
        finally:
            robot.disconnect()
    return 0


def validate_config() -> None:
    if config.IMU_TRANSPORT not in ("serial", "udp"):
        raise SystemExit("IMU_TRANSPORT must be serial or udp")
    if config.MODE not in (1, 2, 3):
        raise SystemExit("MODE must be 1, 2, or 3")
    if not all(math.isfinite(value) and value > 0 for value in
               (config.CONTROL_HZ, config.DIAGNOSTICS_HZ, config.PACKET_TIMEOUT_S)):
        raise SystemExit("CONTROL_HZ, DIAGNOSTICS_HZ and PACKET_TIMEOUT_S must be finite and positive")
    if not 0.0 < config.LOW_PASS_ALPHA <= 1.0:
        raise SystemExit("LOW_PASS_ALPHA must be in (0, 1]")
    if not all(math.isfinite(value) and value > 0 for value in
               (config.MAX_STEP_DEG, config.MAX_STEP_GRIPPER)):
        raise SystemExit("MAX_STEP_DEG and MAX_STEP_GRIPPER must be finite and positive")
    for joint in config.ROBOT_JOINTS:
        low, high = config.JOINT_LIMITS[joint]
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise SystemExit(f"Invalid limits for {joint}")
    gripper_low, gripper_high = config.JOINT_LIMITS["gripper"]
    if not 0 <= gripper_low < gripper_high <= 100:
        raise SystemExit("Gripper limits must be in LeRobot's 0-100 range")
    if config.FIXED_GRIPPER_POSITION is not None:
        if not gripper_low <= config.FIXED_GRIPPER_POSITION <= gripper_high:
            raise SystemExit("FIXED_GRIPPER_POSITION must be within gripper limits")
    for joint in config.HUMAN_JOINTS:
        if not math.isfinite(config.GAINS[joint]):
            raise SystemExit(f"Non-finite gain for {joint}")
        if not math.isfinite(config.DEADBAND_DEG[joint]) or config.DEADBAND_DEG[joint] < 0:
            raise SystemExit(f"Invalid deadband for {joint}")
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
        if config.IMU_TRANSPORT == "serial" and os.path.realpath(config.ROBOT_PORT) == os.path.realpath(config.IMU_SERIAL_PORT):
            raise SystemExit("ROBOT_PORT and IMU_SERIAL_PORT must be different USB devices")


def print_startup_banner(live: bool) -> None:
    mode_names = {
        1: "MODE 1 - IMU input only",
        2: "MODE 2 - robot mapping dry run",
        3: "MODE 3 - live teleoperation",
    }
    print("=" * 72)
    print(mode_names[config.MODE])
    if config.IMU_TRANSPORT == "serial":
        print(f"USB serial: {config.IMU_SERIAL_PORT} at {config.IMU_SERIAL_BAUD} baud")
    else:
        print(f"UDP listen: {config.UDP_BIND_IP}:{config.UDP_PORT}")
    print(f"DRY_RUN: {config.DRY_RUN}")
    print(f"Live robot commands enabled: {live}")
    if live:
        print("Support the arm at connection and exit; disconnect disables motor torque.")
    print(f"Active joints: {', '.join(config.ACTIVE_JOINTS)}; arm units: degrees; gripper: 0-100")
    print("Keys: Enter/c = calibrate and start mapping, Space/q = stop and disconnect")
    print("=" * 72)


def print_diagnostics(receiver: ImuReceiver,
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
        print("Waiting: receive a valid packet, hold neutral pose, then press Enter or c.")


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
