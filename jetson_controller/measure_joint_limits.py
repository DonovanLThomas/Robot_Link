#!/usr/bin/env python3
"""Record hand-guided SO-101 joint extrema using LeRobot with torque disabled."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import select
import sys
import time

import config
from robot_controller import make_follower


def padded_limits(extrema, margin_deg, margin_gripper):
    limits = {}
    for joint in config.ROBOT_JOINTS:
        low, high = extrema[joint]
        margin = margin_gripper if joint == "gripper" else margin_deg
        if not all(math.isfinite(value) for value in (low, high, margin)) or margin <= 0:
            raise ValueError(f"Invalid range or margin for {joint}")
        if joint == "gripper":
            low, high = max(0.0, low), min(100.0, high)
        low = math.ceil((low + margin) * 100) / 100
        high = math.floor((high - margin) * 100) / 100
        if low >= high:
            raise ValueError(f"{joint}: insufficient movement for the margin; repeat the measurement")
        limits[joint] = (low, high)
    return limits


def record_ranges(bus):
    extrema = {joint: [math.inf, -math.inf] for joint in config.ROBOT_JOINTS}
    previous = None
    next_display = 0.0
    while True:
        positions = bus.sync_read("Present_Position")
        for joint in config.ROBOT_JOINTS:
            value = float(positions[joint])
            if not math.isfinite(value):
                raise ValueError(f"Invalid position for {joint}")
            if previous is not None and joint != "gripper" and abs(value - previous[joint]) > 180:
                raise ValueError(f"{joint}: encoder wrap detected; repeat within a smaller continuous range")
            extrema[joint][0] = min(extrema[joint][0], value)
            extrema[joint][1] = max(extrema[joint][1], value)
        previous = positions
        if time.monotonic() >= next_display:
            print(" | ".join(f"{joint}: {bounds[0]:.1f}..{bounds[1]:.1f}"
                             for joint, bounds in extrema.items()), flush=True)
            next_display = time.monotonic() + 0.5
        if select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline() == "":
                raise RuntimeError("Terminal closed; measurement aborted")
            return extrema
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="SO-101 USB port, not the Pico port")
    parser.add_argument("--id", default=config.ROBOT_ID, help="Same ID used by lerobot-calibrate")
    parser.add_argument("--margin-deg", type=float, default=5.0)
    parser.add_argument("--margin-gripper", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=Path("joint_limits.json"))
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("Use an interactive terminal (ssh -t for remote commands)")
    if args.output.exists():
        parser.error("Output exists; choose a new --output filename")
    if not all(math.isfinite(value) and value > 0 for value in
               (args.margin_deg, args.margin_gripper)):
        parser.error("Margins must be finite and positive")

    robot = make_follower(args.port, args.id)
    if not robot.calibration:
        raise RuntimeError("No saved motor calibration for this ID. Use the ID of your existing calibration.")
    print(f"Physical joint motor IDs: {config.ROBOT_MOTOR_IDS}")
    print("Support the arm: recording disables torque. Move each joint by hand, one at a time.")
    print("Stay within unobstructed travel; do not force stops or twist wrist cables.")
    print("Units: arm joints in degrees; gripper in 0-100. This sends no movement targets.")
    input("Press Enter when ready to disable torque and begin recording: ")
    try:
        robot.bus.connect()
        robot.bus.disable_torque()
        if not robot.is_calibrated:
            raise RuntimeError("Saved calibration differs from the motors. Check the ID and existing calibration file.")
        print("Recording ALL six joints. Press Enter when finished; Ctrl+C aborts without saving.")
        extrema = record_ranges(robot.bus)
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=True)

    limits = padded_limits(extrema, args.margin_deg, args.margin_gripper)
    report = {
        "robot_id": args.id,
        "motor_ids": dict(config.ROBOT_MOTOR_IDS),
        "arm_units": "degrees",
        "gripper_units": "0-100",
        "observed_ranges": extrema,
        "suggested_limits": limits,
        "margin_deg": args.margin_deg,
        "margin_gripper": args.margin_gripper,
    }
    with args.output.open("x") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    print(f"\nSaved observed minima/maxima and suggested limits to {args.output}")
    print("Review and narrow these limits for your workspace before copying into config.py:")
    print("JOINT_LIMITS = {")
    for joint, bounds in limits.items():
        print(f'    "{joint}": ({bounds[0]:.2f}, {bounds[1]:.2f}),')
    print("}")
    print("These are observed ranges, not certified mechanical or collision-safe limits.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit("Measurement aborted; no report saved.")
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc))
