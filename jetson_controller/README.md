# Jetson IMU Teleoperation Controller

This directory is the Jetson-side half of the Pico IMU -> SO-101 teleoperation
pipeline. USB serial is the default transport; no Wi-Fi or IP address is needed.
For motor calibration, joint-limit measurement, and first live movement, follow
[the step-by-step live-mode guide](LIVE_MODE.md).

This arm has swapped motor IDs: physical **wrist roll = 6**, **gripper = 5**.
`ROBOT_MOTOR_IDS` in `config.py` applies to both movement and range measurement.
The adapter rebuilds the LeRobot bus mapping and associates saved calibration
entries by motor ID in memory. It does not rewrite IDs or calibration files.
Joint names always mean the physical joints: wrist roll uses degrees and the
gripper uses 0–100 units. Use these repo scripts for the swapped arm; standalone
LeRobot commands use their own default mapping.

The current default is mapping dry run:

- `MODE = 2` (use `--mode 1` for input only)
- `DRY_RUN = True`
- no LeRobot import
- no robot connection
- no motor commands

The intended progression is:

1. `MODE = 1`: receive Pico JSON over USB serial and print human joint fields.
2. `MODE = 2`: press Enter to calibrate neutral pose, map to SO-101 targets,
   apply filtering and safety, and print commands without moving motors.
3. `MODE = 3`: live teleoperation only after setting `DRY_RUN = False`,
   `ROBOT_PORT`, verified gains/signs, and verified joint limits.

## Packet Format

The receiver expects newline-terminated JSON from the Pico over USB serial
(or optional UDP):

```json
{
  "seq": 1234,
  "timestamp_ms": 12345678,
  "shoulder_pan": 15.3,
  "shoulder_lift": 42.7,
  "elbow_flex": 61.2,
  "wrist_flex": -8.4,
  "wrist_roll": 25.1
}
```

Packets are rejected if JSON is malformed, required fields are missing, values
are NaN/infinite/non-numeric, values are implausibly large, or sequence numbers
go backwards.
USB startup messages and raw sensor text are ignored. Partial serial lines are
buffered without blocking the control loop. Restart the controller after a Pico
reboot, since the Pico sequence counter restarts at zero.

## Configure

Edit `jetson_controller/config.py`.

Important fields:

- `IMU_TRANSPORT`: `"serial"` (default) or `"udp"`.
- `IMU_SERIAL_PORT`: Pico USB device, initially `/dev/ttyACM0`; prefer its stable
  `/dev/serial/by-id/` path when available. This is separate from `ROBOT_PORT`.
- `IMU_SERIAL_BAUD`: `115200`.
- `UDP_PORT`: for optional UDP, matches `JETSON_UDP_PORT` in `pico_imu_test/wifi_config.h`.
- `MODE`: `1`, `2`, or `3`.
- `DRY_RUN`: leave `True` until dry-run output is correct.
- `ROBOT_PORT`: fill in only after finding the SO-101 serial device.
- `*_SOURCE`: maps robot joints to packet fields. Later these can point at
  relative-orientation fields when the Pico sends them.
- `*_GAIN` and `*_SIGN`: tune after neutral calibration.
- `JOINT_LIMITS`: conservative placeholders; replace with verified safe values.
- `JOINT_LIMITS_VERIFIED`: must be `True` before live Mode 3 can run.
- `MAX_STEP_DEG`, `DEADBAND_DEG`, `LOW_PASS_ALPHA`: safety and smoothing.

Arm positions use degrees (`use_degrees=True` in LeRobot). The gripper uses
LeRobot's normalized 0–100 units. `FIXED_GRIPPER_POSITION = None` holds the
gripper position captured at neutral calibration; a numeric value chooses a
fixed target, approached with `MAX_STEP_GRIPPER` per update.

## Run on the Jetson

Flash the updated `pico_imu_test` firmware using the
[Pico build instructions](../pico_imu_test/README.md#build-for-pico-2-w), then plug
the Pico into a Jetson USB host port using a USB data cable. The `pico_mac`
firmware only prints a MAC address and cannot supply IMU packets.

After SSHing into the Jetson, copy this updated repository there and run from
its root in your Python environment:

```sh
cd ~/Robot_Link
python3 -m pip install -r jetson_controller/requirements.txt
ls -l /dev/serial/by-id/
python3 jetson_controller/run_teleop.py --mode 1 --serial-port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the Pico device shown on your Jetson; you can pass
the `/dev/serial/by-id/...` path directly. Close any serial monitor or viewer
using that port before starting the controller. No `s` command is needed:
teleoperation JSON streams automatically whenever USB serial is open and all
three sensors read successfully. Mode 1 prints the received joint fields and
packet rate without connecting to the robot.

If opening the port reports permission denied, add your user to `dialout`:

```sh
sudo usermod -aG dialout "$USER"
```

Log out and back in for the group change to apply. On USB disconnection the
controller exits through its robot cleanup path; reconnect and restart it.

If LeRobot is installed in a virtual environment or conda environment, activate
that environment first. This repo does not define the Jetson environment name.

Check which Python sees LeRobot:

```sh
python3 -c "import lerobot; print(lerobot.__file__)"
```

## Optional UDP and Useful Jetson Commands

To use the original network transport, build the Pico with
`-DIMU_ENABLE_WIFI=ON`, configure Wi-Fi/UDP in `wifi_config.h`, and run
`python3 jetson_controller/run_teleop.py --transport udp`.

Find the Jetson IP address:

```sh
hostname -I
ip -4 addr show
```

Verify the UDP listener while `run_teleop.py` is running:

```sh
ss -lunp | grep 5005
```

Find the SO-101 serial port:

```sh
ls -l /dev/serial/by-id/
dmesg | tail -50
```

Check whether LeRobot imports in the active environment:

```sh
python3 -c "from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig; print('SO101 API OK')"
```

Print LeRobot robot feature names without moving the arm after `ROBOT_PORT` is
set in `config.py`:

```sh
python3 -c "import sys; sys.path.insert(0, 'jetson_controller'); import config; from robot_controller import RobotController; r=RobotController(dry_run=False); r.connect(); r.disconnect()"
```

Send a test UDP packet locally without moving the arm:

```sh
python3 -c 'import json,socket,time; p={"seq":1,"timestamp_ms":int(time.time()*1000),"shoulder_pan":0,"shoulder_lift":10,"elbow_flex":45,"wrist_flex":0,"wrist_roll":0}; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto((json.dumps(p)+"\n").encode(),("127.0.0.1",5005))'
```

## Controls

While `run_teleop.py` is running:

- `Enter` or `c`: capture neutral pose in Modes 2/3 after a fresh packet has arrived.
- `Space` or `q`: software stop. The program disconnects and disables motor
  torque; support the arm because it may sag. This is not a hardware emergency stop.

On packet timeout, the controller prints:

```text
IMU DATA TIMEOUT - HOLDING LAST TARGET. Press c with fresh data to resume.
```

No further targets are sent after timeout. The servos retain their last target,
and neutral calibration is cleared. Fresh packets alone do not restart motion;
hold a neutral pose and press `c` again. USB disconnection exits and disables
torque instead. Live startup checks motor calibration and joint bounds with
torque disabled, loads the measured pose as the initial target, then configures
the follower. Support the arm through connection and disconnection.

## Tabletop Mapping Check

Run in an interactive terminal with the IMUs resting still on the table:

```sh
python3 jetson_controller/run_teleop.py --mode 2 --serial-port /dev/ttyACM0
```

Press `c` or Enter and look for `calibrated: True` in the recurring diagnostics.
Mode 1 now explains that calibration requires Mode 2; missing or stale packets
produce a calibration-not-ready message. If keyboard input is unavailable, run
in a terminal directly or allocate a terminal with `ssh -t`.

Leave IMUs 1 and 3 still and slowly tilt IMU 2 (mux channel 1) forward/backward.
Watch `shoulder_lift` and the mapped targets change. The current algorithm
measures tilt, not vertical position: lifting a level sensor up/down does not
produce a sustained angle change. Mode 2 displays targets without moving motors.
Live Mode 3 still requires a calibrated robot, its serial port, verified limits,
and `DRY_RUN = False`.
