# Jetson IMU Teleoperation Controller

This directory is the Jetson-side half of the Pico IMU -> SO-101 teleoperation
pipeline. It starts in safe network-only mode:

- `MODE = 1`
- `DRY_RUN = True`
- no LeRobot import
- no robot connection
- no motor commands

The intended progression is:

1. `MODE = 1`: receive Pico UDP JSON and print human joint fields.
2. `MODE = 2`: press Enter to calibrate neutral pose, map to SO-101 targets,
   apply filtering and safety, and print commands without moving motors.
3. `MODE = 3`: live teleoperation only after setting `DRY_RUN = False`,
   `ROBOT_PORT`, verified gains/signs, and verified joint limits.

## Packet Format

The receiver expects newline-terminated UDP JSON from the Pico:

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

## Configure

Edit `jetson_controller/config.py`.

Important fields:

- `UDP_PORT`: must match `JETSON_UDP_PORT` in `pico_imu_test/wifi_config.h`.
- `MODE`: `1`, `2`, or `3`.
- `DRY_RUN`: leave `True` until dry-run output is correct.
- `ROBOT_PORT`: fill in only after finding the SO-101 serial device.
- `*_SOURCE`: maps robot joints to packet fields. Later these can point at
  relative-orientation fields when the Pico sends them.
- `*_GAIN` and `*_SIGN`: tune after neutral calibration.
- `JOINT_LIMITS`: conservative placeholders; replace with verified safe values.
- `JOINT_LIMITS_VERIFIED`: must be `True` before live Mode 3 can run.
- `MAX_STEP_DEG`, `DEADBAND_DEG`, `LOW_PASS_ALPHA`: safety and smoothing.

The gripper is fixed by `FIXED_GRIPPER_POSITION`.

## Run on the Jetson

After SSHing into the Jetson:

```sh
cd ~/Robot_Link
git pull
python3 jetson_controller/run_teleop.py
```

If LeRobot is installed in a virtual environment or conda environment, activate
that environment first. This repo does not define the Jetson environment name.

Check which Python sees LeRobot:

```sh
python3 -c "import lerobot; print(lerobot.__file__)"
```

## Useful Jetson Commands

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

- `Enter`: capture neutral pose after a valid packet has arrived.
- `Space` or `q`: emergency stop. The program stops sending commands and
  disconnects the robot in `finally`.

On packet timeout, the controller prints:

```text
IMU DATA TIMEOUT - HOLDING POSITION
```

It holds the last safety-limited command rather than continuing stale motion.
