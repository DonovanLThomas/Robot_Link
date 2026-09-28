# SO-101 live movement from Pico IMUs

Run these commands on the Jetson in your existing LeRobot 0.6.2 environment.
Copy the updated `jetson_controller` folder there first. The Pico firmware does
not need another update for these steps. The current default stays Mode 2 with
`DRY_RUN = True` until you explicitly configure live movement.

## 1. Identify both USB devices

Connect the Pico and the SO-101 follower controller to separate Jetson USB host
ports. The follower also needs its normal motor power supply.

```sh
cd ~/Robot_Link
ls -l /dev/serial/by-id/
```

Identify each device by unplugging/replugging one USB cable at a time while no
controller program is running. Prefer the stable `/dev/serial/by-id/...` paths.
The examples below assume `/dev/ttyACM0` is the Pico and `/dev/ttyACM1` is the
robot; substitute your actual devices. Never use the same port for both.

Stop the Mode 2 controller and any other serial monitors before continuing.

## 2. Verify the LeRobot environment

```sh
python3 -m pip show lerobot
python3 -c "from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig; import scservo_sdk; print('SO-101 and Feetech available')"
```

Your editable installation is `/home/dontech/lerobot`. If the Feetech import
is missing, install the extra into the same active Python environment:

```sh
python3 -m pip install -e '/home/dontech/lerobot[feetech]'
```

## 3. Calibrate the robot motors if needed

The IMU neutral calibration from Mode 2 does not calibrate the robot motors.
If the follower already has a valid LeRobot calibration, reuse that robot ID
and skip recalibration. The project uses `ROBOT_ID = "ladon"`; change it if your
existing calibration uses another ID.

For a follower whose motor IDs are already set up, use the official workflow:

```sh
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=ladon
```

Follow the terminal prompts and the
[official calibration video](https://huggingface.co/docs/lerobot/so101#calibration-video):
place the arm in the illustrated middle pose, then move the requested joints
by hand through their travel. Keep the base secured and support the links when
torque is disabled. Do not force a joint past an obstruction or twist cables.
Use this same robot ID for measurement and live operation.

## 4. Measure the minimum and maximum of each joint

```sh
python3 jetson_controller/measure_joint_limits.py \
  --port /dev/ttyACM1 --id ladon \
  --output joint_limits.json
```

1. Support the arm and press Enter to begin. Torque is disabled.
2. Move each of the six joints by hand, one at a time, through the unobstructed
   range you intend to use, including opening and closing the gripper.
3. Watch the printed minimum/maximum values. Arm joints are in degrees;
   the gripper is in 0–100 units.
4. Return the arm to a supported resting position and press Enter to finish.

This script only reads positions and disables torque; it sends no movement
targets. It does not change LeRobot's motor calibration. Ctrl+C aborts without
saving. Torque remains disabled on exit.

The JSON report contains observed ranges and suggested bounds inset by 5
degrees on each side (5 percentage points for the gripper). The script also
prints a `JOINT_LIMITS = {...}` block to paste into `config.py`. It refuses to
produce limits for a joint that did not move far enough for the margins. Use a
new `--output` name when repeating a measurement; existing reports are not
overwritten. Encoder wrap is rejected; use a continuous wrist range that does
not cross the wrap or strain cables.

These are observed ranges, not automatically discovered mechanical maxima or
collision-free limits. Reduce them further around your working area. Per-joint
bounds cannot prevent every self-collision or table collision. Re-measure if
you change motor calibration, horns, or the arm assembly.

## 5. Configure a slow first movement

Edit `jetson_controller/config.py`:

```python
ROBOT_ID = "ladon"
ROBOT_PORT = "/dev/ttyACM1"
IMU_SERIAL_PORT = "/dev/ttyACM0"

MODE = 3
DRY_RUN = False
JOINT_LIMITS_VERIFIED = True

MAX_STEP_DEG = 0.25
MAX_STEP_GRIPPER = 0.5
FIXED_GRIPPER_POSITION = None
SHOULDER_LIFT_GAIN = 0.25
```

Replace the existing `JOINT_LIMITS` dictionary with your reviewed measured
bounds before setting `JOINT_LIMITS_VERIFIED = True`. This flag records your
review; the recorder does not enable live mode for you. Edit the existing gain
assignment above the `GAINS` dictionary, not an extra assignment at file end.

At 30 Hz, a 0.25-degree target step permits at most 7.5 degrees/second of target
change. LeRobot also limits each requested target's distance from the measured
joint position. These are software command limits, not a physical speed guarantee.

Place the robot inside the configured bounds, with the gripper partly open if
your inward margins exclude the endpoints. Keep the base secured and workspace
clear. Support the links at connection: startup briefly disables torque, checks
calibration and bounds, sets the current pose as the initial goal, and then
enables the follower's holding torque.

## 6. Test one joint in live mode

In an interactive terminal (use `ssh -t` for a remote command):

```sh
python3 jetson_controller/run_teleop.py \
  --mode 3 --serial-port /dev/ttyACM0 --robot-port /dev/ttyACM1 \
  --joint shoulder_lift
```

1. Check the banner says Mode 3, live commands enabled, and only shoulder lift active.
2. Hold your IMUs still in their neutral pose. Press `c` or Enter.
3. Make a small tilt. Only shoulder lift receives changing mapped targets;
   other joints and the gripper hold their captured starting positions.
4. Return to neutral. Stop before changing a sign or gain in the config.
5. Press `q`, Space, or Ctrl+C to exit. Exit disables torque, so support the arm.

The keys are a software stop, not an independent emergency-stop circuit. Keep
the motor power cutoff accessible during initial tests.

Repeat the single-joint test with `--joint elbow_flex`, `--joint wrist_flex`,
and `--joint wrist_roll` after verifying each sensor estimate. Set low gains
for the joint being tested. Shoulder pan currently stays neutral: the Pico's
acceleration-based estimator does not measure yaw. The elbow estimate is still
the experimental triangle calculation, not anatomical joint-angle tracking.

## 7. Enable the joints you have checked

Repeat `--joint` to select the verified channels:

```sh
python3 jetson_controller/run_teleop.py \
  --mode 3 --serial-port /dev/ttyACM0 --robot-port /dev/ttyACM1 \
  --joint shoulder_lift --joint wrist_flex
```

Omit `--joint` to use all joints in `ACTIVE_JOINTS`. Each run requires a new
neutral calibration. An IMU timeout holds the last motor target and clears
neutral calibration; fresh packets do not automatically resume motion. Press
`c` with a fresh neutral pose to restart. USB disconnect or an I/O error exits
and disables torque. If the Pico reboots, restart the controller too.

To return to testing without motors, run with `--mode 2`; the program uses the
dry-run robot wrapper in Mode 2 even if `DRY_RUN` remains False. Prefer restoring
`DRY_RUN = True` when finished with live testing.

## LeRobot references and implementation

- [SO-101 setup and calibration](https://huggingface.co/docs/lerobot/so101)
- [Robot API: observations and actions](https://huggingface.co/docs/lerobot/main/api/robots)
- [SO follower implementation and units](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)
- [Motor bus torque and position API](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py)

`robot_controller.py` uses `SO101FollowerConfig(use_degrees=True)`, reads current
positions using `get_observation()`, and sends `<joint>.pos` targets through
`send_action()`. It tracks the action returned by LeRobot after relative-target
clipping. Live startup uses the follower's motor bus to establish a current-pose
goal before `configure()` enables torque. The measurement script uses that bus
with torque disabled throughout. The references were checked against current
LeRobot source declaring version 0.6.2; your editable checkout may differ.
