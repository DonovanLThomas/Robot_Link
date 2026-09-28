"""Configuration for IMU-to-SO-101 teleoperation.

Start with MODE = 1 or MODE = 2 and DRY_RUN = True. Do not enable live motion
until UDP packets, signs, gains, neutral calibration, and joint limits are
verified with the arm powered and supported safely.
"""

MODE = 1
DRY_RUN = True

UDP_BIND_IP = "0.0.0.0"
UDP_PORT = 5005

ROBOT_ID = "ladon"
ROBOT_PORT = ""  # Fill in after checking /dev/serial/by-id or lerobot discovery.

CONTROL_HZ = 30.0
DIAGNOSTICS_HZ = 5.0
PACKET_TIMEOUT_S = 0.5
IMPOSSIBLE_HUMAN_ANGLE_DEG = 360.0

HUMAN_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

ROBOT_JOINTS = HUMAN_JOINTS + ("gripper",)

# The Pico currently sends these standardized fields. Later, these source
# strings can point at relative-orientation channels such as
# "shoulder_relative.pitch" or "elbow_relative.roll".
SHOULDER_PAN_SOURCE = "shoulder_pan"
SHOULDER_LIFT_SOURCE = "shoulder_lift"
ELBOW_FLEX_SOURCE = "elbow_flex"
WRIST_FLEX_SOURCE = "wrist_flex"
WRIST_ROLL_SOURCE = "wrist_roll"

JOINT_SOURCES = {
    "shoulder_pan": SHOULDER_PAN_SOURCE,
    "shoulder_lift": SHOULDER_LIFT_SOURCE,
    "elbow_flex": ELBOW_FLEX_SOURCE,
    "wrist_flex": WRIST_FLEX_SOURCE,
    "wrist_roll": WRIST_ROLL_SOURCE,
}

SHOULDER_PAN_GAIN = 1.0
SHOULDER_LIFT_GAIN = 1.0
ELBOW_FLEX_GAIN = 1.0
WRIST_FLEX_GAIN = 1.0
WRIST_ROLL_GAIN = 1.0

GAINS = {
    "shoulder_pan": SHOULDER_PAN_GAIN,
    "shoulder_lift": SHOULDER_LIFT_GAIN,
    "elbow_flex": ELBOW_FLEX_GAIN,
    "wrist_flex": WRIST_FLEX_GAIN,
    "wrist_roll": WRIST_ROLL_GAIN,
}

SHOULDER_PAN_SIGN = 1
SHOULDER_LIFT_SIGN = 1
ELBOW_FLEX_SIGN = 1
WRIST_FLEX_SIGN = 1
WRIST_ROLL_SIGN = 1

SIGNS = {
    "shoulder_pan": SHOULDER_PAN_SIGN,
    "shoulder_lift": SHOULDER_LIFT_SIGN,
    "elbow_flex": ELBOW_FLEX_SIGN,
    "wrist_flex": WRIST_FLEX_SIGN,
    "wrist_roll": WRIST_ROLL_SIGN,
}

# Conservative placeholders. Replace these with limits from your calibrated
# SO-101 setup before live operation.
JOINT_LIMITS_VERIFIED = False
JOINT_LIMITS = {
    "shoulder_pan": (-30.0, 30.0),
    "shoulder_lift": (-30.0, 30.0),
    "elbow_flex": (-30.0, 30.0),
    "wrist_flex": (-30.0, 30.0),
    "wrist_roll": (-30.0, 30.0),
    "gripper": (0.0, 60.0),
}

MAX_STEP_DEG = 1.0
DEADBAND_DEG = {
    "shoulder_pan": 2.0,
    "shoulder_lift": 2.0,
    "elbow_flex": 2.0,
    "wrist_flex": 2.0,
    "wrist_roll": 2.0,
}

LOW_PASS_ALPHA = 0.25
FIXED_GRIPPER_POSITION = 30.0

DRY_RUN_ROBOT_START_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": FIXED_GRIPPER_POSITION,
}
