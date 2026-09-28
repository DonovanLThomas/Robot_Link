# Robot Link

Robot Link is a hardware teleoperation workspace that connects a Raspberry Pi
Pico 2 W IMU rig to a Jetson-side SO-101 robot arm controller.

The repository is split by runtime target:

| Path | Purpose |
| --- | --- |
| `jetson_controller/` | Python controller that receives Pico IMU packets, maps them to robot joints, applies safety limits, and runs dry or live teleoperation modes. |
| `pico_imu_test/` | Pico 2 W firmware for reading three MPU6050 sensors through a TCA9548A mux and streaming teleoperation JSON over USB, with optional Wi-Fi/UDP support. |
| `pico_imu_test/viewer/` | Desktop Python viewer for inspecting raw IMU acceleration records over USB or TCP. |
| `pico_mac/` | Small Pico firmware used only to print the board MAC address. |

Start with the target-specific docs:

- [Jetson controller setup](jetson_controller/README.md)
- [Live mode checklist](jetson_controller/LIVE_MODE.md)
- [Pico firmware setup](pico_imu_test/README.md)
- [IMU viewer setup](pico_imu_test/viewer/README.md)

## Repository Hygiene

Generated files, local build directories, local credentials, Python bytecode, and
editor state are ignored at the repository root. Keep checked-in files focused
on source code, wiring/setup documentation, examples, and tests.

Do not commit `pico_imu_test/wifi_config.h`; use
`pico_imu_test/wifi_config.example.h` as the tracked template.

## Commit Messages

Use short, specific commit subjects that say what changed and why it matters to
this hardware pipeline. Prefer a scope when the change is local to one part of
the project:

```text
jetson: reject stale IMU packets before live commands
pico: stream teleop JSON when USB serial opens
viewer: add position-mode parser tests
docs: document swapped wrist roll and gripper IDs
repo: ignore generated build and bytecode files
```

Good commit subjects make the need clear. Avoid vague subjects like `update`,
`fix`, or `changes`.

