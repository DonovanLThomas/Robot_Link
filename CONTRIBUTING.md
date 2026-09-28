# Contributing

This project spans firmware, desktop tooling, and robot-control code. Keep each
change focused on one behavior or documentation need so hardware regressions are
easy to review.

## Commit Message Guide

Use this format for most commits:

```text
scope: action and reason
```

Recommended scopes:

| Scope | Use for |
| --- | --- |
| `jetson` | Robot controller, joint mapping, safety checks, serial/UDP receiving, and live-mode behavior. |
| `pico` | Pico IMU firmware, CMake settings, USB serial output, Wi-Fi, UDP, and sensor acquisition. |
| `viewer` | Desktop IMU viewer, parsing, visualization, and viewer tests. |
| `pico_mac` | MAC-address helper firmware only. |
| `docs` | Setup guides, wiring notes, operating procedures, and troubleshooting. |
| `repo` | Repository organization, ignore rules, editor settings, and other project hygiene. |

Examples:

```text
jetson: clamp gripper targets before sending commands
pico: disable Wi-Fi by default for USB teleop
viewer: record raw serial chunks without blocking plots
docs: add Jetson live-mode startup checklist
repo: ignore generated CMake and Python artifacts
```

If a commit needs more explanation, add a body after a blank line:

```text
jetson: hold last target on IMU timeout

Live mode should stop sending new targets when packets go stale so the arm does
not chase old sensor state after a cable disconnect.
```

## Before Committing

- Keep local credentials out of Git. `pico_imu_test/wifi_config.h` is ignored;
  update `wifi_config.example.h` only when the template changes.
- Keep generated files out of Git, including `__pycache__/`, `.DS_Store`,
  virtual environments, logs, and CMake build output.
- Run the narrowest relevant tests or build checks for the area you changed.

