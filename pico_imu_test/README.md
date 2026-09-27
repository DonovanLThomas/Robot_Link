# Three MPU6050s through a TCA9548A

## Live 3D USB viewer

See [viewer/README.md](viewer/README.md) for the Python 3D viewer that reads the
current `main.c` output, plus setup and demo commands. The current code prints
acceleration in g and gyro in degrees/second; some older firmware details below
describe a previous raw-integer version. Acceleration plots do not measure
physical distances between IMUs. The viewer also accepts externally computed
XYZ positions for displaying distances in meters.

A small C program for the Raspberry Pi Pico SDK. It reads raw accelerometer
and gyroscope integers from mux channels 0, 1 and 2, then prints them over USB
serial about twice per second. Readings scroll so diagnostic messages stay visible.
No robot control, motion mapping, calibration or software filtering is included.

## Wiring

Your stated wiring matches this program:

| Pico | TCA9548A |
| --- | --- |
| 3V3 OUT | VCC |
| GND | GND |
| GP0 (physical pin 1) | SDA |
| GP1 (physical pin 2) | SCL |

| Sensor | SDA | SCL |
| --- | --- | --- |
| IMU 1 | SD0 | SC0 |
| IMU 2 | SD1 | SC1 |
| IMU 3 | SD2 | SC2 |

All modules share 3.3 V and ground. Check that your particular IMU breakout's
power input supports 3.3 V; onboard regulator arrangements vary.

- TCA9548A A0/A1/A2 must be LOW for address `0x70`. Many breakouts already have
  pull-downs. If strapped differently, change `MUX_ADDRESS` in `main.c`.
- The mux RESET pin must stay HIGH, normally through a pull-up to 3.3 V already
  fitted on the breakout. Do not leave it floating if your board lacks one.
- Each IMU's AD0 must have a defined level. LOW selects `0x68`, HIGH selects
  `0x69`; the program checks both. All three can use `0x68`.
- SDA and SCL need pull-ups to 3.3 V on the upstream bus and each used downstream
  channel. Check existing breakout resistors first; if absent, 4.7 kOhm per line
  is a typical starting point for short 100 kHz wiring. Pico internal pull-ups
  alone are weak. Keep all bus pull-ups at 3.3 V for this setup.
- INT, XDA and XCL on the IMUs are not used.

## Build

Install the Pico SDK (2.x for Pico 2), its submodules, CMake, and a complete Arm
embedded GCC toolchain including newlib. The Raspberry Pi Pico VS Code extension
can install these. The SDK's normal extra-output support uses picotool to create
the UF2. Set the SDK path to your installation:

```sh
cd /Users/dono_1k/Collab/pico_imu_test
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build-pico -DPICO_BOARD=pico
cmake --build build-pico -j4
```

For Pico 2, use a separate build directory:

```sh
cmake -S . -B build-pico2 -DPICO_BOARD=pico2
cmake --build build-pico2 -j4
```

The result is `pico_imu_test.uf2` in the selected build directory. Use the build
for your exact board. If GCC reports missing `nosys.specs`, select a complete
Arm embedded toolchain with `-DPICO_TOOLCHAIN_PATH=/path/to/toolchain` in a fresh
build directory; this indicates a toolchain installation issue.

## Flash and view

Both board builds were successfully compiled locally with Pico SDK 2.2.0 and
Arm GCC 13.2.1. Ready-to-flash files are in `build-pico-sdk/pico_imu_test.uf2`
and `build-pico2-sdk/pico_imu_test.uf2`. These have not been tested on hardware.
To rebuild those already configured directories on this computer:

```sh
cmake --build build-pico-sdk -j4
cmake --build build-pico2-sdk -j4
```

1. Hold BOOTSEL while connecting the Pico to your computer with a USB data cable.
2. Copy the appropriate `pico_imu_test.uf2` onto the USB drive that appears.
3. After it reboots, open its USB serial port in a serial monitor. Choose 115200
   baud if asked (USB CDC does not use a physical UART baud rate).
4. The program waits for the serial terminal connection before starting detection.
   On macOS the port is usually `/dev/cu.usbmodem...`; Linux usually
   `/dev/ttyACM0`; Windows uses a COM port. Disable hardware flow control and
   enable DTR if your terminal requires it for the USB connection.

Startup should show:

```text
TCA9548A detected
IMU 1 detected on channel 0 (address 0x68)
IMU 2 detected on channel 1 (address 0x68)
IMU 3 detected on channel 2 (address 0x68)
```

Each update prints six labeled integers per IMU:

```text
IMU 1:
  Accel X: 120
  Accel Y: -85
  Accel Z: 16300
  Gyro X:  15
  Gyro Y:  -20
  Gyro Z:  8
```

These example numbers are illustrative, not measured. At rest, the gyro should
be near zero with some bias/noise. Acceleration includes gravity: with an axis
vertical it should be roughly +16384 or -16384 at the configured +/-2 g range.
Tilt or rotate one sensor at a time; its readings should respond independently.
The sensors are polled sequentially, not sampled simultaneously.

## How it works and errors

`main.c` sets up I2C0 at 100 kHz on GP0/GP1. It disables all mux channels and
checks control-byte readback. Selecting a channel writes `1 << channel` to the
mux: `0x01`, `0x02`, or `0x04`. Only one sensor is connected to the upstream bus
at a time, avoiding address collisions.

For each channel, the program reads WHO_AM_I, resets and wakes the MPU6050,
and selects +/-2 g and +/-250 degrees/s ranges. A 14-byte burst starting at
`0x3B` contains acceleration, temperature and gyro registers. Temperature is
skipped; each axis is decoded as a signed 16-bit value. The device's default
internal signal path is retained; no additional filtering is configured.

Every I2C operation has a timeout and its result is checked. A missing mux is
retried once per second. An IMU missing or failing initialization is marked
unavailable while other sensors continue; fix wiring with power off, then restart
to repeat discovery. Runtime read errors are reported and retried next update.
Old or uninitialized data is never printed as a successful reading.

This targets the MPU6050 register map and WHO_AM_I value `0x68`. A module sold
as “MPU6050-style” may contain a different chip: if an identity mismatch is
reported, check the actual chip before changing the identity check. A downstream
short holding the bus LOW may affect all sensors and require a mux hardware
reset or power cycle; timeouts do not repair electrical faults.

## References

- [TI TCA9548A datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf)
- [TDK/InvenSense MPU6000/MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)
- [Raspberry Pi SDK setup](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html)
- [Pico SDK I2C API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_i2c)
