# Three MPU6050s over USB and Wi-Fi

C firmware for a Raspberry Pi Pico 2 W reads three MPU6050s through a TCA9548A
multiplexer. Human-readable records are printed about twice per second. Each
record contains acceleration in g, gyroscope readings in degrees/second, and
temperature in Celsius. The same records can go to USB serial and one connected
TCP client. Acquisition starts without a USB terminal, so the board can run from
a USB power supply.

An optional UDP teleoperation stream can also send standardized human joint
fields to the Jetson at about 30-50 Hz. This UDP stream is separate from the
existing USB/TCP viewer stream.

The [Python viewer](viewer/README.md) displays the acceleration readings as a
triangle or 3D vectors and can save the received stream. These readings are not
physical positions or distances between the sensors.

## Wiring

| Pico | TCA9548A |
| --- | --- |
| 3V3 OUT | VCC |
| GND | GND |
| GP0 (physical pin 1) | SDA |
| GP1 (physical pin 2) | SCL |

Connect IMUs 1–3 to mux channels 0–2 respectively (SD0/SC0, SD1/SC1,
SD2/SC2). All modules share ground and compatible 3.3 V power.

- Mux A0/A1/A2 must be LOW for address `0x70`; RESET must stay HIGH.
- Each MPU6050 AD0 must be LOW for address `0x68`.
- SDA/SCL need suitable pull-ups to 3.3 V on the upstream and downstream buses.
  Check the breakout resistors; internal Pico pull-ups alone are weak.
- Sensor INT, XDA and XCL are unused.

The firmware wakes each sensor and reads its default +/-2 g and +/-250 deg/s
ranges. It does not check WHO_AM_I or automatically detect other addresses.
Sensors are read sequentially, not simultaneously. Failed initialization requires
a restart after fixing wiring; runtime read errors are retried each cycle.

## Configure Wi-Fi

From this directory:

```sh
cp wifi_config.example.h wifi_config.h
```

Edit `wifi_config.h` locally with your 2.4 GHz Wi-Fi SSID and password (C string
literals). This file is ignored by Git. Do not put credentials in the example.
The default TCP port is 4242. With no local configuration, the firmware builds
and runs with Wi-Fi disabled and USB available. Credentials are embedded in the
compiled firmware, so keep configured UF2 files private.

To enable Jetson teleoperation packets in your local ignored `wifi_config.h`:

```c
#define TELEOP_UDP_ENABLED 1
#define JETSON_IP "192.168.x.x"
#define JETSON_UDP_PORT 5005
#define TELEOP_SEND_PERIOD_MS 25
```

The current UDP JSON packet is:

```json
{
  "seq": 1234,
  "timestamp_ms": 12345678,
  "shoulder_pan": 0.0,
  "shoulder_lift": 12.3,
  "elbow_flex": 65.4,
  "wrist_flex": -3.2,
  "wrist_roll": 8.7
}
```

The first implementation derives these fields from the existing accel readings:
`shoulder_pan` is held at zero because an MPU6050 accelerometer/gyro alone does
not provide reliable yaw, `shoulder_lift` and wrist fields use relative accel
tilt estimates, and `elbow_flex` preserves the existing triangle-angle
experiment at IMU 2. Treat these as a stable networking/mapping contract, not
finished orientation fusion.

Use a normal WPA2-compatible home network or hotspot. Put the laptop on the
same LAN; guest/client isolation can prevent connections. The laptop can be on
a different Wi-Fi band if both bands share the LAN. No internet is required.

## Build for Pico 2 W

Install Pico SDK 2.x with its submodules, CMake, a complete Arm embedded GCC
toolchain and picotool (the Pico VS Code extension can install these).

```sh
cmake -S . -B build-pico2-w -DPICO_BOARD=pico2_w \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_TOOLCHAIN_PATH=/path/to/arm-toolchain
cmake --build build-pico2-w -j4
```

This repository defaults new build directories to `pico2_w`. Existing build
folders retain their board selection: `pico2` is not the wireless board target.
Explicit `pico` and `pico2` builds remain USB-only. On this laptop the configured
`build-pico2-w` directory uses SDK 2.2.0 and Arm GCC 13.2.1. After editing Wi-Fi
credentials, rebuild that directory with the second command above.

## Flash and connect

1. Hold BOOTSEL while connecting the board with a USB data cable.
2. Copy `build-pico2-w/pico_imu_test.uf2` to the drive that appears.
3. Open USB serial at 115200 baud with DTR enabled. Reset the board while the
   terminal is open if you missed startup output, or find its IP in your router's
   DHCP client list. A successful connection prints `Wi-Fi ready: <IP> TCP port 4242`.
4. Start the viewer from its directory:

   ```sh
   python3 imu_viewer.py --host 192.168.1.123 --record readings.txt
   ```

   Replace the example IP with the Pico's address. Install viewer dependencies
   first as described in [viewer/README.md](viewer/README.md).
5. For USB instead, use `python3 imu_viewer.py --port /dev/cu.usbmodem...`.
   Close other programs reading that USB port first.

Once configured, power the Pico from a USB supply to use it without the laptop
cable. A DHCP reservation in your router keeps its IP stable.

## USB setup pause

On a wireless board, USB IMU output starts **paused** so Wi-Fi messages stay
visible. Opening the USB serial monitor prints the current network status,
including the IP if connected, even if you missed startup. Send these commands
(type the letter and use Send/Enter if your monitor requires it):

- `w`: show Wi-Fi status and IP address.
- `s`: resume USB IMU readings.
- `p`: pause USB IMU readings again.

Sensor acquisition and Wi-Fi streaming continue during a USB output pause.
The Python USB viewer sends `s` automatically when it opens the port.
Non-wireless Pico builds retain their default automatic USB streaming.

## Hotspot troubleshooting

The USB monitor reports network-not-found, authentication failure, and waiting
for a DHCP address separately. Send `w` to repeat the current status. A failure
code is a diagnostic clue, not proof of a particular cause.

For an iPhone, enable Allow Others to Join and Maximize Compatibility (if
available) under Settings > Personal Hotspot. Keep that screen open while
connecting. Maximize Compatibility enables 2.4 GHz and WPA2 Personal.
For Android, choose a 2.4 GHz hotspot band and WPA2 security if those options
are available. The Pico 2 W cannot connect to a 5 GHz-only hotspot.
Make sure the hotspot name/password match `wifi_config.h` exactly, then rebuild
and flash after any credential changes. Connect the laptop to the same hotspot.

References: [Apple hotspot troubleshooting](https://support.apple.com/en-us/119837),
[Apple hotspot compatibility](https://support.apple.com/en-ca/guide/security/secfd166f620/web).

## Streaming behavior

Wi-Fi connects asynchronously and retries every 30 seconds while offline. USB
and sensor acquisition continue during connection attempts. The main loop
services the Pico SDK's polling network stack frequently. One TCP viewer is
accepted at a time; additional connections are closed. A slow client whose
send buffer fills is disconnected instead of blocking sensor acquisition.
The viewer expires readings after two seconds and reports a closed connection;
restart it to reconnect. A silent network outage may initially show stale data
until TCP detects the failure. There is no offline storage or replay: samples
acquired while disconnected are not recorded on the laptop.

`--record` appends raw received text, including acceleration, gyro, temperature,
and errors. It does not add timestamps or convert the stream to CSV. TCP carries
plain text without authentication or encryption; use a trusted local network.

Firmware compilation and desktop loopback tests do not verify physical sensor
wiring, RF performance, or your access point. Those need a board test.

Reference: [Pico SDK networking and polling API](https://www.raspberrypi.com/documentation/pico-sdk/networking.html).
