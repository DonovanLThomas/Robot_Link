# Three IMUs: live triangle angles over USB or Wi-Fi

This desktop Python viewer reads the output of the current `../main.c` directly.
Use the firmware Wi-Fi configuration in [the parent README](../README.md) for wireless operation. By default, the three XYZ readings form a 2D
triangle with a labelled angle at each IMU. Flattening preserves the 3D triangle
angles; drawing size is normalized. These are angles between reading endpoints,
not sensor orientations or angles between physical sensor locations.
Overlapping or collinear points show undefined angles. The triangle disappears
when any sensor is missing or its reading is older than two seconds.
Use `--view 3d` for the original rotatable vectors and endpoint differences.

**The current firmware sends acceleration in g, not position.** The plotted
separation is an acceleration-vector difference, not the physical distance
between sensors. Gravity, sensor orientation, and bias affect these values;
each sensor measures in its own local axes. Three IMUs alone do not provide
reliable absolute positions or distances. Double integration drifts and needs
orientation/gravity compensation, calibration, initial conditions, and external
position constraints. For a robot arm, joint angles plus known link lengths can
instead be used with forward kinematics.

## Run (macOS / Linux)

From this `viewer` directory:

```sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 imu_viewer.py --demo
python3 imu_viewer.py --demo --view 3d
python3 imu_viewer.py --list-ports
python3 imu_viewer.py --port /dev/cu.usbmodem1101
python3 imu_viewer.py --host 192.168.1.123
python3 imu_viewer.py --host 192.168.1.123 --view 3d --record readings.txt
```

Replace the port with the one listed for your board. Close the Serial Monitor
before opening this viewer: only one program should read the USB port.
The viewer automatically resumes paused USB output when connecting.
Default baud is 115200; override with `--baud 9600` if needed. Pico USB CDC
does not use a physical UART baud rate. On Windows use `python` and activate
with `.venv\Scripts\activate`; a typical port argument is `--port COM3`.
Use a Python installation with desktop GUI support (Tk or Qt).
After unplugging/reconnecting, restart the viewer and recheck the port name.

For Wi-Fi, replace the example address with the Pico's DHCP IP. The default TCP
port is 4242; use `--tcp-port` if changed in firmware. The laptop and Pico must
be on the same reachable LAN. Only one TCP client is supported. Restart the
viewer after a disconnect; no samples are replayed from an offline period.
USB and Wi-Fi use the same parser and two-second stale-reading timeout.

`--record FILE` appends the raw incoming bytes for either USB or Wi-Fi, including
gyro and temperature lines ignored by the plot. It flushes each received chunk.
It does not add timestamps, and repeated runs append to the same file.

The accepted firmware output is:

```text
IMU 1 - Channel 0
Accel: X=  0.010 Y= -0.020 Z=  1.000 g
Gyro : X=   0.00 Y=   0.00 Z=   0.00 deg/s
```

Repeat for IMU 2 and 3. Startup/temperature/gyro messages are ignored.
Older raw-integer firmware output is not accepted.

## Actual position input

If an external tracking or kinematics system supplies XYZ **positions in meters
in the same coordinate frame**, send these newline-terminated serial lines:

```text
POS,1,0.0,0.0,0.0
POS,2,0.3,0.0,0.0
POS,3,0.3,0.4,0.0
```

Run `python3 imu_viewer.py --port PORT --mode position --view 3d`. The viewer then labels
the axes in meters and reports the three Euclidean distances (0.3, 0.5, 0.4 m
for the example). **Do not put acceleration readings in POS lines.**
Try `python3 imu_viewer.py --demo --mode position` for simulated positions.
Sensors are updated as readings arrive; their timestamps are not synchronized.

Implementation references: [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)
and [Matplotlib animation](https://matplotlib.org/stable/users/explain/animations/animations.html).
