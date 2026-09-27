# Three IMUs: live triangle angles over USB

This desktop Python viewer reads the output of the current `../main.c` directly.
No firmware change is needed. By default, the three XYZ readings form a 2D
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
```

Replace the port with the one listed for your board. Close the Serial Monitor
before opening this viewer: only one program should read the USB port.
Default baud is 115200; override with `--baud 9600` if needed. Pico USB CDC
does not use a physical UART baud rate. On Windows use `python` and activate
with `.venv\Scripts\activate`; a typical port argument is `--port COM3`.
Use a Python installation with desktop GUI support (Tk or Qt).
After unplugging/reconnecting, restart the viewer and recheck the port name.

The accepted firmware output is:

```text
IMU 1 - Channel 0
Accel: X=  0.010 Y= -0.020 Z=  1.000 g
Gyro : X=   0.00 Y=   0.00 Z=   0.00 deg/s
```

Repeat for IMU 2 and 3. Startup/temperature/gyro messages are ignored.
The older raw-integer format described in the parent README is not accepted;
this viewer targets the actual current `main.c` output.

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
