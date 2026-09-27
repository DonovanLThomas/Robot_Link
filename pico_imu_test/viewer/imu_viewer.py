#!/usr/bin/env python3
"""USB serial 3D viewer. See README.md for formats and physical limitations."""
import argparse
import itertools
import math
import re
import time

NUMBER = r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
HEADER = re.compile(r"IMU ([123])(?:\s*-\s*Channel \d+|:)$")
ACCEL = re.compile(r"Accel:\s*X=\s*" + NUMBER + r"\s+Y=\s*" + NUMBER + r"\s+Z=\s*" + NUMBER + r"\s+g$")


class Parser:
    def __init__(self, mode):
        self.mode = mode
        self.sensor = None

    def feed(self, line):
        """Return (sensor ID, XYZ or None for error), ignoring diagnostics."""
        line = line.strip()
        if self.mode == "position":
            parts = line.split(",")
            if len(parts) != 5 or parts[0] != "POS" or parts[1] not in ("1", "2", "3"):
                return None
            try:
                xyz = tuple(float(v) for v in parts[2:])
            except ValueError:
                return None
            return (int(parts[1]), xyz) if all(map(math.isfinite, xyz)) else None
        header = HEADER.fullmatch(line)
        if header:
            self.sensor = int(header[1])
        elif line.startswith("ERROR:"):
            sensor, self.sensor = self.sensor, None
            return (sensor, None) if sensor else None
        elif self.sensor is not None:
            sample = ACCEL.fullmatch(line)
            if sample:
                sensor, self.sensor = self.sensor, None
                xyz = tuple(float(v) for v in sample.groups())
                return (sensor, xyz) if all(map(math.isfinite, xyz)) else None
        return None


class LineBuffer:
    """Keep partial USB reads until newline; discard overlong garbage."""
    def __init__(self):
        self.pending = bytearray()
        self.discard = False

    def feed(self, data):
        for byte in data:
            if byte == 10:
                if not self.discard:
                    yield self.pending.decode("ascii", errors="replace")
                self.pending.clear()
                self.discard = False
            elif not self.discard:
                self.pending.append(byte)
                if len(self.pending) > 1024:
                    self.pending.clear()
                    self.discard = True


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", help="USB port, e.g. /dev/cu.usbmodem1101 or COM3")
    cli.add_argument("--baud", type=int, default=115200)
    cli.add_argument("--mode", choices=("accel", "position"), default="accel")
    cli.add_argument("--list-ports", action="store_true")
    cli.add_argument("--demo", action="store_true", help="Show simulated data without hardware")
    args = cli.parse_args()
    if not args.demo:
        try:
            import serial
            from serial.tools import list_ports
        except ImportError:
            cli.exit(1, "Install dependencies: python3 -m pip install -r requirements.txt\n")
        if args.list_ports:
            ports = list(list_ports.comports())
            for port in ports:
                print(f"{port.device}: {port.description}")
            if not ports:
                print("No serial ports found. Check the USB data cable.")
            return
        if not args.port:
            cli.error("Use --list-ports then --port PORT, or use --demo")
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        cli.exit(1, "Install dependencies: python3 -m pip install -r requirements.txt\n")

    connection = None
    if not args.demo:
        try:
            connection = serial.Serial(args.port, args.baud, timeout=0)
            connection.dtr = True
        except (serial.SerialException, OSError) as exc:
            if connection:
                connection.close()
            cli.exit(1, f"Cannot open USB serial: {exc}\nClose your Serial Monitor first.\n")

    parser, buffer = Parser(args.mode), LineBuffer()
    samples = {}
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(right=0.68, bottom=0.15)
    unit = "g" if args.mode == "accel" else "m"
    title = "Acceleration vectors (not physical positions)" if args.mode == "accel" else "Positions in a shared coordinate frame"
    ax.set_title(title)
    ax.set(xlabel=f"X ({unit})", ylabel=f"Y ({unit})", zlabel=f"Z ({unit})")
    ax.set_box_aspect((1, 1, 1))
    points = {i: ax.plot([], [], [], "o-", color=c, label=f"IMU {i}", markersize=9)[0]
              for i, c in enumerate(("tab:red", "tab:green", "tab:blue"), 1)}
    pairs = list(itertools.combinations(points, 2))
    edges = {pair: ax.plot([], [], [], ":", color="gray")[0] for pair in pairs}
    ax.legend(loc="upper left")
    panel = fig.text(0.70, 0.85, "", va="top", family="monospace", fontsize=10)
    status = fig.text(0.08, 0.04, "")
    fig.text(0.70, 0.12, "Drag to rotate the view.\nData expires after 2 seconds.")
    error = None
    limit = 2.0 if args.mode == "accel" else 1.0

    def update(_):
        nonlocal error, limit
        now = time.monotonic()
        if args.demo:
            for i in points:
                samples[i] = ((math.sin(now + i), math.cos(now + i), 0.3 * i), now)
        elif error is None:
            try:
                for line in buffer.feed(connection.read(min(connection.in_waiting, 65536))):
                    result = parser.feed(line)
                    if result:
                        sensor, xyz = result
                        if xyz is None:
                            samples.pop(sensor, None)
                        else:
                            samples[sensor] = (xyz, now)
            except (serial.SerialException, OSError) as exc:
                error = str(exc)
                samples.clear()
        active = {i: xyz for i, (xyz, stamp) in samples.items() if now - stamp < 2}
        rows = []
        for i, point in points.items():
            xyz = active.get(i)
            if xyz is None:
                point.set_data_3d([], [], [])
                rows.append(f"IMU {i}: no fresh data\n")
            else:
                coordinates = [(0, v) for v in xyz] if args.mode == "accel" else [(v,) for v in xyz]
                point.set_data_3d(*coordinates)
                rows.append(f"IMU {i}\nX {xyz[0]: .3f} {unit}\nY {xyz[1]: .3f} {unit}\nZ {xyz[2]: .3f} {unit}\n")
        rows.append("Vector differences (g)" if args.mode == "accel" else "Distances (m)")
        for a, b in pairs:
            if a in active and b in active:
                edges[a, b].set_data_3d(*zip(active[a], active[b]))
                rows.append(f"{a} - {b}: {math.dist(active[a], active[b]):.3f}")
            else:
                edges[a, b].set_data_3d([], [], [])
                rows.append(f"{a} - {b}: --")
        panel.set_text("\n".join(rows))
        if active:
            limit = max(limit, max(abs(v) for xyz in active.values() for v in xyz) * 1.15)
        ax.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit))
        status.set_text(f"Disconnected: {error}. Restart to reconnect." if error else
                        ("DEMO — simulated data" if args.demo else f"USB: {args.port} | {len(active)}/3 fresh IMUs"))

    try:
        animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
        plt.show()
    finally:
        if connection:
            connection.close()


if __name__ == "__main__":
    main()
