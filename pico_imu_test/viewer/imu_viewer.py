#!/usr/bin/env python3
"""USB serial triangle and 3D viewer. See README.md for formats and physical limitations."""
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


def triangle_geometry(points):
    """Flatten the triangle into its own plane, preserving angles.

    Coordinates are normalized only to keep the drawing comfortably sized.
    """
    def sub(a, b):
        return [x - y for x, y in zip(a, b)]

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def cross(a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

    def norm(a):
        return math.sqrt(dot(a, a))

    # Use the longest side as the baseline; retain original sensor labels.
    pairs = [(0, 1), (0, 2), (1, 2)]
    lengths = [norm(sub(points[j], points[i])) for i, j in pairs]
    scale = max(lengths)
    if scale < 1e-8:
        return [(0.0, 0.0)] * 3, None
    i, j = pairs[lengths.index(scale)]
    k = 3 - i - j
    baseline = sub(points[j], points[i])
    other = sub(points[k], points[i])
    x = dot(other, baseline) / (scale * scale)
    y = norm(cross(baseline, other)) / (scale * scale)
    flat = [None] * 3
    flat[i], flat[j], flat[k] = (0.0, 0.0), (1.0, 0.0), (x, y)
    # A collapsed triangle has no three well-defined interior angles.
    if min(lengths) < 1e-8 or y < 1e-6:
        return flat, None
    angles = []
    for corner in range(3):
        others = [n for n in range(3) if n != corner]
        a = sub(points[others[0]], points[corner])
        b = sub(points[others[1]], points[corner])
        angles.append(math.degrees(math.atan2(norm(cross(a, b)), dot(a, b))))
    return flat, angles


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", help="USB port, e.g. /dev/cu.usbmodem1101 or COM3")
    cli.add_argument("--baud", type=int, default=115200)
    cli.add_argument("--mode", choices=("accel", "position"), default="accel")
    cli.add_argument("--view", choices=("triangle", "3d"), default="triangle")
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
    ax = fig.add_subplot(111, projection="3d" if args.view == "3d" else None)
    fig.subplots_adjust(right=0.68, bottom=0.15)
    unit = "g" if args.mode == "accel" else "m"
    title = "Acceleration vectors (not physical positions)" if args.mode == "accel" else "Positions in a shared coordinate frame"
    if args.view == "triangle":
        ax.set_title("Triangle of XYZ readings" if args.mode == "accel" else "Triangle of positions")
        ax.set_aspect("equal", adjustable="box")
        ax.set(xlim=(-0.22, 1.22), ylim=(-0.22, 1.08))
        ax.axis("off")
        points = {i: ax.plot([], [], "o", color=c, label=f"IMU {i}", markersize=9)[0]
                  for i, c in enumerate(("tab:red", "tab:green", "tab:blue"), 1)}
        labels = {i: ax.annotate("", (0, 0), xytext=(8, 12), textcoords="offset points",
                                color=p.get_color(), fontsize=11) for i, p in points.items()}
    else:
        ax.set_title(title)
        ax.set(xlabel=f"X ({unit})", ylabel=f"Y ({unit})", zlabel=f"Z ({unit})")
        ax.set_box_aspect((1, 1, 1))
        points = {i: ax.plot([], [], [], "o-", color=c, label=f"IMU {i}", markersize=9)[0]
                  for i, c in enumerate(("tab:red", "tab:green", "tab:blue"), 1)}
    pairs = list(itertools.combinations(points, 2))
    edges = {pair: (ax.plot([], [], ":", color="gray")[0] if args.view == "triangle"
                   else ax.plot([], [], [], ":", color="gray")[0]) for pair in pairs}
    ax.legend(loc="upper left")
    panel = fig.text(0.70, 0.85, "", va="top", family="monospace", fontsize=10)
    status = fig.text(0.08, 0.04, "")
    fig.text(0.70, 0.12, ("Angles preserved; size normalized.\nPoints represent XYZ readings.\nData expires after 2 seconds."
                            if args.view == "triangle" else "Drag to rotate the view.\nData expires after 2 seconds."))
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
        if args.view == "triangle":
            flat, angles = triangle_geometry([active[i] for i in (1, 2, 3)]) if len(active) == 3 else (None, None)
            rows = []
            for i, point in points.items():
                xyz = active.get(i)
                rows.append(f"IMU {i}: no fresh data\n" if xyz is None else
                            f"IMU {i}\nX {xyz[0]: .3f} {unit}\nY {xyz[1]: .3f} {unit}\nZ {xyz[2]: .3f} {unit}\n")
                labels[i].set_visible(flat is not None)
                if flat is None:
                    point.set_data([], [])
                else:
                    x, y = flat[i - 1]
                    point.set_data([x], [y])
                    labels[i].xy = (x, y)
                    angle_text = f"{angles[i - 1]:.1f}°" if angles else "undefined"
                    labels[i].set_text(f"IMU {i}: {angle_text}")
            for (a, b), edge in edges.items():
                if flat is None:
                    edge.set_data([], [])
                else:
                    edge.set_data(*zip(flat[a - 1], flat[b - 1]))
            if flat is None:
                rows.append("Waiting for all three IMUs")
            elif angles is None:
                rows.append("Angles undefined:\npoints overlap or\nform a straight line.")
            else:
                rows.append("Triangle angles")
                rows.extend(f"IMU {i}: {angle:.1f}°" for i, angle in enumerate(angles, 1))
            panel.set_text("\n".join(rows))
        else:
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
