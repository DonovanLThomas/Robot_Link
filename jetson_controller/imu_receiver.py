"""USB serial and UDP receivers for Pico teleoperation packets."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import time
from typing import Any

import config


@dataclass(frozen=True)
class ImuPacket:
    seq: int
    timestamp_ms: int
    human: dict[str, float]
    received_at: float
    source: tuple[str, int] | str


class PacketValidationError(ValueError):
    """Raised when a payload is not safe to use as teleop input."""


class ImuReceiver:
    def __init__(self):
        self.last_seq: int | None = None
        self.last_packet: ImuPacket | None = None
        self.valid_packets = 0
        self.invalid_packets = 0
        self._rate_window: list[float] = []

    def last_packet_age(self) -> float | None:
        if self.last_packet is None:
            return None
        return time.monotonic() - self.last_packet.received_at

    def timed_out(self) -> bool:
        age = self.last_packet_age()
        return age is None or age > config.PACKET_TIMEOUT_S

    def packets_per_second(self) -> float:
        now = time.monotonic()
        self._rate_window = [stamp for stamp in self._rate_window if now - stamp <= 1.0]
        return float(len(self._rate_window))

    def _parse_packet(self, payload: bytes, source: tuple[str, int] | str) -> ImuPacket:
        if len(payload) > 2048:
            raise PacketValidationError("payload too large")
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise PacketValidationError(f"malformed JSON ({exc})") from exc
        if not isinstance(message, dict):
            raise PacketValidationError("JSON root must be an object")

        seq = self._required_int(message, "seq")
        timestamp_ms = self._required_int(message, "timestamp_ms")
        if self.last_seq is not None and seq <= self.last_seq:
            raise PacketValidationError(f"old sequence {seq}, last was {self.last_seq}")

        human = {
            joint: self._required_angle(message, joint)
            for joint in config.HUMAN_JOINTS
        }

        now = time.monotonic()
        self.last_seq = seq
        self.valid_packets += 1
        self._rate_window.append(now)
        return ImuPacket(seq=seq, timestamp_ms=timestamp_ms, human=human,
                         received_at=now, source=source)

    @staticmethod
    def _required_int(message: dict[str, Any], field: str) -> int:
        value = message.get(field)
        if not isinstance(value, int):
            raise PacketValidationError(f"missing or non-integer {field}")
        if value < 0:
            raise PacketValidationError(f"{field} must be non-negative")
        return value

    @staticmethod
    def _required_angle(message: dict[str, Any], field: str) -> float:
        value = message.get(field)
        if not isinstance(value, (int, float)):
            raise PacketValidationError(f"missing or non-numeric {field}")
        value = float(value)
        if not math.isfinite(value):
            raise PacketValidationError(f"{field} is not finite")
        if abs(value) > config.IMPOSSIBLE_HUMAN_ANGLE_DEG:
            raise PacketValidationError(f"{field}={value:.1f} outside plausible range")
        return value


class UdpImuReceiver(ImuReceiver):
    def __init__(self, bind_ip: str = config.UDP_BIND_IP, port: int = config.UDP_PORT):
        super().__init__()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind_ip, port))
        self.socket.setblocking(False)

    def close(self) -> None:
        self.socket.close()

    def read_latest(self) -> ImuPacket | None:
        """Drain pending UDP datagrams and return the newest valid packet."""
        newest = None
        while True:
            try:
                payload, source = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                newest = self._parse_packet(payload, source)
            except PacketValidationError as exc:
                self.invalid_packets += 1
                print(f"Rejected IMU packet: {exc}")
        if newest is not None:
            self.last_packet = newest
        return newest


class SerialImuReceiver(ImuReceiver):
    def __init__(self, port: str = config.IMU_SERIAL_PORT,
                 baudrate: int = config.IMU_SERIAL_BAUD):
        super().__init__()
        import serial

        self.port = port
        self.serial = serial.Serial(port, baudrate, timeout=0, exclusive=True)
        self._buffer = bytearray()
        self._discard_line = False

    def close(self) -> None:
        self.serial.close()

    def read_latest(self) -> ImuPacket | None:
        """Read bounded, nonblocking USB data; preserve fragmented JSON lines."""
        newest = None
        chunk = self.serial.read(min(self.serial.in_waiting, 8192))
        for byte in chunk:
            if byte == 10:
                payload = bytes(self._buffer).strip()
                self._buffer.clear()
                discard = self._discard_line
                self._discard_line = False
                if discard or not payload.startswith(b"{"):
                    continue
                try:
                    newest = self._parse_packet(payload, self.port)
                except PacketValidationError as exc:
                    self.invalid_packets += 1
                    print(f"Rejected IMU packet: {exc}")
            elif not self._discard_line:
                self._buffer.append(byte)
                if len(self._buffer) > 2048:
                    self._buffer.clear()
                    self._discard_line = True
                    self.invalid_packets += 1
        if newest is not None:
            self.last_packet = newest
        return newest
