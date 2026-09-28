import json
import unittest
from unittest.mock import Mock, patch

import config
from imu_receiver import SerialImuReceiver, UdpImuReceiver
import run_teleop


def packet(sequence=1, **overrides):
    message = dict(seq=sequence, timestamp_ms=100, **dict.fromkeys(config.HUMAN_JOINTS, 10.0))
    message.update(overrides)
    return json.dumps(message).encode() + b"\r\n"


class SerialReceiverTests(unittest.TestCase):
    def setUp(self):
        self.device = Mock()
        self.device.in_waiting = 0
        self.device.read.return_value = b""
        module = Mock()
        module.Serial.return_value = self.device
        with patch.dict("sys.modules", {"serial": module}):
            self.receiver = SerialImuReceiver("/dev/test-pico")
        module.Serial.assert_called_once_with("/dev/test-pico", 115200, timeout=0, exclusive=True)
        self.addCleanup(self.receiver.close)

    def feed(self, data):
        self.device.in_waiting = len(data)
        self.device.read.return_value = data
        return self.receiver.read_latest()

    def test_fragmented_json_and_diagnostics(self):
        wire = packet()
        for byte in b"USB raw IMU output paused.\r\n" + wire[:-1]:
            self.assertIsNone(self.feed(bytes([byte])))
        result = self.feed(wire[-1:])
        self.assertEqual(result.seq, 1)
        self.assertEqual(result.source, "/dev/test-pico")
        self.assertEqual(self.receiver.invalid_packets, 0)

    def test_latest_valid_packet_and_invalid_data(self):
        result = self.feed(packet(1) + b"{invalid}\n" + packet(2) + packet(1))
        self.assertEqual(result.seq, 2)
        self.assertEqual(self.receiver.valid_packets, 2)
        self.assertEqual(self.receiver.invalid_packets, 2)
        self.assertIsNone(self.feed(b""))
        self.assertEqual(self.receiver.last_packet.seq, 2)

    def test_invalid_angles_do_not_refresh_timeout(self):
        self.assertTrue(self.receiver.timed_out())
        self.feed(packet(1))
        received_at = self.receiver.last_packet.received_at
        for value in (float("nan"), float("inf"), 1000, "bad", None):
            self.assertIsNone(self.feed(packet(2, elbow_flex=value)))
        with patch("imu_receiver.time.monotonic", return_value=received_at + config.PACKET_TIMEOUT_S + 1):
            self.assertTrue(self.receiver.timed_out())

    def test_overlong_line_recovers_at_next_newline(self):
        self.assertIsNone(self.feed(b"{" + b"x" * 3000))
        self.assertLessEqual(len(self.receiver._buffer), 2048)
        result = self.feed(packet(1) + packet(2))
        self.assertEqual(result.seq, 2)
        self.assertEqual(self.receiver.invalid_packets, 1)

    def test_disconnect_propagates_to_controller(self):
        self.device.read.side_effect = OSError("device unplugged")
        with self.assertRaisesRegex(OSError, "unplugged"):
            self.receiver.read_latest()

    def test_read_work_is_bounded(self):
        self.device.in_waiting = 100000
        self.receiver.read_latest()
        self.device.read.assert_called_with(8192)


class UdpReceiverTests(unittest.TestCase):
    def test_udp_still_validates_and_returns_latest(self):
        with patch("imu_receiver.socket.socket") as socket_factory:
            receiver = UdpImuReceiver()
        self.addCleanup(receiver.close)
        socket_factory.return_value.recvfrom.side_effect = [
            (packet(1), ("127.0.0.1", 5005)),
            (packet(2, wrist_roll=float("nan")), ("127.0.0.1", 5005)),
            (packet(3), ("127.0.0.1", 5005)),
            BlockingIOError(),
        ]
        self.assertEqual(receiver.read_latest().seq, 3)
        self.assertEqual(receiver.invalid_packets, 1)


class ControllerSerialTests(unittest.TestCase):
    def test_selected_port_and_disconnect_cleanup(self):
        with patch.object(config, "IMU_TRANSPORT", "serial"), \
             patch.object(config, "IMU_SERIAL_PORT", "/dev/default-pico"), \
             patch.object(config, "MODE", 1), \
             patch.object(config, "DRY_RUN", True), \
             patch("sys.argv", ["run_teleop.py", "--serial-port", "/dev/chosen-pico"]), \
             patch("run_teleop.SerialImuReceiver") as receiver_factory, \
             patch("run_teleop.RobotController") as robot_factory, \
             patch("run_teleop.Keyboard") as keyboard_factory:
            receiver = receiver_factory.return_value
            receiver.read_latest.side_effect = OSError("device unplugged")
            keyboard_factory.return_value.__enter__.return_value.read_key.return_value = None
            self.assertEqual(run_teleop.main(), 1)
            receiver_factory.assert_called_once_with("/dev/chosen-pico", config.IMU_SERIAL_BAUD)
            receiver.close.assert_called_once()
            robot_factory.assert_called_once_with(dry_run=True)
            robot_factory.return_value.send_action.assert_not_called()
            robot_factory.return_value.disconnect.assert_called_once()


if __name__ == "__main__":
    unittest.main()
