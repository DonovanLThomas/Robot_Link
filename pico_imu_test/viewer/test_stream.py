"""Transport regression tests; run with python3 -m unittest discover -s viewer."""
import socket
import unittest
from unittest.mock import patch

from imu_viewer import LineBuffer, Parser, TCPStream


class StreamTests(unittest.TestCase):
    def test_fragmented_records_and_sensor_errors(self):
        wire = (b'Wi-Fi ready: 192.168.1.2 TCP port 4242\r\n'
                b'IMU 1 - Channel 0\r\nAccel: X= 0.010 Y= -0.020 Z= 1.000 g\r\n'
                b'Gyro : X= 0.00 Y= 0.00 Z= 0.00 deg/s\n'
                b'IMU 2 - Channel 1\nERROR: Channel 1 failed initialization or could not be read.\n'
                b'IMU 3 - Channel 2\nAccel: X= 0.5 Y= 0.0 Z= -1.0 g\n')
        # Any TCP packet boundary, including inside a header or numeric value.
        for split in range(len(wire) + 1):
            parser, buffer = Parser('accel'), LineBuffer()
            actual = []
            for chunk in (wire[:split], wire[split:]):
                for line in buffer.feed(chunk):
                    result = parser.feed(line)
                    if result:
                        actual.append(result)
            self.assertEqual(actual, [(1, (0.01, -0.02, 1.0)), (2, None),
                                      (3, (0.5, 0.0, -1.0))])

    def test_idle_data_and_disconnect(self):
        receiver, sender = socket.socketpair()
        self.addCleanup(sender.close)
        with patch('imu_viewer.socket.create_connection', return_value=receiver) as connect:
            stream = TCPStream('pico.local', 4242)
        self.addCleanup(stream.close)
        connect.assert_called_once_with(('pico.local', 4242), timeout=5)
        self.assertEqual(stream.read(), b'')
        sender.sendall(b'IMU 1 - Channel 0\n')
        self.assertEqual(stream.read(), b'IMU 1 - Channel 0\n')
        sender.close()
        with self.assertRaisesRegex(OSError, 'closed'):
            stream.read()

    def test_overlong_line_recovers(self):
        buffer = LineBuffer()
        self.assertEqual(list(buffer.feed(b'x' * 1025 + b'\nIMU 1 - Channel 0\n')),
                         ['IMU 1 - Channel 0'])


if __name__ == '__main__':
    unittest.main()
