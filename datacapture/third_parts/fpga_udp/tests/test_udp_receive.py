"""Exercise the actual compiled fpga_udp receiver on Linux loopback."""
import socket
import threading
import time
import unittest

import numpy as np
import fpga_udp


class NativeSocketTests(unittest.TestCase):
    size = 1466

    def setUp(self):
        self.assertGreaterEqual(getattr(fpga_udp, "read_data_udp_api_version", 0), 2)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", 0))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = self.rx.getsockname()

    def tearDown(self):
        self.rx.close()
        self.tx.close()

    def receive(self, count=8, timeout=.04):
        return fpga_udp.read_data_udp(self.rx.fileno(), count, self.size, timeout)

    def send(self, value, size=None):
        payload = bytes([value]) * (self.size if size is None else size)
        self.tx.sendto(payload, self.address)
        return payload

    def test_empty_timeout_has_no_unreceived_bytes(self):
        start = time.monotonic()
        result = self.receive()
        elapsed = time.monotonic() - start
        self.assertEqual(result.dtype, np.uint8)
        self.assertEqual(result.shape, (0,))
        self.assertGreaterEqual(elapsed, .02)
        self.assertLess(elapsed, .3)

    def test_partial_batch_contains_exact_payload(self):
        expected = self.send(11) + self.send(22)
        result = self.receive()
        self.assertEqual(result.tobytes(), expected)
        self.assertEqual(self.receive(timeout=.01).size, 0)

    def test_full_batch_keeps_buffer_alive_after_next_call(self):
        expected = b"".join(self.send(value) for value in range(4))
        result = self.receive(count=4)
        self.send(91)
        next_result = self.receive(count=1)
        self.assertEqual(next_result.tobytes(), bytes([91]) * self.size)
        self.assertEqual(result.tobytes(), expected)
        self.assertTrue(result.flags.c_contiguous)

    def test_wait_releases_gil_for_python_timer(self):
        fired = []
        timer = threading.Timer(.03, lambda: fired.append(time.monotonic()))
        timer.start()
        try:
            self.receive(timeout=.25)
            ended = time.monotonic()
        finally:
            timer.join(timeout=1)
        self.assertEqual(len(fired), 1)
        self.assertLess(fired[0], ended - .08)

    def test_timeout_bounds_whole_batch_with_continuous_sender(self):
        stop = threading.Event()
        def send_until_stopped():
            while not stop.is_set():
                self.send(37)
                stop.wait(.005)
        sender = threading.Thread(target=send_until_stopped, daemon=True)
        sender.start()
        started = time.monotonic()
        try:
            result = self.receive(count=1000, timeout=.1)
            elapsed = time.monotonic() - started
        finally:
            stop.set()
            sender.join(timeout=1)
        self.assertGreater(result.size, 0)
        self.assertLess(result.size, 1000 * self.size)
        self.assertLess(elapsed, .35)
        self.assertEqual(result.size % self.size, 0)

    def test_invalid_datagram_lengths_are_rejected(self):
        for size in (10, self.size - 1, self.size + 1, self.size + 100):
            with self.subTest(size=size):
                self.send(1, size)
                with self.assertRaises(RuntimeError):
                    self.receive(count=1)

    def test_argument_validation_and_empty_request(self):
        self.assertEqual(self.receive(count=0).size, 0)
        for count, size, timeout in ((-1,1466,.1),(1,0,.1),(1,1465,.1),
                                     (1,65508,.1),(1,1466,0),(1,1466,float("nan"))):
            with self.subTest(count=count, size=size, timeout=timeout):
                with self.assertRaises((ValueError, OverflowError)):
                    fpga_udp.read_data_udp(self.rx.fileno(), count, size, timeout)


if __name__ == "__main__":
    print("Native module:", fpga_udp.__file__, flush=True)
    unittest.main(verbosity=2)
