"""Thin adapter to the bundled third-party fpga_udp C++ receiver."""
from __future__ import annotations
import importlib
import math
import numpy as np


class FpgaUdpReceiver:
    """Receive a batch in C++; Python sees only the bytes actually received.

    No Python socket-receive fallback. API version 2 keeps read_data_udp's four
    arguments and flat uint8 result, while fixing partial returns, deadline and
    GIL handling in the bundled library. The socket has one receiving owner.
    """
    packet_size = 1466  # Existing DCA header (10) + ADC payload (1456).

    def __init__(self, data_socket, *, frame_bytes: int, batch_packets: int | None = None,
                 native_module=None):
        if type(frame_bytes) is not int or frame_bytes < 1:
            raise ValueError("frame_bytes must be positive")
        if batch_packets is None:
            batch_packets = min(4096, math.ceil(frame_bytes / (self.packet_size - 10)))
        if type(batch_packets) is not int or not 1 <= batch_packets <= 4096:
            raise ValueError("UDP batch_packets must be in [1, 4096]")
        self.native = native_module or self.load_library()
        self._check_library(self.native)
        self.socket = data_socket
        self.batch_packets = batch_packets
        # C++ select() owns waiting; nonblocking recvfrom avoids a spurious-ready
        # socket escaping the batch deadline. Python never receives a datagram.
        self.socket.setblocking(False)
        self.calls = self.packets = self.empty_batches = self.partial_batches = 0

    @staticmethod
    def _check_library(module):
        if (getattr(module, "read_data_udp_api_version", 0) < 2
                or not callable(getattr(module, "read_data_udp", None))):
            raise RuntimeError(
                "fpga_udp requires the bundled valid-length/deadline fix (receive API >= 2). "
                "Rebuild third_parts/fpga_udp in the device virtual environment.")

    @classmethod
    def load_library(cls):
        try:
            module = importlib.import_module("fpga_udp")
        except ImportError as exc:
            raise RuntimeError("Live capture requires fpga_udp; build third_parts/fpga_udp on the device") from exc
        cls._check_library(module)
        return module

    def read_packets(self, timeout_s: float) -> np.ndarray:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("UDP receive timeout must be positive and finite")
        data = self.native.read_data_udp(self.socket.fileno(), self.batch_packets,
                                         self.packet_size, float(timeout_s))
        if (not isinstance(data, np.ndarray) or data.dtype != np.uint8 or data.ndim != 1
                or not data.flags.c_contiguous or data.size % self.packet_size
                or data.size > self.batch_packets * self.packet_size):
            raise RuntimeError("fpga_udp returned an invalid packet batch")
        packets = data.reshape(-1, self.packet_size)
        self.calls += 1
        self.packets += len(packets)
        self.empty_batches += int(not len(packets))
        self.partial_batches += int(0 < len(packets) < self.batch_packets)
        return packets  # views retain the native buffer; no per-packet bytes copy

    def diagnostics(self):
        return {"receiver": "fpga_udp.read_data_udp", "receive_api_version": self.native.read_data_udp_api_version,
                "batch_packets": self.batch_packets, "native_calls": self.calls,
                "received_packets": self.packets, "empty_batches": self.empty_batches,
                "partial_batches": self.partial_batches}
