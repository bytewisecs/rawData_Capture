"""Lazy hardware adapters. Importing this module never connects to a device."""
from __future__ import annotations
from collections import deque
import json
from pathlib import Path
import socket
import time

from .config import RadarConfig
from .models import ClockDomain, FramePacket, Timestamp
from .decode import decode_lvds_frame
from .udp import DcaPacketAssembler
from .timing import ClockMapping
from .native_udp import FpgaUdpReceiver


class RadarCLI:
    """Configure while stopped, validate CLI acknowledgements, then explicit start.

    The old serial adapter also receives corrected sensorStart/sensorStop spelling.
    This control-only adapter does not open/read the unused UART data port.
    """
    def __init__(self, port: str, *, timeout_s: float = 2.0, serial_factory=None):
        self.port, self.timeout_s, self.serial_factory = port, timeout_s, serial_factory
        self.serial = None
        self.started = False

    def open(self, config_path: Path):
        factory = self.serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        self.serial = factory(self.port, 115200, timeout=0.05, write_timeout=self.timeout_s)
        # Reopening the host port does not clear a partially typed command on
        # the device. Complete that line and wait for its prompt before sending
        # sensorStop; only this synchronization step ignores stale CLI errors.
        self.serial.reset_input_buffer()
        self.serial.write(b"\n")
        deadline = time.monotonic() + self.timeout_s
        response = bytearray()
        while time.monotonic() < deadline:
            response.extend(self.serial.read(max(1, min(self.serial.in_waiting, 4096))))
            if response.decode("utf-8", errors="replace").rstrip().endswith("mmwDemo:/>"):
                break
        else:
            raise TimeoutError(f"Radar CLI prompt synchronization failed: {response!r}")
        self.command("sensorStop")
        # platform is project metadata, not a TI firmware CLI command.
        for raw in Path(config_path).read_text().splitlines():
            line = raw.split("%", 1)[0].split("#", 1)[0].strip()
            if not line or line.split()[0] in ("sensorStart", "sensorStop", "platform"):
                continue
            self.command(line)

    def command(self, command: str):
        if self.serial is None:
            raise RuntimeError("CLI is not open")
        self.serial.reset_input_buffer()
        self.serial.write((command + "\n").encode("ascii"))
        deadline = time.monotonic() + self.timeout_s
        response = bytearray()
        while time.monotonic() < deadline:
            response.extend(self.serial.read(max(1, min(self.serial.in_waiting, 4096))))
            text = response.decode("utf-8", errors="replace")
            if any(word in text.lower() for word in ("error", "not recognized", "invalid")):
                raise RuntimeError(f"Radar rejected {command!r}: {text}")
            acknowledged = "Done" in text or (command == "sensorStop" and "already stopped" in text.lower())
            # Firmware prints Done before it is ready for the next command.
            # Waiting for the complete prompt avoids dropping the next command's first byte.
            if acknowledged and text.rstrip().endswith("mmwDemo:/>"):
                return text
        raise TimeoutError(f"Radar CLI did not acknowledge {command!r}: {response!r}")

    def start(self):
        # Mark before command so cleanup attempts stop even if the ACK is lost.
        self.started = True
        self.command("sensorStart")

    def close(self):
        if self.serial is None:
            return
        try:
            if self.started:
                self.command("sensorStop")
        finally:
            self.started = False
            self.serial.close()
            self.serial = None


class DcaLiveSource:
    """Reuse DCA control commands and the third-party fpga_udp native receiver.

    open prepares stopped hardware, start begins one fresh stream, close stops it.
    host_ip is REQUIRED: the adapter never changes network/IP settings. Radar times
    are frame-counter estimates anchored to the host start command, not hardware
    exposure timestamps. Unknown uncertainty explicitly prevents synchronized Q.
    """
    def __init__(self, config: RadarConfig, radar_config: Path, dca_config: Path, *,
                 host_ip: str, adc_ip: str, cli_port: str, dca_factory=None, cli_factory=RadarCLI,
                 udp_batch_packets: int | None = None, native_module=None):
        import ipaddress
        ipaddress.IPv4Address(host_ip)
        ipaddress.IPv4Address(adc_ip)
        self.config = config
        self.radar_config, self.dca_config = Path(radar_config), Path(dca_config)
        self.host_ip, self.adc_ip, self.cli_port = host_ip, adc_ip, cli_port
        self.dca_factory, self.cli_factory = dca_factory, cli_factory
        self.dca = self.cli = None
        self.receiver = None
        self.udp_batch_packets, self.native_module = udp_batch_packets, native_module
        self._stream_started = False
        self.assembler = DcaPacketAssembler(config.frame_bytes)
        self._pending = deque()
        self._anchor = None
        self.stale_packets_discarded = 0

    @staticmethod
    def _check(name, result):
        if result != "success":
            raise RuntimeError(f"DCA {name} failed: {result!r}")

    def open(self):
        settings = json.loads(self.dca_config.read_text())["DCA1000Config"]
        if (settings["dataLoggingMode"] != "raw" or settings["lvdsMode"] != 2
                or settings["dataTransferMode"] != "LVDSCapture"
                or settings["dataCaptureMode"] != "ethernetStream"):
            raise ValueError("Live adapter requires raw two-lane LVDS Ethernet capture")
        if RadarConfig.from_ti_cfg(self.radar_config, iq_order=self.config.iq_order) != self.config:
            raise ValueError("Live TI file differs from session configuration")
        # Check the compiled dependency before opening sockets or configuring hardware.
        native = self.native_module or FpgaUdpReceiver.load_library()
        FpgaUdpReceiver._check_library(native)
        factory = self.dca_factory
        if factory is None:
            from .drivers.dca1000 import DCA1000
            factory = DCA1000
        try:
            ethernet = settings.get("ethernetConfig", {})
            self.dca = factory(static_ip=self.host_ip, adc_ip=self.adc_ip,
                data_port=ethernet.get("DCA1000DataPort", 4098),
                config_port=ethernet.get("DCA1000ConfigPort", 4096))
            self.receiver = FpgaUdpReceiver(self.dca.data_socket, frame_bytes=self.config.frame_bytes,
                batch_packets=self.udp_batch_packets, native_module=native)
            self._check("connect", self.dca.sys_alive_check())
            self.cli = self.cli_factory(self.cli_port)
            self.cli.open(self.radar_config)
            self._check("config_fpga", self.dca.config_fpga(str(self.dca_config)))
            self._check("config_record", self.dca.config_record(str(self.dca_config)))
        except BaseException:
            self.close()
            raise

    def start(self, event):
        # UART has confirmed the sensor is stopped. Remove buffered data from a
        # previous acquisition before resetting the DCA stream's byte counter.
        deadline = time.monotonic() + .5
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("UDP data continued while radar was stopped")
            packets = self.receiver.read_packets(min(.01, remaining))
            self.stale_packets_discarded += len(packets)
            if not len(packets):
                break
        self._stream_started = True
        self._check("stream_start", self.dca.stream_start())
        self._anchor = Timestamp(time.monotonic(), ClockDomain.MONOTONIC, estimated=True)
        self.mapping = ClockMapping(Timestamp(0, ClockDomain.RELATIVE), self._anchor,
                                    method="sensor_start_command_and_frame_counter")
        self.cli.start()

    def read(self, timeout_s=0.5):
        if self._anchor is None:
            raise RuntimeError("Live source must be started explicitly")
        deadline = time.monotonic() + timeout_s
        while not self._pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Radar frame not ready")
            packets = self.receiver.read_packets(remaining)
            if not len(packets):
                raise TimeoutError("Radar UDP timeout")
            self._pending.extend(self.assembler.feed_batch(packets))
        data = self._pending.popleft()
        relative = Timestamp(data.frame_id * self.config.frame_period_s, ClockDomain.RELATIVE, estimated=True)
        stamp = self.mapping.map(relative)
        return FramePacket(data.frame_id, stamp, self.config,
                           decode_lvds_frame(data.payload, self.config), data.health,
                           source_timestamp=relative, time_uncertainty_s=None)

    def diagnostics(self):
        result = self.assembler.diagnostics()
        if self.receiver is not None:
            result["native_receive"] = self.receiver.diagnostics()
        result.update(host_ip=self.host_ip, adc_ip=self.adc_ip, assembled_unread_frames=len(self._pending),
                      start_command_monotonic_s=self._anchor.seconds if self._anchor else None,
                      frame_period_s=self.config.frame_period_s, time_uncertainty_s=None,
                      stale_packets_discarded=self.stale_packets_discarded)
        if self.dca:
            result["socket_receive_buffer_bytes"] = self.dca.data_socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        return result

    def close(self):
        errors = []
        if self.cli:
            try:
                self.cli.close()
            except Exception as exc:
                errors.append(exc)
            self.cli = None
        device, self.dca = self.dca, None
        was_started, self._stream_started = self._stream_started, False
        if device:
            try:
                if was_started:
                    self._check("stream_stop", device.stream_stop())
            except Exception as exc:
                errors.append(exc)
            try:
                device.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f"Hardware stop failed: {errors}") from errors[0]
