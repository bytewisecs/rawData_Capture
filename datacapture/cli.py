"""Command-line raw ADC acquisition; configuration paths are project-local."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import threading

from .config import IQOrder, RadarConfig
from .native_udp import FpgaUdpReceiver

ROOT = Path(__file__).resolve().parent.parent


def parser():
    p = argparse.ArgumentParser(description="IWR1843 + DCA1000 raw complex ADC to NPY via fpga_udp")
    limit = p.add_mutually_exclusive_group()
    limit.add_argument("--duration", type=float, default=30.0,
                       help="nominal capture seconds, converted to a whole number of frames (default: 30)")
    limit.add_argument("--frames", type=int, help="capture exactly this many frame slots")
    p.add_argument("--dca-config", type=Path, default=ROOT / "datacapture/cfg/dca1000/dca1000.json")
    p.add_argument("--radar-config", type=Path, help="override the radar CFG referenced by DCA JSON")
    p.add_argument("--host-ip", help="override JSON host address (does not reprogram FPGA Ethernet)")
    p.add_argument("--dca-ip", help="override JSON DCA1000 address")
    p.add_argument("--cli-port", help="override JSON radar userPort")
    p.add_argument("--iq-order", choices=[x.value for x in IQOrder], default="q_first")
    p.add_argument("--output", type=Path, help="exact .npy path; existing files are never overwritten")
    p.add_argument("--output-dir", type=Path, default=ROOT / "dataSet")
    p.add_argument("--queue-frames", type=int, default=32)
    p.add_argument("--idle-timeout", type=float, default=3.0)
    p.add_argument("--check", action="store_true", help="check dependencies, config, ports and buffer without starting radar")
    return p


def preflight(host_ip, dca_ip, cli_port, ethernet, output, expected_bytes):
    import ipaddress
    import serial
    ipaddress.IPv4Address(host_ip)
    ipaddress.IPv4Address(dca_ip)
    native = FpgaUdpReceiver.load_library()
    if not os.path.exists(cli_port) or not os.access(cli_port, os.R_OK | os.W_OK):
        raise RuntimeError("Radar CLI port is missing or inaccessible: " + cli_port)
    rmem = Path("/proc/sys/net/core/rmem_max")
    value = int(rmem.read_text()) if rmem.is_file() else None
    if value is not None and value < 16777216:
        raise RuntimeError("UDP rmem_max=%d; run sudo sysctl -w net.core.rmem_max=16777216" % value)
    for key, default in (("DCA1000ConfigPort", 4096), ("DCA1000DataPort", 4098)):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind((host_ip, int(ethernet.get(key, default))))
    existing = output.parent
    while not existing.exists():
        existing = existing.parent
    free = shutil.disk_usage(str(existing)).free
    if free < expected_bytes + 64 * 1024 * 1024:
        raise RuntimeError("Insufficient free disk space for the requested capture")
    return {"fpga_udp_module": native.__file__, "fpga_udp_api": native.read_data_udp_api_version,
            "pyserial_version": serial.__version__, "rmem_max": value, "disk_free_bytes": free}


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        settings = json.loads(args.dca_config.read_text())
        radar_settings = settings["RadarConfig"]
        ethernet = settings["DCA1000Config"]["ethernetConfig"]
        radar_path = args.radar_config or (args.dca_config.parent / radar_settings["radarCFG"])
        radar_path = radar_path.resolve()
        config = RadarConfig.from_ti_cfg(radar_path, iq_order=IQOrder(args.iq_order))
        if not math.isfinite(args.duration) or args.duration <= 0:
            raise ValueError("duration must be positive and finite")
        frames = args.frames if args.frames is not None else math.ceil(args.duration / config.frame_period_s - 1e-9)
        if frames < 1 or args.queue_frames < 1 or not math.isfinite(args.idle_timeout) or args.idle_timeout <= 0:
            raise ValueError("frames, queue-frames and idle-timeout must be positive")
        host_ip = args.host_ip or ethernet["systemIPAddress"]
        dca_ip = args.dca_ip or ethernet["DCA1000IPAddress"]
        cli_port = args.cli_port or radar_settings["userPort"]
        output = args.output or args.output_dir / ("adc_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".npy")
        output = output.resolve()
        if output.suffix != ".npy":
            raise ValueError("output must end in .npy")
        checks = preflight(host_ip, dca_ip, cli_port, ethernet, output, frames * config.frame_bytes * 2)
        print(json.dumps({"output": str(output), "frames": frames,
                          "adc_shape": [frames] + list(config.shape), "tx_order": config.tx_order,
                          "frame_period_s": config.frame_period_s, "host_ip": host_ip,
                          "dca_ip": dca_ip, "cli_port": cli_port, "checks": checks}, indent=2), flush=True)
        if args.check:
            return 0
        from .hardware import DcaLiveSource
        from .capture import capture
        source = DcaLiveSource(config, radar_path, args.dca_config.resolve(),
                               host_ip=host_ip, adc_ip=dca_ip, cli_port=cli_port)
        stop = threading.Event()
        previous = {}
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stop.set())
        try:
            result = capture(source, config, output, frames, stop=stop,
                             queue_frames=args.queue_frames, idle_timeout=args.idle_timeout,
                             metadata={"radar_config_text": radar_path.read_text(),
                                       "radar_config_sha256": hashlib.sha256(radar_path.read_bytes()).hexdigest(),
                                       "dca_settings": settings, "preflight": checks})
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)
        print(json.dumps({key: result[key] for key in
                          ("status", "shape", "invalid_frames", "wall_elapsed_s", "errors")}, indent=2), flush=True)
        print("Saved: " + str(output), flush=True)
        return 0 if result["status"] == "complete" else 2
    except Exception as exc:
        p.exit(2, type(exc).__name__ + ": " + str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
