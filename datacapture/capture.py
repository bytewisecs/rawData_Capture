"""Bounded raw capture and disk writer; no FFT or online decision processing."""
from __future__ import annotations

from dataclasses import asdict
import queue
import threading
import time

from .recording import NpyRecorder, write_json


def udp_counters():
    from pathlib import Path
    path = Path("/proc/net/snmp")
    if not path.is_file():
        return {}
    lines = path.read_text().splitlines()
    for i, line in enumerate(lines[:-1]):
        if line.startswith("Udp:"):
            return dict(zip(line.split()[1:], map(int, lines[i + 1].split()[1:])))
    return {}


def capture(source, config, path, frames, *, metadata=None, stop=None,
            queue_frames=32, idle_timeout=3.0):
    if frames < 1 or queue_frames < 1 or idle_timeout <= 0:
        raise ValueError("frames, queue_frames and idle_timeout must be positive")
    stop = stop if stop is not None else threading.Event()
    writer = NpyRecorder(path, config.shape)
    pending = queue.Queue(maxsize=queue_frames)
    writer_errors = []
    errors = []
    received = enqueued = 0
    diagnostics = {}
    before = udp_counters()
    started_wall = time.time()
    started = time.monotonic()

    def write_frames():
        try:
            while True:
                frame = pending.get()
                if frame is None:
                    break
                writer.write(frame)
        except BaseException as exc:
            writer_errors.append(str(exc))

    worker = threading.Thread(target=write_frames, name="raw-npy-writer", daemon=True)
    worker.start()
    try:
        source.open()
        if not stop.is_set():
            source.start(None)
            active_start = last_frame = time.monotonic()
            deadline = active_start + frames * config.frame_period_s + max(5.0, idle_timeout)
            while received < frames and not stop.is_set():
                if writer_errors:
                    raise RuntimeError("NPY writer failed: " + writer_errors[0])
                if time.monotonic() >= deadline:
                    raise TimeoutError("Capture exceeded the requested frame window plus timeout margin")
                try:
                    frame = source.read(timeout_s=0.2)
                except TimeoutError:
                    if time.monotonic() - last_frame >= idle_timeout:
                        raise TimeoutError("No complete radar frames for %.1f seconds" % idle_timeout)
                    continue
                if frame.frame_id != received:
                    raise ValueError("Noncontiguous frame IDs: expected %d, got %d" % (received, frame.frame_id))
                received += 1
                last_frame = time.monotonic()
                try:
                    pending.put_nowait(frame)
                except queue.Full:
                    raise RuntimeError("Disk writer queue full; capture stopped without silently dropping frames")
                enqueued += 1
                if received == 1 or received % 100 == 0 or received == frames:
                    print("Captured %d/%d, saved %d" % (received, frames, writer.count), flush=True)
    except KeyboardInterrupt:
        stop.set()
    except Exception as exc:
        errors.append(type(exc).__name__ + ": " + str(exc))
    finally:
        try:
            diagnostics = source.diagnostics()
        except Exception as exc:
            errors.append("diagnostics: " + str(exc))
        try:
            source.close()
        except Exception as exc:
            errors.append("hardware_stop: " + str(exc))
        finish_deadline = time.monotonic() + 15
        while worker.is_alive() and time.monotonic() < finish_deadline:
            try:
                pending.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        worker.join(timeout=15)
        if worker.is_alive():
            errors.append("Disk writer did not finish; .npy.part remains unpublished")
        else:
            try:
                writer.close()
            except Exception as exc:
                errors.append("finalize: " + str(exc))
        errors.extend("writer: " + item for item in writer_errors)
    after = udp_counters()
    delta = {key: after[key] - before.get(key, after[key]) for key in after}
    complete = (received == enqueued == writer.count == frames and not errors
                and not writer.invalid_frames and writer.path.is_file())
    result = {
        "schema_version": 1, "status": "complete" if complete else "incomplete",
        "npy_file": writer.path.name, "dtype": "complex64",
        "axes": ["frame", "loop", "tx_slot", "rx_slot", "adc_sample"],
        "shape": [writer.count] + list(config.shape), "config": asdict(config),
        "config_id": config.config_id, "requested_frames": frames,
        "nominal_duration_s": frames * config.frame_period_s,
        "received_frames": received, "enqueued_frames": enqueued,
        "recorded_frames": writer.count, "invalid_frames": writer.invalid_frames,
        "interrupted": stop.is_set(), "errors": errors,
        "adc_payload_sha256": writer.digest.hexdigest(),
        "started_unix_s": started_wall, "wall_elapsed_s": time.monotonic() - started,
        "diagnostics": diagnostics, "host_udp_counter_delta": delta,
        "timestamp_note": "Host command anchor + frame counter; not a hardware exposure timestamp",
        "metadata": metadata or {},
    }
    write_json(writer.path.with_suffix(".json"), result)
    return result
