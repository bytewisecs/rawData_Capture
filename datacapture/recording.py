"""Stream raw ADC cubes to one standard NPY without holding a trial in RAM."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import struct

import numpy as np


class NpyRecorder:
    """Reserve a fixed NPY v2 header; finalize with the actual saved frame count.

    During capture, the file ends in .npy.part. A controlled stop finalizes a
    loadable short NPY; an abrupt process/power loss leaves .part for diagnosis.
    """
    header_bytes = 4096

    def __init__(self, path: Path, frame_shape):
        self.path = Path(path)
        self.part = self.path.with_suffix(".npy.part")
        self.index_path = self.path.with_suffix(".frames.jsonl")
        self.frame_shape = tuple(frame_shape)
        self.count = self.invalid_frames = 0
        self.digest = hashlib.sha256()
        self.closed = False
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for p in (self.path, self.part, self.index_path, self.path.with_suffix(".json")):
            if p.exists():
                raise FileExistsError("Refusing to overwrite: " + str(p))
        self.file = self.part.open("xb+", buffering=1024 * 1024)
        try:
            self.index = self.index_path.open("x", encoding="utf-8")
            self.file.write(self._header(0))
        except BaseException:
            self.file.close()
            raise

    def _header(self, count):
        description = repr({"descr": "<c8", "fortran_order": False,
                            "shape": (count,) + self.frame_shape})
        length = self.header_bytes - 12
        encoded = description.encode("latin1")
        if len(encoded) >= length:
            raise ValueError("NPY header reservation exceeded")
        return (b"\x93NUMPY\x02\x00" + struct.pack("<I", length)
                + encoded.ljust(length - 1, b" ") + b"\n")

    def write(self, frame):
        if self.closed or frame.adc.shape != self.frame_shape:
            raise ValueError("Recorder closed or frame shape changed")
        adc = np.ascontiguousarray(frame.adc, dtype="<c8")
        raw = memoryview(adc).cast("B")
        row = {"npy_index": self.count, "frame_id": frame.frame_id,
               "timestamp_monotonic_s": frame.timestamp.seconds,
               "timestamp_estimated": frame.timestamp.estimated,
               "relative_time_s": frame.source_timestamp.seconds if frame.source_timestamp else None,
               "time_uncertainty_s": frame.time_uncertainty_s,
               "integrity": frame.health.state.value, "reasons": list(frame.health.reasons),
               "adc_sha256": hashlib.sha256(raw).hexdigest()}
        offset = self.header_bytes + self.count * adc.nbytes
        try:
            if self.file.write(raw) != adc.nbytes:
                raise OSError("Short ADC file write")
            self.index.write(json.dumps(row, allow_nan=False) + "\n")
        except BaseException:
            self.file.seek(offset)
            self.file.truncate()
            raise
        self.digest.update(raw)
        self.count += 1
        self.invalid_frames += int(frame.health.state.value != "valid")

    def close(self):
        if self.closed:
            return
        try:
            self.file.seek(0)
            self.file.write(self._header(self.count))
            self.file.flush()
            os.fsync(self.file.fileno())
            self.index.flush()
            os.fsync(self.index.fileno())
        finally:
            self.file.close()
            self.index.close()
            self.closed = True
        # Hard-link then unlink gives an atomic publish that cannot overwrite.
        os.link(str(self.part), str(self.path))
        self.part.unlink()


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
