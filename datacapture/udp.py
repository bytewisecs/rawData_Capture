"""DCA1000 packet decoding/reassembly only; live reception is owned by fpga_udp."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .models import FrameHealth, Integrity


@dataclass(frozen=True)
class AssembledFrame:
    frame_id: int
    payload: bytes
    health: FrameHealth
    missing_bytes: int


class DcaPacketAssembler:
    """Bounded reordering; missing bytes are zero-filled and explicitly INVALID.

    Header: uint32 sequence, uint48 little-endian byte offset, then payload.
    Offsets are absolute within a fresh RECORD_START stream. Packet boundaries
    may cross frame boundaries. Entire missing frames are emitted as invalid.
    Sequence numbers are metadata; the byte counter determines placement.
    """
    def __init__(self, frame_bytes: int, *, reorder_frames: int = 2, max_gap_frames: int = 32):
        if frame_bytes < 1 or reorder_frames < 1 or max_gap_frames < reorder_frames:
            raise ValueError("Invalid reassembly limits")
        self.frame_bytes, self.reorder_frames = frame_bytes, reorder_frames
        self.max_gap_frames = max_gap_frames
        self._frames = {}
        self._next = 0
        self._highest = -1
        self.packets = self.late_packets = self.duplicate_bytes = 0
        self.missing_bytes = self.invalid_frames = 0

    def feed_batch(self, packets: np.ndarray) -> list[AssembledFrame]:
        """Decode a native batch without copying each packet to Python bytes."""
        if packets.ndim != 2 or packets.dtype != np.uint8 or not packets.flags.c_contiguous:
            raise ValueError("Expected a contiguous uint8 native packet batch")
        output = []
        for packet in packets:
            output.extend(self.feed(packet))
        return output

    def feed(self, packet: bytes) -> list[AssembledFrame]:
        if len(packet) <= 10 or len(packet) > 65507:
            raise ValueError("Invalid DCA UDP packet size")
        sequence = int.from_bytes(packet[:4], "little")
        offset = int.from_bytes(packet[4:10], "little")
        if sequence == 0:
            raise ValueError("DCA packet sequence starts at one")
        payload = memoryview(packet)[10:]
        first, last = offset // self.frame_bytes, (offset + len(payload) - 1) // self.frame_bytes
        if last - self._next > self.max_gap_frames:
            raise ValueError("DCA byte-counter jump or stream not reset at view start")
        self.packets += 1
        if last < self._next:
            self.late_packets += 1
            return []
        self._highest = max(self._highest, last)
        output = []
        # Evict old missing frames BEFORE allocation, bounding memory on large gaps.
        while self._highest - self._next >= self.reorder_frames:
            if self._next >= first:
                break
            output.append(self._emit())
        cursor = 0
        while cursor < len(payload):
            frame_id, within = divmod(offset + cursor, self.frame_bytes)
            length = min(len(payload) - cursor, self.frame_bytes - within)
            if frame_id >= self._next:
                if frame_id not in self._frames:
                    self._frames[frame_id] = [np.zeros(self.frame_bytes, np.uint8),
                                              np.zeros(self.frame_bytes, bool), set(), 0]
                data, received, reasons, _ = self._frames[frame_id]
                part = np.frombuffer(payload[cursor:cursor + length], dtype=np.uint8)
                old = received[within:within + length]
                duplicates = int(np.count_nonzero(old))
                self.duplicate_bytes += duplicates
                self._frames[frame_id][3] += length - duplicates
                if np.any(data[within:within + length][old] != part[old]):
                    reasons.add("conflicting_udp_payload")
                data[within:within + length] = part
                received[within:within + length] = True
            cursor += length
        while self._next <= self._highest:
            current = self._frames.get(self._next)
            complete = current is not None and current[3] == self.frame_bytes
            if not complete and self._highest - self._next < self.reorder_frames:
                break
            output.append(self._emit())
        return output

    def _emit(self):
        current = self._frames.pop(self._next, None)
        if current is None:
            data, reasons, count = np.zeros(self.frame_bytes, np.uint8), set(), 0
        else:
            data, _, reasons, count = current
        missing = self.frame_bytes - count
        if missing:
            reasons.add(f"udp_missing_bytes:{missing}")
        valid = not reasons
        health = FrameHealth(Integrity.VALID if valid else Integrity.INVALID,
                             0 if valid else None, tuple(sorted(reasons)))
        result = AssembledFrame(self._next, data.tobytes(), health, missing)
        self._next += 1
        self.missing_bytes += missing
        self.invalid_frames += int(not valid)
        return result

    def finish(self) -> list[AssembledFrame]:
        """For offline packet files/tests: flush a partial tail with explicit validity."""
        result = []
        while self._next <= self._highest:
            result.append(self._emit())
        return result

    def diagnostics(self):
        return {"packets": self.packets, "late_packets": self.late_packets,
                "duplicate_bytes": self.duplicate_bytes, "missing_bytes": self.missing_bytes,
                "invalid_frames": self.invalid_frames,
                "unfinished_frames": len(self._frames),
                "unfinished_received_bytes": sum(item[3] for item in self._frames.values())}
