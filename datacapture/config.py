"""Validated acquisition layout. No serial, socket, or device imports."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import hashlib
import json
import math
from pathlib import Path


class IQOrder(str, Enum):
    Q_FIRST = "q_first"
    I_FIRST = "i_first"


@dataclass(frozen=True)
class RadarConfig:
    loops: int
    tx_order: tuple[int, ...]  # Physical TX indices in transmission order.
    rx_order: tuple[int, ...]
    samples: int
    sample_rate_hz: float
    slope_hz_per_s: float
    carrier_hz: float
    chirp_period_s: float
    frame_period_s: float
    iq_order: IQOrder = IQOrder.Q_FIRST

    def __post_init__(self) -> None:
        if type(self.loops) is not int or self.loops < 1:
            raise ValueError("loops must be a positive integer")
        if type(self.samples) is not int or self.samples < 4 or self.samples % 2:
            raise ValueError("samples must be even and >= 4")
        for name, order, limit in (("tx_order", self.tx_order, 3),
                                   ("rx_order", self.rx_order, 4)):
            if (not isinstance(order, tuple) or not order
                    or len(set(order)) != len(order)
                    or any(type(i) is not int or not 0 <= i < limit for i in order)):
                raise ValueError(f"Invalid {name}: {order}")
        for name in ("sample_rate_hz", "slope_hz_per_s", "carrier_hz",
                     "chirp_period_s", "frame_period_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not isinstance(self.iq_order, IQOrder):
            raise TypeError("iq_order must be an IQOrder")
        if self.frame_period_s < self.loops * len(self.tx_order) * self.chirp_period_s:
            raise ValueError("frame period is shorter than the chirp burst")

    @property
    def shape(self) -> tuple[int, int, int, int]:
        return self.loops, len(self.tx_order), len(self.rx_order), self.samples

    @property
    def frame_bytes(self) -> int:
        return math.prod(self.shape) * 4  # signed int16 I + signed int16 Q

    @property
    def config_id(self) -> str:
        data = json.dumps(asdict(self), sort_keys=True).encode()
        return hashlib.sha256(data).hexdigest()[:16]

    @classmethod
    def from_ti_cfg(cls, path: str | Path, *,
                    iq_order: IQOrder = IQOrder.Q_FIRST) -> RadarConfig:
        """Support the project's single-profile, complex16, one-TX-per-chirp TDM mode.

        Reject unsupported layouts rather than silently assigning incorrect axes.
        IQ lane order is explicit; it is not inferred from the antenna geometry.
        """
        commands: dict[str, list[list[str]]] = {}
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.split("%", 1)[0].split("#", 1)[0].strip()
            if line:
                words = line.split()
                commands.setdefault(words[0], []).append(words[1:])

        def one(name: str) -> list[str]:
            rows = commands.get(name, [])
            if len(rows) != 1:
                raise ValueError(f"Expected one {name}, got {len(rows)}")
            return rows[0]

        try:
            if one("dfeDataOutputMode") != ["1"] or "subFrameCfg" in commands:
                raise ValueError("Only standard frameCfg TDM mode is supported")
            if one("adcCfg")[:2] != ["2", "1"]:
                raise ValueError("Only 16-bit complex ADC is supported")
            if one("adcbufCfg")[:4] != ["-1", "0", "1", "1"]:
                raise ValueError("Unsupported ADC buffer/lane layout")
            if one("lvdsStreamCfg") != ["-1", "0", "1", "0"]:
                raise ValueError("Only header-free ADC LVDS payloads are supported")
            if any(int(row[1]) != 0 for row in commands.get("bpmCfg", [])):
                raise ValueError("BPM mode is not supported")
            channels = one("channelCfg")
            rx_mask, tx_mask = int(channels[0]), int(channels[1])
            if not 0 < rx_mask < 16 or not 0 < tx_mask < 8:
                raise ValueError("Invalid channel masks")
            profile = one("profileCfg")
            frame = one("frameCfg")
            first, last, loops = map(int, frame[:3])
            if first < 0 or not first <= last < first + 3:
                raise ValueError("Expected one chirp per enabled TX in each loop")
            chirps: dict[int, int] = {}
            for row in commands.get("chirpCfg", []):
                begin, end, profile_id = map(int, row[:3])
                if begin > end or begin < 0:
                    raise ValueError("Invalid chirp range")
                if end < first or begin > last:
                    continue
                if profile_id != int(profile[0]) or any(float(v) != 0 for v in row[3:7]):
                    raise ValueError("Chirp profile/parameter variations are unsupported")
                mask = int(row[7])
                if mask <= 0 or mask & (mask - 1) or not mask & tx_mask:
                    raise ValueError("Each chirp must enable exactly one configured TX")
                for idx in range(max(begin, first), min(end, last) + 1):
                    if idx in chirps:
                        raise ValueError("Overlapping chirp definitions")
                    chirps[idx] = mask.bit_length() - 1
            order = tuple(chirps[idx] for idx in range(first, last + 1))
            enabled_tx = tuple(i for i in range(3) if tx_mask & (1 << i))
            if sorted(order) != list(enabled_tx):
                raise ValueError("Each enabled TX must occur once per loop")
            return cls(
                loops=loops, tx_order=order,
                rx_order=tuple(i for i in range(4) if rx_mask & (1 << i)),
                samples=int(profile[9]),
                sample_rate_hz=float(profile[10]) * 1e3,
                slope_hz_per_s=float(profile[7]) * 1e12,
                carrier_hz=float(profile[1]) * 1e9,
                chirp_period_s=(float(profile[2]) + float(profile[4])) * 1e-6,
                frame_period_s=float(frame[4]) * 1e-3, iq_order=iq_order,
            )
        except (IndexError, KeyError) as exc:
            raise ValueError(f"Incomplete TI configuration: {path}") from exc

