"""Raw acquisition messages extracted from AeroSense_capture."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
import numpy as np
from .config import RadarConfig

class ClockDomain(str, Enum):
    RELATIVE = "relative"
    MONOTONIC = "monotonic"
    UNIX = "unix"
    DEVICE = "device"


@dataclass(frozen=True)
class Timestamp:
    seconds: float
    domain: ClockDomain
    estimated: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.seconds):
            raise ValueError("Timestamp must be finite")
        if not isinstance(self.domain, ClockDomain):
            raise TypeError("Timestamp domain must be a ClockDomain")
        if type(self.estimated) is not bool:
            raise TypeError("estimated must be a boolean")


class Integrity(str, Enum):
    VALID = "valid"
    INVALID = "invalid"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class FrameHealth:
    state: Integrity
    missing_packets: int | None = None
    reasons: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.state, Integrity):
            raise TypeError("Invalid integrity state")
        if self.missing_packets is not None:
            if type(self.missing_packets) is not int or self.missing_packets < 0:
                raise ValueError("missing_packets must be a nonnegative integer")
        if self.state is Integrity.VALID and self.missing_packets != 0:
            raise ValueError("A valid frame requires confirmed zero missing packets")


def readonly_array(value: np.ndarray, dtype: object) -> np.ndarray:
    """Own the buffer so a producer reusing its memory cannot mutate a message."""
    result = np.array(value, dtype=dtype, copy=True)
    if not np.all(np.isfinite(result)):
        raise ValueError("Arrays must not contain NaN or infinity")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class FramePacket:
    frame_id: int
    timestamp: Timestamp
    config: RadarConfig
    adc: np.ndarray  # complex64 [loop, tx_slot, rx_slot, sample]
    health: FrameHealth
    source_timestamp: Timestamp | None = None
    time_uncertainty_s: float | None = None

    def __post_init__(self):
        if type(self.frame_id) is not int or self.frame_id < 0:
            raise ValueError("frame_id must be a nonnegative integer")
        if not np.iscomplexobj(self.adc) or np.shape(self.adc) != self.config.shape:
            raise ValueError("ADC must be complex and match the configured frame shape")
        object.__setattr__(self, "adc", readonly_array(self.adc, np.complex64))

