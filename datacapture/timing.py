"""Explicit affine clock mapping; a software start marker is not exposure sync."""
from __future__ import annotations
from dataclasses import dataclass
import math
from .models import ClockDomain, Timestamp


@dataclass(frozen=True)
class ClockMapping:
    source_origin: Timestamp
    target_origin: Timestamp
    scale: float = 1.0
    uncertainty_s: float | None = None  # None means not bounded/calibrated.
    method: str = "software_anchor"

    def __post_init__(self):
        if not math.isfinite(self.scale) or self.scale <= 0 or not self.method:
            raise ValueError("Clock mapping needs a positive scale and method")
        if self.uncertainty_s is not None and (not math.isfinite(self.uncertainty_s) or self.uncertainty_s < 0):
            raise ValueError("Clock uncertainty must be finite and nonnegative")

    def map(self, timestamp: Timestamp) -> Timestamp:
        if timestamp.domain != self.source_origin.domain:
            raise ValueError("Clock mapping source domain mismatch")
        return Timestamp(
            self.target_origin.seconds + (timestamp.seconds - self.source_origin.seconds) * self.scale,
            self.target_origin.domain,
            timestamp.estimated or self.source_origin.estimated or self.target_origin.estimated
            or self.uncertainty_s != 0,
        )


def synchronized(a: Timestamp, b: Timestamp, *, uncertainty_a_s: float | None,
                 uncertainty_b_s: float | None, tolerance_s: float) -> bool:
    """Conservative match; unknown clock error cannot be called synchronized."""
    if not math.isfinite(tolerance_s) or tolerance_s < 0:
        raise ValueError("Synchronization tolerance must be finite and nonnegative")
    if a.domain != b.domain:
        return False
    if uncertainty_a_s is None or uncertainty_b_s is None:
        return False
    if any(not math.isfinite(x) or x < 0 for x in (uncertainty_a_s, uncertainty_b_s)):
        raise ValueError("Invalid uncertainty")
    return abs(a.seconds - b.seconds) + uncertainty_a_s + uncertainty_b_s <= tolerance_s
