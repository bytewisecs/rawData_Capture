"""Original two-lane complex16 LVDS to ADC frame decoder."""
from __future__ import annotations
import numpy as np
from .config import IQOrder, RadarConfig

def decode_lvds_frame(payload: bytes, config: RadarConfig) -> np.ndarray:
    """Decode the existing header-free, two-lane, complex16 BIN convention.

    Four words are [Q0,Q1,I0,I1] for q_first, [I0,I1,Q0,Q1] for i_first.
    ADC values stay signed int16 until conversion to complex64; never float16.
    """
    if len(payload) != config.frame_bytes:
        raise ValueError(f"Expected {config.frame_bytes} bytes, received {len(payload)}")
    words = np.frombuffer(payload, dtype="<i2").reshape(-1, 4)
    values = np.empty((len(words), 2), dtype=np.complex64)
    if config.iq_order is IQOrder.Q_FIRST:
        values.real, values.imag = words[:, 2:4], words[:, 0:2]
    else:
        values.real, values.imag = words[:, 0:2], words[:, 2:4]
    return values.reshape(config.shape)

