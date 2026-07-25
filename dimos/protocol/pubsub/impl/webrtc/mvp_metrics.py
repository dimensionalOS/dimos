# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import binascii
from dataclasses import dataclass
import time

import numpy as np
from numpy.typing import NDArray

_MAGIC = 0xD105
_PAYLOAD_BYTES = 10
_ENCODED_BYTES = 12
_BITS = _ENCODED_BYTES * 8
_BLOCK_WIDTH = 6
_HEADER_HEIGHT = 48
_DARK = 16
_LIGHT = 240


@dataclass(frozen=True)
class FrameStamp:
    timestamp_ms: int
    sequence: int


def make_stamped_frame(
    *,
    width: int,
    height: int,
    sequence: int,
    timestamp_ms: int | None = None,
) -> NDArray[np.uint8]:
    if width < _BITS * _BLOCK_WIDTH or height < _HEADER_HEIGHT:
        raise ValueError(f"Frame must be at least {_BITS * _BLOCK_WIDTH}x{_HEADER_HEIGHT}")
    timestamp = timestamp_ms if timestamp_ms is not None else time.time_ns() // 1_000_000
    payload = (
        _MAGIC.to_bytes(2, "big")
        + (timestamp & ((1 << 48) - 1)).to_bytes(6, "big")
        + (sequence & 0xFFFF).to_bytes(2, "big")
    )
    crc = binascii.crc_hqx(payload, 0xFFFF)
    encoded = payload + crc.to_bytes(2, "big")

    frame = np.full((height, width, 3), 96, dtype=np.uint8)
    for index in range(_BITS):
        byte = encoded[index // 8]
        bit = (byte >> (7 - index % 8)) & 1
        start = index * _BLOCK_WIDTH
        frame[:_HEADER_HEIGHT, start : start + _BLOCK_WIDTH] = _LIGHT if bit else _DARK
    return frame


def decode_frame_stamp(frame: NDArray[np.uint8]) -> FrameStamp | None:
    if frame.ndim != 3 or frame.shape[1] < _BITS * _BLOCK_WIDTH:
        return None
    encoded = bytearray(_ENCODED_BYTES)
    for index in range(_BITS):
        start = index * _BLOCK_WIDTH
        value = float(frame[:_HEADER_HEIGHT, start : start + _BLOCK_WIDTH].mean())
        if value >= (_DARK + _LIGHT) / 2:
            encoded[index // 8] |= 1 << (7 - index % 8)

    payload = bytes(encoded[:_PAYLOAD_BYTES])
    expected_crc = int.from_bytes(encoded[_PAYLOAD_BYTES:], "big")
    if int.from_bytes(payload[:2], "big") != _MAGIC:
        return None
    if binascii.crc_hqx(payload, 0xFFFF) != expected_crc:
        return None
    return FrameStamp(
        timestamp_ms=int.from_bytes(payload[2:8], "big"),
        sequence=int.from_bytes(payload[8:10], "big"),
    )


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=np.float64), quantile))
