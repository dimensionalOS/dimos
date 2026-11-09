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

"""Pure byte encoders for the inspected PC2 external C++ SONIC protocol.

No transport, controller lifecycle or execution API is provided. Validation
covers wire representation, not joint limits, collision or motion safety.
"""

from __future__ import annotations

from collections.abc import Sequence
import json
import math
from numbers import Real
import struct
from typing import Any

HEADER_SIZE = 1280


def _message(topic: bytes, version: int, fields: list[dict[str, Any]], payload: bytes) -> bytes:
    header = json.dumps(
        {"v": version, "endian": "le", "count": 1, "fields": fields}, separators=(",", ":")
    ).encode("ascii")
    return topic + header.ljust(HEADER_SIZE, b"\0") + payload


def encode_command(*, start: bool, stop: bool, planner: bool) -> bytes:
    """Encode explicit v1 control flags without sending them.

    ``planner`` selects planner (True) or streamed references (False).
    ``stop`` requests controller shutdown/damping, not a task pause. False
    start and stop allow a mode-only request; receipt is not acknowledged.
    """
    for name, value in (("start", start), ("stop", stop), ("planner", planner)):
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be a bool")
    if start and stop:
        raise ValueError("start and stop cannot both be True")
    fields = [{"name": name, "dtype": "u8", "shape": [1]} for name in ("start", "stop", "planner")]
    return _message(b"command", 1, fields, struct.pack("BBB", start, stop, planner))


def _float32_vector(values: Sequence[float], size: int, name: str) -> bytes:
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    for value in values:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise ValueError(f"{name} must contain finite float32 numbers")
        try:
            finite = math.isfinite(value)
        except OverflowError as exc:
            raise ValueError(f"{name} must contain finite float32 numbers") from exc
        if not finite:
            raise ValueError(f"{name} must contain finite float32 numbers")
    try:
        return struct.pack(f"<{size}f", *values)
    except (OverflowError, struct.error) as exc:
        raise ValueError(f"{name} must contain finite float32 numbers") from exc


def encode_token_reference(
    *,
    token: Sequence[float],
    left_hand: Sequence[float],
    right_hand: Sequence[float],
    frame_index: int,
) -> bytes:
    """Encode one v4 reference: 64 latent values and seven motor slots per hand.

    Both hands are required, in the installed publisher's motor-slot order;
    no permutation or anatomical mapping is applied. Tokens are SONIC latent
    references, not Cartesian poses or arm joint goals. ``frame_index`` is a
    nonnegative signed int64; it does not establish acknowledgement or expiry.
    """
    if (
        isinstance(frame_index, bool)
        or not isinstance(frame_index, int)
        or not 0 <= frame_index < 2**63
    ):
        raise ValueError("frame_index must be a nonnegative signed int64")
    payload = (
        _float32_vector(token, 64, "token")
        + struct.pack("<q", frame_index)
        + _float32_vector(left_hand, 7, "left_hand")
        + _float32_vector(right_hand, 7, "right_hand")
    )
    fields = [
        {"name": "token_state", "dtype": "f32", "shape": [1, 64]},
        {"name": "frame_index", "dtype": "i64", "shape": [1]},
        {"name": "left_hand_joints", "dtype": "f32", "shape": [1, 7]},
        {"name": "right_hand_joints", "dtype": "f32", "shape": [1, 7]},
    ]
    return _message(b"pose", 4, fields, payload)
