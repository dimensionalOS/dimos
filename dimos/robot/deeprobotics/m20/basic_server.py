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

"""Framing for the M20's vendor basic_server (AOS): a 16-byte header, then a PatrolDevice JSON body."""

from datetime import datetime
import json
import struct
from typing import Any

MAGIC = bytes.fromhex("eb91eb90")
JSON_ENCODING = 1
HEADER = struct.Struct("<4sHHB7s")


def encode_frame(message_type: int, command: int, items: dict[str, Any], message_id: int) -> bytes:
    payload = json.dumps(
        {
            "PatrolDevice": {
                "Type": message_type,
                "Command": command,
                "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Items": items,
            }
        },
        separators=(",", ":"),
    ).encode()
    if len(payload) > 0xFFFF:
        raise ValueError("M20 basic_server payload exceeds the APDU limit")
    return HEADER.pack(MAGIC, len(payload), message_id & 0xFFFF, JSON_ENCODING, b"\0" * 7) + payload


def decode_frame(frame: bytes) -> dict[str, Any] | None:
    """A frame's PatrolDevice dict, or None for anything that is not a JSON basic_server frame."""
    if len(frame) < HEADER.size:
        return None
    magic, length, _message_id, encoding, _reserved = HEADER.unpack_from(frame)
    if magic != MAGIC or encoding != JSON_ENCODING:
        return None
    try:
        device = json.loads(frame[HEADER.size : HEADER.size + length])["PatrolDevice"]
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError):
        return None
    return device if isinstance(device, dict) else None
