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

"""Wire format for the vr-world <-> Quest WebSocket channel.

Same envelope as memory_browser/memory_world:
``[1B type][4B header_json_len LE][header JSON][payload]`` for binary,
JSON object with a ``type`` field for text.

Server -> client:
  binary: MSG_VOXEL_MAP (full map resend, throttled), MSG_CAMERA (jpeg)
  text:   robot_pose, status
Client -> server:
  text:   drive {x, yaw}, ping, diag
"""

from __future__ import annotations

import json
import struct
from typing import Any

# Binary message types (server -> client).
MSG_VOXEL_MAP = 0x01
MSG_CAMERA = 0x02


def encode_binary(msg_type: int, header: dict[str, Any], payload: bytes) -> bytes:
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return bytes([msg_type & 0xFF]) + struct.pack("<I", len(header_bytes)) + header_bytes + payload


def encode_text(msg_type: str, **fields: Any) -> str:
    return json.dumps({"type": msg_type, **fields}, separators=(",", ":"))


def decode_text(raw: str) -> dict[str, Any]:
    try:
        msg = json.loads(raw)
        return msg if isinstance(msg, dict) else {}
    except (json.JSONDecodeError, ValueError):
        return {}
