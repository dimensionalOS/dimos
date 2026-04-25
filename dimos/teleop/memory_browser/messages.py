"""Wire format for the memory-browser <-> Quest WebSocket channel.

Two flavors travel over one ``/ws_memory`` socket:

* Text frames — JSON control messages, both directions.
* Binary frames — server → client only. Carry image bytes with a small JSON
  header describing them. Layout::

      [1 byte type][4 bytes header_json_len, little-endian][header JSON][JPEG bytes]

The 1-byte type lets the JS client dispatch quickly; the JSON header carries
metadata (id, ts, stream name, layout slot) without needing a binary schema.
"""

from __future__ import annotations

import json
import struct
from typing import Any

# Binary message types
MSG_THUMBNAIL = 0x01
MSG_ACTIVE_FRAME = 0x02
MSG_GLOBAL_MAP = 0x03


def encode_binary(msg_type: int, header: dict[str, Any], payload: bytes) -> bytes:
    """Pack a binary frame with a JSON header and raw payload bytes."""
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return (
        bytes([msg_type & 0xFF])
        + struct.pack("<I", len(header_bytes))
        + header_bytes
        + payload
    )


def encode_text(msg_type: str, **fields: Any) -> str:
    """Pack a text frame as a JSON object with ``type`` plus arbitrary fields."""
    return json.dumps({"type": msg_type, **fields}, separators=(",", ":"))


def decode_text(raw: str) -> dict[str, Any]:
    """Decode a JSON text frame. Returns {} on parse failure."""
    try:
        msg = json.loads(raw)
        return msg if isinstance(msg, dict) else {}
    except (json.JSONDecodeError, ValueError):
        return {}
