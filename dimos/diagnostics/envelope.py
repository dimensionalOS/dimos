# Copyright 2025-2026 Dimensional Inc.
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

"""Explicit traced-link framing. Never sniff or modify a legacy payload."""

from __future__ import annotations

import hashlib
import json
import struct

from dimos.diagnostics.model import MAX_CONTEXT_BYTES, TraceContext

MAGIC = b"DIMOSTR1"
MAX_PAYLOAD = 64 * 1024 * 1024


def pack(link_id: str, message_type: str, context: TraceContext, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    metadata = json.dumps(
        {"link": link_id, "type": message_type, "context": json.loads(context.encode())},
        separators=(",", ":"),
    ).encode()
    if len(metadata) > MAX_CONTEXT_BYTES:
        raise ValueError("metadata too large")
    return MAGIC + struct.pack("!I", len(metadata)) + metadata + payload


def unpack(data: bytes, link_id: str, message_type: str) -> tuple[TraceContext, bytes]:
    if len(data) < 12 or data[:8] != MAGIC:
        raise ValueError("traced link requires v1 envelope")
    size = struct.unpack("!I", data[8:12])[0]
    if size > MAX_CONTEXT_BYTES or len(data) < 12 + size or len(data) - 12 - size > MAX_PAYLOAD:
        raise ValueError("invalid envelope length")
    meta = json.loads(data[12 : 12 + size])
    if (
        not isinstance(meta, dict)
        or meta.get("link") != link_id
        or meta.get("type") != message_type
    ):
        raise ValueError("traced link/type mismatch")
    return TraceContext.decode(json.dumps(meta["context"]).encode()), data[12 + size :]


def traced_channel(run_id: str, link_id: str) -> str:
    """A distinct channel valid as an LCM channel, ROS topic or DDS topic name."""
    return (
        "dimos_trace_v1_" + hashlib.sha256(json.dumps([run_id, link_id]).encode()).hexdigest()[:32]
    )
