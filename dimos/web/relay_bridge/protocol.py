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

"""Wire-protocol mirror of web/shared/protocol.ts.

Pinned by the golden vectors in web/shared/fixtures/ (tested from both pytest
and deno test). Stdlib-only on purpose: importable without aioquic.

Framing (see web/README.md for the upstream-bug rationale):
- Control stream frame: u32-LE length | UTF-8 JSON.
- Datagram: raw UTF-8 JSON, no length prefix.
- Data frame (one message per stream): u32-LE headerLen | u32-LE payloadLen |
  header JSON | payload. Receivers count bytes and must never treat stream
  EOF as a message boundary (Deno 2.6.x delays FIN by up to ~1 s).

Validation policy (mirrored in protocol.ts): decoders validate shape strictly,
and receivers drop invalid or unknown messages -- a peer's bytes must never
kill a session. Framing-level corruption (absurd length prefixes) raises
ProtocolError and kills only the affected stream.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import json
import logging
import struct
from typing import Any, ClassVar, Literal, Union, cast

# Stdlib logging, not dimos.utils.logging_config (which needs structlog):
# this module must stay importable with no dependencies at all.
logger = logging.getLogger(__name__)

PROTOCOL_VERSION = 1

# Reject absurd header lengths before allocating (mirrors protocol.ts).
MAX_HEADER_LEN = 65536

# Upper bound for a whole data frame; guards receivers against buffering a
# hostile/corrupt payloadLen (same constant as the relay's ingress cap).
MAX_DATA_FRAME_BYTES = 64 * 1024 * 1024

Role = Literal["robot", "viewer"]
Delivery = Literal["latest", "reliable"]


class ProtocolError(ValueError):
    pass


@dataclass
class Hello:
    t: ClassVar[str] = "hello"
    v: int
    role: Role


@dataclass
class Welcome:
    t: ClassVar[str] = "welcome"
    v: int


@dataclass
class Ping:
    t: ClassVar[str] = "ping"
    n: int
    ts: float


@dataclass
class Pong:
    t: ClassVar[str] = "pong"
    n: int
    ts: float


@dataclass
class Error:
    t: ClassVar[str] = "error"
    code: str
    message: str


# Teleop datagrams (carried from T6 on; declared so the wire format is pinned
# by fixtures from day one).
@dataclass
class Twist:
    t: ClassVar[str] = "twist"
    vx: float
    wz: float
    seq: int
    ts: float


@dataclass
class Stop:
    t: ClassVar[str] = "stop"
    seq: int
    ts: float


Msg = Union[Hello, Welcome, Ping, Pong, Error, Twist, Stop]

_MSG_TYPES: dict[str, type[Any]] = {
    cls.t: cls for cls in (Hello, Welcome, Ping, Pong, Error, Twist, Stop)
}

# Runtime field validation, derived from the dataclass annotations (mirrors
# MSG_FIELDS in protocol.ts): "string" is a JSON string, "number" any JSON
# number except booleans. An unmapped annotation fails loudly at import.
_KIND_BY_ANNOTATION = {"int": "number", "float": "number", "str": "string", "Role": "string"}
_MSG_FIELD_KINDS: dict[str, dict[str, str]] = {
    # cast: `from __future__ import annotations` makes f.type always a str.
    t: {f.name: _KIND_BY_ANNOTATION[cast("str", f.type)] for f in fields(cls)}
    for t, cls in _MSG_TYPES.items()
}


def _is_kind(value: Any, kind: str) -> bool:
    if kind == "string":
        return isinstance(value, str)
    return isinstance(value, (int, float)) and not isinstance(value, bool)


@dataclass
class FrameHeader:
    """Data-plane frame header.

    `delivery` tells the relay how to forward without a manifest (T1 only; the
    T2+ manifest replaces it). `meta` carries encoding-specific extras.
    """

    ch: str
    seq: int
    ts: float
    delivery: Delivery
    meta: dict[str, Any] | None = None


@dataclass
class DataFrame:
    header: FrameHeader
    payload: bytes


def _dump_json(obj: dict[str, Any]) -> bytes:
    # Canonical form shared with JSON.stringify: compact separators, raw UTF-8.
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode()


def _msg_to_dict(msg: Msg) -> dict[str, Any]:
    out: dict[str, Any] = {"t": type(msg).t}
    for f in fields(msg):
        out[f.name] = getattr(msg, f.name)
    return out


def msg_from_dict(data: dict[str, Any]) -> Msg:
    t = data.get("t")
    if not isinstance(t, str) or t not in _MSG_TYPES:
        raise ProtocolError(f"unknown message type: {t!r}")
    cls = _MSG_TYPES[t]
    for name, kind in _MSG_FIELD_KINDS[t].items():
        if not _is_kind(data.get(name), kind):
            raise ProtocolError(f"invalid {t} message: field {name!r} is not a {kind}")
    msg: Msg = cls(**{f.name: data[f.name] for f in fields(cls)})
    return msg


def encode_control_frame(msg: Msg) -> bytes:
    body = _dump_json(_msg_to_dict(msg))
    return struct.pack("<I", len(body)) + body


class ControlFrameReader:
    """Incremental parser for a control stream (frames may split across chunks).

    Malformed or unknown messages are dropped with a log line (the length
    prefix keeps framing intact); framing errors still raise ProtocolError.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def push(self, chunk: bytes) -> list[Msg]:
        self._buf += chunk
        msgs: list[Msg] = []
        while len(self._buf) >= 4:
            (length,) = struct.unpack_from("<I", self._buf, 0)
            if length > MAX_HEADER_LEN:
                raise ProtocolError(f"control frame too large: {length}")
            if len(self._buf) < 4 + length:
                break
            body_bytes = self._buf[4 : 4 + length]
            del self._buf[: 4 + length]
            try:
                body = json.loads(body_bytes.decode())
                if not isinstance(body, dict):
                    raise ProtocolError("control frame is not a JSON object")
                msgs.append(msg_from_dict(body))
            except ValueError as e:  # ProtocolError, bad JSON, and bad UTF-8
                logger.warning(f"dropping bad control message: {e}")
        return msgs


def encode_datagram(msg: Msg) -> bytes:
    return _dump_json(_msg_to_dict(msg))


def decode_datagram(data: bytes) -> Msg | None:
    """Returns None for datagrams that are not our JSON messages."""
    try:
        body = json.loads(data.decode())
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(body, dict):
        return None
    try:
        return msg_from_dict(body)
    except ProtocolError:
        return None


def _header_to_dict(header: FrameHeader) -> dict[str, Any]:
    out: dict[str, Any] = {
        "ch": header.ch,
        "seq": header.seq,
        "ts": header.ts,
        "delivery": header.delivery,
    }
    if header.meta is not None:
        out["meta"] = header.meta
    return out


def encode_data_frame(header: FrameHeader, payload: bytes) -> bytes:
    hdr = _dump_json(_header_to_dict(header))
    return struct.pack("<II", len(hdr), len(payload)) + hdr + payload


def peek_data_frame_lengths(buf: bytes | bytearray) -> tuple[int, int, int] | None:
    """(headerLen, payloadLen, total) or None if fewer than 8 bytes are available."""
    if len(buf) < 8:
        return None
    header_len, payload_len = struct.unpack_from("<II", buf, 0)
    if header_len > MAX_HEADER_LEN:
        raise ProtocolError(f"data frame header too large: {header_len}")
    total = 8 + header_len + payload_len
    if total > MAX_DATA_FRAME_BYTES:
        raise ProtocolError(f"data frame too large: {total} bytes")
    return header_len, payload_len, total


def _frame_header_from_dict(body: dict[str, Any]) -> FrameHeader:
    ch, seq, ts, delivery = body.get("ch"), body.get("seq"), body.get("ts"), body.get("delivery")
    meta = body.get("meta")
    if (
        not isinstance(ch, str)
        or not _is_kind(seq, "number")
        or not _is_kind(ts, "number")
        or delivery not in ("latest", "reliable")
        or ("meta" in body and not isinstance(meta, dict))
    ):
        raise ProtocolError(f"invalid data frame header: {body!r}")
    # cast: _is_kind proved seq/ts are numbers, which is as precise as JSON
    # gets (no int/float split on the wire, mirroring protocol.ts).
    return FrameHeader(
        ch=ch, seq=cast("int", seq), ts=cast("float", ts), delivery=delivery, meta=meta
    )


def decode_data_frame(frame: bytes | bytearray) -> DataFrame:
    lens = peek_data_frame_lengths(frame)
    if lens is None or len(frame) < lens[2]:
        raise ProtocolError(f"truncated data frame: {len(frame)} bytes")
    header_len, _, total = lens
    view = memoryview(frame)
    try:
        body = json.loads(bytes(view[8 : 8 + header_len]).decode())
    except ValueError as e:  # bad JSON or bad UTF-8
        raise ProtocolError(f"bad data frame header: {e}") from e
    if not isinstance(body, dict):
        raise ProtocolError("data frame header is not a JSON object")
    # The payload slice is the only whole-payload copy on the receive path.
    return DataFrame(
        header=_frame_header_from_dict(body), payload=bytes(view[8 + header_len : total])
    )


class DataFrameReader:
    """Incremental reader for a single-message stream.

    Returns the frame as soon as headerLen + payloadLen bytes have arrived;
    never waits for EOF. Bytes past the frame are ignored.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._done = False

    def push(self, chunk: bytes) -> DataFrame | None:
        if self._done:
            return None
        self._buf += chunk
        lens = peek_data_frame_lengths(self._buf)
        if lens is None or len(self._buf) < lens[2]:
            return None
        self._done = True
        frame = decode_data_frame(self._buf)
        self._buf = bytearray()
        return frame
