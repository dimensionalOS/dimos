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
and deno test). Validation runs on pydantic; nothing here needs aioquic or the
rest of the [web] extra.

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

from dataclasses import dataclass
import struct
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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


class _WireModel(BaseModel):
    # strict: no coercion, so a bool or "1" is not a protocol number (mirrors
    # the typeof checks in protocol.ts). allow_inf_nan=False: Python's JSON
    # parser accepts NaN/Infinity where JSON.parse errors, so the mirror must
    # reject them explicitly -- and must refuse to encode them locally.
    model_config = ConfigDict(strict=True, allow_inf_nan=False)


# Number fields are `int | float`, not `float`: JSON has a single number type
# (booleans excluded above), and plain `float` would coerce ints and serialize
# seq=1 as 1.0, breaking byte-exact encoding against the golden fixtures.


class Hello(_WireModel):
    t: Literal["hello"] = "hello"
    v: int | float
    role: Role


class Welcome(_WireModel):
    t: Literal["welcome"] = "welcome"
    v: int | float


class Ping(_WireModel):
    t: Literal["ping"] = "ping"
    n: int | float
    ts: int | float


class Pong(_WireModel):
    t: Literal["pong"] = "pong"
    n: int | float
    ts: int | float


class Error(_WireModel):
    t: Literal["error"] = "error"
    code: str
    message: str


# Teleop datagrams (carried from T6 on; declared so the wire format is pinned
# by fixtures from day one).
class Twist(_WireModel):
    t: Literal["twist"] = "twist"
    vx: int | float
    wz: int | float
    seq: int | float
    ts: int | float


class Stop(_WireModel):
    t: Literal["stop"] = "stop"
    seq: int | float
    ts: int | float


Msg = Hello | Welcome | Ping | Pong | Error | Twist | Stop

# One pydantic-core pass takes raw peer bytes to a validated message: UTF-8
# decoding, JSON parsing, and shape checks together, discriminated on "t".
_MSG_TA: TypeAdapter[Msg] = TypeAdapter(Annotated[Msg, Field(discriminator="t")])


class FrameHeader(_WireModel):
    """Data-plane frame header.

    `delivery` tells the relay how to forward without a manifest (T1 only; the
    T2+ manifest replaces it). `meta` carries encoding-specific extras.
    """

    ch: str
    seq: int | float
    ts: int | float
    delivery: Delivery
    meta: dict[str, Any] | None = None


@dataclass
class DataFrame:
    header: FrameHeader
    payload: bytes


def msg_from_dict(data: dict[str, Any]) -> Msg:
    try:
        return _MSG_TA.validate_python(data)
    except ValidationError as e:
        raise ProtocolError(f"invalid message: {e}") from e


def _msg_from_json(data: bytes) -> Msg:
    try:
        return _MSG_TA.validate_json(data)
    except ValidationError as e:
        raise ProtocolError(f"invalid message: {e}") from e


def encode_control_frame(msg: Msg) -> bytes:
    body = msg.model_dump_json().encode()
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
            if length == 0 or length > MAX_HEADER_LEN:
                raise ProtocolError(f"invalid control frame length: {length}")
            if len(self._buf) < 4 + length:
                break
            body = bytes(self._buf[4 : 4 + length])
            del self._buf[: 4 + length]
            try:
                msgs.append(_msg_from_json(body))
            except ProtocolError as e:
                logger.warning(f"dropping bad control message: {e}")
        return msgs


def encode_datagram(msg: Msg) -> bytes:
    return msg.model_dump_json().encode()


def decode_datagram(data: bytes) -> Msg | None:
    """Returns None for datagrams that are not our JSON messages."""
    try:
        return _MSG_TA.validate_json(data)
    except ValidationError:
        return None


def encode_data_frame(header: FrameHeader, payload: bytes) -> bytes:
    hdr = header.model_dump_json(exclude_none=True).encode()
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


def decode_data_frame(frame: bytes | bytearray) -> DataFrame:
    lens = peek_data_frame_lengths(frame)
    if lens is None or len(frame) < lens[2]:
        raise ProtocolError(f"truncated data frame: {len(frame)} bytes")
    header_len, _, total = lens
    view = memoryview(frame)
    try:
        header = FrameHeader.model_validate_json(bytes(view[8 : 8 + header_len]))
    except ValidationError as e:
        raise ProtocolError(f"bad data frame header: {e}") from e
    # The payload slice is the only whole-payload copy on the receive path.
    return DataFrame(header=header, payload=bytes(view[8 + header_len : total]))


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
