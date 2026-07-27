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

"""Golden-fixture tests keeping protocol.py byte-identical to protocol.ts."""

import base64
import json
import struct

import pytest

from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.protocol import (
    MAX_DATA_FRAME_BYTES,
    MAX_HEADER_LEN,
    PROTOCOL_VERSION,
    ChannelSpec,
    ControlFrameReader,
    DataFrameReader,
    FrameHeader,
    Hello,
    Ping,
    ProtocolError,
    RobotInfo,
    RobotManifest,
    Robots,
    decode_data_frame,
    decode_datagram,
    encode_control_frame,
    encode_data_frame,
    encode_datagram,
    msg_from_dict,
    peek_data_frame_lengths,
)

FIXTURES = find_web_dir() / "shared" / "fixtures"


def _vectors(name):
    with open(FIXTURES / name) as f:
        return json.load(f)["vectors"]


CONTROL = _vectors("control_frames.json")
DATAGRAMS = _vectors("datagrams.json")
DATA = _vectors("data_frames.json")


def _header(d):
    return FrameHeader(
        ch=d["ch"], seq=d["seq"], ts=d["ts"], delivery=d["delivery"], meta=d.get("meta")
    )


def test_protocol_version():
    assert PROTOCOL_VERSION == 1


@pytest.mark.parametrize("vector", CONTROL, ids=[v["name"] for v in CONTROL])
def test_control_frame_encode_matches_golden(vector):
    msg = msg_from_dict(vector["message"])
    assert encode_control_frame(msg) == base64.b64decode(vector["b64"])


def test_control_frame_reader_decodes_golden_stream():
    stream = b"".join(base64.b64decode(v["b64"]) for v in CONTROL)
    msgs = ControlFrameReader().push(stream)
    assert msgs == [msg_from_dict(v["message"]) for v in CONTROL]


def test_control_frame_reader_survives_every_split():
    stream = b"".join(base64.b64decode(v["b64"]) for v in CONTROL)
    expected = [msg_from_dict(v["message"]) for v in CONTROL]
    for split in range(len(stream) + 1):
        reader = ControlFrameReader()
        msgs = reader.push(stream[:split]) + reader.push(stream[split:])
        assert msgs == expected, f"split at {split}"


def test_control_frame_reader_rejects_absurd_length():
    with pytest.raises(ProtocolError):
        ControlFrameReader().push(struct.pack("<I", MAX_HEADER_LEN + 1))


def test_control_frame_reader_rejects_zero_length():
    # An encoder can never produce one, so treat it as framing corruption
    # instead of warn-per-4-bytes on a hostile chunk of zeros.
    with pytest.raises(ProtocolError):
        ControlFrameReader().push(bytes(4))


@pytest.mark.parametrize("vector", DATAGRAMS, ids=[v["name"] for v in DATAGRAMS])
def test_datagram_golden_roundtrip(vector):
    msg = msg_from_dict(vector["message"])
    raw = base64.b64decode(vector["b64"])
    assert encode_datagram(msg) == raw
    assert decode_datagram(raw) == msg


def test_datagram_junk_returns_none():
    assert decode_datagram(b"\xff\x00\x80") is None
    assert decode_datagram(b"[1,2]") is None
    assert decode_datagram(b'{"x":1}') is None


@pytest.mark.parametrize("vector", DATA, ids=[v["name"] for v in DATA])
def test_data_frame_encode_matches_golden(vector):
    frame = encode_data_frame(_header(vector["header"]), base64.b64decode(vector["payload_b64"]))
    assert frame == base64.b64decode(vector["frame_b64"])


@pytest.mark.parametrize("vector", DATA, ids=[v["name"] for v in DATA])
def test_data_frame_decode_roundtrips_golden(vector):
    frame = decode_data_frame(base64.b64decode(vector["frame_b64"]))
    assert frame.header == _header(vector["header"])
    assert frame.payload == base64.b64decode(vector["payload_b64"])


def test_data_frame_reader_completes_at_byte_count_split_anywhere():
    vector = next(v for v in DATA if v["name"] == "image_latest_meta")
    frame_bytes = base64.b64decode(vector["frame_b64"])
    for split in range(len(frame_bytes) + 1):
        reader = DataFrameReader()
        first = reader.push(frame_bytes[:split])
        second = reader.push(frame_bytes[split:])
        if split < len(frame_bytes):
            assert first is None, f"complete before full frame at split {split}"
        out = first or second
        assert out is not None, f"incomplete after full frame at split {split}"
        assert out.header == _header(vector["header"])
        assert out.payload == base64.b64decode(vector["payload_b64"])


def test_data_frame_reader_ignores_bytes_past_frame():
    vector = next(v for v in DATA if v["name"] == "odom_reliable")
    frame_bytes = base64.b64decode(vector["frame_b64"])
    out = DataFrameReader().push(frame_bytes + b"\x00" * 32)
    assert out is not None
    assert out.header == _header(vector["header"])


def test_peek_and_decode_guard_truncation_and_absurd_headers():
    assert peek_data_frame_lengths(b"\x00" * 7) is None
    frame_bytes = base64.b64decode(DATA[0]["frame_b64"])
    with pytest.raises(ProtocolError):
        decode_data_frame(frame_bytes[:-1])
    with pytest.raises(ProtocolError):
        peek_data_frame_lengths(struct.pack("<II", MAX_HEADER_LEN + 1, 0))


def _raw_data_frame(header_bytes: bytes, payload: bytes = b"") -> bytes:
    return struct.pack("<II", len(header_bytes), len(payload)) + header_bytes + payload


@pytest.mark.parametrize(
    "header_bytes",
    [
        b"\xff\xfe\xfd",  # invalid UTF-8
        b"{not json",  # malformed JSON
        b"[1,2]",  # not a JSON object
        json.dumps({"ch": "c", "ts": 1.0, "delivery": "latest"}).encode(),  # missing seq
        json.dumps({"ch": "c", "seq": "x", "ts": 1.0, "delivery": "latest"}).encode(),  # seq type
        json.dumps({"ch": "c", "seq": 1, "ts": 1.0, "delivery": "bogus"}).encode(),  # delivery
    ],
    ids=["bad_utf8", "bad_json", "not_object", "missing_seq", "seq_wrong_type", "bad_delivery"],
)
def test_decode_data_frame_rejects_malformed_header(header_bytes):
    with pytest.raises(ProtocolError):
        decode_data_frame(_raw_data_frame(header_bytes))


def test_peek_rejects_oversize_total():
    with pytest.raises(ProtocolError):
        peek_data_frame_lengths(struct.pack("<II", 2, MAX_DATA_FRAME_BYTES))


def test_msg_from_dict_validates_types():
    assert msg_from_dict({"t": "ping", "n": 1, "ts": 2.5}) is not None
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "ping", "n": "1", "ts": 2.5})  # n not a number
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "ping", "ts": 2.5})  # missing n
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "bogus"})  # unknown type
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "ping", "n": True, "ts": 2.5})  # bool is not a number
    # Mirrored-validator parity: protocol.ts must also reject prototype-chain
    # keys instead of resolving them through Object.prototype (protocol_test.ts
    # asserts the same three).
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "toString"})
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "constructor"})
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "hasOwnProperty"})


def test_msg_from_dict_validates_nested_session_shapes():
    robot = {"id": "go2-lab", "name": "Go2 Lab", "model": "unitree-go2"}
    spec = {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 20.5}
    full = {"t": "hello", "v": 1, "role": "robot", "robot": robot, "manifest": {"channels": [spec]}}
    assert msg_from_dict(full) == Hello(
        v=1,
        role="robot",
        robot=RobotInfo(id="go2-lab", name="Go2 Lab", model="unitree-go2"),
        manifest=RobotManifest(
            channels=[
                ChannelSpec(ch="odom", encoding="pose.json.v1", delivery="reliable", maxHz=20.5)
            ]
        ),
    )
    # hello stays valid without the optional robot/manifest (viewer form).
    assert msg_from_dict({"t": "hello", "v": 1, "role": "viewer"}) == Hello(v=1, role="viewer")
    bad = [
        # Optional means absent-or-valid: explicit null is rejected on the
        # wire (local construction with robot=None stays fine: absent).
        {"t": "hello", "v": 1, "role": "robot", "robot": None},
        {**full, "robot": {"id": 5, "name": "x", "model": "m"}},
        {**full, "manifest": {"channels": [{**spec, "maxHz": "20"}]}},
        {**full, "manifest": {"channels": [{**spec, "delivery": "bogus"}]}},
        {**full, "manifest": {"channels": robot}},
        {"t": "robots", "robots": {}},
        {"t": "robots", "robots": [{"id": "a", "name": "b"}]},
        {"t": "robots"},
        {"t": "manifest", "channels": [spec]},
        {"t": "watch"},
        {"t": "subs", "chs": ["a", 5], "n": 1},
        {"t": "subs", "chs": ["a"]},
    ]
    for data in bad:
        with pytest.raises(ProtocolError):
            msg_from_dict(data)


def test_encode_omits_absent_optional_fields():
    # A viewer hello must stay byte-identical to its T1 wire form: no
    # "robot":null / "manifest":null keys (JSON.stringify omits undefined).
    assert encode_datagram(Hello(v=1, role="viewer")) == b'{"t":"hello","v":1,"role":"viewer"}'


def test_nested_roundtrip_returns_models():
    msg = Robots(robots=[RobotInfo(id="a", name="A", model="m")])
    decoded = decode_datagram(encode_datagram(msg))
    assert decoded == msg
    assert isinstance(decoded.robots[0], RobotInfo)
    # Extra wire keys inside nested objects are ignored (forward compat).
    extra = {"t": "watch", "robotId": "r", "later": 1}
    assert msg_from_dict(extra) == msg_from_dict({"t": "watch", "robotId": "r"})


def test_non_finite_numbers_rejected():
    # Python's JSON parser accepts NaN/Infinity where the TS mirror's
    # JSON.parse errors; the validator must reject them, and a local encode
    # must fail fast instead of emitting wire JSON the relay cannot parse.
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "ping", "n": 1, "ts": float("nan")})
    with pytest.raises(ProtocolError):
        msg_from_dict({"t": "ping", "n": float("inf"), "ts": 2.5})
    assert decode_datagram(b'{"t":"ping","n":7,"ts":NaN}') is None
    with pytest.raises(ValueError):
        Ping(n=1, ts=float("nan"))


def test_data_frame_header_rejects_non_finite():
    hdr = b'{"ch":"c","seq":1,"ts":1e999,"delivery":"latest"}'
    with pytest.raises(ProtocolError):
        decode_data_frame(_raw_data_frame(hdr))


def test_huge_int_is_a_valid_number():
    # Arbitrary-precision ints are legal JSON and always finite; they must
    # pass (a math.isfinite check would raise OverflowError on them).
    msg = msg_from_dict({"t": "ping", "n": 10**400, "ts": 2.5})
    assert isinstance(msg, Ping) and msg.n == 10**400


def test_control_reader_drops_invalid_keeps_valid_neighbors():
    hello = encode_control_frame(
        msg_from_dict({"t": "hello", "v": PROTOCOL_VERSION, "role": "viewer"})
    )
    ping = encode_control_frame(msg_from_dict({"t": "ping", "n": 3, "ts": 4.5}))
    junk = struct.pack("<I", len(b"null")) + b"null"  # well-framed, invalid message
    msgs = ControlFrameReader().push(hello + junk + ping)
    assert msgs == [
        msg_from_dict({"t": "hello", "v": PROTOCOL_VERSION, "role": "viewer"}),
        msg_from_dict({"t": "ping", "n": 3, "ts": 4.5}),
    ]
