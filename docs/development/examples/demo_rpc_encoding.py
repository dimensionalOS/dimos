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

"""Demonstrate an RPC round trip with MessagePack + LCM, using in-memory transport.

Run from the repository root:
    uv run --no-sync --with msgpack python docs/development/examples/demo_rpc_encoding.py
"""

from typing import Any

import msgpack

from dimos.msgs.geometry_msgs.Vector3 import Vector3

LCM_EXT = 1  # This example reserves extension code 1 for LCM messages.
VECTOR3_TYPE = "geometry_msgs.Vector3"


def calculate(vector: Vector3, scale: float) -> Vector3:
    """The business function only handles decoded Python values."""
    return Vector3(vector.x * scale, vector.y * scale, vector.z * scale)


def encode_lcm(value: Any) -> msgpack.ExtType:
    """Store the type name and full LCM bytes when MessagePack encounters a Vector3."""
    if type(value) is not Vector3:
        raise TypeError(f"Unsupported type: {type(value).__name__}")
    content = msgpack.packb({"type": VECTOR3_TYPE, "data": value.lcm_encode()}, use_bin_type=True)
    return msgpack.ExtType(LCM_EXT, content)


def decode_lcm(code: int, data: bytes) -> Vector3:
    """Select the decoder using the agreed extension code and type name."""
    if code != LCM_EXT:
        raise ValueError(f"Unknown extension: {code}")
    content = msgpack.unpackb(data, raw=False)
    if content["type"] != VECTOR3_TYPE:
        raise ValueError(f"Unknown LCM type: {content['type']}")
    vector = Vector3.lcm_decode(content["data"])
    assert type(vector) is Vector3
    return vector


def pack(value: Any) -> bytes:
    payload: bytes = msgpack.packb(value, default=encode_lcm, use_bin_type=True)
    return payload


def unpack(payload: bytes) -> Any:
    return msgpack.unpackb(payload, ext_hook=decode_lcm, raw=False)


def handle_calculate(request: bytes) -> bytes:
    """The server receives and returns bytes; Zenoh would transport them in a real RPC."""
    params = unpack(request)
    vector = params["args"][0]
    scale = params["kwargs"]["scale"]
    assert type(vector) is Vector3
    assert type(scale) is float
    print(f"3. Server decodes: {type(vector).__name__}{vector.as_tuple}, scale={scale}")

    result = calculate(*params["args"], **params["kwargs"])
    print(f"4. Execute calculate: Vector3{result.as_tuple}")
    reply = pack(result)
    print(f"5. Server encodes the return value: {len(reply)} bytes")
    return reply


def main() -> None:
    print("In-memory demo: in a real RPC, Zenoh transports the request and reply bytes.")
    print("1. Client call: calculate(Vector3(1, 2, 3), scale=2.0)")
    request = pack({"args": [Vector3(1, 2, 3)], "kwargs": {"scale": 2.0}})
    print(f"2. Client encodes the request: {len(request)} bytes")
    print(f"   Request in hex: {request.hex()}")

    # Omit ext_hook to inspect what a standard MessagePack decoder can recognize.
    wire = msgpack.unpackb(request, raw=False)
    extension = wire["args"][0]
    assert isinstance(extension, msgpack.ExtType)
    content = msgpack.unpackb(extension.data, raw=False)
    print(f"   Standard MessagePack decoding: args[0] is ExtType, code={extension.code}")
    print(f"   Extension content: type={content['type']}, data={len(content['data'])} LCM bytes")
    print(f"   Regular argument: scale={wire['kwargs']['scale']}, no LCM decoder needed")

    reply = handle_calculate(request)
    answer = unpack(reply)
    assert type(answer) is Vector3
    assert answer.as_tuple == (2.0, 4.0, 6.0)
    print(f"6. Client decodes: {type(answer).__name__}{answer.as_tuple}")


if __name__ == "__main__":
    main()
