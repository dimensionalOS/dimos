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

"""One in-memory CBOR RPC call with native values, LCM, pickle, and CDDL.

Run from the repository root:
    uv run --no-sync --with cbor2==6.1.5 --with pycddl==0.6.4 python -m dimos.spec.demo_cbor_rpc

This is a trusted, same-process example, not a replacement for ZenohRPC.
"""

from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import pickle
from typing import Any

import cbor2
from pycddl import Schema

from dimos.msgs.geometry_msgs.Vector3 import Vector3

RPC_VERSION = "dimos-cbor/1"
LCM_TAG = 60003  # Provisional draft numbers, not registered or private-use allocations.
PICKLE_TAG = 60004
PICKLE_PROTOCOL = 5
VECTOR3_TYPE = "geometry_msgs.Vector3"


@dataclass
class Label:
    name: str


def to_wire(value: Any) -> Any:
    """Use an explicit native subset so tuple/Enum types are not silently lost."""
    if type(value) in (type(None), bool, float, str, bytes):
        return value
    if type(value) is int and -(2**64) <= value < 2**64:
        return value
    if type(value) is list:
        return [to_wire(item) for item in value]
    if type(value) is dict and all(type(key) is str for key in value):
        return {key: to_wire(item) for key, item in value.items()}
    if type(value) is Vector3:
        return cbor2.CBORTag(LCM_TAG, [VECTOR3_TYPE, value.lcm_encode()])
    return cbor2.CBORTag(PICKLE_TAG, pickle.dumps(value, protocol=PICKLE_PROTOCOL))


def restore(value: Any, *, allow_pickle: bool = False) -> Any:
    """Restore extensions only after validating the complete wire message."""
    if isinstance(value, cbor2.CBORTag):
        if value.tag == LCM_TAG:
            type_id, payload = value.value
            if type_id != VECTOR3_TYPE:
                raise ValueError(f"Unregistered LCM type: {type_id}")
            return Vector3.lcm_decode(payload)
        if value.tag == PICKLE_TAG:
            if not allow_pickle:
                raise ValueError("Pickle requires a trusted peer and explicit opt-in")
            return pickle.loads(value.value)
        raise ValueError(f"Unknown tag: {value.tag}")
    if isinstance(value, list):
        return [restore(item, allow_pickle=allow_pickle) for item in value]
    if isinstance(value, dict):
        return {key: restore(item, allow_pickle=allow_pickle) for key, item in value.items()}
    return value


def pack(message: dict[str, Any]) -> bytes:
    return cbor2.dumps(to_wire(message))


def unpack(payload: bytes, schema: Schema) -> dict[str, Any]:
    """Validate the envelope and retain tags for the caller's routing/trust checks."""
    schema.validate_cbor(payload)
    stream = BytesIO(payload)
    wire: dict[str, Any] = cbor2.load(
        stream, read_size=1, allow_duplicate_keys=False, allow_indefinite=False
    )
    if stream.read(1):
        raise ValueError("Expected exactly one CBOR message")
    return wire


def calculate(vector: Vector3, scale: float, label: Label) -> dict[str, Any]:
    return {"vector": Vector3(vector.x * scale, vector.y * scale, vector.z * scale), "label": label}


def run_demo(schema: Schema) -> None:
    request = {
        "rpc": RPC_VERSION,
        "id": "demo:1",
        "method": "Calculator/calculate",
        "params": {"args": [Vector3(1, 2, 3)], "kwargs": {"scale": 2.0, "label": Label("demo")}},
    }
    request_bytes = pack(request)
    wire = unpack(request_bytes, schema)
    print(f"Request: {len(request_bytes)} bytes, method={wire['method']}")
    print(f"  vector: tag {wire['params']['args'][0].tag} (LCM)")
    print(f"  label: tag {wire['params']['kwargs']['label'].tag} (pickle)")
    print(f"  scale: {wire['params']['kwargs']['scale']} (native CBOR)")

    # This fixed server expects Calculator/calculate; only its own local input is trusted.
    assert wire["method"] == "Calculator/calculate"
    params = restore(wire["params"], allow_pickle=True)
    result = calculate(*params["args"], **params["kwargs"])
    reply_bytes = pack({"rpc": RPC_VERSION, "id": wire["id"], "result": result})

    reply = unpack(reply_bytes, schema)
    assert reply["id"] == request["id"]
    answer = restore(reply["result"], allow_pickle=True)
    assert type(answer["vector"]) is Vector3
    assert answer["vector"].as_tuple == (2.0, 4.0, 6.0)
    assert type(answer["label"]) is Label
    assert answer["label"].name == "demo"
    print(f"Reply: {len(reply_bytes)} bytes; request and reply passed CDDL validation")
    print(f"Result: Vector3{answer['vector'].as_tuple}, {answer['label']}")


def main() -> None:
    schema_dir = Path(__file__).resolve().parents[2] / "docs/development/cbor-rpc"
    contract = "\n".join(
        (schema_dir / name).read_text() for name in ("rpc.cddl", "values.cddl", "extensions.cddl")
    )
    run_demo(Schema(contract))


if __name__ == "__main__":
    main()
