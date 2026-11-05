# Zenoh RPC Encoding Proposal

## Proposed approach

**Keep Zenoh, replace pickle in RPC with MessagePack, and reuse existing LCM encoding.**

Zenoh transports the data, MessagePack packs arguments and results, and LCM encodes existing message types. Callers continue to use the existing @rpc, @skill, and Spec APIs.

Clients and servers upgrade together, so compatibility with old pickle nodes is unnecessary. This proposal covers Zenoh RPC; pickle in local worker IPC and other transports is handled separately.

## Encoding payloads

| Payload | Encoding |
| --- | --- |
| Numbers, strings, bytes, lists, and plain dicts | Use MessagePack directly |
| LCM messages | Store a type identifier and the original LCM bytes; the receiver uses the matching decoder |
| NumPy arrays | Store dtype, shape, and raw bytes |
| Tuples, enums, dataclasses, etc. | Add type identifiers to restore the original Python types |
| RPCClient, Transport, tasks, and Blueprint | Provide dedicated adapters for these objects |

These types can be nested. For example, a dict can contain a PoseStamped, an array, and a string. The encoder recursively processes each value and packs them into one binary payload. Bytes stay binary, without base64 conversion. [MessagePack specification](https://github.com/msgpack/msgpack/blob/master/spec.md)

Custom objects need registered encoding and decoding rules. Unsupported types raise an explicit error, with no fallback to pickle. **Compatibility means preserving the original types and data, not just producing a dict with similar contents.**

## Minimal example

This example uses msgpack-python to demonstrate a complete in-memory round trip for `calculate(2, b=3)`. In a real RPC, Zenoh transports the request and reply bytes.

```python
import msgpack


def calculate(a: int, b: int) -> int:
    return a + b


# Client: encode positional and keyword arguments as binary data.
request = msgpack.packb({"args": [2], "kwargs": {"b": 3}}, use_bin_type=True)

# Server: decode the arguments, call calculate, and encode the result.
params = msgpack.unpackb(request, raw=False)
result = calculate(*params["args"], **params["kwargs"])
reply = msgpack.packb(result, use_bin_type=True)

# Client: decode the reply to recover the Python value.
answer = msgpack.unpackb(reply, raw=False)
assert answer == 5
print(answer)
```

```results
5
```

When arguments contain LCM messages or custom objects, the shared codec uses the corresponding adapters to pack them and restore their types on unpacking. The way callers invoke `calculate` stays the same.

### LCM arguments and return values

For `calculate(Vector3(1, 2, 3), scale=2.0)`, Vector3 uses LCM encoding, scale uses MessagePack's native float encoding, and the return value is also a Vector3.

To keep this example minimal, MessagePack extension code 1 represents Vector3, and the extension payload contains the full LCM bytes directly.

```python
from typing import Any

import msgpack

from dimos.msgs.geometry_msgs.Vector3 import Vector3


def calculate(vector: Vector3, scale: float) -> Vector3:
    return Vector3(vector.x * scale, vector.y * scale, vector.z * scale)


def encode_lcm(value: Any) -> msgpack.ExtType:
    if type(value) is Vector3:
        return msgpack.ExtType(1, value.lcm_encode())
    raise TypeError(f"Unsupported type: {type(value).__name__}")


def decode_lcm(code: int, data: bytes) -> Vector3:
    if code == 1:
        return Vector3.lcm_decode(data)
    raise ValueError(f"Unknown extension: {code}")


# Client: MessagePack automatically calls encode_lcm for the nested Vector3.
request = msgpack.packb(
    {"args": [Vector3(1, 2, 3)], "kwargs": {"scale": 2.0}},
    default=encode_lcm,
    use_bin_type=True,
)

# Server: restore the Vector3, then call calculate as usual.
params = msgpack.unpackb(request, ext_hook=decode_lcm, raw=False)
assert type(params["args"][0]) is Vector3
result = calculate(*params["args"], **params["kwargs"])
reply = msgpack.packb(result, default=encode_lcm, use_bin_type=True)

# Client: restore the return value as a Vector3 too.
answer = msgpack.unpackb(reply, ext_hook=decode_lcm, raw=False)
assert type(answer) is Vector3
assert (answer.x, answer.y, answer.z) == (2.0, 4.0, 6.0)
print(answer.x, answer.y, answer.z)
```

```results
2.0 4.0 6.0
```

To support multiple LCM types in the full implementation, add a type identifier inside a shared LCM extension and select the decoder from a registry. Move encode_lcm/decode_lcm into the shared codec so business functions do not need to handle encoding.

A runnable script is available at [demo_rpc_encoding.py](/docs/development/examples/demo_rpc_encoding.py). It adds a type name inside the LCM extension and prints each step: encoding, type identification, server execution, and result decoding. Transport is simulated in memory.

Run `uv run --no-sync --with msgpack python docs/development/examples/demo_rpc_encoding.py` from the repository root. The final decoded value is `Vector3(2.0, 4.0, 6.0)`.

## Implementation steps

1. **Add a shared codec.** Provide encode/decode functions for the types above, initially using msgpack-python. Requests, return values, and exceptions all go through it.
2. **Integrate with the existing ZenohRPC.** Replace pickle calls in [zenohrpc.py](/dimos/protocol/rpc/zenohrpc.py), add an encoding version identifier, and preserve existing calling conventions and exception propagation.
3. **Cover special payloads before switching everyone over.** Start with RPCClient and Transport, which module wiring needs, then cover business objects, tasks, and Blueprint. Verify encoding round trips, blueprint startup, RPC calls, and shutdown.

Three issues in the current code also need attention:

- [StreamDescriptor](/dimos/core/module.py) contains embedded pickle bytes. Replace this encoding together with its [reader](/dimos/porcelain/remote_module_source.py).
- [GraspCandidate](/dimos/msgs/manipulation_msgs/GraspCandidate.py) uses pickle in its encode method. It needs a dedicated adapter and cannot be treated as an LCM message directly.
- Existing LCM encoding for Image and PointCloud2 may change timestamps, array writability, or precision. Verify these separately and add adapters where needed.

Tasks and Blueprint contain behavior and state, so converting them to plain dicts is insufficient. Full compatibility with existing payloads requires completing their adapters.

This is an implementation proposal; the runtime has not been changed yet. Build the codec and adapters for existing types first, then integrate them with Zenoh RPC.
