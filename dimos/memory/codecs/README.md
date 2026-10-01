# Storage codecs

Codecs convert Python values to bytes for blob-backed stores such as SQLite.
Generated messages use the same CDR bytes as typed transports. Raw images retain
their pixels and encoding; lossy compression is an explicit conversion to a
`sensor_msgs/msg/CompressedImage` value before storage.

| Codec | Values | Stored representation |
|-------|--------|-----------------------|
| `CdrCodec` | Generated messages | Encapsulated CDR, with no private wrapper |
| `Lz4Codec` | Values accepted by its inner codec | LZ4 frame around the inner bytes |
| `PickleCodec` | Python objects | Python pickle; separate from typed message storage |

```python
from dimos_generated.sensor_msgs.msg import Image, CompressedImage
from dimos.memory.codecs.base import codec_for

codec_for(Image)            # CdrCodec(Image), lossless
codec_for(CompressedImage)  # CdrCodec(CompressedImage)
codec_for(dict)             # PickleCodec()
```

SQLite persists the codec ID and Python payload type so reopening a stream uses
the same codec. Use `codec="lz4+cdr"` for lossless blob compression. Old `lcm`,
`lz4+lcm`, and `jpeg` storage codec IDs are rejected; no legacy decoder is retained.

MCAP uses `cdr` channels with embedded `ros2msg` definitions and chunk compression.
The reader resolves installed message types by qualified schema name. Unknown
schemas remain inspectable as raw bytes; payload-module metadata never triggers
an arbitrary import. Compressed images remain CompressedImage values when read.
Multiple channels on one topic must agree on schema and timestamp policy.

A custom codec only needs `encode(value) -> bytes` and `decode(bytes) -> value`.
Implement the `Codec` protocol and supply the instance explicitly to a stream.
Adding a generated message does not require a new storage codec.
