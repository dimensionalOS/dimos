# codecs

Encode/decode payloads for persistent storage. Codecs convert typed Python objects to `bytes` and back, used by backends that store observation data as blobs.

## Protocol

```python
class Codec(Protocol[T]):
    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...
```

## Built-in codecs

| Codec | Type | Notes |
|-------|------|-------|
| `PickleCodec` | Any Python object | Fallback. Uses `HIGHEST_PROTOCOL`. |
| `JpegCodec` | `Image` | Lossy compression via TurboJPEG. ~10-20x smaller. Preserves `frame_id` in header. |
| `LercCodec` | Depth `Image` | Bounded-error compression for `float32` meters and `uint16` millimeters. Maximum error: 5 mm. |
| `LcmCodec` | `DimosMsg` subclasses | Uses `lcm_encode()`/`lcm_decode()`. Zero-copy for LCM message types. |
| `ZstdCodec` | Any codec output | Lossless level-3 Zstandard wrapper. Use `zstd+lcm` for typed messages. |

## Auto-selection

`codec_for(payload_type)` picks the right codec:

```python
from dimos.memory.codecs import codec_for

codec_for(Image)        # → JpegCodec(quality=50)
codec_for(SomeLcmMsg)   # → LcmCodec(SomeLcmMsg)   (if has lcm_encode/lcm_decode)
codec_for(dict)         # → PickleCodec()            (fallback)
codec_for(None)         # → PickleCodec()
```

## Depth images

Use the `lerc` codec explicitly for recorded depth streams:

```python
depth = store.stream("depth_image", Image, codec="lerc")
```

Use `zstd+lcm` when the recording must preserve every original depth value:

```python
depth = store.stream("depth_image", Image, codec="zstd+lcm")
```

`LercCodec` accepts `DEPTH/float32` images in meters and `DEPTH16/uint16`
images in millimeters. It limits the error of every valid decoded sample to 5
mm. Invalid float samples decode as `NaN`; invalid uint16 samples decode as
zero. Shape, dtype, timestamp, frame ID, and the valid-pixel mask survive the
round trip.

The error bound is fixed because the SQLite stream registry stores codec IDs,
not codec constructor parameters. Do not add an unpersisted quality option.
See [Depth image compression](../../../docs/capabilities/memory/depth_compression.md)
for the rationale and comparison tool.

Run the manual benchmark against its default LFS recording:

```bash
uv run python -m dimos.memory.codecs.tool_depth_benchmark
```

Pass any number of SQLite recordings, extracted depth-frame directories, or
named LFS datasets:

```bash
uv run python -m dimos.memory.codecs.tool_depth_benchmark \
  recording.db g1_zed rgbd_frames
```

For a quick check without LFS data:

```bash
uv run python -m dimos.memory.codecs.tool_depth_benchmark --synthetic uint16
uv run python -m dimos.memory.codecs.tool_depth_benchmark --synthetic float32
```

## Writing a new codec

1. Create `dimos/memory/codecs/mycodec.py`:

```python
class MyCodec:
    def encode(self, value: MyType) -> bytes:
        ...

    def decode(self, data: bytes) -> MyType:
        ...
```

2. Add a branch in `codec_for()` in `base.py` to auto-select it for the relevant type.

3. Add a test case to `test_codecs.py` — the grid fixture makes this easy:

```python
@pytest.fixture(params=[..., ("mycodec", MyCodec(), sample_value)])
def codec_case(request):
    ...
```

No base class needed — `Codec` is a protocol. Just implement `encode` and `decode`.
