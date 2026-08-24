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
| `JpegCodec` | `Image` | Lossy JPEG for visual images; lossless JPEG XL for depth. Preserves metadata. |
| `LercCodec` | Depth `Image` | Bounded-error compression for `float32` meters and `uint16` millimeters. Maximum error: 5 mm. |
| `LcmCodec` | `DimosMsg` subclasses | Uses `lcm_encode()`/`lcm_decode()`. Zero-copy for LCM message types. |

## Auto-selection

`codec_for(payload_type)` picks the right codec:

```python
from dimos.memory.codecs import codec_for

codec_for(Image)        # → JpegCodec: JPEG for visual images, JPEG XL for depth
codec_for(SomeLcmMsg)   # → LcmCodec(SomeLcmMsg)   (if has lcm_encode/lcm_decode)
codec_for(dict)         # → PickleCodec()            (fallback)
codec_for(None)         # → PickleCodec()
```

## Depth images

The default `Image` codec stores `DEPTH/float32` and `DEPTH16/uint16` images as
lossless JPEG XL. Visual images continue to use ordinary lossy JPEG. Record a
depth stream without a codec override to use this format-aware default:

```python no-result
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image

with TemporaryDirectory() as directory:
    with SqliteStore(path=str(Path(directory) / "recording.db")) as store:
        store.stream("depth_image", Image)
```

Use the `lerc` codec explicitly when a 5 mm error bound is acceptable:

```python no-result
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image

with TemporaryDirectory() as directory:
    with SqliteStore(path=str(Path(directory) / "recording.db")) as store:
        store.stream("depth_image", Image, codec="lerc")
```

`LercCodec` accepts `DEPTH/float32` images in meters and `DEPTH16/uint16`
images in millimeters. It limits the error of every valid decoded sample to 5
mm. Invalid float samples decode as `NaN`; invalid uint16 samples decode as
zero. Shape, dtype, timestamp, frame ID, and the valid-pixel mask survive the
round trip. Different invalid float sentinels, such as infinity, are therefore
canonicalized rather than preserved bit-for-bit.

The error bound is fixed because the SQLite stream registry stores codec IDs,
not codec constructor parameters. Do not add an unpersisted quality option.
See [Depth image compression](../../../docs/capabilities/memory/depth_compression.md)
for the rationale and comparison tool.

The manual benchmark processes every frame in every detected depth stream.
Pass any number of SQLite recordings, extracted depth-frame directories, or
named LFS datasets, and choose a new or empty output directory:

```sh skip
uv run python -m dimos.memory.codecs.tool_depth_benchmark \
  recording.db g1_zed rgbd_frames \
  --output /tmp/depth-codecs
```

Repeat `--stream` to restrict SQLite recordings with several depth streams.
The output directory receives `results.json` and `results.md` with total size,
p50/p95 wall and process-CPU timing, and fidelity measurements. After all
streams finish, the terminal prints a compact report with per-stream results
and highlights; the artifact files retain the complete measurements.

For a quick harness check without LFS data:

```sh
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
