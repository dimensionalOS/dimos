# Depth image compression

Depth compression must preserve geometry, not visual similarity. A small mean
error does not compensate for a large error at an object boundary: that error
creates a false 3D point between foreground and background surfaces.

## Two compression layers

Per-frame codecs and recording-container codecs solve different problems:

```text
depth array -> per-frame codec -> typed message -> container chunk -> recording
               LERC, PNG, RVL                    LZ4 or Zstd
```

[MCAP](https://mcap.dev/spec) compresses chunks of messages with LZ4 or Zstd.
This generic compression is lossless and complements a depth-aware codec; it
does not define depth precision, invalid pixels, or image reconstruction.

ROS 2's
[`compressed_depth_image_transport`](https://github.com/ros-perception/image_transport_plugins/tree/rolling/compressed_depth_image_transport)
uses PNG or RVL. Both preserve native `16UC1` values. For `32FC1`, the plugin
first converts floating-point depth to quantized inverse depth, so its PNG path
is not bit-exact. RVL is a useful fast `uint16` codec, but dimOS does not depend
on its native implementation.

The experimental ROS
[`depthz` proposal](https://github.com/ros-perception/image_transport_plugins/pull/238)
combines depth prediction, bounded quantization, and Zstd. It closely matches
dimOS's requirements but remains an unmerged, unreviewed proposal.

RealSense documents a
[depth-colorization method](https://dev.realsenseai.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras/)
that maps depth into hue before JPEG, WebP, or video compression. This approach
can use common hardware codecs, but it reduces depth to about 1,529 hue levels
and can create flying pixels, depth inversion, and color-boundary gaps. It is
better suited to bandwidth-constrained visual streaming than metric recording.

## Default: lossless JPEG XL

The default `Image` storage codec uses lossless JPEG XL for `DEPTH/float32` and
`DEPTH16/uint16`. It uses ordinary lossy JPEG for visual image formats. This
runtime dispatch keeps depth metric while retaining the existing visual-image
tradeoff:

```python no-result
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image

with TemporaryDirectory() as directory:
    with SqliteStore(path=str(Path(directory) / "recording.db")) as store:
        store.stream("depth_image", Image)
```

Use LERC when bounded error saves enough space to justify losing millimeter
precision.

## Why LERC

[Limited Error Raster Compression (LERC)](https://github.com/Esri/lerc) is an
Apache-2.0 numeric-raster codec. It supports integer and floating-point arrays,
validity masks, zero-error operation, and an explicit maximum error per sample.
dimOS already receives LERC through its `imagecodecs` dependency.

The `lerc` storage codec uses one contract:

| Input | Units | Maximum error | Invalid output |
|---|---:|---:|---|
| `DEPTH/float32` | meters | `0.005` m | `NaN` |
| `DEPTH16/uint16` | millimeters | `5` mm | `0` |

The codec also preserves shape, dtype, timestamp, frame ID, and the valid-pixel
mask. Invalid float sentinels are canonicalized to `NaN`, so even zero-error
LERC is exact for valid samples rather than bit-exact for the whole array. LERC
remains opt-in.

```python no-result
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image

with TemporaryDirectory() as directory:
    with SqliteStore(path=str(Path(directory) / "recording.db")) as store:
        store.stream("depth_image", Image, codec="lerc")
```

## Compare codecs

The benchmark processes complete depth streams through the same four codecs
available to recordings: LCM, LZ4-wrapped LCM, the default `jpeg` codec (which
uses lossless JPEG XL for depth), and 5 mm LERC. A source can be a SQLite
recording, a directory containing `depth/*.png` or timestamped
`depth/*.pickle` frames, or a named LFS dataset. PNG depth is interpreted as
`uint16` millimeters; float32 pickle arrays are interpreted as meters. Only
load pickle data from trusted recordings. Frames are streamed instead of
retained in memory.

```sh skip
# Benchmark every frame in one SQLite depth stream.
uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db \
  --stream depth_image --output /tmp/depth-codecs

# Auto-detect every depth stream across several local or named LFS recordings.
uv run python -m dimos.memory.codecs.tool_depth_benchmark \
  recording.db g1_zed rgbd_frames --output /tmp/depth-codecs

```

Check the harness without downloading recordings:

```sh
uv run python -m dimos.memory.codecs.tool_depth_benchmark --synthetic uint16
```

```results
──────────────────────────── Depth codec benchmark ─────────────────────────────
Inputs    synthetic-uint16
Streams   1
Frames    5
Elapsed   0.04 s

─────────────────────────── synthetic-uint16 — depth ───────────────────────────
5 frames · 240x320 · uint16 · DEPTH16
╭─────────┬───────────┬───────┬─────────────┬─────────────┬───────────────╮
│         │           │       │   Encode ms │   Decode ms │               │
│ Codec   │ Avg/frame │ Ratio │   p50 / p95 │   p50 / p95 │ Fidelity      │
├─────────┼───────────┼───────┼─────────────┼─────────────┼───────────────┤
│ lcm     │ 150.1 KiB │ 1.00x │ 0.01 / 0.02 │ 0.02 / 0.02 │ exact         │
│ lz4+lcm │ 134.5 KiB │ 1.12x │ 0.19 / 0.20 │ 0.06 / 0.06 │ exact         │
│ jpeg    │  37.5 KiB │ 4.00x │ 0.47 / 0.49 │ 0.62 / 0.65 │ exact         │
│ lerc    │  33.4 KiB │ 4.49x │ 0.72 / 0.77 │ 0.24 / 0.26 │ ≤5 mm         │
│         │           │       │             │             │ RMSE 2.867 mm │
╰─────────┴───────────┴───────┴─────────────┴─────────────┴───────────────╯
Highlights
• Best exact compression: jpeg — 4.00x
• Fastest exact encode: lcm — 0.01 ms p50
• Fastest exact decode: lcm — 0.02 ms p50
• LERC (≤5 mm): 4.49x — RMSE 2.867 mm
```

Real-recording runs require `--output`; the directory must be new or empty.
They write `results.json` and `results.md`. Ratios use total raw and encoded
bytes across the stream. Timing includes p50, p95, total wall and process-CPU
time, plus effective frames per second. Fidelity includes global maximum,
mean, and root-mean-square depth error and invalid-mask mismatches. The command
fails if a codec violates its fidelity contract or a stream changes format,
dtype, or dimensions. After every stream completes, the terminal prints one
summary with the wall-time and fidelity figures most useful for choosing a
codec; the artifact files retain the complete measurements.
