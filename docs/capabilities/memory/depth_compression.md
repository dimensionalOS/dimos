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

```python
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

```python
store.stream("depth_image", Image, codec="lerc")
```

## Lossless Zstd

Use `zstd+lcm` when every depth value must round-trip exactly. `ZstdCodec`
compresses the bytes produced by its inner codec with Zstandard level 3. The
level is fixed, so the persisted codec ID contains no untracked constructor
configuration.

```python
store.stream("depth_image", Image, codec="zstd+lcm")
```

Zstd usually costs slightly more CPU than LZ4 but compresses structured depth
messages more effectively. It remains lossless: depth values, invalid values,
and metadata decode byte-for-byte through the inner LCM codec.

## Compare codecs

The benchmark processes complete depth streams and compares raw LCM, LZ4,
Zstd, PNG, lossless JPEG XL, reversible ZFP, and several LERC error bounds.
Unsupported input-dtype combinations are reported as `N/A`. A source can be a
SQLite recording, a directory containing `depth/*.png` or timestamped
`depth/*.pickle` frames, or a named LFS dataset. PNG depth is interpreted as
`uint16` millimeters; float32 pickle arrays are interpreted as meters. Only
load pickle data from trusted recordings. Frames are streamed instead of
retained in memory.

```bash
# Benchmark every frame in one SQLite depth stream.
uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db \
  --stream depth_image --output /tmp/depth-codecs

# Auto-detect every depth stream across several local or named LFS recordings.
uv run python -m dimos.memory.codecs.tool_depth_benchmark \
  recording.db g1_zed rgbd_frames --output /tmp/depth-codecs

# Check the harness without downloading recordings.
uv run python -m dimos.memory.codecs.tool_depth_benchmark --synthetic uint16
```

Real-recording runs require `--output`; the directory must be new or empty.
They write `results.json` and `results.md`. Ratios use total raw and encoded
bytes across the stream. Timing includes p50, p95, total wall and process-CPU
time, plus effective frames per second. Fidelity includes global maximum,
mean, and root-mean-square depth error and invalid-mask mismatches. The command
fails if a codec violates its fidelity contract or a stream changes format,
dtype, or dimensions.
