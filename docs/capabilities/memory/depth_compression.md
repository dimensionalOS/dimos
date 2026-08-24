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
is not bit-exact. RVL is a useful fast `uint16` codec, but DimOS does not depend
on its native implementation.

The experimental ROS
[`depthz` proposal](https://github.com/ros-perception/image_transport_plugins/pull/238)
combines depth prediction, bounded quantization, and Zstd. It closely matches
DimOS's requirements but remains an unmerged, unreviewed proposal.

RealSense documents a
[depth-colorization method](https://dev.realsenseai.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras/)
that maps depth into hue before JPEG, WebP, or video compression. This approach
can use common hardware codecs, but it reduces depth to about 1,529 hue levels
and can create flying pixels, depth inversion, and color-boundary gaps. It is
better suited to bandwidth-constrained visual streaming than metric recording.

## Why LERC

[Limited Error Raster Compression (LERC)](https://github.com/Esri/lerc) is an
Apache-2.0 numeric-raster codec. It supports integer and floating-point arrays,
validity masks, lossless operation, and an explicit maximum error per sample.
DimOS already receives LERC through its `imagecodecs` dependency.

The `lerc` storage codec uses one contract:

| Input | Units | Maximum error | Invalid output |
|---|---:|---:|---|
| `DEPTH/float32` | meters | `0.005` m | `NaN` |
| `DEPTH16/uint16` | millimeters | `5` mm | `0` |

The codec also preserves shape, dtype, timestamp, frame ID, and the valid-pixel
mask. It is opt-in; existing `lz4+lcm` recorder settings remain unchanged.

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

The benchmark samples recorded frames and compares raw LCM, LZ4, Zstd, PNG,
lossless JPEG XL, reversible ZFP, and several LERC error bounds. Cells that do
not support an input dtype are omitted. A source can be a SQLite recording, a
directory containing `depth/*.png` or timestamped `depth/*.pickle` frames, or
a named LFS dataset. PNG depth is interpreted as `uint16` millimeters; float32
pickle arrays are interpreted as meters. Only load pickle data from trusted
recordings.

```bash
# Pull and sample the default RealSense LFS recording.
uv run python -m dimos.memory.codecs.tool_depth_benchmark

# Use another SQLite recording or select a stream explicitly.
uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db \
  --stream depth_image --frames 20 --repeats 3

# Compare several local or named LFS recordings in one run.
uv run python -m dimos.memory.codecs.tool_depth_benchmark \
  recording.db g1_zed rgbd_frames --frames 20 --repeats 3

# Write machine-readable results.
uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db \
  --json /tmp/depth-codecs.json
```

The table reports bytes per frame, compression ratio, encode and decode wall
time, encode and decode process-CPU time, maximum and root-mean-square depth
error, and invalid-mask mismatches. Use maximum error and mask mismatches as
hard fidelity checks; use size and timing to choose among candidates that pass
those checks.
