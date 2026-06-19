# H.264 memory2 storage benchmark

This report compares memory2 image storage size for the same frames stored with the default JPEG codec and the opt-in H.264 codec.

## Method

Blueprint: `demo-h264-storage-benchmark`

The benchmark source publishes identical `Image` frames to two recorder streams:

- `jpeg_image` uses the default memory2 `Image` codec (`JpegCodec`).
- `h264_image` uses `codec="h264"` and receives encoded H.264 images through `H264LcmTransport(decode_images=False)`.

The reporter measures compact SQLite snapshot sizes with SQLite backup, so active WAL/SHM sidecars do not skew the comparison.

## Public video sample

Source video:

- URL: `https://raw.githubusercontent.com/opencv/opencv/master/samples/data/vtest.avi`
- Local path used for the run: `/tmp/opencode/dimos-h264-benchmark-vtest.avi`
- File size: 8,131,690 bytes
- Source dimensions: 768 x 576
- Source FPS: 10
- Source frame count: 795

Benchmark settings:

- Frames recorded: 150
- Recorded dimensions: 320 x 240
- Publish rate: 15 FPS
- H.264 bitrate: 1,500,000 bps
- H.264 keyframe interval: 30 frames
- H.264 profile/preset/tune: baseline / veryfast / zerolatency
- B-frames: disabled

Command:

```bash
rm -f benchmark_jpeg.db benchmark_jpeg.db-wal benchmark_jpeg.db-shm \
      benchmark_h264.db benchmark_h264.db-wal benchmark_h264.db-shm

DIMOS_H264_BENCHMARK_VIDEO=/tmp/opencode/dimos-h264-benchmark-vtest.avi \
  uv run dimos run demo-h264-storage-benchmark --daemon

sleep 22
uv run dimos log -n 80
uv run dimos stop
```

## Result

| Codec | DB path | Rows | Blob rows | Blob bytes | DB size |
|---|---:|---:|---:|---:|---:|
| JPEG | `benchmark_jpeg.db` | 150 | 150 | 1,586,940 | 1,884,160 bytes (1.80 MiB) |
| H.264 | `benchmark_h264.db` | 150 | 150 | 1,008,355 | 1,126,400 bytes (1.07 MiB) |

H.264 used 59.8% of the JPEG storage size and saved 757,760 bytes, a 40.2% reduction for this sample.

## Direct ffmpeg H.264 comparison

To estimate the cost of per-frame Foxglove-style storage versus a continuous H.264 stream, the same 150 frames were encoded directly with ffmpeg using similar H.264 settings:

```bash
ffmpeg -y -v error \
  -i /tmp/opencode/dimos-h264-benchmark-vtest.avi \
  -vf "scale=320:240,fps=15" \
  -frames:v 150 \
  -c:v libx264 \
  -b:v 1500k -maxrate 1500k -bufsize 3000k \
  -profile:v baseline -preset veryfast -tune zerolatency \
  -g 30 -keyint_min 30 -sc_threshold 0 -bf 0 \
  -pix_fmt yuv420p \
  -f h264 /tmp/opencode/dimos-h264-benchmark-direct.h264
```

| Output | Size |
|---|---:|
| Direct ffmpeg Annex B H.264 elementary stream | 1,603,706 bytes (1.53 MiB) |
| Direct ffmpeg MP4 container | 1,606,038 bytes (1.53 MiB) |
| memory2 H.264 SQLite DB | 1,126,400 bytes (1.07 MiB) |
| memory2 H.264 blob payloads only | 1,008,355 bytes (0.96 MiB) |

In this run, memory2 H.264 storage was smaller than the direct ffmpeg elementary stream. That means this benchmark does not show a storage-efficiency penalty from the per-frame Annex B access-unit layout. It mostly shows that the current aiortc/libx264 path and the direct ffmpeg command did not produce identical rate-control output, even with similar nominal settings.

The storage overhead within memory2 was measurable: the H.264 DB was 118,045 bytes larger than its stored blob payloads, or 11.7% over the blob bytes. That overhead includes observation metadata, SQLite page overhead, and one encoded-image envelope per frame.

## Notes

- The benchmark measures SQLite DB size, not raw compressed frame bytes alone. Observation metadata and blob table overhead are included for both codecs.
- The direct ffmpeg comparison is not a quality-matched encoder benchmark. It uses similar nominal settings to the DimOS H.264 config, but aiortc/PyAV and ffmpeg rate control can still choose different actual bit allocation.
- The sample video already contains temporal structure. Synthetic frames from the same benchmark blueprint produced a larger reduction in one local run: JPEG 2,109,440 bytes, H.264 983,040 bytes, a 53.4% reduction.
- H.264 results depend on bitrate, keyframe interval, resolution, motion, and scene texture.
