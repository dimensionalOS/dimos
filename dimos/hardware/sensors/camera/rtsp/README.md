# RTSP camera

`RtspCamera` turns an RTSP H.265 stream into dimOS streams. `url` is required.

## How it works

- `video`: the encoded H.265 access units, untouched. Nothing is re-encoded.
- `color_image`: decoded frames (PyAV), capped at `color_hz`, for consumers on the same machine.
- `color_jpeg`: a small JPEG (`jpeg_hz`, `jpeg_max_width`) for a slow operator link.
- Frames are stamped at packet read minus `capture_latency_s` (default 0), on the system clock.
- `url` is an RTSP URL, a file path (replayed through the same code, converted to Annex-B)
  or `synthetic` (a clip generated at start).

RPCs:

- `set_video_enabled(enabled) -> bool` stops or resumes `video`; the decoded outputs keep running.
- `set_jpeg_rate(hz) -> float` sets the `color_jpeg` rate; `hz <= 0` stops it.
- Both return the value in effect and hold until changed or the module restarts.
- `sensor_stats()` returns per-stream counters.

## Run

`dimos run rtsp-camera` needs `--rtspcamera.url`; without it the run stops with
`rtspcamera.url: Field required`.

```bash
dimos run rtsp-camera-vis --rtspcamera.url=rtsp://<camera>:8554/<path>
dimos run rtsp-camera-vis --rtspcamera.url=clip.mp4     # replay a capture
dimos run rtsp-camera-vis --rtspcamera.url=synthetic    # no camera
dimos run rtsp-camera --rtspcamera.url=synthetic        # the module alone, no viewer
```

The JPEG path needs the system library: `sudo apt install libturbojpeg`. Without it
`color_jpeg` switches itself off; `video` and `color_image` keep running.

## Test

No camera needed.

```bash
uv sync --extra px4
uv run pytest dimos/hardware/sensors/camera/rtsp
```
