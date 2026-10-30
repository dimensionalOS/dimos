# RTSP camera

`RtspCamera` turns an RTSP H.264 or H.265 stream into dimOS streams. `url` is required.

- `video`: the encoded access units, untouched. Nothing is re-encoded.
- `color_image`: decoded frames (PyAV), capped at `color_hz`, for consumers on the same machine.
- `color_jpeg`: a small JPEG (`jpeg_hz`, `jpeg_max_width`) for a slow operator link. It needs
  the system library (`sudo apt install libturbojpeg`); without it `color_jpeg` switches
  itself off and the rest keeps running.
- Frames are stamped at packet read minus `capture_latency_s` (default 0), on the system clock.
- `url` is an RTSP URL, or a file path to replay a capture through the same code.

RPCs: `set_video_enabled(enabled)` stops or resumes `video`; `set_jpeg_rate(hz)` sets the
`color_jpeg` rate (`hz <= 0` stops it); `get_sensor_stats()` returns per-stream counters.

## Run

```bash
uv sync --extra px4 --inexact
dimos run rtsp-camera-vis --rtspcamera.url=rtsp://<camera>:8554/<path>
dimos run rtsp-camera-vis --rtspcamera.url=clip.mp4     # replay a capture
```
