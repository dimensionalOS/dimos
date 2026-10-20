# Sensor Streams

Dimos uses reactive streams (RxPY) to handle sensor data. This approach naturally fits robotics where multiple sensors emit data asynchronously at different rates, and downstream processors may be slower than the data sources.

## Guides

| Guide                                        | Description                                                   |
|----------------------------------------------|---------------------------------------------------------------|
| [ReactiveX Fundamentals](/docs/usage/sensor_streams/reactivex.md)       | Observables, subscriptions, and disposables                   |
| [Advanced Streams](/docs/usage/sensor_streams/advanced_streams.md)      | Backpressure, parallel subscribers, synchronous getters       |
| [Quality-Based Filtering](/docs/usage/sensor_streams/quality_filter.md) | Select highest quality frames when downsampling streams       |
| [Temporal Alignment](/docs/usage/sensor_streams/temporal_alignment.md)  | Match messages from multiple sensors by timestamp             |
| [Storage & Replay](/docs/usage/sensor_streams/storage_replay.md)        | Record sensor streams to disk and replay with original timing |

## Quick Example

```python skip
from reactivex import operators as ops
from dimos.utils.reactive import backpressure
from dimos.types.timestamped import align_timestamped
from dimos.msgs.sensor_msgs.Image import sharpness_barrier

# Camera at 30fps, lidar at 10Hz
camera_stream = camera.observable()
lidar_stream = lidar.observable()

# Pipeline: filter blurry frames -> align with lidar -> handle slow consumers
processed = (
    camera_stream.pipe(
        sharpness_barrier(10.0),  # Keep sharpest frame per 100ms window (10Hz)
    )
)

aligned = align_timestamped(
    backpressure(processed),     # Camera as primary
    lidar_stream,                # Lidar as secondary
    match_tolerance=0.1,
)

aligned.subscribe(lambda pair: process_frame_with_pointcloud(*pair))
```

## Generated camera values in the CDR proposal

`Webcam` produces `dimos.messages.sensor_msgs.msg.Image` values. It converts
OpenCV BGR frames to `rgb8` and applies the selected stereo crop before setting
`width`, `height` and `step`. `CameraModule` exposes these generated ports:

| Port | Value type | Contents |
| --- | --- | --- |
| `color_image` | `sensor_msgs/msg/Image` | Pixels and capture Header |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration and optical frame |
| `tf` | `tf2_msgs/msg/TFMessage` | Mount and optical-frame transforms |

Read pixels with the array adapter. Keep the generated value for typed ports
and recording:

```python
from dimos.messages.sensor_msgs.msg import Image
from dimos.msgs.image import image_view


def mean_red(image: Image) -> float:
    assert image.encoding == "rgb8"
    return float(image_view(image)[..., 0].mean())
```

Configured calibration remains a generated `CameraInfo` instance. When no valid
intrinsics are supplied, the existing nominal pinhole behavior uses the emitted
frame size, including a stereo crop. Metadata publishes a fresh Header timestamp
without changing the configured mount transform.

`GstreamerCameraModule.video` also emits generated `Image` values, with `bgr8`
pixels. It copies padded rows before releasing the SDK buffer and keeps source
PTS plus the configured offset in integer nanoseconds. Its existing rejection
of pre-2000 source timestamps remains in place.

These ports need explicit CDR transport wiring until the default-transport
cutover is complete. Native V4L2, RealSense and ZED producers are separate
migration slices. This change does not start cameras or switch robot blueprints.

Run the offline producer checks from the checkout:

```sh
python -m pytest --noconftest -o addopts='' -q \
  dimos/hardware/sensors/camera/test_webcam.py \
  dimos/hardware/sensors/camera/test_webcam_backend.py \
  dimos/hardware/sensors/camera/test_module.py \
  dimos/hardware/sensors/camera/gstreamer/test_generated_frames.py
```

The GStreamer cases use a real in-memory `appsrc`/`appsink` pipeline, not a camera
or TCP stream. They require PyGObject and the GStreamer core, app and video
libraries/typelibs; they are skipped when that optional binding is absent.
No test installs system libraries. Constructing a viewer blueprint defers
backend allocation until startup, so importing a camera module opens no viewer
multicast socket.
