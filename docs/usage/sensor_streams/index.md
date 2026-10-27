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

## Read camera pixels

Camera streams carry `Image` messages from `dimos.messages.sensor_msgs.msg`.
Use `image.view` to borrow a read-only pixel array, or `image.to_rgb` for an
independent RGB array:

```python
import numpy as np
import reactivex as rx
from reactivex import operators as ops

from dimos.message_utils import image, time

frame = image.from_array(
    np.full((2, 3, 3), 128, dtype=np.uint8),
    encoding="rgb8",
    header=time.header_now("camera_optical"),
)
means = []
rx.just(frame).pipe(
    ops.map(lambda message: float(image.view(message)[..., 0].mean())),
).subscribe(means.append)
assert means == [128.0]
```

Keep the `Image` message when publishing to another typed port or recording a
stream. LCM and Zenoh transports select the encoding from the message type.

## Camera ports

`CameraModule` exposes these ports:

| Port | Message type | Contents |
| --- | --- | --- |
| `color_image` | `sensor_msgs/msg/Image` | Pixels and capture header |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration and optical frame |
| `tf` | `tf2_msgs/msg/TFMessage` | Mount and optical-frame transforms |

`Webcam` converts OpenCV BGR frames to `rgb8`. Stereo cropping changes the
emitted image dimensions and row stride. Supply a `CameraInfo` calibration
for calibrated projection; without valid intrinsics, the camera uses a nominal
pinhole model based on the emitted frame size.

`GstreamerCameraModule.video` emits `bgr8` images. Capture timestamps use the
source PTS plus the configured offset. Source timestamps before 2000 are
rejected.

### V4L2

`V4L2Camera.jpeg_out` carries `sensor_msgs/msg/CompressedImage` messages with
JPEG data. The native worker handles device capture, clock correction, and
JPEG encoding.

Build the worker with Nix:

```sh
cd dimos/hardware/sensors/camera/v4l2/rust
nix build path:.
```

### RealSense

`RealSenseCamera` exposes these ports:

| Ports | Message type |
| --- | --- |
| `color_image`, `depth_image`, `infrared_left`, `infrared_right` | `sensor_msgs/msg/Image` |
| `camera_info`, `depth_camera_info`, `infrared_left_camera_info`, `infrared_right_camera_info` | `sensor_msgs/msg/CameraInfo` |
| `pointcloud` | `sensor_msgs/msg/PointCloud2` |
| `imu` | `sensor_msgs/msg/Imu` |
| `imu_info` | `dimos_msgs/msg/ImuInfo` |
| `tf` | `tf2_msgs/msg/TFMessage` |

The image headers identify the capture time and frame. Calibration messages
provide intrinsics for their corresponding image streams, while TF messages
connect the camera, optical, and IMU frames.

Build the native worker with Nix:

```sh
cd dimos/hardware/sensors/camera/realsense/rust
nix build path:.
```
