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
cutover is complete. ZED producers remain a separate migration slice. This change does not start cameras or switch robot blueprints.

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

### Native V4L2 producer

`V4L2Camera.jpeg_out` now uses generated `sensor_msgs/msg/CompressedImage`
values. The Rust worker serializes those values with the SDK CDR codec. Device
selection, driver-clock correction, JPEG encoding and retries keep their
existing behavior.

The independent Nix build prepares the generated Rust owner from fixed-hash
project and upstream inputs. It calls the same Python source-preparation and
code-generation functions as the main package build; it does not build the
whole Python runtime or commit upstream `.msg` copies. The SDK source and
message configuration are pinned to a published project commit. Updating that
pin is a maintainer action when the SDK or built-in schemas change.

Build without opening a device:

```sh
cd dimos/hardware/sensors/camera/v4l2/rust
nix build path:.
```

For a repository developer using Cargo, first install the checkout normally so
the package backend prepares `build/message-source/rust`, then run library
acceptance only:

```sh
uv pip install -e .
cargo test --manifest-path dimos/hardware/sensors/camera/v4l2/rust/Cargo.toml \
  --locked --lib --jobs 2
```

This needs the native Rust/C toolchains. The library tests check CDR, clock
correction and software JPEG with synthetic data; they do not open V4L2 or start
the worker. Jetson driver validation and physical capture remain separate
hardware acceptance.

### Native RealSense producer

`RealSenseCamera` uses generated values on all camera ports:

| Ports | Value type |
| --- | --- |
| `color_image`, `depth_image`, `infrared_left`, `infrared_right` | `sensor_msgs/msg/Image` |
| `camera_info`, `depth_camera_info`, `infrared_left_camera_info`, `infrared_right_camera_info` | `sensor_msgs/msg/CameraInfo` |
| `pointcloud` | `sensor_msgs/msg/PointCloud2` |
| `imu` | `sensor_msgs/msg/Imu` |
| `imu_info` | `dimos_msgs/msg/ImuInfo` |
| `tf` | `tf2_msgs/msg/TFMessage` |

The IMU noise message is generated from the project-owned definition; it no
longer uses a handwritten LCM fingerprint/encoder. Calibration, RGBD projection,
IMU pairing, frame names and capture configuration retain their existing roles.
The Rust TF handle retains its lookup/publish operations. RealSense selects CDR
through explicit codec hooks; an existing bare `#[tf]` declaration retains its
existing wire format. The graph's existing floating-point seconds API remains
unchanged; raw messages use the generated Time fields.

The RealSense Nix build uses the same isolated source assembly as V4L2, with its
own pinned SDK/configuration snapshot. To build and run library regression tests
without starting a camera:

```sh
cd dimos/hardware/sensors/camera/realsense/rust
nix build path:.#tests
```

For a Cargo developer with the checkout installed and `librealsense2` available
through pkg-config:

```sh
cargo test --manifest-path dimos/hardware/sensors/camera/realsense/rust/Cargo.toml \
  --locked --bin realsense_native --jobs 2 -- tests:: --skip bench::
```

These tests use real SDK intrinsics values and synthetic frame data. They check
canonical CDR in both byte orders, normalized time, calibration, point/color
alignment and source headers. They do not enumerate or capture from devices.
Nix execution, physical camera behavior and platform-specific drivers require
separate verification; the local regression build used an isolated official
librealsense 2.56.5 build tree without system installation.
