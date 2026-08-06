# PX4 MAVSDK Stack

The PX4 stack runs in DimOS and publishes camera, point-cloud, mapping, and
flight-control data to a remote `dimos-viewer` through the standard Rerun
visualization modules.

## Blueprints

PX4 blueprints are registered for the DimOS CLI:

```bash
dimos list
dimos run px4-basic
```

`px4-basic` composes `FlightController`, PointLIO, the MID360 mount transform,
ray-tracing map, camera pipeline, and remote viewer integration.

Use the independent debug blueprints to isolate hardware and transport issues:

```python
from dimos.robot.drone.px4.blueprints.basic.debug import px4_video_debug

px4_video_debug.build().loop()
```

- `px4-video-debug`: camera and H.264/raw image streaming.
- `px4-flight-debug`: MAVSDK flight control and viewer teleoperation route.
- `px4-mapping-debug`: PointLIO, MID360 transform, ray-tracing map, and viewer.
- `px4-gazebo-harmonic`: PX4 SITL MAVSDK control and Gazebo UDP/RTP H.264 video.

The viewer backend follows `global_config` when the blueprint module is
imported. Configure `global_config.viewer` before importing a blueprint when a
non-default backend is needed. These blueprints do not require DimOS to run on
the viewer machine.

## Configuration

The default MAVLink endpoint is `serial:///dev/ttyTHS3:921600`. Override it for
SITL or another transport:

```bash
export DIMOS_MAVSDK_CONNECTION_URL=udpin://0.0.0.0:14540
```

The single local `Px4GstTeeCamera` always publishes both representations:
raw `color_image` via pSHM at `/color_image`, and H.264 `video_h264` via typed
LCM at `/video_h264`. Camera metadata and TF are emitted only when a valid
calibration is configured. Rerun records H.264 at `drone/video` by default;
select raw JPEG-compressed display/recording for encoder diagnostics without
changing the camera pipeline:

```bash
export DIMOS_CAMERA_RERUN_MODE=color_image
```

Set `DIMOS_PX4_CAMERA_CALIBRATION_YAML` to a camera-info YAML file to publish
intrinsics and the identity `base_link -> camera_link` mount. Missing files or
zero focal lengths suppress calibration output rather than fabricating values.

## PX4 Gazebo Harmonic SITL

`px4-gazebo-harmonic` consumes the upstream PX4 `GstCameraSystem` stream. It
uses MAVSDK `udpin://0.0.0.0:14540` and UDP/RTP H.264 on port `5600` by default.
Use a PX4 Gazebo model with a camera, such as `gz_x500_mono_cam`.

Install the DimOS drone extra before starting. On Ubuntu 22.04, PyGObject for
the project Python requires these system build dependencies:

```bash
sudo apt-get install -y gobject-introspection libgirepository1.0-dev libcairo2-dev pkg-config
uv sync --extra drone
```

```bash
# Start PX4 and Gazebo Harmonic in another terminal.
cd /path/to/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_mono_cam

# Start DimOS after PX4 reports its MAVLink endpoint.
cd /path/to/dimos
dimos run px4-gazebo-harmonic
```

The startup wait defaults to 10 seconds. Override it only when PX4 needs longer
to establish the Onboard MAVLink link:

```bash
export DIMOS_MAVSDK_CONNECTION_TIMEOUT_S=20
```

For a separately launched gz-server, start it with the official
`simulation-gazebo` script, then use:

```bash
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam
```

Override the Gazebo UDP port only when the upstream PX4 world plugin has been
configured to send to the same port:

```bash
export DIMOS_PX4_CAMERA_SOURCE=udp-rtp-h264
export DIMOS_PX4_CAMERA_UDP_PORT=5600
dimos run px4-gazebo-harmonic
```

QGroundControl and DimOS cannot both receive the unicast stream on UDP `5600`.
Close QGroundControl video reception before starting DimOS, or configure the
PX4 `GstCameraSystem` world plugin to send video to another UDP port and set
`DIMOS_PX4_CAMERA_UDP_PORT` to match.

## Safety

Validate command paths against PX4 SITL before connecting real hardware. Do not
arm or enter Offboard mode during telemetry-only checks.
