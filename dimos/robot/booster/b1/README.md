# Booster B1 native integration

The B1 locomotion connection uses Booster's C++ SDK through a dimOS
`NativeModule`. It does not import or build `booster_robotics_sdk_python`.

The Booster C++ SDK is pinned to an exact upstream commit and the native binary
is built by the Nix flake in `cpp/`. The first blueprint start runs `nix build`;
subsequent starts reuse `cpp/result/bin/booster_b1_native` and the Nix store
cache.

```bash
ROBOT_INTERFACE=<iface> \
dimos run booster-b1-keyboard-teleop
```

To build it directly:

```bash
cd dimos/robot/booster/b1/cpp
nix build .#booster-b1-native
```

The repackaged, architecture-specific SDK closure is also available as
`nix build .#booster-sdk`.

The CMake project is an internal part of the Nix derivation and is not a
standalone build interface. Nix supplies all dependency and source paths to
CMake.

The driver enters prepare mode on startup, clips commands to 0.2 m/s in X and
Y and 1.0 rad/s in yaw, and sends a zero command after 0.25 seconds without a
new command. In the keyboard teleop blueprint, the first forward command
switches the robot to walking mode; movement starts with the next command after
the mode change is confirmed. SIGINT/SIGTERM sends a final zero command and
returns the robot to prepare mode.

Hardware behavior requires a B1 on the selected network interface and is not
run in unit tests.

## Native RGB-D camera

The camera bridge uses Booster SDK DDS subscribers for its configured color,
depth, and calibration channels and publishes native dimOS streams. The topic
defaults live in `camera_config.py`. Integer depth samples are converted to
float32 meters using the configured `depth_scale` (0.001 by default).

Image subscriptions use best-effort delivery and a capacity-one, drop-oldest
executor queue by default to minimize latency. For lossy links where complete
frame delivery matters more than freshness, set
`-o boostercamera.image_reliable=true`.

On the robot, leave `ROBOT_INTERFACE` unset so the SDK uses its default local
transports:

```bash
unset ROBOT_INTERFACE
dimos run booster-b1-camera
```

For an offboard host, restrict DDS to the interface connected to the robot:

```bash
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera
```

The four topic config fields select the camera streams: `color_topic`,
`depth_topic`, `color_camera_info_topic`, and `depth_camera_info_topic`.
Set `publish_rate_hz` to cap each image stream's DimOS publishing rate;
excess frames are dropped before the native bridge converts or publishes them.
Camera-info messages are not throttled:

```bash
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera \
  -o boostercamera.publish_rate_hz=10
```

The bridge collects aggregate color and depth latency once per second without
writing the metrics to the process log. The
`frame_age_ms` values measure from the camera's source header timestamp through
the bridge's LCM publish, while `bridge_ms` isolates time spent in the bridge
callback. It also reports frame rate, payload bandwidth, invalid timestamps,
and negative ages. Source and host clocks must be synchronized for frame age to
be meaningful; use PTP where supported, or chrony otherwise. A nonzero
`negative_ages` count normally indicates clock skew or a clock adjustment.

## Python RGB-D camera

The Python implementation uses `booster-robotics-sdk-python==1.5.6` and exposes
the same four dimOS streams from a dedicated Python worker. Depth acquisition is
disabled by default; enable it for RGB-D consumers with
`-o boostercamerapython.depth_enabled=true`:

```bash
uv sync --extra booster
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera-python
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera-python \
  -o boostercamerapython.depth_enabled=true
```

To cap each enabled image stream's DimOS publishing rate, set
`publish_rate_hz`. Excess frames are dropped before decoding or conversion;
camera-info messages are not throttled:

```bash
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera-python \
  -o boostercamerapython.publish_rate_hz=10
```

As with the native camera, leave `ROBOT_INTERFACE` unset when running directly
on the robot. The Python bridge decodes compressed color frames into NumPy
images, preserves raw color formats and source timestamps, converts integer
depth to float32 meters, and forwards calibration and ROI fields. Its DDS
subscriber bindings do not expose the native SDK's reliability and queue
options, so `image_reliable` is available only on `booster-b1-camera`.

It publishes once-per-second metrics for the enabled image and camera-info
streams without writing them to the process log. For image streams, payload
bandwidth measures the decoded NumPy images published to dimOS. Camera-info
payload bandwidth uses the encoded DimOS message size. `bridge_ms` includes
conversion and publishing work. Configure the reporting cadence with
`metrics_interval_seconds`.

The Python camera publishes these aggregates as a typed `camera_metrics`
diagnostic stream. Compose the camera and its optional Rerun dashboard in one
run so they share a single coordinator and recording:

```bash
ROBOT_INTERFACE=<iface> dimos run \
  booster-b1-camera-python \
  booster-b1-camera-rerun
```

The bridge converts the numeric diagnostics into scalar time series under
`metrics/camera/{color,depth,color_camera_info,depth_camera_info}` for frame
age, bridge time, frame rate, payload bandwidth, and timestamp error counts.
The camera-info metrics appear in the dashboard's third tab. No camera-specific
Rerun connection or application ID is used. The charts are scoped to this
Booster-specific Rerun blueprint; the default `dimos rerun-bridge` layout
remains robot-agnostic.
