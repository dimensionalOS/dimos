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
run in unit tests. The tests cover native build configuration and blueprint
wiring only.

## RGB-D camera

The camera bridge uses Booster SDK DDS subscribers for its configured color,
depth, and calibration channels and publishes native dimOS streams. The topic
defaults live in `camera.py`. Integer depth samples are converted to float32
meters using the configured `depth_scale` (0.001 by default).

Image subscriptions are reliable by default so fragmented RGB-D samples can be
reassembled across the robot link. For a best-effort-only ROS 2 publisher, set
`-o boostercamera.image_reliable=false`.

```bash
ROBOT_INTERFACE=<iface> dimos run booster-b1-camera
```

The four topic config fields select the camera streams: `color_topic`,
`depth_topic`, `color_camera_info_topic`, and `depth_camera_info_topic`.
