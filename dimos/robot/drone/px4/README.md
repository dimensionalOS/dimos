# PX4 Module

## Demo

| Hardware: camera, MID360 point cloud, and voxel map | Simulation: Gazebo gimbal camera |
|---|---|
| ![PX4 hardware frontend](https://github.com/user-attachments/assets/adaaa34b-1ced-4229-98da-02f1e9f4e9f5) | ![PX4 Gazebo Harmonic simulation frontend](https://github.com/user-attachments/assets/19358c12-3d02-4c18-b7b1-687e05fb6f2c) |

DimOS integration for PX4 drones using MAVSDK. The module combines flight control, MID360 localization, voxel mapping, GStreamer video distribution, Rerun visualization, Gazebo Harmonic simulation, and optional LLM agent control. The hardware stack currently targets a PX4 flight controller connected at `serial:///dev/ttyTHS3:921600` and a Livox MID360 at `192.168.1.3`.

## Quick Start

The following commands assume that the [installation](#installation) is complete. They explicitly provide the minimum hardware inputs used by the current setup: MID360 network addresses, MAVSDK serial connection, and V4L2 camera pipeline. Replace the device paths and local host address when the hardware differs.

### Hardware stack: `px4-basic`

```bash
dimos run px4-basic \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50 \
  --flightcontroller.connection-url='serial:///dev/ttyTHS3:921600' \
  --flightcontroller.connection-timeout-s=20 \
  --gsteecamera.input-pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1' \
  --gsteecamera.input-format=raw
```

For a headless onboard computer that only accepts a remote `dimos-viewer`, use:

```bash
dimos --viewer rerun --rerun-open none --no-rerun-web --rerun-host 0.0.0.0 run px4-basic \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50 \
  --flightcontroller.connection-url='serial:///dev/ttyTHS3:921600' \
  --flightcontroller.connection-timeout-s=20 \
  --gsteecamera.input-pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1' \
  --gsteecamera.input-format=raw
```

Connect from the remote computer, replacing `<ONBOARD_IP>` with the address of the computer running DimOS:

```bash
dimos-viewer \
  --connect rerun+http://<ONBOARD_IP>:9877/proxy \
  --ws-url ws://<ONBOARD_IP>:3030/ws
```

`dimos-viewer` currently sends directional commands only. It does not arm, take off, or enter Offboard mode. Before its direction controls can work, open another terminal on the onboard computer, enter `dimos shell`, verify that the flight area is safe, and run these commands in order:

```bash
dimos shell
```

```python
app.FlightController.arm()
app.FlightController.takeoff(3.0)
# Wait until the vehicle reaches a safe altitude and stabilizes, then run:
app.FlightController.enter_offboard()
```

The remote viewer controls become active only after `enter_offboard()` returns `offboard mode entered`. Run `app.FlightController.hold()` to stop directional control, then `app.FlightController.land()` after confirming that the landing area is safe. Do not call `disarm()` while airborne.

### Hardware stack with agent: `px4-agentic`

```bash
export OPENAI_API_KEY=sk-...

dimos run px4-agentic \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50 \
  --flightcontroller.connection-url='serial:///dev/ttyTHS3:921600' \
  --flightcontroller.connection-timeout-s=20 \
  --gsteecamera.input-pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1' \
  --gsteecamera.input-format=raw
```

In another terminal, inspect the available tools or send a request:

```bash
dimos mcp list-tools
dimos agent-send "take off to 2 meters"
```

### Simulation: `px4-gazebo-harmonic`

The simulation scene comes from [PX4-AeroFusion-Sim](https://github.com/weyne-Jiang/PX4-AeroFusion-Sim). Start PX4 SITL and Gazebo in terminal 1:

```bash
cd /path/to/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal_windy
```

After PX4 reports its MAVLink endpoint, start DimOS in terminal 2. Use `dimos status`, `dimos log -f`, and `dimos stop` to inspect or stop the running stack.

```bash
cd /path/to/dimos
dimos run px4-gazebo-harmonic
```

## Blueprints and Modules

The PX4 integration provides three runnable blueprints. `px4-basic` is the complete hardware stack for flight, localization, mapping, video, and visualization; `px4-agentic` adds an LLM agent; and `px4-gazebo-harmonic` targets Gazebo SITL without PointLIO or voxel mapping.

| Blueprint | Composition and data flow |
|-----------|---------------------------|
| `px4-basic` | Connects real PX4 hardware, MID360, and a camera, providing external vision, voxel mapping, and Rerun visualization |
| `px4-agentic` | Adds `McpServer`, `McpClient`, and `WebInput` to `px4-basic`, exposing `@skill` flight methods to the LLM agent |
| `px4-gazebo-harmonic` | Connects to PX4 SITL at `udpin://0.0.0.0:14540`, receives Gazebo H.264 video on UDP `5600`, and shows only the camera view in Rerun |

`px4-basic` and `px4-agentic` use these core modules:

| Module | Purpose |
|--------|---------|
| `FlightController` | Direct MAVSDK connection, telemetry, flight skills, Offboard velocity control, and external-vision forwarding |
| `PointLio` | MID360 point-cloud processing and lidar odometry |
| `Mid360MountStaticTf` | Calibrated static transform from `mid360_link` to `base_link` |
| `RayTracingVoxelMap` | Local and global voxel maps with ray-traced clearing |
| `GsTeeCamera` | One GStreamer input split into raw BGR and Annex-B H.264 streams |
| Rerun visualization modules | Camera, point-cloud, map, TF, and teleoperation visualization |

## Available Skills

`FlightController` provides the flight skills below. They can be called through `dimos shell` in every PX4 blueprint. `px4-agentic` also exposes them to the agent through MCP.

| Skill | Parameters | Purpose and prerequisites |
|-------|------------|---------------------------|
| `arm()` | None | Arm the motors; first verify that the flight area is safe and PX4 permits arming |
| `disarm()` | None | Disarm the motors; call only after landing and when PX4 permits it |
| `takeoff(altitude=3.0)` | `altitude`: height above the takeoff point in meters | Take off to the target height; normally call after `arm()` succeeds |
| `land()` | None | Land at the current position |
| `enter_offboard()` | None | Send a zero-velocity setpoint and enter Offboard mode; must succeed before viewer direction controls are used |
| `exit_offboard()` | None | Exit Offboard mode |
| `hold()` | None | Exit Offboard control and switch PX4 to Hold mode |
| `move(forward=0, left=0, up=0, yaw_rate=0)` | Body-FLU velocity in m/s; counter-clockwise `yaw_rate` in rad/s | Set a body-frame velocity; requires Offboard mode |
| `goto(north, east, down, yaw=0)` | Local-NED position in meters; clockwise `yaw` in degrees | Set a local position and heading target; requires Offboard mode and valid local position data |
| `hover()` | None | Hold the latest local-NED position and heading; requires valid position data |

For example, run the following basic flight sequence in `dimos shell`. A successful RPC return means that the command was sent, not that the maneuver has completed, so check PX4 telemetry between steps. In particular, wait until the vehicle reaches a safe altitude and stabilizes before entering Offboard mode.

```python
app.FlightController.arm()
app.FlightController.takeoff(3.0)
# Wait until the vehicle reaches a safe altitude and stabilizes.
app.FlightController.enter_offboard()
app.FlightController.move(forward=0.5)
app.FlightController.hold()
app.FlightController.land()
```

With `px4-agentic`, list the skills actually registered with MCP using:

```bash
dimos mcp list-tools
```

## Installation

### Python dependencies

Run the following command from the DimOS repository root. The `drone` extra installs MAVSDK, pymavlink, and PyGObject:

```bash
uv sync --extra drone
```

### System dependencies

Ubuntu 22.04 requires the following system packages. Jetson hardware encoding additionally requires the NVIDIA GStreamer plugins provided by JetPack, including `nvv4l2h264enc` and `nvvidconv`.

```bash
sudo apt-get update
sudo apt-get install -y \
  gobject-introspection libgirepository1.0-dev libcairo2-dev pkg-config \
  python3-gi python3-gi-cairo \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

### Native PointLIO and ray-tracing modules

Nix with flakes enabled is required. Build both native executables once:

```bash
cd dimos/hardware/sensors/lidar/pointlio/cpp
nix build -L .#pointlio_native

cd ../../../../../mapping/ray_tracing/rust
nix build -L path:.
```

Each build creates a `result` link in its module directory. The corresponding DimOS `NativeModule` starts `result/bin/pointlio_native` or `result/bin/voxel_ray_tracing` from that directory.

## Hardware Configuration

### MAVLink

The default MAVLink endpoint is `serial:///dev/ttyTHS3:921600`, with a 10-second connection timeout. Override the endpoint with `--flightcontroller.connection-url` when the flight controller uses another serial device, and adjust the timeout with `--flightcontroller.connection-timeout-s`. The following complete hardware example changes only the serial device to `/dev/ttyUSB0`:

```bash
dimos run px4-basic \
  --flightcontroller.connection-url='serial:///dev/ttyUSB0:921600' \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50 \
  --gsteecamera.input-pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1' \
  --gsteecamera.input-format=raw
```

The SITL endpoint `udpin://0.0.0.0:14540` belongs to `px4-gazebo-harmonic` and should not be mixed into a `px4-basic` hardware configuration that also enables PointLIO.

### MID360 and external vision

The MID360 address is `192.168.1.3`. `--pointlio.host-ip` must be an address on the local lidar-facing interface in the same subnet. It may be omitted when PointLIO can select that interface automatically.

Point-LIO publishes the sensor pose and the mount publisher completes this TF chain:

```text
odom -> mid360_link -> base_link
```

`mid360_link -> base_link` applies the inverse of the calibrated 15-degree sensor mount transform. `FlightController` uses the same conversion to derive the body pose, converts FLU to FRD, and sends it to PX4 through MAVSDK `set_vision_position_estimate`.

## Video Pipeline

`GsTeeCamera` accepts a trusted GStreamer pipeline ending in raw video or H.264 and always publishes both outputs. Rerun records H.264 at `drone/video`; raw lidar visualization is limited to 5 Hz, while camera calibration and camera TF publication remain outside this stack.

| Stream | Transport | Content |
|--------|-----------|---------|
| `color_image` at `/color_image` | pSHM | Raw BGR frames |
| `video_h264` at `/video_h264` | Typed LCM | Annex-B H.264 access units |

Raw input is split before conversion and encoding. `bitrate` is measured in bits per second and `gop` is the maximum keyframe interval in frames; both NVV4L2 and X264 use these values:

```bash
dimos run px4-basic \
  --gsteecamera.input-pipeline='videotestsrc is-live=true' \
  --gsteecamera.input-format=raw \
  --gsteecamera.encoder=x264enc \
  --gsteecamera.bitrate=2000000 \
  --gsteecamera.gop=30 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

H.264 input is forwarded without re-encoding and decoded only for the BGR branch. This mode rejects explicit `encoder`, `bitrate`, or `gop` settings because they would be unused:

```bash
dimos run px4-basic \
  --gsteecamera.input-pipeline='rtspsrc location=rtsp://camera/stream latency=50 ! rtph264depay ! h264parse config-interval=-1' \
  --gsteecamera.input-format=h264 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

## File Structure

PX4 blueprints and dedicated modules live under `dimos/robot/drone/px4/`. PointLIO and ray tracing are shared DimOS modules under `dimos/hardware/sensors/lidar/pointlio/` and `dimos/mapping/ray_tracing/`, respectively:

```text
dimos/robot/drone/px4/
├── blueprints/
│   ├── basic/
│   │   ├── px4_basic.py             # Hardware and Gazebo blueprints
│   │   └── test_px4_basic.py
│   └── agentic/
│       └── px4_agentic.py           # MCP and LLM agent composition
├── flight_control.py                # MAVSDK flight control and external vision
├── gstreamer_tee_camera.py          # Raw BGR and H.264 GStreamer tee
├── mid360_mount_tf.py               # Calibrated MID360-to-body transform
├── test_external_vision.py
├── test_gstreamer_tee_camera.py
├── test_mid360_mount_tf.py
└── README.md
```

## Validation and Safety

Validate command paths against PX4 SITL before connecting real hardware, and do not arm or enter Offboard mode during telemetry-only checks. Run the following code checks:

```bash
uv run pytest dimos/robot/drone/px4 -q
uv run ruff check dimos/robot/drone/px4
```
