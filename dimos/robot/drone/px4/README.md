# PX4 Module

DimOS integration for PX4 drones using MAVSDK. The module combines flight
control, MID360 localization, voxel mapping, GStreamer video distribution,
Rerun visualization, Gazebo Harmonic simulation, and optional LLM agent
control.

The hardware stack currently targets a PX4 flight controller connected at
`serial:///dev/ttyTHS3:921600` and a Livox MID360 at `192.168.1.3`.

## Quick Start

The following commands assume that the [installation](#installation) is
complete. They explicitly provide the minimum hardware inputs used by the
current setup: MID360 network addresses, MAVSDK serial connection, and V4L2
camera pipeline. Replace the device paths and local host address when the
hardware differs.

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

Terminal 1:

```bash
cd /path/to/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_mono_cam
```

After PX4 reports its MAVLink endpoint, start DimOS in terminal 2:

```bash
cd /path/to/dimos
dimos run px4-gazebo-harmonic
```

Use `dimos status`, `dimos log -f`, and `dimos stop` to inspect and stop a
running stack.

## Blueprints and Modules

### `px4-basic`

The complete hardware stack for flight, localization, mapping, video, and
visualization.

| Module | Purpose |
|--------|---------|
| `FlightController` | Direct MAVSDK connection, telemetry, flight skills, Offboard velocity control, and external-vision forwarding |
| `PointLio` | MID360 point-cloud processing and lidar odometry |
| `Mid360MountStaticTf` | Calibrated static transform from `mid360_link` to `base_link` |
| `RayTracingVoxelMap` | Local and global voxel maps with ray-traced clearing |
| `GsTeeCamera` | One GStreamer input split into raw BGR and Annex-B H.264 streams |
| Rerun visualization modules | Camera, point-cloud, map, TF, and teleoperation visualization |

### `px4-agentic`

Builds on `px4-basic` and adds `McpServer`, `McpClient`, and `WebInput`. Flight
methods marked with `@skill` become MCP tools available to the LLM agent. The
MCP server uses the configured DimOS MCP port.

### `px4-gazebo-harmonic`

Runs `FlightController` against PX4 SITL at `udpin://0.0.0.0:14540`, consumes
the Gazebo UDP/RTP H.264 stream on port `5600`, and publishes it through the
same DimOS camera streams used by the hardware stack. PointLIO and voxel
mapping are not part of this simulation blueprint. Its Rerun layout therefore
contains one large camera view and no 3D mapping view.

## Installation

### Python dependencies

From the DimOS repository root:

```bash
uv sync --extra drone
```

The `drone` extra installs MAVSDK, pymavlink, and PyGObject.

### System dependencies

On Ubuntu 22.04:

```bash
sudo apt-get update
sudo apt-get install -y \
  gobject-introspection libgirepository1.0-dev libcairo2-dev pkg-config \
  python3-gi python3-gi-cairo \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

Jetson hardware encoding additionally requires the NVIDIA GStreamer plugins
provided by JetPack, including `nvv4l2h264enc` and `nvvidconv`.

### Native PointLIO and ray-tracing modules

Nix with flakes enabled is required. Build both native executables once:

```bash
cd dimos/hardware/sensors/lidar/pointlio/cpp
nix build -L .#pointlio_native

cd ../../../../../mapping/ray_tracing/rust
nix build -L path:.
```

Each build creates a `result` link in its module directory. The corresponding
DimOS `NativeModule` starts `result/bin/pointlio_native` or
`result/bin/voxel_ray_tracing` from that directory.

## Hardware Configuration

### MAVLink

The default endpoint is `serial:///dev/ttyTHS3:921600`. Override it when the
flight controller uses another serial device or network transport:

```bash
dimos run px4-basic \
  --flightcontroller.connection-url=udpin://0.0.0.0:14540 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

The connection timeout defaults to 10 seconds and can be changed with
`--flightcontroller.connection-timeout-s`.

### MID360 and external vision

The MID360 address is `192.168.1.3`. `--pointlio.host-ip` must be an address on
the local lidar-facing interface in the same subnet. It may be omitted when
PointLIO can select that interface automatically.

Point-LIO publishes the sensor pose and the mount publisher completes this TF
chain:

```text
odom -> mid360_link -> base_link
```

`mid360_link -> base_link` applies the inverse of the calibrated 15-degree
sensor mount transform. `FlightController` uses the same conversion to derive
the body pose, converts FLU to FRD, and sends it to PX4 through MAVSDK
`set_vision_position_estimate`.

## Video Pipeline

`GsTeeCamera` accepts a trusted GStreamer pipeline ending in raw video or
H.264. It always publishes both outputs:

| Stream | Transport | Content |
|--------|-----------|---------|
| `color_image` at `/color_image` | pSHM | Raw BGR frames |
| `video_h264` at `/video_h264` | Typed LCM | Annex-B H.264 access units |

Rerun records H.264 at `drone/video`. Raw lidar visualization is limited to
5 Hz. Camera calibration and camera TF publication are intentionally outside
this stack.

Raw input is split before conversion and encoding:

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

For raw input, `bitrate` is in bits per second and `gop` is the maximum
keyframe interval in frames. Both NVV4L2 and X264 consume these values.

H.264 input is forwarded without re-encoding and decoded only for the BGR
branch:

```bash
dimos run px4-basic \
  --gsteecamera.input-pipeline='rtspsrc location=rtsp://camera/stream latency=50 ! rtph264depay ! h264parse config-interval=-1' \
  --gsteecamera.input-format=h264 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

Explicit `encoder`, `bitrate`, or `gop` overrides are rejected for H.264 input
because they would otherwise be unused.

## Gazebo Harmonic Details

Use a PX4 model with a camera, such as `gz_x500_mono_cam`. For a separately
launched gz-server, start it with the official `simulation-gazebo` script and
then run:

```bash
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam
```

If PX4 needs longer to establish its Onboard MAVLink link:

```bash
dimos run px4-gazebo-harmonic \
  --flightcontroller.connection-timeout-s=20
```

To change the Gazebo video port, configure the upstream PX4 world plugin and
override the complete input pipeline:

```bash
dimos run px4-gazebo-harmonic \
  --gsteecamera.input-pipeline='udpsrc port=5601 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! h264parse config-interval=-1' \
  --gsteecamera.input-format=h264
```

QGroundControl and DimOS cannot both receive the unicast stream on UDP `5600`.
Close QGroundControl video reception before starting DimOS, or configure the
PX4 `GstCameraSystem` world plugin and the DimOS input pipeline to use another
port.

## File Structure

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
├── README.md
└── README.zh-CN.md
```

PointLIO and ray tracing are shared DimOS modules under
`dimos/hardware/sensors/lidar/pointlio/` and
`dimos/mapping/ray_tracing/` respectively.

## Validation and Safety

```bash
uv run pytest dimos/robot/drone/px4 -q
uv run ruff check dimos/robot/drone/px4
```

Validate command paths against PX4 SITL before connecting real hardware. Do
not arm or enter Offboard mode during telemetry-only checks.
