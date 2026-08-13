# PX4 模块

本模块通过 MAVSDK 将 PX4 无人机集成到 DimOS，组合飞行控制、MID360 定位、
体素建图、GStreamer 视频分流、Rerun 可视化、Gazebo Harmonic 仿真，以及可选的
LLM 智能体控制。

当前实机技术栈默认连接位于 `serial:///dev/ttyTHS3:921600` 的 PX4 飞控，以及
地址为 `192.168.1.3` 的 Livox MID360。

## 快速开始

以下命令假定已经完成[安装](#安装指南)，并显式提供当前环境启动所需的最小硬件
输入：MID360 网络地址、MAVSDK 串口连接和 V4L2 相机管线。硬件不同时，应替换
对应的设备路径和本机地址。

### 实机基础蓝图：`px4-basic`

```bash
dimos run px4-basic \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50 \
  --flightcontroller.connection-url='serial:///dev/ttyTHS3:921600' \
  --flightcontroller.connection-timeout-s=20 \
  --gsteecamera.input-pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1' \
  --gsteecamera.input-format=raw
```

### 实机智能体蓝图：`px4-agentic`

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

在另一个终端中查看可用工具或向智能体发送请求：

```bash
dimos mcp list-tools
dimos agent-send "起飞到 2 米高度"
```

### 仿真蓝图：`px4-gazebo-harmonic`

终端 1：

```bash
cd /path/to/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_mono_cam
```

PX4 输出 MAVLink 端点后，在终端 2 启动 DimOS：

```bash
cd /path/to/dimos
dimos run px4-gazebo-harmonic
```

使用 `dimos status`、`dimos log -f` 和 `dimos stop` 检查及停止运行中的技术栈。

## 蓝图与模块

### `px4-basic`

用于飞行、定位、建图、视频和可视化的完整实机技术栈。

| 模块 | 作用 |
|------|------|
| `FlightController` | 直接连接 MAVSDK，处理遥测、飞行技能、Offboard 速度控制和外部视觉转发 |
| `PointLio` | 处理 MID360 点云并输出激光雷达里程计 |
| `Mid360MountStaticTf` | 发布从 `mid360_link` 到 `base_link` 的标定静态变换 |
| `RayTracingVoxelMap` | 通过光线追踪清除动态物体，生成局部和全局体素地图 |
| `GsTeeCamera` | 将一路 GStreamer 输入分流为原始 BGR 和 Annex-B H.264 |
| Rerun 可视化模块 | 显示相机、点云、地图和 TF，并提供遥控输入 |

### `px4-agentic`

在 `px4-basic` 基础上增加 `McpServer`、`McpClient` 和 `WebInput`。
标记为 `@skill` 的飞行方法会成为 LLM 智能体可调用的 MCP 工具。MCP 服务使用
DimOS 配置的 MCP 端口。

### `px4-gazebo-harmonic`

使用 `udpin://0.0.0.0:14540` 连接 PX4 SITL，在端口 `5600` 接收 Gazebo 的
UDP/RTP H.264 视频，并通过与实机相同的 DimOS 相机数据流发布。该仿真蓝图不包含
PointLIO 和体素建图，因此其 Rerun 布局只显示一个大尺寸相机画面，不显示 3D
建图视图。

## 安装指南

### Python 依赖

在 DimOS 仓库根目录执行：

```bash
uv sync --extra drone
```

`drone` 可选依赖包含 MAVSDK、pymavlink 和 PyGObject。

### 系统依赖

Ubuntu 22.04：

```bash
sudo apt-get update
sudo apt-get install -y \
  gobject-introspection libgirepository1.0-dev libcairo2-dev pkg-config \
  python3-gi python3-gi-cairo \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

Jetson 硬件编码还需要 JetPack 提供的 NVIDIA GStreamer 插件，包括
`nvv4l2h264enc` 和 `nvvidconv`。

### PointLIO 和光线追踪原生模块

需要启用 flakes 的 Nix。首次运行前构建两个原生可执行文件：

```bash
cd dimos/hardware/sensors/lidar/pointlio/cpp
nix build -L .#pointlio_native

cd ../../../../../mapping/ray_tracing/rust
nix build -L path:.
```

每个构建都会在对应模块目录创建 `result` 链接。相应的 DimOS `NativeModule`
会从该目录启动 `result/bin/pointlio_native` 或
`result/bin/voxel_ray_tracing`。

## 实机配置

### MAVLink

默认端点为 `serial:///dev/ttyTHS3:921600`。飞控使用其他串口或网络传输时，
可以覆盖该配置：

```bash
dimos run px4-basic \
  --flightcontroller.connection-url=udpin://0.0.0.0:14540 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

连接超时时间默认为 10 秒，可以通过
`--flightcontroller.connection-timeout-s` 修改。

### MID360 与外部视觉

MID360 地址为 `192.168.1.3`。`--pointlio.host-ip` 必须是本机连接激光雷达、
且与其处于同一子网的网络接口地址。当 PointLIO 能自动选择该接口时，可以省略
此参数。

Point-LIO 发布传感器位姿，安装变换发布器补全以下 TF 链：

```text
odom -> mid360_link -> base_link
```

`mid360_link -> base_link` 使用经过标定的 15 度传感器安装变换的逆变换。
`FlightController` 使用相同转换得到机体位姿，将 FLU 转换为 FRD，然后通过
MAVSDK `set_vision_position_estimate` 发送给 PX4。

## 视频管线

`GsTeeCamera` 接受末端为原始视频或 H.264 的可信 GStreamer 管线，并始终发布
两路输出：

| 数据流 | 传输 | 内容 |
|--------|------|------|
| `/color_image` 上的 `color_image` | pSHM | 原始 BGR 帧 |
| `/video_h264` 上的 `video_h264` | 类型化 LCM | Annex-B H.264 访问单元 |

Rerun 在 `drone/video` 记录 H.264。原始激光雷达数据的可视化频率限制为 5 Hz。
相机标定和相机 TF 发布有意不包含在此技术栈中。

原始视频输入会在转换和编码前分流：

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

对于原始输入，`bitrate` 单位为 bit/s，`gop` 表示以帧数计的最大关键帧间隔。
NVV4L2 和 X264 都会使用这些参数。

H.264 输入不会重新编码，只在 BGR 分支进行解码：

```bash
dimos run px4-basic \
  --gsteecamera.input-pipeline='rtspsrc location=rtsp://camera/stream latency=50 ! rtph264depay ! h264parse config-interval=-1' \
  --gsteecamera.input-format=h264 \
  --pointlio.lidar-ip=192.168.1.3 \
  --pointlio.host-ip=192.168.1.50
```

H.264 输入模式会拒绝显式设置 `encoder`、`bitrate` 或 `gop`，因为这些参数
不会被使用。

## Gazebo Harmonic 说明

应使用带相机的 PX4 模型，例如 `gz_x500_mono_cam`。如果 gz-server 单独启动，
应先使用官方 `simulation-gazebo` 脚本启动，然后执行：

```bash
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam
```

如果 PX4 建立 Onboard MAVLink 链路需要更长时间：

```bash
dimos run px4-gazebo-harmonic \
  --flightcontroller.connection-timeout-s=20
```

如需修改 Gazebo 视频端口，应配置上游 PX4 world 插件，并覆盖完整输入管线：

```bash
dimos run px4-gazebo-harmonic \
  --gsteecamera.input-pipeline='udpsrc port=5601 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! h264parse config-interval=-1' \
  --gsteecamera.input-format=h264
```

QGroundControl 和 DimOS 无法同时接收 UDP `5600` 上的单播视频流。启动 DimOS
前应关闭 QGroundControl 的视频接收，或者将 PX4 `GstCameraSystem` world 插件
和 DimOS 输入管线配置为使用其他端口。

## 文件结构

```text
dimos/robot/drone/px4/
├── blueprints/
│   ├── basic/
│   │   ├── px4_basic.py             # 实机及 Gazebo 蓝图
│   │   └── test_px4_basic.py
│   └── agentic/
│       └── px4_agentic.py           # MCP 和 LLM 智能体组合
├── flight_control.py                # MAVSDK 飞行控制和外部视觉
├── gstreamer_tee_camera.py          # 原始 BGR 与 H.264 视频分流
├── mid360_mount_tf.py               # MID360 到机体的标定变换
├── test_external_vision.py
├── test_gstreamer_tee_camera.py
├── test_mid360_mount_tf.py
├── README.md
└── README.zh-CN.md
```

PointLIO 和光线追踪是 DimOS 的共享模块，分别位于
`dimos/hardware/sensors/lidar/pointlio/` 和
`dimos/mapping/ray_tracing/`。

## 验证与安全

```bash
uv run pytest dimos/robot/drone/px4 -q
uv run ruff check dimos/robot/drone/px4
```

连接实机前，应先在 PX4 SITL 中验证命令路径。仅检查遥测数据时，不要解锁，
也不要进入 Offboard 模式。
