# FAST-LIO2 Native Module (C++)

Real-time LiDAR SLAM using FAST-LIO2 with integrated Livox Mid-360 driver.
Binds Livox SDK2 directly into FAST-LIO-NON-ROS: SDK callbacks feed
CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM. Registered
(world-frame) point clouds and odometry are published on LCM.

## Build

### Nix (recommended)

```bash
cd dimos/hardware/sensors/lidar/fastlio2/cpp
nix build .#fastlio2_native
```

Binary lands at `result/bin/fastlio2_native`.

The flake pulls Livox SDK2 from the livox sub-flake and
[FAST-LIO-NON-ROS](https://github.com/leshy/FAST-LIO-NON-ROS) from GitHub
automatically.

### Native (CMake)

Requires:
- CMake >= 3.14
- [LCM](https://lcm-proj.github.io/) (`pacman -S lcm` or build from source)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) installed to `/usr/local`
- Eigen3, PCL (common, filters), yaml-cpp, Boost, OpenMP
- [FAST-LIO-NON-ROS](https://github.com/leshy/FAST-LIO-NON-ROS) checked out locally

```bash
cd dimos/hardware/sensors/lidar/fastlio2/cpp
cmake -B build -DFASTLIO_DIR=$HOME/coding/FAST-LIO-NON-ROS
cmake --build build -j$(nproc)
cmake --install build
```

Binary lands at `result/bin/fastlio2_native` (same location as nix).

If `-DFASTLIO_DIR` is omitted, CMake auto-fetches FAST-LIO-NON-ROS from GitHub.

## Network setup

The Mid-360 communicates over USB ethernet. Configure the interface:

```bash
sudo nmcli con add type ethernet ifname usbeth0 con-name livox-mid360 \
    ipv4.addresses 192.168.1.5/24 ipv4.method manual
sudo nmcli con up livox-mid360
```

This persists across reboots. The lidar defaults to `192.168.1.155`.

## Usage

Normally launched by `FastLio2` via the NativeModule framework:

```python
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.core.blueprints import autoconnect

autoconnect(
    FastLio2.blueprint(host_ip="192.168.1.5"),
    SomeConsumer.blueprint(),
).build().loop()
```

### Manual invocation (for debugging)

```bash
./result/bin/fastlio2_native \
    --lidar '/pointcloud#sensor_msgs.PointCloud2' \
    --odometry '/odometry#nav_msgs.Odometry' \
    --host_ip 192.168.1.5 \
    --lidar_ip 192.168.1.155 \
    --config_path ../config/mid360.yaml
```

Topic strings must include the `#type` suffix -- this is the actual LCM channel
name used by dimos subscribers.

For full vis:
```sh
rerun-bridge
```

For LCM traffic:
```sh
lcm-spy
```

## Configuration

FAST-LIO2 config files live in `config/`. The YAML config controls filter
parameters, EKF tuning, and point cloud processing settings.

## File overview

| File                      | Description                                                  |
|---------------------------|--------------------------------------------------------------|
| `main.cpp`                | Livox SDK2 + FAST-LIO2 integration, EKF SLAM, LCM publishing |
| `cloud_filter.hpp`        | Point cloud filtering (range, voxel downsampling)            |
| `voxel_map.hpp`           | Global voxel map accumulation                                |
| `dimos_native_module.hpp` | Reusable header for parsing NativeModule CLI args            |
| `config/`                 | FAST-LIO2 YAML configuration files                           |
| `flake.nix`               | Nix flake for hermetic builds                                |
| `CMakeLists.txt`          | Build config, fetches dimos-lcm headers automatically        |
| `../module.py`            | Python NativeModule wrapper (`FastLio2`)                     |
