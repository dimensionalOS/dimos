# Livox Mid-360 Native Module (C++)

Native C++ driver for the Livox Mid-360 LiDAR. Publishes PointCloud2 and IMU
data directly on LCM, bypassing Python for minimal latency.

## Build

### Nix (recommended)

```bash
cd dimos/hardware/sensors/lidar/livox/cpp
nix build .#mid360_native
```

Binary lands at `result/bin/mid360_native`.

To build just the Livox SDK2 library:

```bash
nix build .#livox-sdk2
```

### Native (CMake)

Requires:
- CMake >= 3.14
- [LCM](https://lcm-proj.github.io/) (`pacman -S lcm` or build from source)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) installed to `/usr/local`

Installing Livox SDK2 manually:

```bash
cd ~/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

Then build:

```bash
cd dimos/hardware/sensors/lidar/livox/cpp
cmake -B build
cmake --build build -j$(nproc)
cmake --install build
```

Binary lands at `result/bin/mid360_native` (same location as nix).

CMake automatically fetches [dimos-lcm](https://github.com/dimensionalOS/dimos-lcm)
for the C++ message headers on first configure.

## Network setup

The Mid-360 communicates over USB ethernet. Configure the interface:

```bash
sudo nmcli con add type ethernet ifname usbeth0 con-name livox-mid360 \
    ipv4.addresses 192.168.1.5/24 ipv4.method manual
sudo nmcli con up livox-mid360
```

This persists across reboots. The lidar defaults to `192.168.1.155`.

## Usage

Normally launched by `Mid360` via the NativeModule framework:

```python
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.core.blueprints import autoconnect

autoconnect(
    Mid360.blueprint(host_ip="192.168.1.5"),
    SomeConsumer.blueprint(),
).build().loop()
```

### Manual invocation (for debugging)

```bash
./result/bin/mid360_native \
    --pointcloud '/pointcloud#sensor_msgs.PointCloud2' \
    --imu '/imu#sensor_msgs.Imu' \
    --host_ip 192.168.1.5 \
    --lidar_ip 192.168.1.155 \
    --frequency 10
```

Topic strings must include the `#type` suffix -- this is the actual LCM channel
name used by dimos subscribers.

View data in another terminal:

For full vis:
```sh
rerun-bridge
```

For LCM traffic:
```sh
lcm-spy
```

## File overview

| File                      | Description                                              |
|---------------------------|----------------------------------------------------------|
| `main.cpp`                | Livox SDK2 callbacks, frame accumulation, LCM publishing |
| `dimos_native_module.hpp` | Reusable header for parsing NativeModule CLI args        |
| `flake.nix`               | Nix flake for hermetic builds                            |
| `CMakeLists.txt`          | Build config, fetches dimos-lcm headers automatically    |
| `../module.py`            | Python NativeModule wrapper (`Mid360`)                   |
