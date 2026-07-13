# How to Optimize Point-LIO Configs

1. Record mid360 with PCAP enabled — see [Recording a Map (Go2 + Mid-360)](go2_mid360.md)
2. Use `pcap_to_db.py` to generate alternative lidar/odom outcomes - renders to rerun

# Modules

- **`VirtualMid360`** — replays the pcap (aliasing the host/lidar IPs onto a
  dummy interface on Linux, or `lo0` on macOS).
- **`PointLio`** — an unmodified, live Point-LIO that consumes the replay as if
  it were real hardware.
- **`PointlioRecorder`** — appends the `pointlio_odometry` / `pointlio_lidar`
  streams into the db.

### Pcap to DB

```bash
PCAP_EXAMPLE="mid360_shake_stairs/mid360_shake_stairs.pcap"
DB="mem2.db"

python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --db "$DB" \
    --pcap "$PCAP_EXAMPLE" \
    --filter-size-surf 0.15  \
    --filter-size-map 0.5 \
    --no-imu-en

# ^ should
# 1. open up rerun with aggregated map + odom path
# 2. add pointlio stream to the .db file

# generate a map
dimos map global --lidar pointlio_lidar --pgo-tol=0 --no-carve
```

#### Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--pcap` | *(required)* | Livox Mid-360 pcap (a missing path is fetched via `get_data`) |
| `--db` | `<pcap>.db` | Target memory2 db. Existing → append/align; missing → built from scratch (or fetched via `get_data`) |
| `--rate` | `1.0` | Replay-speed multiplier |
| `--odom-freq` | `30.0` | Point-LIO odometry rate (Hz) |
| `--max-sensor-sec` | `0` (whole pcap) | Stop after N sensor seconds |
| `--warmup-sec` | `4.0` | Seconds the fake lidar waits before streaming (lets Point-LIO come up) |
| `--no-rrd` | off | Skip writing the `<db>.rrd` quick-look |
| `--voxel` | `0.2` | Voxel size (m) for the `.rrd` aggregated map |
| `--host-ip` | `192.168.1.5` | Host IP (override to run two replays at once) |
| `--lidar-ip` | `192.168.1.155` | Synthetic lidar IP |
| `--alias-iface` | `dimos-mid360` | Dummy iface the host/lidar IPs live on |
| `--no-network-setup` | off | Don't let the module alias the NIC via sudo — you've set up the IPs + routes yourself |

#### Tuning flags

| Flag | Default | Meaning |
|------|---------|---------|
| `--con-frame` | off | Accumulate multiple sweeps into one frame |
| `--con-frame-num` | `1` | Sweeps per accumulated frame (`con_frame`) |
| `--cut-frame` | off | Split each sweep into time sub-frames |
| `--cut-frame-time-interval` | `0.1` | Sub-frame interval (s) when `cut_frame` |
| `--time-lag-imu-to-lidar` | `0.0` | IMU→lidar clock offset (s) |
| `--lidar-type` | `avia` | Driver branch: `avia` (Livox Mid-360) / `velodyne` / `ouster` / `hesai` / `unilidar` |
| `--scan-line` | `4` | Number of scan lines |
| `--scan-rate` | `10` | Scan rate (Hz) |
| `--timestamp-unit` | `nanosecond` | Per-point timestamp unit: `second` / `millisecond` / `microsecond` / `nanosecond` |
| `--blind` | `0.5` | Spherical min range (m); nearer points dropped |
| `--point-filter-num` | `3` | Keep every Nth raw point (1 = all) |
| `--use-imu-as-input` | off | IMU-as-input model (default robust IMU-as-output) |
| `--prop-at-freq-of-imu` | on | Propagate state at IMU frequency |
| `--check-satu` | on | Zero residuals on saturated IMU samples |
| `--init-map-size` | `10` | Initial iVox map size |
| `--space-down-sample` | on | Voxel-downsample each scan (leaf = `--filter-size-surf`) |
| `--satu-acc` | `3.0` | Accel saturation threshold (g) |
| `--satu-gyro` | `35.0` | Gyro saturation threshold (deg/s) |
| `--acc-norm` | `1.0` | IMU accel unit (1 = g, 9.81 = m/s²) |
| `--plane-thr` | `0.1` | Plane-fit residual threshold (m) |
| `--filter-size-surf` | `0.2` | Pre-KF scan downsample leaf (m) |
| `--filter-size-map` | `0.5` | Persistent map voxel leaf (m) |
| `--ivox-grid-resolution` | `2.0` | iVox local-map grid (m) |
| `--ivox-nearby-type` | `nearby6` | iVox neighbour stencil: `center` / `nearby6` / `nearby18` / `nearby26` |
| `--cube-side-length` | `1000.0` | Map cube side length (m) |
| `--det-range` | `100.0` | Max detection range (m) |
| `--fov-degree` | `360.0` | Horizontal FOV (deg) |
| `--imu-en` | on | Use the IMU |
| `--start-in-aggressive-motion` | off | Skip the static IMU-init assumption |
| `--extrinsic-est-en` | off | Online-estimate the IMU→lidar extrinsic |
| `--imu-time-inte` | `0.005` | IMU integration step (s) |
| `--lidar-meas-cov` | `0.01` | Lidar measurement covariance |
| `--acc-cov-input` | `0.1` | Accel process cov (input model) |
| `--vel-cov` | `20.0` | Velocity process covariance |
| `--gyr-cov-input` | `0.01` | Gyro process cov (input model) |
| `--gyr-cov-output` | `1000.0` | Gyro process cov (output model) |
| `--acc-cov-output` | `500.0` | Accel process cov (output model) |
| `--b-gyr-cov` | `0.0001` | Gyro-bias random-walk covariance |
| `--b-acc-cov` | `0.0001` | Accel-bias random-walk covariance |
| `--imu-meas-acc-cov` | `0.01` | Accel measurement covariance |
| `--imu-meas-omg-cov` | `0.01` | Gyro measurement covariance |
| `--match-s` | `81.0` | Point-to-plane match scale |
| `--gravity-align` | on | Align initial gravity to −Z |
| `--gravity` | `0 0 -9.81` | Gravity vector: `x y z` (m/s²) |
| `--gravity-init` | `0 0 -9.81` | Initial gravity estimate: `x y z` (m/s²) |
| `--extrinsic-t` | `-0.011 -0.02329 0.04412` | IMU→lidar translation: `x y z` (m) |
| `--extrinsic-r` | identity | IMU→lidar rotation: 9 values row-major |
| `--publish-odometry-without-downsample` | off | Publish odom per scan, no downsample |
| `--odom-only` | off | Odometry only, skip map publishing |

#### MacOS Caveats

The module aliases the synthetic IPs onto `lo0`, which needs sudo. A tty-less
worker can't prompt, so set up the interface by hand, then pass
`--no-network-setup`:

```bash
sudo ifconfig lo0 alias 192.168.1.5 netmask 255.255.255.0
sudo ifconfig lo0 alias 192.168.1.155 netmask 255.255.255.0
sudo route -n add -host 224.1.1.5 -interface lo0
sudo route -n add -host 255.255.255.255 -interface lo0

python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --pcap "$PCAP" --no-network-setup
```

### Replay a pcap to a module

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.visualization.vis_module import vis_module

replay = autoconnect(
    VirtualMid360.blueprint(
        pcap="recordings/run1.pcap",
        # lidar_ip="192.168.1.155",
    ),
    PointLio.blueprint(
        # lidar_ip="192.168.1.155",
    ),
    vis_module("rerun"),
).global_config(n_workers=3)
ModuleCoordinator.build(replay).loop()
```

## Notes

- Replay runs in **real time** and Point-LIO is **not deterministic**, so
  successive runs differ.
