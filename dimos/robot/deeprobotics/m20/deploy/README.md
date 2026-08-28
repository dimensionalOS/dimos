# M20 lidar deployment

DimOS consumes one merged cloud on GOS. The verified data path is:

```text
front lidar 10.21.33.201 -- MSOP 6691 / DIFOP 7781 --+
                                                        +--> NOS multicast-relay.service
rear lidar  10.21.33.202 -- MSOP 6692 / DIFOP 7782 --+       --> GOS rsdriver.service
                                                               --> DDS /LIDAR/POINTS
                                                               --> m20_ros_bridge
                                                               --> local LCM lidar stream
                                                               --> RayTracingVoxelMap
```

The two lidars are already extrinsically merged by the vendor driver. On the
inspected robot, `/LIDAR/POINTS` is reliable/volatile at about 9.5 Hz, with
29,000-81,000 points and 0.8-2.1 MB per cloud. Its 26-byte point layout is:

| field | type | byte offset |
|---|---|---:|
| `x` | float32 | 0 |
| `y` | float32 | 4 |
| `z` | float32 | 8 |
| `intensity` | float32 | 12 |
| `ring` | uint16 | 16 |
| `timestamp` | float64 | 18 |

## Persistent robot setup

NOS must have its existing relay enabled. Its vendor unit already uses
`Restart=always` and `RestartSec=2`:

```bash
sudo systemctl enable --now multicast-relay.service
systemctl is-enabled multicast-relay.service
systemctl is-active multicast-relay.service
```

Run those commands on NOS (`10.21.31.106`).

GOS runs `rsdriver.service` as root for real-time scheduling. Fast DDS therefore
creates root-owned shared-memory files that an unprivileged DimOS process cannot
attach to. From a DimOS checkout on GOS, install the checked-in permission helper
and systemd drop-in, then enable the driver:

```bash
sudo install -D -o root -g root -m 0755 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-rsdriver-shm-permissions \
  /usr/local/libexec/dimos-m20-rsdriver-shm-permissions
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/rsdriver.service.d/10-dimos-shm-permissions.conf \
  /etc/systemd/system/rsdriver.service.d/10-dimos-shm-permissions.conf
sudo systemctl daemon-reload
sudo systemctl enable --now rsdriver.service
```

The drop-in is outside the vendor package, so it survives package replacement.
The vendor package's post-install script may disable `rsdriver.service`; re-run
the `enable --now` command after a driver or firmware update.

## Run DimOS locally on GOS

Load the local-only LCM URL before launching DimOS. Its explicit 16 MiB receive
buffer is needed for fragmented multi-megabyte clouds:

```bash
set -a
source dimos/robot/deeprobotics/m20/deploy/dimos-m20.env
set +a
dimos --rerun-open none --rerun-host 0.0.0.0 \
  run deeprobotics-m20-kronknav
```

Attach a viewer from another computer without moving the DimOS graph off GOS:

```bash
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

The default blueprint never creates a `/NAV_CMD` publisher. Use the separate
`deeprobotics-m20-kronknav-control` blueprint only when motion ownership is
intentional; it still starts disarmed.

## Health and recovery contract

Before starting DimOS, both checks below must print `enabled` and `active` on
their respective hosts:

```bash
systemctl is-enabled multicast-relay.service  # NOS
systemctl is-active multicast-relay.service

systemctl is-enabled rsdriver.service         # GOS
systemctl is-active rsdriver.service
```

As the normal `user` account on GOS, this measures the DDS stream itself rather
than merely checking that the driver process exists:

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic hz --wall-time --window 100 /LIDAR/POINTS
```

The native bridge additionally validates every cloud and publishes
`lidar_ready` at 10 Hz. Five missed nominal frames (0.5 seconds) make it false.
The bridge logs `M20 lidar stream is missing or stale` once on loss and
`M20 lidar stream is healthy` once on recovery. Both the native `/NAV_CMD`
watchdog and `M20Connection` require fresh lidar, so loss immediately forces
zero velocity and disarms the Python gate. DDS rematches automatically after an
`rsdriver.service` restart; DimOS does not need to restart.

Process supervision cannot make a disconnected or unpowered sensor produce
data. The contract is therefore: restart crashed vendor processes, detect an
invalid or absent stream within 0.5 seconds, fail closed, expose the state, and
recover automatically when valid clouds return.
