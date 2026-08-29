# M20 lidar deployment

DimOS consumes one merged cloud on GOS. The verified data path is:

```text
front lidar 10.21.33.201 -- MSOP 6691 / DIFOP 7781 --+
                                                        +--> NOS multicast-relay.service
rear lidar  10.21.33.202 -- MSOP 6692 / DIFOP 7782 --+       --> GOS rsdriver.service
                                                               --> DDS /LIDAR/POINTS
                                                               --> m20_ros_bridge
                                                               --> local LCM raw lidar + IMU
                                                               --> M20PointLio
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

NOS must have its existing relay enabled. Install the checked-in supervisor and
drop-in before enabling it:

```bash
sudo install -D -o root -g root -m 0755 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-multicast-relay-supervisor \
  /usr/local/libexec/dimos-m20-multicast-relay-supervisor
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/multicast-relay.service.d/10-dimos-network-readiness.conf \
  /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf
sudo systemctl daemon-reload
sudo systemctl enable --now multicast-relay.service
systemctl is-enabled multicast-relay.service
systemctl is-active multicast-relay.service
```

Run those commands on NOS (`10.21.31.106`).

The vendor Python process starts four forwarding threads but does not propagate
a worker-thread failure to systemd. It can therefore remain `active` while one
or both MSOP streams are dead. The supervisor waits for both NOS Ethernet
addresses and multicast routes before launch, then restarts the service if the
process has fewer than its expected four forwarding workers.

The control blueprint owns `/NAV_CMD`. The M20 manual explicitly requires the
vendor `planner.service` to be stopped before an external publisher uses that
topic. Install the checked-in ownership condition on NOS so a boot script cannot
silently start a second command owner:

```bash
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/planner.service.d/10-dimos-command-ownership.conf \
  /etc/systemd/system/planner.service.d/10-dimos-command-ownership.conf
sudo systemctl unmask planner.service
sudo systemctl daemon-reload
sudo systemctl stop planner.service
systemctl is-active planner.service  # must print inactive
```

To deliberately restore the vendor planner, create
`/etc/dimos/enable-vendor-m20-planner` and start the service. Never run it while
the DimOS control blueprint owns `/NAV_CMD`.

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

## Cable-free operator access

GOS has no Wi-Fi radio. Use the vendor-managed AP on AOS while DimOS continues
to run entirely on GOS. The inspected robot exposes `m20_24G`; NetworkManager
gives clients an address in `10.21.41.0/24`, with AOS at `10.21.41.1`, and
routes them to the internal `10.21.31.0/24` network.

The AP credential is robot configuration and is intentionally not stored in
this repository. It can be read or changed on AOS using the vendor Wi-Fi tools.
The vendor `start.service` launches `loop_start_ap.sh` at boot, which
recreates the AP when `wlan0` is down. No DimOS service is required on AOS.

If an old office-client experiment is installed, return AOS to the vendor AP:

```bash
sudo systemctl disable --now dimos-m20-office-wifi.service 2>/dev/null || true
sudo systemctl disable --now wifi-office-autoswitch.service 2>/dev/null || true
sudo systemctl disable --now zenoh-router.service 2>/dev/null || true
sudo nmcli connection modify office5g connection.autoconnect no 2>/dev/null || true
sudo nmcli connection down office5g 2>/dev/null || true
```

After a few seconds, verify on AOS:

```bash
iw dev wlan0 info
ip -4 address show dev wlan0
```

The output must show `type AP`, the intended SSID, and
`10.21.41.1/24`. Connect the developer computer to that SSID; it should use
DHCP. On macOS, add the robot-subnet route explicitly so a simultaneous USB
phone tether remains the internet default:

```bash
networksetup -setadditionalroutes \
  "Wi-Fi" 10.21.31.0 255.255.255.0 10.21.41.1
route -n get 10.21.31.104
ping 10.21.31.104
ssh user@10.21.31.104
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

Use a separate USB phone tether if the developer computer also needs internet;
the robot AP is the robot route, not the office internet connection. No DimOS
module, LCM traffic, ROS/DDS traffic, or Zenoh router runs on AOS.

## Run DimOS locally on GOS

Load the local-only LCM URL before launching DimOS. Its explicit 16 MiB receive
buffer is needed for fragmented multi-megabyte clouds. The native ROS bridge
also needs the vendor Foxy library path and Fast DDS profile in its inherited
environment:

```bash
source /opt/ros/foxy/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
set -a
source dimos/robot/deeprobotics/m20/deploy/dimos-m20.env
set +a
dimos --rerun-open none --rerun-host 0.0.0.0 \
  run deeprobotics-m20-kronknav
```

After a daemon launch, use `dimos status` and `dimos log` to confirm that both
`M20ROSBridge` and `M20PointLio` remained alive. A successful viewer connection
alone proves only the visualization process, not the sensor bridge.

Attach a viewer over direct robot Ethernet or the onboard AP without moving the
DimOS graph off GOS:

```bash
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

The default blueprint never creates a `/NAV_CMD` publisher. Use the separate
`deeprobotics-m20-kronknav-control` blueprint only when motion ownership is
intentional; it still starts disarmed. Its single `M20Connection.standup()` RPC
switches `basic_server` to navigation usage mode (`Type=1101`, `Command=5`,
`Mode=1`), transitions to RL Control, selects gait `0x3002`, and arms commands.

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
