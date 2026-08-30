# M20 deployment

The X20/M20 integration has one runnable blueprint:
`deeprobotics-m20-kronknav-control`. It starts command output disarmed.

The sensor path stays on GOS and never sends raw lidar through LCM:

```text
M20 lidar + IMU -- Fast DDS --> M20PointLio -- LCM --> mapping/navigation
M20 state/control -- Fast DDS <--> M20ROSBridge -- LCM <--> M20Connection
```

`/LIDAR/POINTS` is the vendor-merged `base_link` cloud. Its point records are
little-endian `float32 x/y/z/intensity`, `uint16 ring`, and `float64 timestamp`
at byte offsets 0, 4, 8, 12, 16, and 18 respectively.

## Persistent robot setup

On NOS (`10.21.31.106`), install the lidar relay supervisor and reserve
`/NAV_CMD` for DimOS:

```bash
sudo install -D -o root -g root -m 0755 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-multicast-relay-supervisor \
  /usr/local/libexec/dimos-m20-multicast-relay-supervisor
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/multicast-relay.service.d/10-dimos-network-readiness.conf \
  /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/planner.service.d/10-dimos-command-ownership.conf \
  /etc/systemd/system/planner.service.d/10-dimos-command-ownership.conf
sudo systemctl unmask planner.service
sudo systemctl daemon-reload
sudo systemctl stop planner.service
sudo systemctl enable --now multicast-relay.service
```

On GOS (`10.21.31.104`), install the Fast DDS shared-memory permission hooks:

```bash
sudo install -D -o root -g root -m 0755 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-rsdriver-shm-permissions \
  /usr/local/libexec/dimos-m20-rsdriver-shm-permissions
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/rsdriver.service.d/10-dimos-shm-permissions.conf \
  /etc/systemd/system/rsdriver.service.d/10-dimos-shm-permissions.conf
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-fastdds-permissions.path \
  /etc/systemd/system/dimos-m20-fastdds-permissions.path
sudo install -D -o root -g root -m 0644 \
  dimos/robot/deeprobotics/m20/deploy/dimos-m20-fastdds-permissions.service \
  /etc/systemd/system/dimos-m20-fastdds-permissions.service
sudo systemctl daemon-reload
sudo systemctl enable --now rsdriver.service dimos-m20-fastdds-permissions.path
```

The `rsdriver` hook fixes existing Fast DDS files after driver start. The path
unit handles files created later. Keep both.

## Run on GOS

The native module declarations carry their own ROS library path, RMW selection,
and Fast DDS profile. No ROS setup script or environment file is required:

```bash
cd /var/opt/robot/data/dimos-m20-kronknav
source .venv/bin/activate
LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=0&recv_buf_size=16777216' \
  dimos --rerun-open none --rerun-host 0.0.0.0 \
  run deeprobotics-m20-kronknav-control
```

Attach and enable control deliberately:

```bash
dimos --transport lcm shell
```

```python
app.M20Connection.standup()
```

`standup()` selects the navigation usage mode, enters RL Control, chooses gait
`0x3002`, and arms bounded velocity output.

## Minimal checks

```bash
# NOS
systemctl is-active multicast-relay.service
systemctl is-active planner.service  # must be inactive

# GOS
systemctl is-active rsdriver.service
systemctl is-active dimos-m20-fastdds-permissions.path
dimos status
dimos log
```

To restore the vendor planner instead of DimOS control, create
`/etc/dimos/enable-vendor-m20-planner` on NOS and start `planner.service`. Never
run both command owners at once.
