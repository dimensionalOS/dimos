# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

`galaxea-dimos` is Galaxea's own ROS 2 driver with local bug fixes applied; it
is not part of dimos. `canfd.sh` ships with the robot and brings up the CAN FD
interfaces the driver needs. Boot the driver with the standalone stack, which
bypasses the stock `moca_adapter` and runs the chassis gatekeeper on-robot:

```bash
bash ~/canfd.sh
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script
./robot_startup.sh kill
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
```

## Environment

- `ROS_DOMAIN_ID=1` (new-gen V2.3.0), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- ROS 2 Humble ships `rclpy` built for CPython 3.10, so the venv must use the
  robot's system interpreter. `.python-version` says 3.12, so pass `--python`
  explicitly rather than relying on direnv:

  ```bash
  sudo apt-get install -y libturbojpeg   # pyturbojpeg needs the native lib
  uv sync --python /usr/bin/python3.10 --python-preference only-system \
          --no-default-groups --extra base --extra manipulation --extra cpu
  uv pip install python-socketio         # runtime dep of the viewer, only declared in the lint group
  ```

  `--all-extras` does not work on the robot's arm64 board: the `scene` extra
  needs `usd-core` (x86_64/macOS wheels only) and `mapping` needs
  `gtsam-extended` (arm64 Linux wheels start at cp311). `mapping` is also
  reachable through `unitree` and `all`, so excluding it by name is not enough.
- The planning model fetches the vendor URDF from the pinned upstream repo.

## Blueprints

```bash
dimos run r1pro-coordinator      # the base: connection, head cameras, Mid-360 + Point-LIO, viewer (WASD drives)
dimos run r1pro-nav              # base + head depth + 3D ray-traced map + MLS planner (click a goal)
dimos run r1pro-manipulation     # dual-arm planning (experimental)
dimos run r1pro-planar-preview   # planar-base planning preview on mock hardware
```

`r1pro-coordinator` is the one standard R1 blueprint: `R1ProConnection` (chassis
`cmd_vel`, joints, wrist cameras, wheel odometry; it boots the vendor stack if it
is not already running), the head cameras (V4L2, hardware-synced), and our
Mid-360 driver into Point-LIO. `r1pro-nav` builds on the same pieces.

## Point-LIO and head depth

`base_link` is placed by Point-LIO on the chassis Mid-360, not by wheel odometry.
The connection still publishes wheel odometry on `odometry`, just not on tf.
`r1pro-manipulation` builds on `r1pro_control` alone and keeps wheel odometry on tf.
`r1pro-nav` adds a dense cloud from the left head camera: Depth Anything,
calibrated per pixel to the last few seconds of Point-LIO scans (`Depth2DepthCloud`).
The Mid-360 driver, Point-LIO and the head depth are native binaries built on
first run, so `cargo` (and on an Orin, `nvcc` for CUDA) must be on the path.

**Transport.** The blueprints run on zenoh. On LCM, the vendor's
`realsense2_camera` holds LCM's default port, so set
`LCM_DEFAULT_URL=udpm://239.255.76.67:7767?ttl=0`.

**The head cameras.** The vendor's `signal_camera` node holds both head eyes; the
connection stops it (SIGINT) before anything starts, so our V4L2 cameras can open them.

**The lidar.** Our Mid-360 driver takes the sensor from the vendor's
`livox_ros_driver2` (a Livox streams to whoever asked last), and
`/hdas/lidar_chassis_left` goes silent until the vendor driver is restarted. To
give it back, restart it in its tmux session `hdas` (kill by PID; `pkill -f`
over ssh matches your own ssh command):

```bash
pgrep -a livox_ros_driver2      # note the pid
kill <pid>
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script/boot/modules/hdas
tmux send-keys -t hdas './start_livox_lidar.sh' Enter
```

The lidar and host addresses are `R1PRO_CHASSIS_LIDAR_IP` /
`R1PRO_CHASSIS_LIDAR_HOST_IP` in `config.py` (`--mid360.lidar_ip` /
`--mid360.host_ip` override them).
