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
dimos run r1pro-coordinator     # connection + coordinator + viewer
dimos run r1pro-teleop          # + chassis teleop from the viewer
dimos run r1pro-nav             # + click-to-drive nav (costmap + A*)
dimos run r1pro-manipulation    # + dual-arm planning (experimental)
dimos run r1pro-planar-preview   # planar-base planning preview with fake hardware
dimos run r1pro-pointlio --g.transport lcm   # coordinator + Point-LIO on the chassis lidar
```

## Point-LIO

`r1pro-pointlio` places `base_link` by Point-LIO on the chassis Mid-360 instead
of wheel odometry; `chassis_odom` keeps its name.
The Mid-360 driver and Point-LIO are native binaries built on first run, so
`cargo` must be on the path.

**Transport.** Run with `--g.transport lcm`. The vendor's `realsense2_camera`
holds LCM's default port, so set
`LCM_DEFAULT_URL=udpm://239.255.76.67:7767?ttl=0`.

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
