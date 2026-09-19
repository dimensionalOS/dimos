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

## Point-LIO on the chassis lidar

`r1pro-pointlio` replaces the wheel odometry with lidar-inertial odometry from
the chassis Mid-360. The connection's own `odom` (integrated from the speed the
chassis was *commanded*, which smears a map around a spin) is switched off, and
`base_link` hangs off Point-LIO through the lidar mount
(`dimos/robot/galaxea/r1pro/lio.py`). Downstream consumers keep reading
`chassis_odom`; it just comes from the lidar now.

**What keeps running.** All of the vendor's ROS nodes stay up: the chassis and
arm controllers, the cameras, the IMUs. dimos does not replace any of them.
Point-LIO opens its *own* connection to the Mid-360 beside the vendor's
`livox_ros_driver2`.

**What stops.** A Livox streams to the host that last asked it to. As soon as
Point-LIO starts, the vendor driver receives nothing: `/hdas/lidar_chassis_left`
keeps its publisher and goes silent, and it does not recover when Point-LIO
exits, because `livox_ros_driver2` never asks again. Nothing in the vendor's
navigation runs while dimos owns the sensor, and the sensor stays with dimos
until the vendor driver is restarted.

**Giving the sensor back.** The vendor driver runs in the tmux session `hdas`,
started by
`~/galaxea-dimos/install/startup_config/share/startup_config/script/boot/modules/hdas/start_livox_lidar.sh`.
Kill it by PID (never `pkill -f` over ssh -- the pattern matches your own ssh
command) and re-run that script from its own directory, in that session:

```bash
pgrep -a livox_ros_driver2      # note the pid
kill <pid>
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script/boot/modules/hdas
tmux send-keys -t hdas './start_livox_lidar.sh' Enter
```

**Network.** Point-LIO needs the lidar's IP and the host NIC it pushes to. On
an R1 they are read from the vendor's own
`~/galaxea-dimos/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
(override the location with `DIMOS_R1_MID360_CONFIG`), or set
`DIMOS_POINTLIO_LIDAR_IP` / `DIMOS_POINTLIO_HOST_IP` to skip the file. The
blueprint refuses to start, naming the file and the variables, when neither
can answer, or when no local interface sits on the lidar's subnet.

**Transport.** Run with `--g.transport lcm`. Over zenoh the C++ estimator's
cloud is dropped by the Rust voxel map (`Received Data for unknown expr_id`)
with no warning naming the stream. The vendor's `realsense2_camera` holds LCM's
default port, so use `LCM_DEFAULT_URL=udpm://239.255.76.67:7767?ttl=0`.

**Build.** The estimator is a native binary built on first run with
`nix build -L .#pointlio_native` in `dimos/hardware/sensors/lidar/pointlio/cpp`;
on an Orin that is about twenty minutes, once.
