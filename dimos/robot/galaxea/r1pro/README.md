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
dimos's own Mid-360 driver opens a *second* connection to the sensor beside
the vendor's `livox_ros_driver2`, and feeds the Rust Point-LIO: the estimator
deskews a sweep by a per-point time offset the vendor's cloud does not carry,
and it needs the IMU inside the sensor, not the chassis IMU.

**What stops.** A Livox streams to the host that last asked it to. As soon as
Point-LIO starts, the vendor driver receives nothing: `/hdas/lidar_chassis_left`
keeps its publisher and goes silent, and it does not recover when the driver
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

**Network.** The driver needs the lidar's IP and the host NIC it pushes to:
`R1PRO_CHASSIS_LIDAR_IP` / `R1PRO_CHASSIS_LIDAR_HOST_IP` in
`dimos/robot/galaxea/r1pro/config.py`, the same addresses as the vendor's
`MID360_config.json`; `--mid360.lidar_ip` / `--mid360.host_ip` override them.
The blueprint refuses to start when no local interface sits on the lidar's
subnet. The vendor driver holds the same host ports (56101, 56201, ...); both
bind them with `SO_REUSEADDR`, and the device streams to whichever asked last.

**Time.** A Mid-360 with no time source stamps its packets with its own
uptime. The estimator carries its output onto the host's clock from packet
arrival times (the minimum of host-minus-device over a 30 s window), so its
odometry, tf and cloud resolve against the cameras' stamps.

**Transport.** Run with `--g.transport lcm`. Over zenoh the C++ estimator's
cloud was dropped by the Rust voxel map (`Received Data for unknown expr_id`)
with no warning naming the stream; the Rust estimator has not been tried
there. The vendor's `realsense2_camera` holds LCM's default port, so use
`LCM_DEFAULT_URL=udpm://239.255.76.67:7767?ttl=0`.

**Build.** The driver and the estimator are native binaries
(`target/release/mid360_native`, `target/release/pointlio_native`) built on
first run with `cargo build --release`. On an Orin `cargo` is not on the
path: build once inside `nix develop path:nix/rust`.

## Recording stereo calibration data

The head's two eyes are calibrated as a pair from a recording, not on the
robot: `calibrate_stereo.py` fits the baseline and how the right eye is aimed
relative to the left (`right_{roll,pitch,yaw}_rad`) against the chassis lidar
and writes `~/.dimos/r1pro/calibration.json`
(`dimos/robot/galaxea/r1pro/stereo_calibration.py` is the schema and loader;
`DIMOS_R1_STEREO_CALIBRATION` points it elsewhere, and with no file the
committed rig numbers are used). The recording it consumes comes from this,
on the robot:

```bash
dimos run r1pro-calibration-recorder \
    --record sqlite --record-engine rust \
    --record-topics head_left_color,head_right_color,head_left_info,head_right_info,pointlio_lidar,pointlio_odometry,tf
```

That is both eyes and both infos at 30 fps, Point-LIO's deskewed cloud and
pose, and tf -- with the wrist cameras off and nothing that plans, so the
Orin's CPU goes to the frames. The engine must be `rust` (the robot's sqlite
cannot write the python recorder's JSONB), and the estimator's stamps have to
be on the host's clock, which the Rust Point-LIO does by design (see *Time*
above).

**Driving.** Slowly, for 60-120 s, with the floor and at least one wall in the
head's view at 1-6 m the whole time. Put in a couple of gentle turns -- a turn
is what separates a yaw error from a baseline error; a straight line cannot --
and keep people from walking through the frame, since the fit assumes the
scene held still between one eye's exposure and the other's.

**Check it before fitting.** A recorder that fell behind leaves gaps that look
like motion to the fit, and the file does not say it dropped anything:

```bash
python -m dimos.robot.galaxea.r1pro.recording_rates <recording.db> \
    --require head_left_color=28 --require head_right_color=28
```

prints count, first/last stamp, mean Hz and the min/median/max Hz over
10-second windows (`--window-s`) per stream, and exits 1 when a `--require`
is not met. The window minimum is where a stall shows; the cameras arrive at
about 28 Hz, so a mean under that means frames were lost.
