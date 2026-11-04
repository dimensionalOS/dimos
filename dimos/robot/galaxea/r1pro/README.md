# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

`galaxea-dimos` is Galaxea's own ROS 2 driver with local bug fixes applied; it
is not part of dimos. `R1ProConnection` boots it, when it is not running, with
[`vendor_setup.sh`](/dimos/robot/galaxea/r1pro/vendor_setup.sh): the standalone stack
(no `moca_adapter`, chassis gatekeeper on-robot) without the vendor head camera and
Livox drivers. HDAS brings up CAN FD itself, so `~/canfd.sh` is not needed. Pick a
vendor profile instead with `--r1proconnection.vendor-profile ATCStandard/R1PROVRTeleop`.

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
dimos run r1pro-coordinator     # connection + coordinator + Point-LIO + viewer
dimos run r1pro-teleop          # + chassis teleop from the viewer
dimos run r1pro-nav             # + click-to-drive nav (costmap + A*)
dimos run r1pro-manipulation    # + dual-arm planning (experimental)
dimos run r1pro-planar-preview   # planar-base planning preview with fake hardware
```

## Point-LIO

Every blueprint built on `r1pro-coordinator` places `base_link` by Point-LIO on
the chassis Mid-360 instead of wheel odometry; `chassis_odom` keeps its name.
`r1pro-manipulation` builds on `r1pro_control` alone and keeps wheel odometry.
The Mid-360 driver and Point-LIO are native binaries built on first run, so
`cargo` must be on the path.

**Transport.** Run with `--g.transport lcm`. The vendor's `realsense2_camera`
holds LCM's default port, so set
`LCM_DEFAULT_URL=udpm://239.255.76.67:7767?ttl=0`.

**Other vendor profiles.** With a `--r1proconnection.vendor-profile` that starts
the vendor head camera and Livox drivers, the head cameras retry until the device
is free and our Mid-360 driver competes with the vendor's for the sensor.

The lidar and host addresses are `R1PRO_CHASSIS_LIDAR_IP` /
`R1PRO_CHASSIS_LIDAR_HOST_IP` in `config.py` (`--mid360.lidar_ip` /
`--mid360.host_ip` override them).


## Manipulation grippers

`r1pro-manipulation` uses standard coordinator hardware components and gripper
control tasks for the left and right jaws. Arm/torso planning remains 18 DOF;
the two gripper joints are controlled separately from arm trajectories.

Use Python 3.10 for ROS Humble. With ROS and the vendor workspace sourced in the
same ROS domain as the existing vendor stack, and that stack
already running, start the manipulation blueprint:

```bash
# Initializes real control interfaces; clear the workspace and have physical E-stop ready.
# Never run this alongside another R1 control chain.
dimos run r1pro-manipulation --viewer none \
  --r1proconnection.boot-vendor-stack false \
  --disable head-left-camera --disable head-right-camera \
  --r1proconnection.enable-wrist-color false \
  --r1proconnection.enable-wrist-depth false \
  --manipulationmodule.default-speed-scale 0.05
```

Use `dimos shell` from another terminal with the same environment and runtime
state directory. Read capabilities and measured openings before commanding:

```python
m = app.ManipulationModule
m.list_planning_groups()
m.get_state().groups["left_arm"].gripper_position
m.get_state().groups["right_arm"].gripper_position
```

With empty jaws and hands clear, execute each motion command separately and
observe the selected jaw before continuing:

```python
m.set_gripper_position(0.9, planning_group="left_arm")
m.get_state().groups["left_arm"].gripper_position
# After confirming arrival and that the other jaw stayed still:
m.set_gripper_position(1.0, planning_group="left_arm")
```

Opening is normalized: 0 closed, 1 open. Use `right_arm` for the other jaw;
`torso` and an omitted group are rejected. Command acceptance is not proof of
arrival or successful grasping. Native vendor position targets are 0–100 mm;
measured openings are not clipped, including small excursions outside travel.

The adapters do not publish jaw targets on connect/activate/disconnect or while
the tasks are idle. Missing/stale feedback, required arm/torso fault reports,
reported faults and conflicting target endpoints inhibit output. Invalidated
feedback must be observed by the control read tick before old intent can be
used again; the task drops that intent and requires a new command. Unscoped
broadcast gripper commands are disabled for this dual-jaw blueprint.

Dedicated gripper fault status is unknown until the vendor supplies it. There
is no verified grip-force measurement or vendor cancel operation. Software stop
prevents subsequent commands but cannot cancel a target already accepted by the
vendor controller; use the physical E-stop for unexpected motion. Do not treat
this blueprint as a completed perception or automatic pick-and-place stack.
