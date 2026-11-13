# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

See the [teleoperation backend and frontend integration contract](/docs/development/r1pro-teleop-backend.md) for shared speed limits, episode RPCs, and policy-ready recording.

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

## Control path

`R1ProConnection` is a connection module: the coordinator talks to it
directly, with no transport adapters in between. `position_command` carries
all 18 upper-body joints' targets (rad, `r1pro/<joint>`, positions only;
the vendor tracker's speed is a tracking-speed cap, not a velocity command),
`velocity_command` carries a velocity-only `JointState` for body resources
`r1pro/base_forward`, `r1pro/base_lateral`, `r1pro/base_turn` (bounded by
±1/±1 m/s and ±2 rad/s, rejected rather than clipped). These are actuator
rates, not planar generalized coordinates. Upper-body planning and the
geometric `R1PRO_PLANAR_BASE` helper retain their ordinary position semantics.

Chassis speeds flow `/cmd_vel` → coordinator `twist_command` → the registered
`twist_velocity` task `vel_chassis` → ordinary joint arbitration →
`r1pro_velocity_command`. Explicit axes are `linear.x`, `linear.y`, `angular.z`.
The native body command reaches the SDK unchanged, with no yaw read, frame
roundtrip, or odometry prerequisite. The 0.2 s operator-input timeout remains.

**Measured feedback.** The deployed `/motion_control/chassis_speed`
(`geometry_msgs/TwistStamped`) input is unchanged. Every valid measured sample,
including the first, publishes native body rates on `joint_state` in `base_link`,
with no positions. Only valid subsequent intervals integrate local wheel
odometry on the separate `odometry` (`nav_msgs/Odometry`) port. Its pose is in
`odom`, its twist in `base_link`; pose, twist and optional `odom`/TF outputs
retain the actual sample timestamp. This is relative dead reckoning, not map
localization, and never integrates commands.

Source timestamp validation (0.2 s age limit) remains. Invalid samples clear
the integration time anchor; backwards/duplicate timestamps and gaps over 1 s
rebase it without inventing motion or resetting accumulated pose. These
odometry checks do not gate native body writes. Arms, sensors and deadman
lifecycle remain unchanged. Hardware timing/accuracy are not validated here.

**Deadman.** While the coordinator runs it sends a complete command every
tick, so the deadman stays quiet. When commands stop, estop, or the
coordinator going down, `R1ProConnection` halts within
`deadman_timeout_s` (0.1 s): the chassis is told zero speed once and the
arms are held where they were last measured. Stale feedback
(`reading_timeout_s`, 0.2 s) also stops commands until fresh readings
return. This replaces the old adapter path, where an estopped robot simply
kept its last position command with no deadman. After clearing software E-stop,
fresh operator velocity input is required; pre-stop intent is not replayed.

**Grippers.** Each jaw (`r1pro/left_gripper`, `r1pro/right_gripper`) is an
ordinary position joint in metres over the vendor's 0–100 mm range. Each uses
the unchanged gripper task and a separate input binding. Both measured jaw
positions are required for initial coordinator readiness. The R1Pro connection
owns feedback/fault checks, millimetre conversion and duplicate suppression;
an initial hold at the measured opening does not issue a vendor target.
Stale or faulted feedback blocks vendor writes. An interrupted target is not
replayed merely because feedback returns; replace it with a different target.
The generic task's readback remains its last measurement, not a freshness signal.

**Stop is not a cancel.** `stop()`/`halt()` and the deadman only stop *us*
from sending new targets. They are not a vendor cancel and not a physical
E-stop: a jaw target the vendor controller already accepted keeps executing
on the vendor side, and we have no verified way to recall it. The safety
guarantee ends at "we never re-send an interrupted intent" — not at "the
jaw stops moving". For a real emergency stop use the robot's own E-stop
hardware.

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
