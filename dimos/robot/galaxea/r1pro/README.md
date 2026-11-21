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
(no `moca_adapter`, chassis gatekeeper on-robot) without the vendor head camera,
wrist camera and Livox drivers. HDAS brings up CAN FD itself, so `~/canfd.sh` is not
needed. Pick a vendor profile instead with
`--r1proconnection.vendor-profile ATCStandard/R1PROVRTeleop`.

**Wrist cameras.** dimos reads both wrist D405s' colour straight off V4L2
(`r1pro_control`), 640x480 JPEG at 30 fps on `wrist_{left,right}_color`. Galaxea's
wrist driver opens colour and depth on both at 1280x720, more than their shared USB
controller carries, so every stream stalls. If a vendor profile or a reboot started
it (`start_realsense_camera_r1pro.sh` in the `hdas` tmux session), stop it with Ctrl-C
in that pane; until then the wrist cameras log and retry.

## Environment

- `ROS_DOMAIN_ID=1` (new-gen V2.3.0), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- ROS 2 Humble ships `rclpy` built for CPython 3.10, so the venv must use the
  robot's system interpreter. `.python-version` says 3.12, so pass `--python`
  explicitly rather than relying on direnv:

  ```bash
  sudo apt-get install -y libturbojpeg0-dev   # pyturbojpeg needs the native lib
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

## Run45 policy simulation (no ROS or hardware)

Use the DimOS `sim`, `cpu`, and `planning` extras. Point `R1PRO_SIM_BUNDLE` at
an unpacked `r1pro-web-demo` checkout (or an identical offline copy), with
`assets/r1pro-holonomic-run45.xml`, `assets/BUNDLE.json`, `assets/meshes/`,
and `models/r1pro_run45_upperdr_8000u.onnx`. Neither meshes nor weights are
vendored. This mirrors DimOS's separation of code from MuJoCo assets, but
uses explicit local paths rather than fetching an unpublished bundle. The
run45 XML hash is checked; `R1PRO_POLICY_ONNX` can override the checkpoint
with another export of the same ABI.

```sh
CUDA_VISIBLE_DEVICES="" R1PRO_SIM_BUNDLE=/home/cc/work/r1pro-web-demo \
  dimos run r1pro-mujoco-policy --g.zenoh-domain r1pro-layer2 \
  --g.zenoh-scout-addr 239.77.38.96:17696
```

Viser renders the loaded plant on loopback port 8080. Open `/teleop` on the
WebXR HTTPS port 8443 from the headset on a trusted network (the stock
WebXR server binds all interfaces and uses a local certificate). With both
controllers tracked, release a grip, then hold both grips to anchor both
wrists. Release either to stop the policy; the per-hand triggers use the
normal gripper tasks. The visible and collision fingers are welded in the
run45 model; the gripper streams model ideal jaw opening in metres, not
manipulation/contact. No camera/LiDAR or physical drivers are launched.

For a programmatic goal, stream a fresh `PoseArray` on `wrist_targets`,
ordered `[left_wrist, right_wrist]`, with Unix acquisition stamp and `odom`
frame (metres, unit xyzw quaternion). `wrist_poses` carries the measured
poses in that frame. Stop publishing to let intent expire, or send an
empty stamped array to withdraw immediately. Programmatic targets take
precedence over grip-relative WebXR while active.

The 50 Hz task uses synchronized upper joint, root odometry and wrist
samples. `state_history` is float32 `[1,10,60]`, oldest first; each row is
`[q(18), dq(18), root-local omega(3), previous raw action(21)]`, with ten
copies of the initial row on engage and zero initial raw action.
`urr_goal` is float32 `[1,60]`: slots 1 and 2 are the left and right wrist,
`[root-local XYZ, first two rotation columns, valid]`; other slots are
zero. `command[0:3]` is body twist and `command[3:21]` is upper position.
There is no command shaping; out-of-contract commands are rejected by
the existing named-interface checks, not clipped. Native MuJoCo force
limits remain active. The physical plant has a free root and uses ten 2 ms
steps per simulation cycle. The Coordinator and simulation have independent
50 Hz clocks joined by transport, not a lockstep real-time guarantee.
