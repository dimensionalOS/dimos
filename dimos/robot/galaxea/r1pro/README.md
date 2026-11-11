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
dimos run r1pro-vr-teleop        # Quest VR: both arms + torso, safe fake backend
```

## Quest VR teleop

`r1pro-vr-teleop` is the operator stack for a Meta Quest browser: both
controllers drive both arms and the top torso pitch/yaw joints through one
`teleop_ik` task. The lower two pitch joints are controlled independently
by the joystick lift task. Top pitch has
5× posture cost and a 0.15 rad/s velocity cap; arm and yaw costs are unchanged. Each trigger drives its own jaw through
the existing gripper task with a separate input binding; the jaws never enter the IK model or a
trajectory. Left thumbstick controls forward/lateral velocity and right
thumbstick x controls yaw, reusing Go2's thumbstick mapping and deadzone.
`ArmTeleopModule` publishes original `left_joy` / `right_joy` packets, not
robot movement commands. `JoystickTeleopMapper` applies the axis mapping,
deadzone, scales and input expiry, then publishes body-frame `Twist` and
`Float32` lift speed (m/s). The existing `twist_velocity` task maps the Twist
onto the named base joints; the connection still owns vendor conversion.
Hold right-stick click to enable body motion: left X/Y translate, right X
turns and right Y lifts/lowers. Default caps are 0.3 m/s, 0.5 rad/s and
0.03 m/s. `make_r1pro_vr_teleop(joystick={...})` overrides mapper config.
The R1Pro height task owns only torso joints 1/2; IK owns torso 3/4 and arms.
The lift follows the pinned URDF's two-link geometry, retaining fore/aft
position, with a 0.45 m minimum hip-relative height and 0.15 rad/s joint cap.
Full extension is the upper boundary, so raising an already upright waist does
nothing. Joystick lift and `full_torso=True` are rejected as overlapping owners.
Both tasks read measured feedback independently; neither feeds the other.
Raw Joy is emitted only on receipt: old input is not periodically re-stamped.
Stale/disabled input zeros body intent; the lift task independently times out
and clears its target on E-stop or preemption. Software stop is not vendor cancel.
Operator hold is explicitly scoped to the upper-body joints;
it does not claim jaw positions or chassis velocity interfaces. E-stop stops command
publication and invalidates velocity intent. After clearing it, fresh velocity
input may drive the chassis; the arm position hold still awaits acknowledgement.

It starts on the **fake backend by default** — `FakeR1ProConnection`, an
in-memory closed loop on the same contract (`position_command`, `velocity_command`,
`joint_state`, the same 18-joint order, limits and metres for jaw openings).
It pulls in no ROS 2, vendor stack, camera, lidar, CAN or robot; the Quest
input visibly moves the simulated joints and jaws through the coordinator's
`joint_state` feedback and the viewer. Selecting the real robot is explicit
and only assembles the stack; starting it against hardware is a separate,
deliberate operation:

```bash
dimos run r1pro-vr-teleop                       # fake, safe anywhere
# real (assembly only, does NOT start hardware):
#   python -c "from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop \
#       import make_r1pro_vr_teleop; make_r1pro_vr_teleop(backend='real')"
```

Both backends share the Quest input module, the task set, the IK model and
the coordinator; only the connection module differs. Opening
`https://<host>:8443/teleop` in the headset connects the browser; the actual
headset session is verified by a human, not by CI.

In fake mode, `FakeR1ProConnection.base_state()` reads the in-memory integrated
chassis x/y/yaw and vx/vy/wz over RPC (`dimos shell`); joint feedback is also
available on `coordinator_joint_state`. No physical base driver is started.

**Deadman and session teardown.** Middle-finger grips engage both hands and
capture the controller references. Releasing either grip, controller input
going stale, or a coordinator E-STOP ends the session and clears the
references, so the next engagement starts from a fresh capture and no old
intent replays. The connection rejects jaw writes while feedback is invalid,
and feedback recovery alone does not reauthorize an interrupted target. A
different target is required; the unchanged generic task does not provide an
intent-generation signal for reissuing the same opening.

**Stop is not a cancel.** As with the rest of the R1 Pro stack, stopping the
coordinator, the deadman, or an E-STOP only stops *us* from sending new
targets. None of it is a vendor cancel or a physical E-stop; a command the
vendor controller already accepted can keep executing. Use the robot's own
E-stop hardware for a real emergency stop.

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
