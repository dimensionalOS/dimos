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

## Spatial body-follow teleop

`dimos run r1pro-spatial` keeps the DimOS Desktop spatial workspace, head and
wrist RGB-D views, lidar map, and its visibility toggle. Its hardware control
path uses the connection/coordinator contract from the VR teleop stack.

In XR, choose **Calibrate body**, release both grips, and extend both arms
forward at shoulder height with controllers pointing forward during the
three-second countdown. This calibrates arm reach, wrist alignment, and the
estimated body heading. Then hold both middle-finger grips to enable the arms;
triggers close the corresponding jaws. Releasing either grip holds both arms.
Re-engagement uses the same absolute body mapping: it never captures a new
robot/controller displacement offset. Full human extension requests 98% of
the robot's neutral shoulder-to-gripper reach to avoid a straight-arm singularity.
The robot approaches that target at bounded speed rather than snapping to it.

Tracked shoulders are used when the browser exposes body tracking. Otherwise,
the interface reports estimated shoulders from headset position and a calibrated
heading; looking sideways does not turn arm targets. This fallback does not
measure torso rotation or elbows. Recalibrate after changing your body heading.
Session exit, reference-space reset, or a change of tracking source invalidates
calibration. Hold/Resume and ordinary grip release preserve it.

Both arms and top waist pitch/yaw share one Pink solve. The solve's root is
`torso_link2`, with targets transformed using measured TF, so lowering the robot
does not silently freeze the lower linkage at zero in IK. The right stick's
vertical axis changes height through a coordinator task; its pitch compensation
yields to active arm IK. Grippers use the connection's metre-valued joints and
separate tasks. Base velocity also passes through coordinator arbitration.

Offline checks cover frame mapping at nonzero torso height, bounded reaching,
release/re-engagement, timeout, and E-stop. They do not establish on-robot tracking
quality or collision avoidance: this differential IK path has no self-collision
constraints and does not mirror measured human elbow poses. `r1pro-sim2` now uses the same absolute body mapping and bimanual solve.
CC's `r1pro-vr-teleop` retains its original input mode.

For tracked hands (including AVP WebXR), calibrate with palms down and hands
open, then choose **Follow hands**. Open hands open the jaws; fists close them.
Tracking loss or opening Controls holds both arms until Follow hands is selected
again. Raise your left palm and select Controls with the other hand to summon
one fixed-size menu. Its View tab offers workspace scaling, map visibility, and
**Robot eyes** in sim2: a life-size view from the head camera using observed
RGB-D surfaces. It is not a panoramic rendered scene.

## Quest VR teleop

`r1pro-vr-teleop` is the operator stack for a Meta Quest browser: both
controllers drive the whole 18-DOF upper body (torso 4 + arm 7 + arm 7)
through one `teleop_ik` task, so the torso participates in the same IK solve
as the arms rather than being pinned. Each trigger drives its own jaw through
a separate feedback-gated gripper task; the jaws never enter the IK model or a
trajectory. Left thumbstick controls forward/lateral velocity and right
thumbstick x controls yaw, reusing Go2's thumbstick mapping and deadzone.
`ArmThumbstickTeleopModule.twist_command` feeds the coordinator's `vel_chassis`
velocity task, then `r1pro_cmd_vel` reaches the selected connection. The same
path applies to fake and real, with limits ±1/±1 m/s and ±2 rad/s. Centred,
missing or stale sticks command zero; arm poses and triggers never drive the
chassis. Thumbsticks do not require the arm grips. No keyboard/nav input is wired.
Operator hold arbitrates arm positions and zero base speeds, excluding the
command-only jaws. E-stop clear leaves the hold latched until acknowledged.

It starts on the **fake backend by default** — `FakeR1ProConnection`, an
in-memory closed loop on the same contract (`position_command`, `cmd_vel`,
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
intent replays. A jaw whose feedback drops out loses its emitted target; a
target latched *during* the outage has never been sent and goes out on the
first tick whose feedback covers the jaw again. That replay-on-recovery is
the framework's current gripper semantic, exercised by the tests and still
unreviewed as queueing behaviour.

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
`cmd_vel` carries chassis speeds (clamped to ±1/±1 m/s, ±2 rad/s), and
feedback goes out on `joint_state`. Every command is checked against the
URDF limits first; an out-of-range command is refused whole.

Chassis speeds flow `/cmd_vel` bus → the coordinator's `twist_command` →
the `vel_chassis` task's arbitration, as before, teleop and nav publishers
are unchanged.

**Deadman.** While the coordinator runs it sends a complete command every
tick, so the deadman stays quiet. When commands stop, estop, or the
coordinator going down, `R1ProConnection` halts within
`deadman_timeout_s` (0.1 s): the chassis is told zero speed once and the
arms are held where they were last measured. Stale feedback
(`reading_timeout_s`, 0.2 s) also stops commands until fresh readings
return. This replaces the old adapter path, where an estopped robot simply
kept its last position command with no deadman.

**Grippers.** Each jaw (`r1pro/left_gripper`, `r1pro/right_gripper`, opening
in metres over the vendor's 0–100 mm range) is a command-only contract joint
driven by its own feedback-gated gripper task — never held, never zero-filled,
never sent anything while idle. A target whose feedback goes stale, malformed
or faulted is interrupted: it is not re-sent until the task withdraws it or an
explicit new command replaces it, and the readback is withheld meanwhile.

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
