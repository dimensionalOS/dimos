# Test YAM, OpenArm, and Metal connections

These robots use `ConnectionModule` directly. Each connection owns one native
`can_motor_control.Robot`, its CAN buses, gripper calibration, state reader,
and MIT commands. Position-control stacks retire their former whole-body
adapters and registry entries. Seeed B601-DM and OpenYAM hand-guided collection
use the same connection path: the generic Damiao whole-body adapter and its
OpenYAM/Seeed compatibility factories are gone, so every Damiao robot now has
one owner and one gain declaration.
Damiao connections are hosted directly: the coordinator in the same worker
reads the connection at the start of each control tick and writes the
arbitrated result in that same tick, with no command or reading message in
between. `joint_state` remains the measured-telemetry port for other
consumers.

## Run software tests first

From the checkout of [PR #4960](https://github.com/dimensionalOS/dimos/pull/4960), run:

```bash
gh pr checkout 4960
uv run pytest dimos/robot/manipulators/damiao/test_damiao_connection.py \
    dimos/robot/manipulators/damiao/test_damiao_native.py \
    dimos/robot/manipulators/damiao/test_damiao_startup.py \
    dimos/robot/manipulators/damiao/test_damiao_model.py
```

These tests use authored URDF fixtures and fake native APIs. They cover partial
commands, exact MIT gains and ordering, calibrated grippers, limits, gravity,
activation rollback, deadman, faults, and thread/resource cleanup. They do not
open physical CAN interfaces.

Run the migrated robot compositions as well:

```bash
uv run pytest dimos/robot/manipulators/openyam \
    dimos/robot/manipulators/dual_openyam \
    dimos/robot/manipulators/openarm
```

The default test filter excludes hardware-runner and native-recorder tests.
The local Pink/model checks use geometry assets and do not actuate hardware.
They check namespaced planner/IK models, independent dual-YAM grips, OpenArm's
both-grip policy, and the seven-coordinate Quest collection profile.

## Start a mock robot

Select the independent mock connection explicitly with `--simulation`:

```bash
uv run dimos --simulation run makermods-metal-connection-control --daemon
# Or: uv run dimos --simulation run openyam-connection-control --daemon
# Or: uv run dimos --simulation run openarm-connection-control --daemon
uv run dimos status
uv run dimos shell
```

The following Python snippets require the live `dimos shell` session.
Inspect the connection and coordinator there:

```python skip
modules()
app.MockJointConnection.status()
app.ControlCoordinator.get_joint_positions()
```

Every simulated arm uses `app.MockJointConnection`; no native connection is started.
The connection status must report `connected=True`, with no error or rejection.
The coordinator must return 7 coordinates for Metal/YAM, or 16 for OpenArm.
Mock mode echoes commanded positions. It does not simulate dynamics or contact.

A connection reports ready only after calibration, feedback validation, and
telemetry-publisher startup succeed. Its ready log names the actual connection
class; `arm` is the deployed joint namespace. The coordinator waits for all initial
connection readings before publishing aggregate state, so calibration does not send
empty state frames to the planner.

For the mock Metal, test its gripper:

```python skip
app.ControlCoordinator.task_invoke("arm_gripper", "set_normalized", {"values": [0.6]})
app.ControlCoordinator.get_joint_positions()
```

The call returns `True`. On a subsequent state read, `arm/joint7` is near
-0.03 m (Metal's finger is closed at 0 and fully open at -0.05), while the six
arm positions are unchanged. In the simple OpenArm control
blueprint, use `arm_left_gripper` or `arm_right_gripper` for the selected jaw.

For a small arm trajectory in the mock Metal, use:

```python skip
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

q = app.ControlCoordinator.get_joint_positions()["arm/joint2"]
app.ControlCoordinator.execute_trajectory(JointTrajectory(
    joint_names=["arm/joint2"],
    points=[
        TrajectoryPoint(time_from_start=0.0, positions=[q]),
        TrajectoryPoint(time_from_start=0.5, positions=[q - 0.02]),
    ],
))
app.ControlCoordinator.get_joint_positions()
```

The result is `ACCEPTED`; a subsequent read shows joint2 moved by -0.02 rad,
while the other arm coordinates and gripper retain their targets.

To exercise the actual coordinator/deadman path:

```python skip
app.ControlCoordinator.set_estop(True)
app.MockJointConnection.status()
```

The coordinator stops writing commands to the hosted connection. After the
configured deadman interval, status reports `deadman_fired=True` and the mock
holds its pose.
`app.ControlCoordinator.set_estop(False)` resumes the command stream.

Exit the shell, then stop the mock:

```bash
uv run dimos stop
```

## Select physical CAN interfaces

Physical startup enables motors and runs the upstream gripper endpoint
calibration, so the grippers move. Prepare the supported robot and clear its
jaws before starting. Use the declared `can-motor-control==0.0.12` dependency;
Metal requires its custom `MotorProfile` API.

After the mock checks, choose existing, configured interfaces explicitly:

```bash
uv run dimos run makermods-metal-connection-control --can-port can0
uv run dimos run openyam-connection-control --can-port can0
uv run dimos run openarm-connection-control --left-can-port can1 --right-can-port can0
uv run dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1
uv run dimos run seeed-b601-dm-planner-coordinator --can-port can0
```

Run only the command for your robot. OpenArm and dual YAM require both selectors,
and they must differ. On macOS, selectors are gs_usb serial numbers. A `/dev/...`
selector on Linux or macOS uses the native Damiao `DmSerialBus` USB-CAN bridge.
The modules
do not configure interfaces or request sudo. They add no startup home motion.
Native startup requires explicit CAN selectors. `--simulation` composes a separate
`MockJointConnection`; it never opens the native connection.

Check live `status()` and joint readings before issuing motion. Confirm left/right
mapping and measured positions. Use a supported small motion within the actual
joint limits, then verify the other arm and uncommanded joints hold. Check each
gripper independently. Test coordinator estop, deadman hold, and graceful shutdown
with the arm supported. A physical emergency stop remains the operator's hardware
control; the coordinator's software estop waits for the connection deadman.

The software tests do not establish physical calibration, full-gain stability,
firmware scaling, or hardware acceptance. No physical robot was actuated for this PR.

## Use the retained planner and teleop entries

Existing entry names now compose the direct connection too:

- YAM: `coordinator-openyam`, `openyam-planner-coordinator`,
  `keyboard-teleop-openyam`, `keyboard-teleop-openyam-planner`,
  and `teleop-webxr-openyam`.
- Dual YAM: `coordinator-dual-openyam`, `dual-openyam-planner-coordinator`,
  and `teleop-webxr-dual-openyam`.
- OpenArm: `coordinator-openarm`, `openarm-planner-coordinator`,
  and `teleop-webxr-openarm`.

Deployed joint names use source `arm`. Metal has `arm/joint1..6`, YAM has
`arm/yam_joint1..6`, dual YAM has `arm/left_joint1..6` and `arm/right_joint1..6`,
and OpenArm has `arm/openarm_left_joint1..7` and the corresponding right names.
Each gripper is its URDF finger joint, in the URDF's unit: YAM `arm/joint7`,
Metal `arm/joint7` (dual Metal `arm/left_joint7` and `arm/right_joint7`), B601-DM
`arm/finger_left`, and OpenArm `arm/openarm_left_finger_joint1` and
`arm/openarm_right_finger_joint1`. The other finger copies it. Each gripper
task's `open_at` says which end of the joint's range is open. Dual YAM's URDF
has its finger joints fixed, so it keeps `arm/left_gripper` and
`arm/right_gripper`, from 0 closed to 1 open.

Planner groups, task joints and the coordinator's model come from each robot's
`config.py`. Existing recordings using the old names retain their original
schema; newly collected recordings use the deployed names.

`ManipulationModuleConfig.gripper_tasks` maps planning-group IDs to complete
coordinator task names; grippers are not configured on `RobotModelConfig`.
Single YAM maps `manipulator` to `arm_gripper`, and Metal maps `arm` to
`metal_gripper`. Dual YAM maps `left_manipulator` / `right_manipulator` to
`left_arm_gripper` / `right_arm_gripper`; dual Metal and OpenArm teleop use
`left_arm` / `right_arm` with those same task names. This mapping drives both
normalized gripper commands and feedback; an unbound group has no gripper API.

OpenYAM policy rollout and Quest collection use direct connections and namespaced
seven-coordinate state/action vectors. `openyam-teach-collection` uses
`OpenYamTeachConnection`: zero stiffness, teaching damping, gravity
compensation and passive gripper commands on every tick, declared to the
coordinator as zero-stiffness joints. Its seven-coordinate recording schema uses
the deployed `arm/...` names. The direct stacks take
`--openyamteachconnection.can-port` (or `--can-port`).

Hand guiding refuses to start unless the robot declares zero stiffness. For a
connection, `connect_damiao` derives that declaration from the connection's own
gains: a whole arm group counts when every `kp` is zero, and a gripper counts
when it is listed in `passive_grippers`. The coordinator carries it as
`zero_stiffness_joints` and hands it to the task factory; a position-holding
connection such as `OpenYamConnection` declares nothing and is rejected.
