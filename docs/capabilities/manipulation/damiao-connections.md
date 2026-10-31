# Test YAM, OpenArm, and Metal connections

These robots use `ConnectionModule` directly. Each connection owns one native
`can_motor_control.Robot`, its CAN buses, gripper calibration, state reader,
and MIT commands. The former whole-body adapters, registry entries, and
`*_hardware` factories have been removed. Coordinators communicate through
`position_command` and `joint_state` streams.

## Run software tests first

From the checkout of [PR #4960](https://github.com/dimensionalOS/dimos/pull/4960), run:

```bash
gh pr checkout 4960
uv run pytest dimos/robot/manipulators/common/test_damiao_connection.py \
    dimos/robot/manipulators/common/test_damiao_native.py \
    dimos/robot/manipulators/common/test_damiao_model.py
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

Run one command at a time, without CAN selectors:

```bash
uv run dimos run makermods-metal-connection-control --daemon
# Or: uv run dimos run openyam-connection-control --daemon
# Or: uv run dimos run openarm-connection-control --daemon
uv run dimos status
uv run dimos shell
```

In the RPC shell, inspect the connection and coordinator:

```python
modules()
app.MetalConnection.status()
app.ControlCoordinator.get_joint_positions()
```

For YAM or OpenArm, use `app.OpenYamConnection` or `app.OpenArmConnection`.
The connection status must report `connected=True`, with no error or rejection.
The coordinator must return 7 coordinates for Metal/YAM, or 16 for OpenArm.
Mock mode echoes commanded positions. It does not simulate dynamics or contact.

For the mock Metal, test its gripper:

```python
app.ControlCoordinator.task_invoke("arm_gripper", "set_normalized", {"values": [0.6]})
app.ControlCoordinator.get_joint_positions()
```

The call returns `True`. On a subsequent state read, `arm/gripper` is near 0.6,
while the six arm positions are unchanged. In the simple OpenArm control
blueprint, use `arm_left_gripper` or `arm_right_gripper` for the selected jaw.

For a small arm trajectory in the mock Metal, use:

```python
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

```python
app.ControlCoordinator.set_estop(True)
app.MetalConnection.status()
```

The coordinator stops publishing commands. After the configured deadman
interval, status reports `deadman_fired=True` and the mock holds its pose.
`app.ControlCoordinator.set_estop(False)` resumes the command stream.

Exit the shell, then stop the mock:

```bash
uv run dimos stop
```

## Select physical CAN interfaces

Physical startup enables motors and runs the upstream gripper endpoint
calibration, so the grippers move. Prepare the supported robot and clear its
jaws before starting. Use the declared `can-motor-control>=0.0.11` dependency;
Metal requires its custom `MotorProfile` API.

After the mock checks, choose existing, configured interfaces explicitly:

```bash
uv run dimos run makermods-metal-connection-control --can-port can0
uv run dimos run openyam-connection-control --can-port can0
uv run dimos run openarm-connection-control --left-can-port can1 --right-can-port can0
uv run dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1
```

Run only the command for your robot. OpenArm and dual YAM require both selectors,
and they must differ. On macOS, selectors are gs_usb serial numbers. The modules
do not configure interfaces or request sudo. They add no startup home motion.
No selectors, or a nonempty global simulation mode, selects mock operation.

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
  `teleop-webxr-openyam`, and `openyam-quest-collection`.
- Dual YAM: `coordinator-dual-openyam`, `dual-openyam-planner-coordinator`,
  and `teleop-webxr-dual-openyam`.
- OpenArm: `coordinator-openarm`, `openarm-planner-coordinator`,
  and `teleop-webxr-openarm`.

Deployed joint names use source `arm`. Metal has `arm/joint1..6`, YAM has
`arm/yam_joint1..6`, dual YAM has `arm/left_joint1..6` and `arm/right_joint1..6`,
and OpenArm has `arm/openarm_left_joint1..7` and the corresponding right names.
Single grippers are `arm/gripper`; bimanual grippers are `arm/left_gripper` and
`arm/right_gripper`. Openings are calibrated 0 closed to 1 open, not distances.
Logical gripper links do not animate the physical finger meshes.

Planner groups, task joints, and Quest recording names derive from the same
`Connection.models()` schema. Existing recordings using the old bare names retain
their original schema; newly collected Quest recordings use the deployed names.
