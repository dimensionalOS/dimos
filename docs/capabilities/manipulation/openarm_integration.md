# OpenArm integration

dimOS drives the [OpenArm](https://openarm.dev) bimanual platform as one
`OpenArmConnection`. It owns both seven-joint arms and two normalized grippers,
with one CAN bus per arm.

Related:

- Hardware and C++ reference: [enactic/openarm_can](https://github.com/enactic/openarm_can)
- Robot model: [enactic/openarm_description](https://github.com/enactic/openarm_description)
- General arm integration: [adding_a_custom_arm.md](/docs/capabilities/manipulation/adding_a_custom_arm.md)

## Architecture

```text
ControlCoordinator (100 Hz)
  arm_position_command -> OpenArmConnection.position_command
  arm_joint_state      <- OpenArmConnection.joint_state
                            └── can-motor-control Robot
                                  ├── left CAN bus: left_arm + left_gripper
                                  └── right CAN bus: right_arm + right_gripper
```

The connection owns native activation, feedback, position targets, gravity
compensation, command timeout holds, and shutdown. The coordinator uses standard
streams and has no hardware adapters. The `openarm_hardware` factory and
`OpenArmDamiaoAdapter` have been removed.

Deployed blueprints use source `arm`. Their command and feedback vectors contain
`arm/openarm_left_joint1..7`, `arm/openarm_right_joint1..7`,
`arm/left_gripper`, and `arm/right_gripper`, in that order. Gripper opening is
normalized from `0.0` closed to `1.0` open. A standalone connection defaults to
source `openarm`; its messages use that prefix instead.

Per arm, shoulder to wrist, send IDs `0x01..0x07` select two DM8009, two DM4340,
and three DM4310 motors. Feedback IDs are `send | 0x10`. Each gripper is a DM4310
at send ID `0x08`, feedback ID `0x18`.

Gravity compensation and planning use the official bimanual OpenArm v2.0 Xacro,
pinned to commit `1fba2cbc05001f05b4514120b70130b4ac06f409` and loaded lazily
through the robot asset cache. The physical model fixes the fingers and
preflights all fourteen arm coordinates before enabling the motors. Logical
gripper coordinates appear only in the control model.

Planning treats both arms as one robot with `left_arm`, `right_arm`, and
`both_arms` groups. Planning models, groups, tasks, and streams use the same
source-prefixed joint names. Link and grasp-frame names retain their upstream
names, allowing cross-arm collision checks.

## Bring-up

```bash
dimos run openarm-planner-coordinator # in-memory connection
dimos run teleop-webxr-openarm

dimos hardware can setup can0
dimos hardware can setup can1
dimos run openarm-planner-coordinator --left-can-port can1 --right-can-port can0
dimos run teleop-webxr-openarm --left-can-port can1 --right-can-port can0
```

All OpenArm blueprints use in-memory positions unless both CAN selectors are
explicit. `--simulation` also selects in-memory operation. Empty, repeated, or
partial selectors are rejected. Linux assigns interface names by enumeration;
select the interfaces that correspond to the rig's left and right arms.

Starting a physical connection enables the motors and runs the native gripper
calibration. Calibration moves the grippers. The connection accepts commands
only after activation completes. Each arm receives its command sequentially;
there is no atomic transaction across both buses.

## Blueprints

| Blueprint | Contents |
|---|---|
| `coordinator-openarm` | connection and coordinator with a fourteen-joint trajectory task |
| `openarm-planner-coordinator` | bimanual planner, connection, and coordinator |
| `teleop-webxr-openarm` | bimanual WebXR task, independent grippers, planner, and Viser |
| `openarm-connection-control` | connection and coordinator with trajectory and gripper tasks |

## WebXR controls and safety

Both controllers feed one fourteen-joint IK task. Hold both middle-finger grips
to engage it. Releasing either grip stops arm output and clears both controller
references. Each index-finger trigger controls the same side's gripper while
that hand's grip is held. Manual arm and gripper tasks have priority 20 over
priority-10 trajectories. Taking control aborts the active trajectory.

The arm velocity profile is `(1, 1, 1, 1, 2, 2, 2)` rad/s per side, with a
5 Hz joint-command filter. OpenArm retains its Pink position, orientation, and
posture objectives.

The connection derives angular limits from the physical model. It clamps
feedback up to `0.05 rad` beyond a limit. Larger excursions latch a fault,
disable the motors, and require a new connection. Missing commands trigger a
hold at measured positions. These software checks do not establish physical
firmware behavior or confirm that every motor supplied fresh feedback.

## Files

| Path | Role |
|---|---|
| `dimos/robot/manipulators/openarm/connection.py` | immutable topology, gains, CAN selection, and direct connection |
| `dimos/robot/manipulators/openarm/config.py` | pinned model, canonical joints and planning configuration |
| `dimos/robot/manipulators/openarm/blueprints/` | connection, coordinator, planner, and WebXR compositions |
| `dimos/robot/manipulators/common/damiao_connection.py` | shared native lifecycle and in-memory operation |

## Validation

```bash
uv run pytest dimos/robot/manipulators/openarm \
    dimos/robot/manipulators/common/test_damiao_connection.py \
    dimos/control/test_coordinator_connections.py
```

Focused tests exercise source-prefixed commands through an in-memory connection,
both-grip activation, independent grippers, and preserved tuning. Tests marked
`self_hosted` also check official model geometry and Pink solver behavior.
Physical activation, gripper calibration, bus mapping, and fault recovery still
need validation on a supported rig.
