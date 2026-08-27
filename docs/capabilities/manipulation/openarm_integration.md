# OpenArm Integration

dimOS drives the [OpenArm](https://openarm.dev) bimanual platform (two 7-DOF
arms + grippers, Damiao motors, one CAN bus per arm) as a single whole-body
device through the generic Damiao adapter stack introduced for OpenYAM.

Related:
- Upstream hardware + C++ reference: [enactic/openarm_can](https://github.com/enactic/openarm_can)
- Robot model: [enactic/openarm_description](https://github.com/enactic/openarm_description)
- How to integrate any new arm: [adding_a_custom_arm.md](/docs/capabilities/manipulation/adding_a_custom_arm.md)

## Architecture

```
ControlCoordinator (100 Hz)
  └── HardwareComponent "openarm" (WHOLE_BODY, 16 joints)
        └── OpenArmDamiaoAdapter          # dimos/hardware/whole_body/openarm_damiao/
              └── DamiaoWholeBodyAdapter  # generic Damiao lifecycle + gravity comp
                    └── can-motor-control # Rust CAN transport + Damiao codec (PyPI)
```

One adapter owns both arms: bus `left` (default `can1`) and bus `right`
(default `can0`) are commanded together in one synchronized tick per control
cycle. The command vector order is `left_arm/joint1..7`, `right_arm/joint1..7`,
`left_arm/gripper`, `right_arm/gripper`; gripper joints are normalized
(`0.0` closed, `1.0` open).

Per arm, shoulder to wrist (send ids `0x01..0x07`, feedback `send | 0x10`):
2x DM8009, 2x DM4340, 3x DM4310, plus a DM4310 gripper at `0x08`.

Gravity compensation and planning use the official bimanual OpenArm v2.0
Xacro. dimOS pins the description repository to an immutable commit and checks
it out lazily through the robot asset cache. Gravity compensation locks the
finger joints and preflights the remaining 14 joints against the declared arm
order before enabling the motors.

Planning treats the two arms as one robot with `left_manipulator` and
`right_manipulator` planning groups because collision exclusions cannot span
robots.

## Bring-up

```bash
dimos run openarm-planner-coordinator # mock hardware
dimos run teleop-webxr-openarm         # mock WebXR teleoperation

dimos hardware can setup can0
dimos hardware can setup can1
dimos run openarm-planner-coordinator --left-can-port can1 --right-can-port can0
dimos run teleop-webxr-openarm --left-can-port can1 --right-can-port can0
```

Linux assigns `can0`/`can1` in USB enumeration order. If the arms come up
swapped, exchange the two explicit CLI values. Supplying only one interface is
rejected so physical operation can never depend on USB/CAN enumeration defaults.

## Blueprints

| Blueprint | Contents |
|---|---|
| `coordinator-openarm` | coordinator + trajectory task over both arms |
| `openarm-planner-coordinator` | planner (bimanual model) + coordinator |
| `teleop-webxr-openarm` | bimanual WebXR teleoperation + planner + Viser |

All OpenArm blueprints use the in-memory whole-body adapter by default. Passing
both `--left-can-port` and `--right-can-port` selects the physical adapter.

## WebXR controls and safety

The WebXR blueprint drives both arms through one bimanual IK task. Hold both
controllers' primary buttons to engage it. Releasing either button stops arm
output and clears both controller references. Each trigger publishes normalized
opening to a dedicated gripper task on the same side. Planned trajectories run
at a higher priority and preempt streaming teleoperation.

The Damiao adapter derives angular joint limits from the official robot model.
It clamps encoder feedback up to `0.05 rad` beyond a limit; larger excursions
latch a fault, disable the motors, and prevent reactivation until the adapter is
reconnected.

## Files

| Path | Role |
|---|---|
| `dimos/hardware/whole_body/openarm_damiao/adapter.py` | physical topology, motors, buses, and gravity model |
| `dimos/robot/manipulators/openarm/config.py` | pinned model, joints, gains, hardware, and planning config |
| `dimos/robot/manipulators/openarm/blueprints/` | coordinator and planner blueprints |

## Validation

```bash
uv run pytest dimos/hardware/whole_body/openarm_damiao \
    dimos/hardware/whole_body/damiao \
    dimos/robot/manipulators/openarm \
    dimos/hardware/test_adapter_registries.py
```
