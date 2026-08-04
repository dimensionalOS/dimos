---
title: "OpenArm Integration"
---

DimOS drives the [OpenArm](https://openarm.dev) bimanual platform (two 7-DOF
arms + grippers, Damiao motors, one CAN bus per arm) as a single whole-body
device through the generic Damiao adapter stack introduced for OpenYAM.

Related:
- Upstream hardware + C++ reference: [enactic/openarm_can](https://github.com/enactic/openarm_can)
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
2x DM8006, 2x DM4340, 3x DM4310, plus a DM4310 gripper at `0x08`.

Gravity compensation uses the bimanual URDF
(`openarm_description/urdf/robot/openarm_v10_bimanual.urdf`, resolved lazily
from LFS at connect time) and is preflighted against the declared joint order
before the motors enable.

## Bring-up

```bash
dimos can setup can0
dimos can setup can1
dimos run keyboard-teleop-openarm
```

Linux assigns `can0`/`can1` in USB enumeration order. If the arms come up
swapped, override the mapping through
`DamiaoRuntimeConfig(bus_addresses={"left": ..., "right": ...})` rather than
editing the adapter topology.

## Blueprints

| Blueprint | Contents |
|---|---|
| `coordinator-openarm` | coordinator + trajectory task over both arms |
| `openarm-planner-coordinator` | planner (per-side models) + coordinator |
| `keyboard-teleop-openarm` | keyboard + per-arm EEF twist + gripper servo + viser |
| `keyboard-teleop-openarm-planner` | teleop + planner + preempting trajectory task |

All blueprints run against the in-memory whole-body adapter under
`--simulation`; the physical adapter is selected automatically otherwise.

The keyboard jogs the left arm (`eef_twist_left_arm`); the right arm's twist
task holds its anchor pose. `[` opens and `]` closes both grippers together
via a single servo task over both gripper joints.

## Files

| Path | Role |
|---|---|
| `dimos/hardware/whole_body/openarm_damiao/adapter.py` | physical topology (motors, buses, gravity URDF) |
| `dimos/robot/manipulators/openarm/config.py` | joints, gains, hardware + planning model configs |
| `dimos/robot/manipulators/openarm/blueprints/` | coordinator/planner/teleop blueprints |

## Validation

```bash
uv run pytest dimos/hardware/whole_body/openarm_damiao \
    dimos/hardware/test_adapter_registries.py
```
