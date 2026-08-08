---
title: "OpenArm Integration"
---

DimOS supports the [OpenArm v2.0](https://openarm.dev) bimanual platform as one
robot with two 7-DOF arms and two grippers. Physical operation uses one Linux
SocketCAN interface per arm. Start with fake hardware, verify the arm-to-bus
mapping, and keep an E-STOP within reach before enabling motors.

## Install

From a DimOS source checkout, install the manipulation dependencies:

```bash skip
uv sync --extra manipulation --inexact
```

Physical OpenArm support is Linux-only. Fake hardware and planning can run on
other supported platforms.

## Prepare the CAN interfaces

Bring up both interfaces and verify their status:

```bash skip
dimos hardware can setup can0
dimos hardware can setup can1
dimos hardware can status can0
dimos hardware can status can1
```

The default mapping is left arm on `can1` and right arm on `can0`. Linux names
USB CAN adapters in enumeration order, so confirm the mapping whenever adapters
are reconnected. A swapped mapping can command the wrong arm.

## Run with fake hardware

Run keyboard teleoperation in simulation mode:

```bash skip
dimos --simulation run keyboard-teleop-openarm
```

The Quest blueprint is fake-hardware-first and needs no simulation flag:

```bash skip
dimos run teleop-quest-openarm
```

Open `https://<host-ip>:8443/teleop` in the Quest browser. Accept the local
certificate warning if prompted.

## Run physical hardware

The keyboard blueprint selects physical hardware when `--simulation` is absent
and uses the default CAN mapping:

```bash skip
dimos run keyboard-teleop-openarm
```

Quest teleoperation selects physical hardware only when both bus mappings are
provided after the blueprint name:

```bash skip
dimos run teleop-quest-openarm \
  --left-can-port can1 \
  --right-can-port can0
```

Providing only one port is rejected. These are blueprint options; the global
`--can-port` option does not configure OpenArm. Other OpenArm blueprints do not
currently expose per-arm remapping options, so use the Quest blueprint when the
machine's interface names differ from the defaults.

## Controls and runtime behavior

Quest uses one bimanual IK task for both arms. Hold both controllers' primary
buttons to engage it. Releasing either button stops arm and gripper output and
clears both controller references. Each controller trigger operates the
gripper on the same side.

Stale controller input, task preemption, E-STOP, or coordinator shutdown also
clears the teleoperation session. Planned trajectories have priority 20 and
preempt the Quest task at priority 10. Planning commands the fourteen arm
joints; the grippers remain under Quest control.

Keyboard teleoperation currently jogs the left arm while the right arm holds
its anchor pose. The keyboard blueprint does not bind gripper controls.

## Available blueprints

| Blueprint | Hardware selection | Purpose |
| --- | --- | --- |
| `coordinator-openarm` | Fake with `--simulation`; physical otherwise | Coordinator and trajectory control |
| `openarm-planner-coordinator` | Fake with `--simulation`; physical otherwise | Bimanual planner and coordinator |
| `keyboard-teleop-openarm` | Fake with `--simulation`; physical otherwise | Keyboard Cartesian jogging and Viser |
| `keyboard-teleop-openarm-planner` | Fake with `--simulation`; physical otherwise | Keyboard teleoperation with planning |
| `teleop-quest-openarm` | Fake by default; physical only with both CAN options | Quest bimanual teleoperation, planning, and Viser |

## Troubleshooting

### The wrong arm moves

Stop the blueprint immediately. Confirm which USB adapter Linux assigned to
each interface, then pass the corrected left and right ports to the Quest
blueprint. Do not assume `can0` always identifies the same physical adapter.

### One arm does not connect

Check both interfaces with `dimos hardware can status`. The Quest blueprint
requires both explicit ports for physical operation and rejects a partial
mapping.

### Quest does not connect

Confirm that the headset can reach `https://<host-ip>:8443/teleop`, accept the
certificate warning, and check that the host firewall permits the connection.

### Quest runs but the arms do not move

Confirm that both primary buttons remain held. If the model moves in Viser but
hardware does not, inspect command-tracking and joint-limit warnings before
raising any safety threshold.

### Motion is slow, unstable, or stops near a limit

Do not change the canonical startup pose to hide the symptom. Follow
[Pink IK Configuration and Tuning](/docs/capabilities/manipulation/pink_ik_tuning.md)
to verify the model, tune objective weights, and bound streaming commands.

## Further reading

- [Quest Teleoperation](https://github.com/dimensionalOS/dimos/blob/main/dimos/teleop/quest/README.md)
- [Pink IK Configuration and Tuning](/docs/capabilities/manipulation/pink_ik_tuning.md)
- [Adding a Custom Arm](/docs/capabilities/manipulation/adding_a_custom_arm.md)
- [Planning Groups](/docs/capabilities/manipulation/planning_groups.md)
- [Upstream OpenArm CAN reference](https://github.com/enactic/openarm_can)
