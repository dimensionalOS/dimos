---
title: "Manipulation"
description: "Plan collision-free motion, teleoperate, collect demonstrations, and train policies — on any supported arm."
---

DimOS manipulation gets a robot arm from "plugged in" to "doing useful work". You can plan collision-free motion, teleoperate with a VR headset or keyboard, record demonstrations, and train a policy that runs back on the same robot.

## Choose your workflow

| I want to... | Start here |
|--------------|-----------|
| **Move an arm right now** | `dimos run keyboard-teleop-xarm7` — no hardware needed, runs against a mock arm |
| **Plan collision-free motion** | [The planning stack](/docs/capabilities/manipulation/planning.md) |
| **Collect demos and train a policy** | [The learning loop](/docs/capabilities/manipulation/learning.md) |
| **Let an agent compose manipulation skills** | [Agentic xArm simulation](/docs/capabilities/manipulation/agentic.md) |
| **Connect my own arm** | [Adding a custom arm](/docs/capabilities/manipulation/adding_a_custom_arm.md) |

## How the stack fits together

Three layers, top to bottom:

```
ManipulationModule        "move the gripper to this pose"
  planning + kinematics    plans a collision-free joint path (RRT over a
                           Drake world), solving IK along the way
        │ trajectory, via RPC
        ▼
ControlCoordinator        100Hz loop. Runs a trajectory task that streams
  trajectory execution     the plan, arbitrates who owns which joint
        │ joint commands
        ▼
Hardware adapter          one small class per arm; wraps the vendor SDK
```

**The planning layer is where you should spend your time.** It is where "do the task" turns into motion — and it is deliberately pluggable. The planner, the IK solver, and the world model are all protocol-based: you can swap any of them, or bypass planning entirely and drive the coordinator from a learned policy. The layers below rarely need touching once your arm is integrated.

Teleop slots in beside planning, not above it: a Quest headset or keyboard streams end-effector targets to the coordinator, which solves IK in-loop. That path is what makes demonstration collection feel direct.

## What runs today

| Arm | Teleop | Planning | Perception | Learning |
|-----|--------|----------|------------|----------|
| XArm 6 / 7 | keyboard, VR | ✓ | ✓ (RealSense) | VR collect |
| Piper | keyboard, VR | ✓ | — | VR collect |
| Galaxea A1Z | hand-teach | — | UVC camera | hand-teach → policy |
| A-750 | keyboard | ✓ | — | — |
| OpenArm (bimanual) | keyboard | ✓ | — | — |

Several workflows also run without hardware through a mock arm or MuJoCo, including the quick start below. A1Z hand-teach and learned-policy execution require the real arm.

Per-arm setup lives with the platform, not here: [A-750](/docs/platforms/arms/a750.md), [OpenArm](/docs/platforms/arms/openarm.md), and the [Galaxea A1Z hardware and learning guide](/dimos/robot/manipulators/galaxea_a1z/README.md).

## Try it in two minutes

```bash
uv sync --extra manipulation --inexact
dimos run keyboard-teleop-xarm7
```

A Meshcat window opens with the arm. Drive the end-effector with `W/A/S/D` (XY), `Q/E` (Z), `R/F/T/G/Y/H` (roll/pitch/yaw). Everything you see — the IK, the 100Hz loop, the visualization — is the same stack that runs on real hardware; only the adapter is fake.

Then pick your workflow from the table above.
