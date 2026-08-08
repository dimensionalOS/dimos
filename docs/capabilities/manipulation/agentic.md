---
title: "Agentic xArm Pick and Place"
---

`xarm-grasp-sim-agent` runs xArm7 MuJoCo simulation, ground-truth object
geometry, GraspGenX, planning, MCP, and the built-in agent together.

```bash
uv sync --extra manipulation --extra graspgenx --inexact
uv run dimos run xarm-grasp-sim-agent --daemon
```

The real box-filling product is:

```bash
uv run dimos run xarm-box-filling --daemon
```

Both products expose the same generic workflow: `scan_objects`,
`select_object`, `pick_selected`, and `place_at`. A scan creates a numbered
snapshot; selection pins its exact object and provider proposals; picking
performs fresh feasibility checks and requires gripper feedback before
reporting success. All public positions and geometry are in `world`.

The box-filling module derives from the generic module and adds
`select_destination_container` and `place_in_destination`. Those application
skills compute opening fit and release clearance without leaking box policy
into the generic API.

The planner checks collision-free preparation and pre-grasp motion. Straight
TCP contact, retreat, place-lower, and place-retract legs intentionally bypass
collision queries while retaining sequential IK, joint limits, timing, and
execution checks. A target-aware collision policy is a later safety extension.

After the gripper closes, failures never reopen it automatically because an
object may be held. Missing feedback and an empty-close reading are failures.
