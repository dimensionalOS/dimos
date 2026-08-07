---
title: "Agentic xArm Simulation"
---

`xarm-perception-sim-agent` runs the xArm perception, planning, MuJoCo
simulation, MCP server, and built-in agent together. It is **simulation-only**; This guide uses this blueprint to provide a walk-through of dimos's agentic manipulation stack.

See the [manipulation capability overview](/docs/capabilities/manipulation/) for
the underlying planning and perception stack.

## Prerequisites

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The built-in agent requires an `OPENAI_API_KEY`.


## Start and stop

Run in the foreground:

```bash
uv run dimos run xarm-perception-sim-agent
```

Or run it as a daemon:

```bash
uv run dimos run xarm-perception-sim-agent --daemon
```

Inspect and control the run from another terminal:

```bash
uv run dimos status
uv run dimos log
uv run dimos stop
```

Use `dimos log -f` to follow the log while the run is active.

## Learned grasp-to-pick pipeline

The real-hardware `xarm-graspgenx-agent` blueprint adds GraspGenX proposals to
the xArm perception stack. Install the optional runtime and start it with:

```bash
uv sync --extra graspgenx --extra manipulation --inexact
uv run dimos run xarm-graspgenx-agent
```

`pick` remains the only high-level picking tool. It resolves one current
object, obtains that object's planning-frame point cloud, requests ranked
GraspGenX candidates, and rejects candidates that fail pre-grasp, grasp, or
retreat inverse kinematics. During planning, the selected target is
temporarily removed from the collision scene while all other obstacles remain
active. The selected candidate then runs through prepare, approach, grasp,
close, verify, and retreat phases.

Use the stable object ID returned by `scan_objects` whenever names are
ambiguous. A name is accepted only when it identifies exactly one current
detection; an object-ID prefix must also be unique. Existing
`xarm-perception` and `xarm-perception-sim` blueprints retain their explicit
heuristic grasp fallback and do not load the optional GraspGenX runtime.

The learned pipeline configuration lives in
`dimos/robot/manipulators/xarm/grasp_config.py`. It records the xArm gripper
sweep volume and the transform from GraspGenX's gripper frame to the planned
TCP. `PickAndPlaceModuleConfig` controls the planning frame, maximum point
cloud age, candidate-check limit, TCP approach direction, approach/retreat
offsets, heuristic fallback, and closure-feedback verification thresholds.
Changing the frame transform, approach direction, or closure threshold
requires robot-specific calibration.

Failures are phase-specific and stop motion immediately. Before closure, a
failed transaction leaves the gripper in its current safe state. After a
successful close command, failures never automatically reopen the gripper;
the result includes `object_may_be_held=true`, and an operator or agent should
inspect state before issuing another motion. Target collision geometry is
restored on every exit path, and restoration errors are reported without
hiding the primary failure.

The current verification is a closure-position proxy: an xArm gripper that
stops above the calibrated empty-close threshold is treated as holding
something. It does not measure grasp force, detect slip, or prove that the
intended object was acquired. Force/torque or tactile feedback is required for
those stronger guarantees. The shipped learned-grasp blueprint keeps this
proxy disabled until the open, empty-close, and representative held-object
positions have been measured on the target xArm; enable
`grasp_verification.enabled` only after recording that calibration.

## Daily interaction

For normal interactive use, start the human-friendly terminal client:

```bash
uv run dimos humancli
```

It connects to the running agent so you can send prompts and read responses in
one session.

### Try these prompts

Start with a non-motion state check:

```text
Report the current robot state without moving.
```

Scan the scene for objects. This moves the arm to its observation pose:

```text
Scan for objects.
```

Try basic motion commands:

```text
Move 10 cm to the left.
```

```text
Move 10 cm above the detected object's pose.
```

## Debugging and testing interfaces

Use `agent-send` for one-shot LCM input when testing or diagnosing the agent:

```bash
uv run dimos agent-send "Report the current robot state and visible objects; do not move the arm or gripper."
```

The blueprint also includes an MCP server. Use these commands for direct
server inspection and tool-level testing:

```bash
uv run dimos mcp status
uv run dimos mcp list-tools
```

For example:

```bash
uv run dimos mcp call get_robot_state
uv run dimos mcp call look
uv run dimos mcp call scan_objects
```
