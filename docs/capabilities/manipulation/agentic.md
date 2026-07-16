---
title: "Agentic xArm Simulation"
---

# Agentic xArm simulation

`xarm-perception-sim-agent` runs the xArm perception, planning, MuJoCo
simulation, MCP server, and built-in agent together. It is **simulation-only**;
it does not control a physical arm.

See the [manipulation capability overview](/docs/capabilities/manipulation/) for
the underlying planning and perception stack.

## Prerequisites

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The built-in agent requires an `OPENAI_API_KEY` for OpenAI Platform API access.
Your OpenAI account must have billing enabled or available API credits. Do not
put the key in this document or in shell history shared with others.

The local LCM UDP multicast/loopback path must be available between the DimOS
processes. On a restricted environment, allow local multicast traffic on the
loopback interface; otherwise the agent input and module communication may not
arrive.

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

`look` observes the current camera view without moving the arm. `scan_objects`
is a single initialization-viewpoint perception snapshot: it first moves the
simulated arm to its init position and then refreshes detections. It is not a
guaranteed object inventory.
