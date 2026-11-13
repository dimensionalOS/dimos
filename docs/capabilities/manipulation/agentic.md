# Agentic xArm Simulation

`xarm-perception-sim-agent` runs the xArm perception, planning, MuJoCo
simulation, MCP server, and built-in agent together. It is **simulation-only**; This guide uses this blueprint to provide a walk-through of dimos's agentic manipulation stack.

See the [manipulation capability overview](/docs/capabilities/manipulation/index.md) for
the underlying planning and perception stack.

## Prerequisites

Use the [official installer](/docs/installation/index.md) first; its default extras include manipulation. To add the dependencies to an existing developer environment:

```bash
uv sync --extra manipulation --inexact
```

This installs perception, EdgeTAM, MuJoCo, web/MCP, and agent dependencies
together. The built-in agent requires an `OPENAI_API_KEY`; EdgeTAM requires a
CUDA or MPS device. Robot assets and model weights are downloaded separately.


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

Find the objects on the table. The arm looks from two poses first:

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
# localize confirms an object from two camera poses, so look from a second one first
uv run dimos mcp call move_to_pose --json-args '{"x": 0.4, "y": 0.1, "z": 0.35}'
uv run dimos mcp call localize --json-args '{"objects": "cup", "start": -30, "duration": 30}'
```
