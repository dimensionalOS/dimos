---
title: "Agentic xArm Simulation"
---

# Agentic xArm simulation

`xarm-perception-sim-agent` runs the xArm perception, planning, MuJoCo
simulation, MCP server, and built-in agent together. It is **simulation-only**;
it does not control a physical arm.

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

## Ask the built-in agent

Send text to the running agent over LCM:

```bash
uv run dimos agent-send "Report the current robot state and visible objects; do not move the arm or gripper."
```

A minimal safe prompt for state and perception checks is:

```text
Do not move the arm, gripper, or any object. Report the current robot state and inspect perception only.
```

## Inspect through MCP

The blueprint includes an MCP server. Check that it is running and inspect the
available tools:

```bash
uv run dimos mcp status
uv run dimos mcp list-tools
```

These inspection calls are useful for checking state and perception:

```bash
uv run dimos mcp call get_robot_state
uv run dimos mcp call look
uv run dimos mcp call scan_objects
```

`look` observes the current camera view without moving the arm. `scan_objects`
is a single initialization-viewpoint perception snapshot: it first moves the
simulated arm to its init position and then refreshes detections. It is not a
guaranteed object inventory. Detection and planning can fail, so inspect each
result and reset or restart the simulation when needed rather than issuing
repeated motion commands.

## Motion and convergence

An accepted command can complete before the MuJoCo joint positions have fully
converged. Before issuing sequential manipulation commands, call
`get_robot_state` and verify that the arm is at the expected pose. If a motion
or plan fails, inspect the state and output, then reset or restart instead of
blindly retrying commands. This page does not describe pick/place as a reliable
workflow.
