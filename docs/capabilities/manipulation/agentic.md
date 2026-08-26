# Agentic xArm Simulation

`xarm-perception-sim-agent` runs the xArm perception, planning, MuJoCo
simulation, MCP server, and built-in agent together. It is **simulation-only**; This guide uses this blueprint to provide a walk-through of dimos's agentic manipulation stack.

See the [manipulation capability overview](/docs/capabilities/manipulation/index.md) for
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

`PickAndPlaceModule` is robot-independent. It composes object-scene,
grasp-provider, candidate-filter, and execution Specs; `ManipulationModule`
provides the planner, motion, and gripper execution capabilities in xArm
blueprints.

Its canonical workflow is `scan_objects(prompts)`, `get_object(object_id)`,
`select_grasp(object_id, rank=0)`, `get_grasp_candidates()`,
`pick_selected(robot_name=None)`, and
`place_at(x, y, z, robot_name=None)`. Use the stable `Object.object_id`
returned by a scan to select an exact object. The core workflow does not model
tables, boxes, or scene obstacles and does not publish visualization state.

## Direct interaction

Use the MCP server to inspect the canonical skills after launching an agent
blueprint:

```bash
uv run dimos mcp status
uv run dimos mcp list-tools
```
