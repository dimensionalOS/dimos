# The Agentive Operating System for Physical Space

Dimensional is the modern operating system for generalist robotics. We are setting the
next-generation SDK standard, integrating with the majority of robot manufacturers.

With a simple install and no ROS required, build physical applications entirely in Python
that run on any humanoid, quadruped, or drone. Dimensional is agent native — "vibecode" your
robots in natural language and build (local and hosted) multi-agent systems that work
seamlessly with your hardware. Agents run as native modules, subscribing to any embedded
stream, from perception (lidar, camera) and spatial memory down to control loops and motor
drivers.

Current version is {{ release }}.

:::{important}
This is a pre-release beta. Direct your favorite agent (OpenClaw, Claude Code, etc.) to
[AGENTS.md](https://github.com/dimensionalOS/dimos/blob/main/AGENTS.md) and the
[Agent CLI and MCP] interfaces to start building Dimensional applications.
:::

## Capabilities at a glance

::::{grid} 1 1 2 2
:gutter: 2

:::{grid-item-card} Navigation & mapping
:link: capabilities/navigation/index
:link-type: doc

**SLAM**, dynamic obstacle avoidance, route planning, and autonomous exploration
via both DimOS native and ROS integrations.
:::

:::{grid-item-card} Perception

Detectors, 3D projections, VLMs, and audio processing.
:::

:::{grid-item-card} Agents
:link: capabilities/agents/index
:link-type: doc

Agentive control and MCP. Example: *"Hey robot, go find the kitchen."*
:::

:::{grid-item-card} Spatial memory
:link: capabilities/memory/index
:link-type: doc

Spatio-temporal RAG, dynamic memory, object localization and permanence.
:::
::::

## Start here

::::{grid} 1 1 2 2
:gutter: 2

:::{grid-item-card} Quickstart
:link: quickstart
:link-type: doc

Install DimOS, run your first blueprint, and inspect the running system.
:::

:::{grid-item-card} Platforms
:link: platforms/index
:link-type: doc

Go2 quadruped and G1 humanoid setup, simulation, and blueprints for real hardware.
:::
::::

## Hardware

:::{list-table}
   :header-rows: 1
   :widths: 20 80

   * - Category
     - Supported platforms
   * - Quadruped
     - Unitree Go2 pro/air (stable), Unitree B1 (experimental)
   * - Humanoid
     - Unitree G1 (beta)
   * - Arm
     - xArm (beta), AgileX Piper (beta)
   * - Drone
     - MAVLink (alpha), DJI Mavic (alpha)
   * - Misc
     - Force Torque Sensor (experimental)
:::

## Installation

### Interactive install

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

See `scripts/install.sh --help` for non-interactive and advanced options.

### Manual system install

To set up your system dependencies, follow one of these guides:

- [Ubuntu 22.04 or 24.04](installation/ubuntu.md) (stable)
- [NixOS or Nix-managed Linux](installation/nix.md) (stable)
- [macOS 12.6 or newer](installation/osx.md) (alpha)

See [System Requirements](requirements.md) for tested configurations and dependency tiers.

### Python install

#### Quick start

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install 'dimos[base,unitree]'

# Replay a recorded quadruped session (no hardware needed).
# NOTE: the first run shows a black rerun window while ~75 MB downloads from LFS.
dimos --replay run unitree-go2
```

```bash
# Install with simulation support.
uv pip install 'dimos[base,unitree,sim]'

# Run a quadruped in MuJoCo simulation.
dimos --simulation run unitree-go2

# Run a humanoid in simulation.
dimos --simulation run unitree-g1-sim
```

```bash
# Control a real robot (Unitree quadruped over WebRTC).
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

## Featured runfiles

:::{list-table}
   :header-rows: 1
   :widths: 50 50

   * - Run command
     - What it does
   * - :bash:`dimos --replay run unitree-go2`
     - Quadruped navigation replay — SLAM, costmap, A* planning
   * - :bash:`dimos --replay --replay-db go2_bigoffice run unitree-go2-memory`
     - Quadruped temporal memory replay
   * - :bash:`dimos --simulation run unitree-go2-agentic`
     - Quadruped agentic + MCP server in simulation
   * - :bash:`dimos --simulation run unitree-g1-sim`
     - Humanoid in MuJoCo simulation
   * - :bash:`dimos --replay run drone-basic`
     - Drone video + telemetry replay
   * - :bash:`dimos run demo-camera`
     - Webcam demo — no hardware needed
:::

See [Blueprints](usage/blueprints.md) for the complete blueprint guide.

## Agent CLI and MCP

The `dimos` CLI manages the full lifecycle — run blueprints, inspect state, interact
with agents, and call skills via MCP.

```bash
dimos run unitree-go2-agentic --daemon   # Start in background
dimos status                             # Check what's running
dimos log -f                             # Follow logs
dimos agent-send "explore the room"      # Send agent a command
dimos mcp list-tools                     # List available MCP skills
dimos mcp call relative_move --arg forward=0.5  # Call a skill directly
dimos stop                               # Shut down
```

See [CLI Reference](usage/cli.md) for the complete CLI reference.

## Using DimOS as a Library

The example below is a simple robot-connection module that publishes a stream of
[`Image`][Image] frames, and a listener that subscribes to them.
DimOS modules are subsystems that communicate using standardized messages over typed
[`In`][In] / [`Out`][Out] streams, with remotely
callable methods marked by the [`rpc()`][rpc] decorator.

```{literalinclude} code/index.py
:pyobject: RobotConnection
```

```{literalinclude} code/index.py
:pyobject: Listener
```

Compose the modules with [`autoconnect()`][autoconnect] — which
connects streams by `(name, type)` — and run them by handing the resulting blueprint to
[`build()`][build], then
[`loop()`][loop]:

```{literalinclude} code/index.py
:lines: 2-
:pyobject: run_connection
```

### Blueprints

Blueprints are instructions for how to construct and wire modules. Each
[`Module`][Module] exposes a [`blueprint`][blueprint] factory, and
[`autoconnect()`][autoconnect] composes several into a single
[`Blueprint`][blueprints-Blueprint]. Blueprints can be composed, remapped,
or have transports overridden with
[`transports()`][transports] when
[`autoconnect()`][autoconnect] cannot
resolve conflicting names or message types on its own.

The example below connects the image stream from a Unitree Go2 to an
MCP-backed agent for
reasoning and action execution, pinning the `color_image` stream onto an explicit
[`LCMTransport`][LCMTransport]:

```{literalinclude} code/index.py
:lines: 2-
:pyobject: run_agentic_blueprint
```

### API reference

See [API Reference](api.rst) for the full API reference.

## Development

```bash
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

# Run the default test suite (uv run syncs deps on demand).
uv run pytest --numprocesses=auto dimos
```

### Multi-language support

Python is the glue and prototyping language, but many languages are supported via LCM
interop. See the language interop examples for
[C++](https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/cpp/),
[Lua](https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/lua/), and
[TypeScript](https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/ts/).

## Table of Contents

```{toctree}
:maxdepth: 2
:name: mastertoc

Introduction <self>
Quickstart <quickstart>
installation/index
usage/index
capabilities/index
platforms/index
development/index
coding-agents/index
api
```

[Image]: #dimos.msgs.sensor_msgs.Image.Image
[In]: #dimos.core.stream.In
[Out]: #dimos.core.stream.Out
[rpc]: #dimos.core.core.rpc
[autoconnect]: #dimos.core.coordination.blueprints.autoconnect
[build]: #dimos.core.coordination.module_coordinator.ModuleCoordinator.build
[loop]: #dimos.core.coordination.module_coordinator.ModuleCoordinator.loop
[Module]: #dimos.core.module.Module
[blueprint]: #dimos.core.module.Module.blueprint
[blueprints-Blueprint]: #dimos.core.coordination.blueprints.Blueprint
[transports]: #dimos.core.coordination.blueprints.Blueprint.transports
[LCMTransport]: #dimos.core.transport.LCMTransport
