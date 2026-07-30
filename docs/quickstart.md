# Quickstart

Install DimOS, run a replay or simulation, then explore the CLI and
MCP. No hardware is required.

In this quickstart, you will replay a Unitree Go2 office navigation session
without hardware, then see how to switch to simulation or a live robot.

If you use coding agents (OpenClaw, Claude Code, or similar), point them at
[AGENTS.md](https://github.com/dimensionalOS/dimos/blob/main/AGENTS.md).

## System requirements

:::{list-table}
   :header-rows: 1
   :widths: 20 40 40

   * - Component
     - Minimum
     - Recommended
   * - OS
     - Ubuntu 22.04, macOS 12.6+
     - Ubuntu 24.04
   * - Python
     - 3.12
     - Latest
   * - RAM
     - 16 GB
     - 32 GB+
   * - Disk
     - 10 GB SSD
     - 25 GB+ SSD
   * - CPU
     - 8-core Intel / AMD
     - 12+ cores
   * - GPU (optional)
     - NVIDIA RTX 3000+ (8 GB VRAM)
     - RTX 4070+ (12 GB+ VRAM)
:::

:::{note}
A GPU is required only for perception, VLMs, and AI features. It is optional for
basic robot control.
:::

## Interactive install

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

## Manual system install

If you prefer to install system dependencies yourself, follow the guide for your OS:

:::{list-table}
   :header-rows: 1
   :widths: 30 70

   * - OS guide
     - Notes
   * - [Ubuntu](installation/ubuntu.md)
     - Primary tested path
   * - [Nix](installation/nix.md)
     - Flakes and development shell
   * - [macOS](installation/osx.md)
     - Homebrew-based; less mature than Linux
:::

## Python environment

DimOS targets Python 3.12. These examples use [uv](https://docs.astral.sh/uv/);
plain `python -m venv` and `pip` work too.

```bash
uv venv --python "3.12"
source .venv/bin/activate   # Windows: .venv\Scripts\activate
```

## Install DimOS

```bash
uv pip install 'dimos[base,unitree]'
```

Extras keep installations lean: `base` provides the runtime, modules, transports, and
CLI; `unitree` adds WebRTC and skills for the Go2 and G1, whether real or replayed.

### Replay a recorded session (no hardware)

On the first run, the Rerun window may stay black briefly while roughly 75 MB of data
downloads from LFS.

```bash
dimos --replay run unitree-go2
```

### Simulation (MuJoCo)

```bash
uv pip install 'dimos[base,unitree,sim]'
dimos --simulation run unitree-go2
dimos --simulation run unitree-g1-sim   # humanoid
```

### Real robot (Unitree Go2 over WebRTC)

```bash
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

Do not skip the [Unitree Go2 platform guide](platforms/quadruped/go2/index.md);
latency, time synchronization, and safety practices matter.

## Featured runfiles

:::{list-table}
   :header-rows: 1
   :widths: 55 45

   * - Command
     - What it does
   * - :bash:`dimos --replay run unitree-go2`
     - Quadruped navigation replay — SLAM, costmap, A* planning
   * - :bash:`dimos --replay --replay-db go2_bigoffice run unitree-go2-memory`
     - Quadruped temporal memory replay
   * - :bash:`dimos --simulation run unitree-go2-agentic`
     - Quadruped agentic stack and MCP server in simulation
   * - :bash:`dimos --simulation run unitree-g1-sim`
     - Humanoid in MuJoCo simulation
   * - :bash:`dimos --replay run drone-basic`
     - Drone video and telemetry replay
   * - :bash:`dimos --replay run drone-agentic`
     - Drone and LLM agent with flight skills in replay
   * - :bash:`dimos run demo-camera`
     - Webcam demo; no hardware needed
   * - :bash:`dimos run keyboard-teleop-xarm7`
     - Keyboard teleoperation with mock xArm7 (``dimos[manipulation]`` extra)
   * - :bash:`dimos --simulation run unitree-go2-agentic-ollama`
     - Quadruped agentic stack with a local LLM (requires ``ollama serve``)
:::

See [Blueprints](usage/blueprints.md) for the complete blueprint reference.

## Agent CLI and MCP

The `dimos` CLI runs blueprints, inspects state, talks to agents, and invokes
skills through MCP.

```bash
dimos run unitree-go2-agentic --daemon
dimos status
dimos log -f
dimos agent-send "explore the room"
dimos mcp list-tools
dimos mcp call relative_move --arg forward=0.5
dimos stop
```

See [CLI Reference](usage/cli.md) for the complete CLI reference.

## What next?

::::{grid} 1 1 2 2
:gutter: 2

:::{grid-item-card} Add an LLM agent
:link: capabilities/agents/index
:link-type: doc

Natural-language control and MCP-exposed skills.
:::

:::{grid-item-card} Pick your platform
:link: platforms/index
:link-type: doc

Hardware support, simulation, and bring-up guides.
:::

:::{grid-item-card} Core concepts
:link: usage/index
:link-type: doc

Modules, streams, and blueprints behind every workflow.
:::

:::{grid-item-card} Capabilities
:link: capabilities/index
:link-type: doc

Navigation, perception, spatial memory, and manipulation.
:::
::::
