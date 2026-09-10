# Quickstart

Start by [creating an SDK application](/docs/installation/installer.md). Run its
synthetic image example, edit the generated source, and run `pytest`.

Once that works, the same application environment can run the built-in examples
below. Replay workloads download recorded data; hardware workloads require the
corresponding robot. Activate your application before running these commands.

## Run your first replay

dimOS applications are launched from **blueprints**. A blueprint is a ready to run bundle of modules that you start by name with `dimos run`. The `--replay` flag feeds it recorded sensor data instead of connecting to a robot:

```bash
dimos --replay run unitree-go2
```

**What you should see:** a [Rerun](https://rerun.io) visualization window opens, and after a short wait it fills with the robot's camera feed, a LiDAR point cloud, and a map of an office being built up as the robot navigates through it, with its planned path drawn on top.

!!! note

    On the first run, roughly **75 MB** of recorded session data is downloaded before anything appears, so the Rerun window may stay black for a minute or two. That is normal. If it stays black well beyond the download, check the terminal output for errors.


Congratulations, you have a full dimOS navigation stack running on recorded data. Everything below is optional and independent, so pick whichever branch matches your goal.

## Next steps

### Simulation (MuJoCo)

Instead of replaying recorded data, you can run the robot in a physics simulation. Install the `sim` extra and pass `--simulation`:

```bash
uv pip install 'dimos[base,unitree,sim]'
dimos --simulation run unitree-go2       # quadruped
dimos --simulation run unitree-g1-sim    # humanoid
```

### Real robot

With a Unitree Go2 on the same network, point dimOS at its IP address and drop the `--replay` flag:

```bash
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

!!! warning

    Before driving real hardware, read the [Unitree Go2 platform guide](/docs/platforms/quadruped/go2/index.md). It covers network setup, latency, time sync, and the safety habits that keep you and the robot out of trouble. Do not skip it.


### LLM agent

The agentic blueprints add an LLM agent that controls the robot through natural language. The default agent uses OpenAI's `gpt-4o`, so you need an `OPENAI_API_KEY` in your environment before starting it. Other providers and local models are covered in the [agents guide](/docs/capabilities/agents/index.md).

```bash
export OPENAI_API_KEY=<YOUR_KEY>
dimos --simulation run unitree-go2-agentic --daemon   # run in the background
dimos agent-send "explore the room"                   # talk to the agent
```

Every robot skill is also exposed over MCP, which means external tools and coding agents can call them directly:

```bash
dimos mcp list-tools
dimos mcp call move_to --arg x=0.5 --arg relative=true
```

Manage the background run with `dimos status`, `dimos log -f`, and `dimos stop`. The full command reference is in the [CLI guide](/docs/usage/cli.md).

## More blueprints to try

| Command | What it does |
| --- | --- |
| `dimos --replay run unitree-go2` | Quadruped navigation replay with SLAM, costmap, and A-star planning |
| `dimos --replay --replay-db go2_bigoffice run unitree-go2-memory` | Quadruped spatial memory replay |
| `dimos --simulation run unitree-go2-agentic` | Quadruped LLM agent plus MCP server in simulation (needs `OPENAI_API_KEY`) |
| `dimos --simulation run unitree-g1-sim` | Humanoid in MuJoCo simulation |
| `dimos --replay run drone-basic` | Drone video and telemetry replay |
| `dimos --replay run drone-agentic` | Drone with an LLM agent and flight skills, replayed |
| `dimos run demo-camera` | Webcam demo, no robot needed |
| `dimos run keyboard-teleop-xarm7` | Keyboard teleop with a mock xArm7 (needs the `manipulation` extra) |
| `dimos --simulation run unitree-go2-agentic-ollama` | Quadruped agent using a local LLM (needs Ollama running via `ollama serve`) |

To learn how blueprints are composed, or to write your own, see the [blueprints guide](/docs/usage/blueprints.md).

## What next?

<div class="grid cards" markdown>

-   [**Add an LLM agent**](/docs/capabilities/agents/index.md)

    Natural language control, agent configuration, and MCP-exposed skills.

-   [**Pick your platform**](/docs/platforms/quadruped/go2/index.md)

    Hardware support and bring-up guides for the Go2 quadruped and G1 humanoid.

-   [**Core concepts**](/docs/usage/index.md)

    Modules, streams, and blueprints, the building blocks behind every workflow.

-   [**Capabilities**](/docs/capabilities/navigation/index.md)

    Navigation, perception, spatial memory, and manipulation in depth.

</div>
