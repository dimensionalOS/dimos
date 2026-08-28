# Welcome to dimOS

**dimOS** is an open source operating system for generalist robotics, built by Dimensional. Write one Python program and run it on a quadruped, a humanoid, a drone, or an arm.

dimOS is agent native. Every skill a robot exposes is also an MCP tool. An LLM or a coding agent such as Claude Code discovers those tools and calls them directly, with no glue code to write.

## The problem

Every robot ships its own SDK. Code written for one rarely survives the move to another. ROS is the usual way out, but you take on its build system and its runtime whole, before writing any behaviour of your own. Most of it also needs the hardware in front of you, so your work waits on a robot that is shared, remote, or still in a box.

## How dimOS solves it

- **Behaviour lives in modules, not in an SDK.** A module does one job. It declares the streams it needs as typed fields, so a module written for a quadruped runs on an arm once something publishes what it asked for.

- **No workspace to build.** dimOS installs with pip on your laptop and reaches the robot over the network. Streams travel over LCM, shared memory, DDS, Zenoh, or ROS 2. You pick the transport per stream, and ROS is one option rather than the foundation.

- **Nothing waits on hardware.** The same blueprint runs against a recorded session or in MuJoCo before it runs on a robot. The behaviour you debug is the behaviour that ships.

- **Agents are modules too.** An LLM subscribes to the same streams your control loops use. It calls the same skills, and it takes its instructions in plain language.

<div class="grid cards dim-cta" markdown>

-   :material-rocket-launch: **[Quickstart](/docs/quickstart.md)**

    Install dimOS and replay a real robot session on your laptop in a few minutes. No robot needed.

</div>

## Capabilities

Real robots, running dimOS.

<div class="grid cards dim-media" markdown>

-   [![Navigation and mapping](assets/readme/navigation.gif)](https://x.com/stash_pomichter/status/2010471593806545367)

    **[Navigation and mapping](/docs/capabilities/navigation/index.md)**

    SLAM, dynamic obstacle avoidance, route planning, and autonomous exploration.

-   [![Agentic control](assets/readme/agentic_control.gif)](https://x.com/stash_pomichter/status/2015912688854200322)

    **[Agentic control and MCP](/docs/capabilities/agents/index.md)**

    Talk to the robot. Every skill is also exposed as an MCP tool.

-   [![Spatial memory](assets/readme/spatial_memory.gif)](https://x.com/stash_pomichter/status/1980741077205414328)

    **[Spatial memory](/docs/capabilities/memory/index.md)**

    Spatio-temporal RAG, object localization, and permanence. Navigate back to what the robot saw.

-   <a href="https://x.com/dimensionalos/status/2077476353960722507"><video src="assets/readme/teleop.mp4" autoplay loop muted playsinline preload="metadata" aria-label="dimTELE: remote control any robot from anywhere in the world"></video></a>

    **[Remote teleoperation](/docs/capabilities/teleoperation/hosted.md)**

    **dimTELE** gives you remote control of any robot, from anywhere in the world, at ultra-low latency.

-   [![Perception](assets/readme/perception.png)](/docs/capabilities/perception/index.md)

    **[Perception](/docs/capabilities/perception/index.md)**

    Detect objects in camera, place them in 3D, with VLMs and audio built in.

-   <a href="https://x.com/swstica/status/2092438167618261161"><video src="assets/readme/manipulation.mp4" autoplay loop muted playsinline preload="metadata" aria-label="A robot arm scanning a scene, planning a grasp, and dropping a bottle into a container on spoken instruction"></video></a>

    **[Manipulation](/docs/capabilities/manipulation/index.md)**

    Scan the scene, generate grasp candidates, plan, and execute from a spoken instruction.

</div>

## Start here

<div class="grid cards dim-tiles" markdown>

-   :material-rocket-launch: **[Quickstart](/docs/quickstart.md)**

    Replay a real robot session on your laptop. No robot needed.

-   :material-chip: **[Platforms](/docs/platforms/quadruped/go2/index.md)**

    Unitree Go2 and G1 setup, in simulation and on real hardware.

</div>
