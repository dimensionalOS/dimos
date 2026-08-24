---
title: "Welcome to dimOS"
description: "An open source operating system for generalist robotics. Python-first, ROS-optional, and agent native."
---

# Welcome to dimOS

**dimOS** is a Python framework for generalist robotics, built by Dimensional. You install it on your laptop with pip, and it talks to robots over the network. ROS is optional.

<div class="grid cards" markdown>

-   **Write in Python**

    One codebase for humanoids, quadrupeds, drones, and arms.

-   **Laptop first**

    Develop against recorded sessions and simulation before any hardware.

-   **Agent native**

    LLM agents are modules: they see streams, call skills, and take natural language.

</div>

!!! note

    No robot or GPU needed to try it. The [quickstart](/docs/quickstart.md) replays a recorded session on your laptop in a few minutes.

## See it in action

Agents on real hardware, driven by natural language.

<div class="grid cards" markdown>

-   [**Drone follow**](https://x.com/stash_pomichter/status/2029755300509827184)

    *"Follow the next white car that comes through the intersection"*, a MAVLink drone agent on X.

-   [**Office patrol**](https://x.com/stash_pomichter/status/2042035279708778791)

    *"Patrol the office… if you see someone in a hoodie, follow them"*, an autonomous Go2 agent on X.

</div>

## Capabilities at a glance

<div class="grid cards" markdown>

-   [**Navigation & mapping**](/docs/capabilities/navigation/index.md)

    Map a space while driving, avoid obstacles, and plan routes, with native modules or ROS.

-   [**Remote teleoperation**](/docs/capabilities/teleoperation/hosted.md)

    **dimTELE**: low-latency WebRTC from a browser, phone, or VR headset. No port forwarding.

-   [**Agents**](/docs/capabilities/agents/index.md)

    Natural-language control, with every skill also exposed as an MCP tool.

-   [**Spatial memory**](/docs/capabilities/memory/index.md)

    Remember what was seen and where, then navigate back to it.

-   [**Manipulation**](/docs/capabilities/manipulation/index.md)

    Motion planning and teleop for arms like xArm7, A-750, and OpenArm.

-   [**Perception**](/docs/capabilities/perception/index.md)

    Detect objects in camera, place them in 3D, with VLMs and audio built in.

</div>

## Start here

<div class="grid cards" markdown>

-   [**Quickstart**](/docs/quickstart.md)

    Replay a real robot session on your laptop. No robot needed.

-   [**Platforms**](/docs/platforms/quadruped/go2/index.md)

    Unitree Go2 and G1 setup for simulation and real hardware.

</div>
