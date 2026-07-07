---
title: "Unitree Go2"
---

- [Setup your Dog](/docs/platforms/quadruped/go2/setup.md) — requirements, install, connecting to your Go2, and agentic control
- [Simulation](/docs/platforms/quadruped/go2/simulation.md) — try it with no hardware via replay or MuJoCo
- [Mapping & Navigation](/capabilities/navigation/index) — live nav, premap recording, and relocalization

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `dimos run unitree-go2-basic` | Connection + visualization (no navigation) |
| `dimos run unitree-go2` | Full navigation stack |
| `dimos run unitree-go2-agentic` | Navigation + LLM agent + MCP tool access |
| `dimos run unitree-go2-agentic-ollama` | Agent with local Ollama models |
| `dimos run unitree-go2-spatial` | Navigation + spatial memory |
| `dimos run unitree-go2-detection` | Navigation + object detection |
| `dimos run unitree-go2-memory` | Navigation + record `lidar`/`odom`/`color_image` to `.db` |
| `dimos run unitree-go2-relocalization` | Navigation + align live scans to a saved `.pc2.lcm` premap |

## Deep Dive

- [Navigation overview](/capabilities/navigation/index) — live mapping vs premap relocalization
- [Navigation stack](/capabilities/navigation/deep_dive) — column-carving voxel mapping, costmap generation, A* planning
- [Relocalization](/capabilities/navigation/relocalization) — record → `dimos map global --export` → replay or live deploy
- [Visualization](/docs/usage/visualization.md) — Rerun, performance tuning
- [Data Streams](/docs/usage/data_streams) — RxPY streams, backpressure, quality filtering
- [Transports](/docs/usage/transports/index.md) — LCM, SHM, DDS
- [Blueprints](/docs/usage/blueprints.md) — composing modules
