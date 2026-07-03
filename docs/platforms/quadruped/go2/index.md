---
title: "Unitree Go2"
---

- [Setup your Dog](/platforms/quadruped/go2/setup) — requirements, install, connecting to your Go2, and agentic control
- [Simulation](/platforms/quadruped/go2/simulation) — try it with no hardware via replay or MuJoCo
- [Mapping & Navigation](/capabilities/navigation/index) — map, premap, relocalize on replay or a live Go2

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `dimos run unitree-go2-basic` | Connection + visualization (no navigation) |
| `dimos run unitree-go2` | Full navigation stack |
| `dimos run unitree-go2-agentic` | Navigation + LLM agent + MCP tool access |
| `dimos run unitree-go2-agentic-ollama` | Agent with local Ollama models |
| `dimos run unitree-go2-spatial` | Navigation + spatial memory |
| `dimos run unitree-go2-detection` | Navigation + object detection |

## Deep Dive

- [Navigation Stack](/capabilities/navigation/deep_dive) — column-carving voxel mapping, costmap generation, A* planning
- [Visualization](/usage/visualization) — Rerun, performance tuning
- [Data Streams](/usage/data_streams) — RxPY streams, backpressure, quality filtering
- [Transports](/usage/transports/index) — LCM, SHM, DDS
- [Blueprints](/usage/blueprints) — composing modules
