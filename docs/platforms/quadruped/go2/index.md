(doc-platforms-quadruped-go2-index-unitree-go2)=

# Unitree Go2

- [Setup your Dog](setup.md) — requirements, install, connecting to your Go2, and agentic control
- [Simulation](simulation.md) — try it with no hardware via replay or MuJoCo
- [Mapping & Navigation](../../../capabilities/navigation/index.md) — live nav, premap recording, and relocalization

(doc-platforms-quadruped-go2-index-available-blueprints)=

## Available Blueprints

| Blueprint                              | Description                                                |
| -------------------------------------- | ---------------------------------------------------------- |
| `dimos run unitree-go2-basic`          | Connection + visualization (no navigation)                 |
| `dimos run unitree-go2`                | Full navigation stack                                      |
| `dimos run unitree-go2-agentic`        | Navigation + LLM agent + MCP tool access                   |
| `dimos run unitree-go2-agentic-ollama` | Agent with local Ollama models                             |
| `dimos run unitree-go2-spatial`        | Navigation + spatial memory                                |
| `dimos run unitree-go2-detection`      | Navigation + object detection                              |
| `dimos run unitree-go2-memory`         | Navigation + record `lidar`/`odom`/`color_image` to `.db`  |
| `dimos run unitree-go2-relocalization` | Navigation + align live scans to a saved `.pc2.lcm` premap |

(doc-platforms-quadruped-go2-index-deep-dive)=

## Deep Dive

- [Navigation overview](../../../capabilities/navigation/index.md) — live mapping vs premap relocalization
- [Navigation stack](../../../capabilities/navigation/deep_dive.md) — column-carving voxel mapping, costmap generation, A\* planning
- [Relocalization](../../../capabilities/navigation/relocalization.md) — record → `dimos map global --export` → replay or live deploy
- [Visualization](../../../usage/visualization.md) — Rerun, performance tuning
- [Data Streams](../../../usage/data_streams/index.md) — RxPY streams, backpressure, quality filtering
- [Transports](../../../usage/transports/index.md) — LCM, SHM, DDS
- [Blueprints](../../../usage/blueprints.md) — composing modules

```{toctree}
:hidden: true
:maxdepth: 1

setup
simulation
```
