# Unitree Go2

- [Setup your Dog](/docs/platforms/quadruped/go2/setup.md): requirements, install, and connecting to your Go2
- [Simulation](/docs/platforms/quadruped/go2/simulation.md): try it with no hardware via replay or MuJoCo
- [Mapping & Navigation](/docs/capabilities/navigation/index.md): live nav, premap recording, and relocalization

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `dimos run unitree-go2` | Full navigation stack on the real robot (`ROBOT_IP`) |
| `dimos run unitree-go2 --simulation mujoco` | Same stack on dim-go2-sim's MuJoCo Go2 |
| `dimos run unitree-go2 --replay.db <x.db\|x.mcap>` | Same stack on a recording |
| `dimos --record run unitree-go2` | Navigation + record every stream to `recordings/<run-id>/memory.db` ([Recording](/docs/usage/recording.md)) |
| `dimos run unitree-go2 --localmaprelocalization.map-file=<map>` | Navigation relocalized on a saved map (`<map>.pc2.lcm`) |

## Deep Dive

- [Navigation overview](/docs/capabilities/navigation/index.md): live mapping vs premap relocalization
- [Navigation stack](/docs/capabilities/navigation/deep_dive.md): column-carving voxel mapping, costmap generation, A* planning
- [Mapping and relocalization](/docs/capabilities/navigation/index.md): relocalizing on a saved map
- [Visualization](/docs/usage/visualization.md): Rerun, performance tuning
- [Data Streams](/docs/usage/data_streams/index.md): RxPY streams, backpressure, quality filtering
- [Transports](/docs/usage/transports/index.md): Zenoh, LCM, DDS
- [Blueprints](/docs/usage/blueprints.md): composing modules
