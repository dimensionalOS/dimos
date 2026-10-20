# Go2 Navigation Overview

The Go2 navigation stack uses a simple **column-carving voxel map** strategy: each new LiDAR frame replaces the corresponding region of the global map entirely, ensuring the map always reflects the latest observations. Map live as you drive, or return to a known space using a saved premap and relocalization.

![Live Go2 navigation in Rerun](assets/noros_nav.gif)

## go2

| Workflow                    | When to use                                                            | Blueprint                    | Docs                                                               |
|-----------------------------|------------------------------------------------------------------------|------------------------------|--------------------------------------------------------------------|
| **Live mapping**            | Explore a new space, closing loops live to correct drift               | `unitree-go2`                | [Navigation deep dive](/docs/capabilities/navigation/deep_dive.md) |
| **Premap + relocalization** | Return to a known space and plan on a loop-closed map                  | `unitree-go2 --map-file=…`   | [Relocalization](/docs/capabilities/navigation/relocalization.md)  |

`unitree-go2` closes loops live with pose-graph optimization (PGO), and relocalizes against a premap when given `--map-file`. For spaces you return to across sessions, record once, run PGO offline, then relocalize against the exported premap at runtime.

For hardware setup, simulation, and the full blueprint list, see the [Go2 platform guide](/docs/platforms/quadruped/go2/index.md).

## Generic

| Workflow                 | When to use                                                            | Blueprint     | Docs                                                |
|--------------------------|------------------------------------------------------------------------|---------------|-----------------------------------------------------|
| **Simulation, no robot** | Run the raycaster and planner against a photoreal scan of a real house | `habitat-nav` | [Habitat](/docs/capabilities/navigation/habitat.md) |
