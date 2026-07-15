---
title: "Go2 Navigation Overview"
description: "Live column-carving navigation and premap relocalization for the Unitree Go2."
---

The Go2 navigation stack uses a simple **column-carving voxel map** strategy: each new LiDAR frame replaces the corresponding region of the global map entirely, ensuring the map always reflects the latest observations. Map live as you drive, or return to a known space using a saved premap and relocalization.

![Live Go2 navigation in Rerun](assets/noros_nav.gif)

## Choose your workflow

| Workflow | When to use | Blueprint | Docs |
|----------|-------------|-----------|------|
| **Live mapping** | Explore a new space where the map updates every frame | `unitree-go2` | [Navigation deep dive](/docs/capabilities/navigation/deep_dive.md) |
| **Premap + relocalization** | Return to a known space and plan on a loop-closed map | `unitree-go2-relocalization` | [Relocalization](/docs/capabilities/navigation/relocalization.md) |

Live column-carving maps are fast and reactive, but odometry drifts over long distances. For spaces you revisit, record once, run pose-graph optimization (PGO) offline, then relocalize against the exported premap at runtime.

For hardware setup, simulation, and the full blueprint list, see the [Go2 platform guide](/docs/platforms/quadruped/go2/index.md).
