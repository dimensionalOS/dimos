---
myst:
  substitutions:
    live_go2_navigation_in_rerun: |-
      ```{image} https://raw.githubusercontent.com/dimensionalOS/dimos-docs-assets/main/capabilities/navigation/assets/noros_nav.gif
      ```
---

(doc-capabilities-navigation-index-go2-navigation-overview)=

# Go2 Navigation Overview

The Go2 navigation stack uses a simple **column-carving voxel map** strategy: each new LiDAR frame replaces the corresponding region of the global map entirely, ensuring the map always reflects the latest observations. Map live as you drive, or return to a known space using a saved premap and relocalization.

{{ live_go2_navigation_in_rerun }}

(doc-capabilities-navigation-index-choose-your-workflow)=

## Choose your workflow

:::{list-table}
   :header-rows: 1

   * - Workflow
     - When to use
     - Blueprint
     - Guide
   * - **Live mapping**
     - Explore a new space while the map updates every frame
     - ``unitree-go2``
     - [Navigation deep dive](deep_dive.md)
   * - **Premap and relocalization**
     - Return to a known space and plan on a loop-closed map
     - ``unitree-go2-relocalization``
     - [Relocalization](relocalization.md)
:::

Live column-carving maps are fast and reactive, but odometry drifts over long distances. For spaces you revisit, record once, run pose-graph optimization (PGO) offline, then relocalize against the exported premap at runtime.

For hardware setup, simulation, and the full blueprint list, see the [Go2 platform guide](../../platforms/quadruped/go2/index.md).

```{toctree}
:hidden: true
:maxdepth: 1

deep_dive
relocalization
```
