"""Run the ZED Mini → voxel map → OccupancyGrid costmap pipeline.

Usage:
    python -m dimos.navigation.camera_nav.run_zed_costmap

What runs:
    ZEDCamera (VGA, NEURAL depth, VIO tracking)
    → ZEDNavBridge (depth backproject → world-frame cloud + odometry)
    → VoxelGridMapper (0.05 m voxel hash map, column carving)
    → CostMapper (height-gradient OccupancyGrid, 0.05 m/cell)
    → Rerun web viewer (open browser at http://localhost:9090)

The OccupancyGrid emitted on world/global_costmap is ready for
ReplanningAStarPlanner — add a goal source to complete the nav stack.
"""

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.camera_nav.blueprint_zed_costmap import zed_costmap_pipeline

if __name__ == "__main__":
    ModuleCoordinator.build(zed_costmap_pipeline).loop()
