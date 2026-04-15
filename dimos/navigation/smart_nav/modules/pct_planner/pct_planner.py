"""PCTPlanner NativeModule: C++ 3D point-cloud-tomography route planner.

Ported from PCT_planner (point cloud tomography + A* + GPMP). Slices an
explored-area point cloud into traversability layers, plans across floors
with A*, smooths the result with GPMP, and publishes lookahead waypoints
for the local planner.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PCTPlannerConfig(NativeModuleConfig):
    """Config for the PCT planner native module."""

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/pct_planner"
    # Local nix build during development. Switch to a github URL for release.
    build_command: str | None = (
        "nix build path:$HOME/repos/dimos-module-pct-planner --no-write-lock-file"
    )

    # Loop / frame
    update_rate: float = 5.0
    frame_id: str = "map"

    # Tomogram grid
    resolution: float = 0.075
    slice_dh: float = 0.4
    slope_max: float = 0.45
    step_max: float = 0.5
    cost_barrier: float = 100.0
    kernel_size: int = 11
    safe_margin: float = 0.3
    inflation: float = 0.2
    interval_min: float = 0.5
    interval_free: float = 0.65
    standable_ratio: float = 0.5

    # Waypoint follower
    lookahead_distance: float = 1.25


class PCTPlanner(NativeModule):
    """PCT (Point Cloud Tomography) planner: 3D multi-floor global route planner.

    Rebuilds a tomogram every time it receives a new explored-areas point cloud
    and plans across floors with A* + GPMP. Publishes lookahead waypoints at
    ``update_rate`` for the local planner to follow.

    Ports:
        explored_areas (In[PointCloud2]): Accumulated mapped point cloud.
        odometry (In[Odometry]): Vehicle state.
        goal (In[PointStamped]): Navigation goal.
        way_point (Out[PointStamped]): Lookahead waypoint.
        goal_path (Out[NavPath]): Full planned path.
        tomogram (Out[PointCloud2]): Tomogram visualization.
    """

    config: PCTPlannerConfig

    explored_areas: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
    goal_path: Out[NavPath]
    tomogram: Out[PointCloud2]
