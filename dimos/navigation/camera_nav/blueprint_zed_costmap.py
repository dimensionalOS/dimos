"""ZED Mini → world-frame voxel map → OccupancyGrid costmap.

Full pipeline using existing dimos modules — no reimplementation:

  ZEDCamera          stereo depth + colour + VIO tracking
      ↓ depth_image, color_image, depth_camera_info, TF
  ZEDNavBridge       backproject depth → world-frame PointCloud2 + Odometry
      ↓ lidar (PointCloud2 world-frame)
  VoxelGridMapper    hash-map voxel accumulation with column carving
      ↓ global_map (PointCloud2 world-frame, 0.05 m voxels)
  CostMapper         height-gradient → OccupancyGrid (0.05 m/cell, −1/0/1-100)
      ↓ global_costmap (OccupancyGrid)
  RerunBridgeModule  web viewer: current frame + growing map + costmap

The costmap is ready to feed ReplanningAStarPlanner — plug in a goal source
and the planner to complete the nav stack.

Usage (via run_zed_costmap.py):
  python -m dimos.navigation.camera_nav.run_zed_costmap

Key design choices:
  enable_fill_mode=False  no interpolation at depth edges (kills white speckles)
  base_transform=None     ZED publishes world→camera_link directly (1-hop TF)
                          ZEDNavBridge resolves it without an extra robot base frame
  VoxelGridMapper device  CUDA:0 with automatic CPU:0 fallback (macOS safe)
  carve_columns=True      erases stale voxels in columns the camera now sees as free
  initial_safe_radius=1.0 first metre around start is marked free so planner has
                          a clear zone to begin planning from
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.camera_nav.viz import cloud_points, costmap_viz, pinhole_setup
from dimos.navigation.camera_nav.zed_nav_bridge import ZEDNavBridge
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule

_RERUN = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "world/lidar":        cloud_points,    # current depth frame (ZEDNavBridge)
        "world/global_map":   cloud_points,    # growing voxel map (VoxelGridMapper)
        "world/global_costmap": costmap_viz,   # height-cost grid (CostMapper)
        "world/camera_info":  pinhole_setup,   # camera model overlay
    },
)

zed_costmap_pipeline = autoconnect(
    ZEDCamera.blueprint(
        resolution="VGA",
        enable_depth=True,
        depth_mode="NEURAL",
        enable_fill_mode=False,     # no edge interpolation — prevents white speckles
        enable_pointcloud=False,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
        base_transform=None,        # world→camera_link direct; ZEDNavBridge handles it
    ),
    ZEDNavBridge.blueprint(
        camera_name="camera",
        stride=2,                   # half-res backprojection keeps PointCloud2 small
        max_depth=8.0,
        max_freq=10.0,
        tf_timeout=1.0,
    ),
    VoxelGridMapper.blueprint(
        voxel_size=0.05,
        device="CUDA:0",            # auto-falls back to CPU:0 on macOS
        carve_columns=True,
        emit_every=1,
    ),
    CostMapper.blueprint(
        initial_safe_radius_meters=1.0,
    ),
    _RERUN,
)
