"""Runs on the machine wired to both the Mid-360 and the D435i (same rig as
dimos/robot/assembly/mid360_realsense_30.py): per-frame lidar-vs-stereo benchmark.

Compares three point clouds every `period_s` (see BenchConfig):
  - lidar           Mid-360 raw frame (reference / "ground truth")
  - realsense_raw   RealSense SDK-deprojected cloud (no filtering, no odometry)
  - stereo          our StereoPointCloud output (filtered, ICP-aligned to lidar for scoring)

Metrics: Chamfer distance, accuracy/completeness/F-score (ETH3D convention),
voxel occupancy IoU, range-binned point density. Logged to console.

Usage:
    DIMOS_MID360_LIDAR_IP=192.168.1.155 python scripts/lidar_stereo_benchmark.py
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.utils.cli.lidar_stereo_bench import LidarStereoBenchmark
from dimos.perception.stereo_point_cloud.filtered_realsense import FilteredRealSenseCamera
from dimos.perception.stereo_point_cloud.module import StereoPointCloud

lidar_stereo_benchmark = (
    autoconnect(
        Mid360.blueprint(),
        FilteredRealSenseCamera.blueprint(
            enable_depth=True, enable_pointcloud=True, publish_color=False
        ).remappings([
            (FilteredRealSenseCamera, "pointcloud", "realsense_raw"),
        ]),
        StereoPointCloud.blueprint(world_frame="stereo_odom").remappings([
            (StereoPointCloud, "frame_cloud", "stereo"),
        ]),
        LidarStereoBenchmark.blueprint(),
    )
    .global_config(n_workers=4)
)

if __name__ == "__main__":
    ModuleCoordinator.build(lidar_stereo_benchmark).loop()
