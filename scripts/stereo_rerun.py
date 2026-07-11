"""Camera → StereoPointCloud → Rerun only. No robot, no nav, no costmap.

Usage:
    python scripts/stereo_rerun.py
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.perception.stereo_point_cloud.filtered_realsense import FilteredRealSenseCamera
from dimos.perception.stereo_point_cloud.module import StereoPointCloud
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

stereo_rerun = autoconnect(
    FilteredRealSenseCamera.blueprint(enable_depth=True, enable_pointcloud=False, publish_color=False),
    StereoPointCloud.blueprint(),
    RerunBridgeModule.blueprint(rerun_open="web"),
    RerunWebSocketServer.blueprint(),
)

if __name__ == "__main__":
    ModuleCoordinator.build(stereo_rerun).loop()
