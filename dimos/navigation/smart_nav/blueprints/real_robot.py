# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real robot blueprint: runs on physical hardware with FastLio2 SLAM.

Uses the existing dimos FastLio2 NativeModule for SLAM with a Livox Mid-360
lidar, replacing the Unity simulator with real sensor data.

FastLio2 outputs ``lidar`` (not ``registered_scan``), so we remap it.
No camera ports — lidar-only setup.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smart_nav.modules.tui_control.tui_control import TUIControlModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.vis_module import vis_module


def _rerun_blueprint() -> Any:
    """Rerun layout for lidar-only (no camera panel)."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def _terrain_map_override(cloud: Any) -> Any:
    """Render terrain_map colored by obstacle cost (intensity field)."""
    return cloud.to_rerun(colormap="turbo", size=0.04)


rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "visual_override": {
        "world/terrain_map": _terrain_map_override,
    },
}


def make_real_robot_blueprint(
    host_ip: str = "192.168.1.5",
    lidar_ip: str = "192.168.1.155",
):
    """Create a real robot blueprint with configurable network settings."""
    return autoconnect(
        FastLio2.blueprint(host_ip=host_ip, lidar_ip=lidar_ip),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(),
        LocalPlanner.blueprint(),
        PathFollower.blueprint(),
        TUIControlModule.blueprint(),
        vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    ).remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
        ]
    )


real_robot_blueprint = make_real_robot_blueprint()


def main() -> None:
    real_robot_blueprint.build().loop()


if __name__ == "__main__":
    main()
