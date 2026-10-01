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

"""Pick camera waypoints by clicking on a premap.

dimos run animation-waypoints --waypointpicker.map-file=<premap stem>
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.animation.waypoints import (
    WaypointPicker,
    render_camera_path,
    render_waypoints,
)
from dimos.visualization.vis_module import vis_module


def _map_points(cloud: PointCloud2) -> Any:
    return cloud.to_rerun(mode="points", ui_radius=1.0)


animation_waypoints = autoconnect(
    WaypointPicker.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/global_map": _map_points,
                "world/waypoints": render_waypoints,
                "world/camera_path": render_camera_path,
            },
        },
    ),
).global_config(n_workers=4)
