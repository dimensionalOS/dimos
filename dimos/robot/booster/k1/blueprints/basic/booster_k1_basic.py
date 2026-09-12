# Copyright 2025-2026 Dimensional Inc.
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

"""Basic Booster K1 blueprint: connection + camera visualization."""

from typing import Any

import rerun.blueprint as rrb

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.protocol.service.system_configurator.clock_sync import ClockSyncConfigurator
from dimos.robot.booster.k1.connection import K1Connection
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _k1_rerun_blueprint() -> rrb.Blueprint:
    """Camera feed + 3D world view side by side."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
    )


rerun_config: dict[str, Any] = {
    "blueprint": _k1_rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
    },
}


booster_k1_basic = (
    autoconnect(
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=rerun_config,
        ),
        K1Connection.blueprint(),
    )
    .global_config(n_workers=4, robot_model="booster_k1")
    .configurators(ClockSyncConfigurator())
)

__all__ = ["booster_k1_basic"]
