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

import platform
from typing import Any

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.booster.k1.connection import K1Connection
from dimos.visualization.vis_module import vis_module

# High-bandwidth camera frames go over shared memory (esp. needed on macOS UDP).
_mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    ("color_image", Image): pSHMTransport(
        "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
}

_transports_base = (
    autoconnect() if platform.system() == "Linux" else autoconnect().transports(_mac_transports)
)


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _k1_rerun_blueprint() -> Any:
    """Camera feed + 3D world view side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config = {
    "blueprint": _k1_rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
    },
    "max_hz": {
        "world/color_image": 0,
    },
}

_with_vis = autoconnect(
    _transports_base,
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)

booster_k1_basic = autoconnect(
    _with_vis,
    K1Connection.blueprint(),
).global_config(n_workers=4, robot_model="booster_k1")

__all__ = [
    "rerun_config",
    "booster_k1_basic",
]
