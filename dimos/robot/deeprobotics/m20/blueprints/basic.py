#!/usr/bin/env python3
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

"""Basic Lynx M20 blueprint: front-camera video + a Rerun viewer.

``MovementManager`` muxes movement sources (``nav_cmd_vel`` / ``tele_cmd_vel`` /
``clicked_point``) into the single ``cmd_vel`` the connection consumes; wire a
teleop or nav source into the manager's inputs to drive.
"""

import platform
from typing import Any

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.visualization.vis_module import vis_module

# Images are large; keep them on shared memory rather than UDP (esp. on macOS).
_image_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    (name, Image): pSHMTransport(name, default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE)
    for name in ("color_image", "color_image_rear")
}

_transports_base = (
    autoconnect() if platform.system() == "Linux" else autoconnect().transports(_image_transports)
)


def _m20_rerun_blueprint() -> Any:
    """Front + rear wide-angle cameras, side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="M20 Front"),
            rrb.Spatial2DView(origin="world/color_image_rear", name="M20 Rear"),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config = {
    "blueprint": _m20_rerun_blueprint,
    "max_hz": {
        "world/color_image": 0,
        "world/color_image_rear": 0,
    },
}

_with_vis = autoconnect(
    _transports_base,
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)

m20_basic = autoconnect(
    _with_vis,
    M20Connection.blueprint(ip="m20"),
    MovementManager.blueprint(),
).global_config(n_workers=3)


m20_onboard = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            "max_hz": {"world/grid_map_3d": 1, "world/aligned_points": 1},
        },
    ),
).global_config(n_workers=2)


__all__ = [
    "m20_basic",
    "m20_onboard",
    "rerun_config",
]
