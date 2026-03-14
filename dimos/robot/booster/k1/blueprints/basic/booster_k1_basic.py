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

"""Basic Booster K1 blueprint: connection + visualization."""

import platform

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.booster.k1.connection import k1_connection
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

_mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    ("color_image", Image): pSHMTransport(
        "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
}

_transports_base = (
    autoconnect() if platform.system() == "Linux" else autoconnect().transports(_mac_transports)
)


def _k1_blueprint():
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.2),
                ),
            ),
            rrb.Spatial2DView(
                name="Camera",
                origin="world/color_image",
            ),
        ),
    )


rerun_config = {
    "pubsubs": [LCM(autoconf=True)],
    "blueprint": _k1_blueprint,
    "visual_override": {
        "world/camera_info": lambda camera_info: camera_info.to_rerun(
            image_topic="/world/color_image",
            optical_frame="camera_optical",
        ),
    },
}

match global_config.viewer_backend:
    case "rerun":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(_transports_base, rerun_bridge(**rerun_config))
    case "rerun-web":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(_transports_base, rerun_bridge(viewer_mode="web", **rerun_config))
    case _:
        with_vis = autoconnect(_transports_base)

booster_k1_basic = autoconnect(
    with_vis,
    k1_connection(),
    websocket_vis(),
).global_config(n_dask_workers=4, robot_model="booster_k1")

__all__ = ["booster_k1_basic"]
