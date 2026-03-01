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

"""Basic DimSim blueprint — connection + visualization."""

import platform

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.sim.bridge import sim_bridge
from dimos.robot.sim.tf_module import sim_tf
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

_mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    ("color_image", Image): pSHMTransport(
        "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
}

_transports_base = (
    autoconnect() if platform.system() == "Linux" else autoconnect().transports(_mac_transports)
)

rerun_config = {
    "pubsubs": [LCM(autoconf=True)],
    "visual_override": {
        "world/camera_info": lambda camera_info: camera_info.to_rerun(
            image_topic="/world/color_image",
            optical_frame="camera_optical",
        ),
        "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1, mode="boxes"),
        "world/navigation_costmap": lambda grid: grid.to_rerun(
            colormap="Accent",
            z_offset=0.015,
            opacity=0.2,
            background="#484981",
        ),
        "world/pointcloud": None,
        "world/depth_image": None,
    },
    "static": {
        "world/tf/base_link": lambda rr: [
            rr.Boxes3D(
                half_sizes=[0.3, 0.15, 0.12],
                colors=[(0, 180, 255)],
            ),
            rr.Transform3D(parent_frame="tf#/base_link"),
        ]
    },
}

match global_config.viewer_backend:
    case "foxglove":
        from dimos.robot.foxglove_bridge import foxglove_bridge

        with_vis = autoconnect(
            _transports_base,
            foxglove_bridge(shm_channels=["/color_image#sensor_msgs.Image"]),
        )
    case "rerun":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(_transports_base, rerun_bridge(**rerun_config))
    case "rerun-web":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(_transports_base, rerun_bridge(viewer_mode="web", **rerun_config))
    case _:
        with_vis = _transports_base

sim_basic = autoconnect(
    with_vis,
    sim_bridge(),
    sim_tf(),
    websocket_vis(),
).global_config(n_dask_workers=4, robot_model="dimsim")

__all__ = ["sim_basic"]
