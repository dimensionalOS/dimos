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

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.sim.bridge import sim_bridge
from dimos.robot.sim.tf_module import sim_tf
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis


class _SimLCM(LCM):  # type: ignore[misc]
    """LCM that JPEG-decodes image topics and standard-decodes everything else."""

    _JPEG_TOPICS = frozenset({"/color_image"})

    def decode(self, msg: bytes, topic: Any) -> Any:  # type: ignore[override]
        if getattr(topic, "topic", None) in self._JPEG_TOPICS:
            return Image.lcm_jpeg_decode(msg)
        return super().decode(msg, topic)


# DimSim sends JPEG-compressed images over LCM — use JpegLcmTransport to decode.
_transports_base = autoconnect().transports(
    {("color_image", Image): JpegLcmTransport("/color_image", Image)}
)


def _convert_camera_info(camera_info: Any) -> Any:
    # Log pinhole under TF tree (3D frustum) — NOT at the image entity.
    # Pinhole at the image entity blocks rerun's 2D viewer.
    import rerun as rr

    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    return [
        (
            "world/tf/camera_optical",
            rr.Pinhole(
                focal_length=[fx, fy],
                principal_point=[cx, cy],
                width=camera_info.width,
                height=camera_info.height,
                image_plane_distance=1.0,
            ),
        ),
        (
            "world/tf/camera_optical",
            rr.Transform3D(parent_frame="tf#/camera_optical"),
        ),
    ]


def _convert_color_image(image: Any) -> Any:
    # Log image at both:
    #   world/color_image                  — 2D view (no pinhole ancestor)
    #   world/tf/camera_optical/image      — 3D view (child of pinhole)
    rerun_data = image.to_rerun()
    return [
        ("world/color_image", rerun_data),
        ("world/tf/camera_optical/image", rerun_data),
    ]


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(voxel_size=0.1, mode="boxes")


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.3, 0.15, 0.12],
            colors=[(0, 180, 255)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


rerun_config = {
    "pubsubs": [_SimLCM(autoconf=True)],
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/color_image": _convert_color_image,
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
        "world/pointcloud": None,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
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
).global_config(n_workers=4, robot_model="dimsim")

__all__ = ["sim_basic"]
