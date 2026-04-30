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

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.sim.bridge import sim_bridge
from dimos.robot.sim.tf_module import sim_tf
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


class _SimLCM(LCM):  # type: ignore[misc]
    """LCM that JPEG-decodes image topics and standard-decodes everything else."""

    _JPEG_TOPICS = frozenset({"/color_image"})

    def decode(self, msg: bytes, topic: Any) -> Any:  # type: ignore[override]
        topic_str = getattr(topic, "topic", "") or ""
        # topic.topic may include type suffix: "/color_image#sensor_msgs.Image"
        bare_topic = topic_str.split("#")[0]
        if bare_topic in self._JPEG_TOPICS:
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
    #   camera/color_image                 — 2D view outside the 3D world tree
    #   world/tf/camera_optical/image      — 3D view (child of pinhole)
    rerun_data = image.to_rerun()
    return [
        ("camera/color_image", rerun_data),
        ("world/tf/camera_optical/image", rerun_data),
    ]


def _convert_depth_image(image: Any) -> Any:
    # Keep depth in the 2D pane only.
    # Logging it under the pinhole makes rerun reconstruct a 3D depth wedge,
    # which is visually noisy and can be mistaken for a map artifact.
    rerun_data = image.to_rerun()
    return [("camera/depth_image", rerun_data)]


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(voxel_size=0.1, mode="boxes")


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _suppress(_msg: Any) -> None:
    return None


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.3, 0.15, 0.12],
            colors=[(0, 180, 255)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _static_camera_pinhole(rr: Any) -> list[Any]:
    """Log pinhole at rerun startup — ensures it exists before the first image."""
    import math
    import os

    fov_deg = int(os.environ.get("DIMSIM_CAMERA_FOV", "46"))
    w, h = 640, 288
    fx = (w / 2) / math.tan(math.radians(fov_deg) / 2)
    return [
        rr.Pinhole(
            focal_length=[fx, fx],
            principal_point=[w / 2, h / 2],
            width=w,
            height=h,
            image_plane_distance=1.0,
        ),
        rr.Transform3D(parent_frame="tf#/camera_optical"),
    ]


def _sim_rerun_blueprint() -> Any:
    """Show color + depth feeds next to the 3D world view."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="camera/color_image", name="Camera"),
                rrb.Spatial2DView(origin="camera/depth_image", name="Depth"),
                row_shares=[1, 1],
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.2),
                ),
            ),
            column_shares=[1, 2],
        ),
    )


rerun_config = {
    "blueprint": _sim_rerun_blueprint,
    "pubsubs": [_SimLCM()],
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/color_image": _convert_color_image,
        "world/depth_image": _convert_depth_image,
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
        "world/pointcloud": _suppress,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
        "world/tf/camera_optical": _static_camera_pinhole,
    },
}

# Skip rerun/foxglove bridges in headless mode (CI, e2e tests). Without this,
# RerunBridgeModule fails to find the viewer binary in PATH and errors at start.
_headless = os.environ.get("DIMSIM_HEADLESS", "").strip() in ("1", "true")

if not _headless and global_config.viewer == "foxglove":
    from dimos.robot.foxglove_bridge import FoxgloveBridge

    with_vis = autoconnect(
        _transports_base,
        FoxgloveBridge.blueprint(shm_channels=["/color_image#sensor_msgs.Image"]),
    )
elif not _headless and global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    with_vis = autoconnect(
        _transports_base,
        RerunBridgeModule.blueprint(**rerun_config),
    )
else:
    with_vis = _transports_base

sim_basic = autoconnect(
    with_vis,
    sim_bridge(),
    sim_tf(),
    WebsocketVisModule.blueprint(),
).global_config(n_workers=4, robot_model="dimsim")

__all__ = ["sim_basic"]
