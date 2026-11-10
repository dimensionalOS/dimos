#!/usr/bin/env python3
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

"""Replay-and-mesh blueprint for the Unitree G1.

Replays a recorded G1 SQLite dataset (defaulting to ``g1_sf_office.db`` or
``--replay-db``) via ``G1Replay`` and reconstructs a colored 3D triangle mesh
in real time from the RealSense RGB-D stream via ``RgbdMesh``.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.mapping.experimental.mesh import MESH_ENTITY, MeshColours
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.stereo.rgbd_mesh import RgbdMesh
from dimos.robot.unitree.g1.g1_replay import DEFAULT_G1_DATASET, G1Replay
from dimos.visualization.vis_module import vis_module


def _g1_replay_mesh_rerun_blueprint() -> Any:
    """Split layout: camera + depth feeds on the left, 3D mesh view on the right."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.Spatial2DView(origin="world/realsense_depth_image", name="Depth"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                line_grid=rrb.LineGrid3D(visible=False),
                overrides={
                    "world/realsense_depth_image": rrb.EntityBehavior(visible=False),
                    "world/pointlio_lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
    )


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.20, 0.7],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _convert_camera_info(camera_info: CameraInfo) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame=camera_info.frame_id or "d435_color_optical_frame",
    )


_replay_mesh_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _g1_replay_mesh_rerun_blueprint,
        "visual_override": {
            "world/realsense_camera_info": _convert_camera_info,
            "world/realsense_depth_camera_info": None,
            "world/pointlio_lidar": None,
            MESH_ENTITY: MeshColours(alpha=1.0),
        },
        "static": {"world/robot_body": _static_robot_body},
        "tf_axes": 0.5,
        "memory_limit": "1GB",
    },
)

_rgbd_mesh = RgbdMesh.blueprint().remappings(
    [
        (RgbdMesh, "depth_image", "realsense_depth_image"),
        (RgbdMesh, "camera_info", "realsense_camera_info"),
    ]
)

unitree_g1_replay_mesh = autoconnect(
    G1Replay.blueprint(),
    _rgbd_mesh,
    _replay_mesh_vis,
).global_config(
    n_workers=4,
    robot_model="unitree_g1",
    replay=True,
    replay_db=DEFAULT_G1_DATASET,
)


if __name__ == "__main__":
    coordinator = ModuleCoordinator.build(unitree_g1_replay_mesh)
    coordinator.loop()
