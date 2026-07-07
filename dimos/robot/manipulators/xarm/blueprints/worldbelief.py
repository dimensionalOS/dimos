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

"""xArm6 WorldBelief blueprint — log-first, pulled perception (no live detection).

No continuous perception module: detection+lift+embeddings run only when asked, over
the memory2 recording. The recorder exposes ``scan`` and ``recall`` as MCP skills
"""

from __future__ import annotations

import math

from functools import partial
from typing import Any

import rerun.blueprint as rrb

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config, xarm6_hardware
from dimos.robot.manipulators.xarm.worldbelief_recorder import XArm6WorldBeliefRecorder
from dimos.visualization.rerun.bridge import RerunBridgeModule


XARM6_WORLDBELIEF_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),
)

XARM6_WORLDBELIEF_HOME_JOINTS = (
    0.0,
    math.radians(-22.3),
    math.radians(-22.3),
    0.0,
    math.radians(-22.3),
    0.0,
)


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    return {
        "/color_image": "world/color_camera/color_image",
        "/camera_info": "world/color_camera",
        "/depth_image": "world/depth_camera/depth_image",
        "/depth_camera_info": "world/depth_camera",
        "/detections_3d": "world/detections_3d",  # published on demand by recorder.scan()
        "/pointcloud": "world/pointcloud",  # detected-object clouds (colored per object id)
    }.get(topic_name, f"world/{topic_name.lstrip('/')}")


def _camera_info_to_rerun(msg: Any, image_topic: str) -> list[tuple[str, Any]]:
    return msg.to_rerun(image_topic=image_topic, optical_frame=getattr(msg, "frame_id", None))


def _rerun_blueprint() -> rrb.Blueprint:
    # no annotated_image (no live detector) -> show the raw color camera + the 3D world
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_camera/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
        )
    )


_hw = xarm6_hardware("arm", gripper=True, home_joints=list(XARM6_WORLDBELIEF_HOME_JOINTS))

xarm6_worldbelief = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm6_model_config(
                name="arm",
                add_gripper=True,
                # full arm chain to tf (matches the original) so the whole arm is visible
                # in `dimos mem rerun` and the recorder captures world->link6->camera.
                tf_extra_links=[
                    "link_base",
                    "link1",
                    "link2",
                    "link3",
                    "link4",
                    "link5",
                    "link6",
                    "link_eef",
                ],
                home_joints=list(XARM6_WORLDBELIEF_HOME_JOINTS),
            ),
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
        floor_z=-0.02,
    ),
    RealSenseCamera.blueprint(
        width=640,
        height=480,
        fps=15,
        base_frame_id="link6",
        base_transform=XARM6_WORLDBELIEF_CAMERA_TRANSFORM,
    ),
    RerunBridgeModule.blueprint(
        blueprint=_rerun_blueprint,
        topic_to_entity=_topic_to_entity,
        visual_override={
            "world/color_camera": partial(
                _camera_info_to_rerun, image_topic="world/color_camera/color_image"
            ),
            "world/depth_camera": partial(
                _camera_info_to_rerun, image_topic="world/depth_camera/depth_image"
            ),
        },
        max_hz={
            "world/color_camera/color_image": 10.0,
            "world/depth_camera/depth_image": 5.0,
            "world/detections_3d": 10.0,
            "world/pointcloud": 5.0,
        },
    ),
    XArm6WorldBeliefRecorder.blueprint(),
    McpServer.blueprint(),
    coordinator(
        hardware=[_hw],
        tasks=[trajectory_task(_hw, name="traj_xarm")],
    ),
).global_config(n_workers=7)
