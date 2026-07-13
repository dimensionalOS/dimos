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

"""xArm6 WorldBelief perception stack."""

from __future__ import annotations

from functools import partial
from typing import Any, cast

import rerun.blueprint as rrb

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.worldbelief_module import WorldBeliefModule
from dimos.perception.worldbelief_recorder import WorldBeliefRecorder
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config, xarm6_hardware
from dimos.visualization.rerun.bridge import RerunBridgeModule

XARM6_WORLDBELIEF_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),
)


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    return {
        "/color_image": "world/color_camera/color_image",
        "/camera_info": "world/color_camera",
        "/depth_image": "world/depth_camera/depth_image",
        "/depth_camera_info": "world/depth_camera",
        "/detections_3d": "world/detections_3d",
        "/pointcloud": "world/pointcloud",
    }.get(topic_name, f"world/{topic_name.lstrip('/')}")


def _camera_info_to_rerun(msg: Any, image_topic: str) -> list[tuple[str, Any]]:
    return cast(
        "list[tuple[str, Any]]",
        msg.to_rerun(image_topic=image_topic, optical_frame=getattr(msg, "frame_id", None)),
    )


def _rerun_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_camera/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
        )
    )


_hw = xarm6_hardware("arm")
_hw.auto_enable = True

xarm6_worldbelief = autoconnect(
    # Provides wrist-camera FK/TF.
    ManipulationModule.blueprint(
        robots=[
            make_xarm6_model_config(
                name="arm",
                add_gripper=False,
                # Enables TF publication.
                tf_extra_links=["link_base"],
            ),
        ],
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
    WorldBeliefRecorder.blueprint(
        db_path=STATE_DIR / "worldbelief" / "xarm6" / "recordings" / "xarm6_worldbelief.db",
    ),
    WorldBeliefModule.blueprint(
        db_path=STATE_DIR / "worldbelief" / "xarm6" / "recordings" / "xarm6_worldbelief.db",
        history_path=STATE_DIR / "worldbelief" / "xarm6" / "worldbelief_history.db",
        scan_prompts=[],
        depth_tolerance_s=0.1,
        stationary_hz=4.0,
        yoloe_model_name="yoloe-11l-seg.pt",
        dino_model_name="facebook/dinov2-base",
        clip_model_name="openai/clip-vit-base-patch32",
    ),
    McpServer.blueprint(),
    coordinator(
        hardware=[_hw],
        tasks=[trajectory_task(_hw)],
    ),
).global_config(n_workers=8)
