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

"""xArm6 WorldBelief hardware perception blueprint."""

from __future__ import annotations

import math
from typing import Any

import rerun.blueprint as rrb

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    make_xarm6_model_config,
    xarm6_hardware,
)
from dimos.robot.manipulators.xarm.worldbelief_recorder import (
    XArm6WorldBeliefRecorder,
    xarm6_worldbelief_history_path,
)
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
    math.radians(-20.2),
    0.0,
)


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    return {
        "/annotated_image": "world/color_camera/annotated_image",
        "/color_image": "world/color_camera/color_image",
        "/camera_info": "world/color_camera",
        "/depth_image": "world/depth_camera/depth_image",
        "/depth_camera_info": "world/depth_camera",
        "/detections_2d": "world/detections_2d",
        "/detections_3d": "world/detections_3d",
        "/pointcloud": "world/pointcloud",
    }.get(topic_name, f"world/{topic_name.lstrip('/')}")


def _color_camera_info_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    optical_frame = getattr(msg, "frame_id", None)
    return [
        *msg.to_rerun(image_topic="world/color_camera/color_image", optical_frame=optical_frame),
        *msg.to_rerun(
            image_topic="world/color_camera/annotated_image", optical_frame=optical_frame
        ),
    ]


def _depth_camera_info_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    optical_frame = getattr(msg, "frame_id", None)
    return msg.to_rerun(image_topic="world/depth_camera/depth_image", optical_frame=optical_frame)


def _rerun_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_camera/annotated_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
        )
    )


_xarm6_worldbelief_hw = xarm6_hardware(
    "arm",
    gripper=True,
    home_joints=list(XARM6_WORLDBELIEF_HOME_JOINTS),
)

xarm6_worldbelief = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm6_model_config(
                name="arm",
                add_gripper=True,
                tf_extra_links=["link6"],
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
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        belief_engine="world_belief",
        prompt_mode="prompt",
        text_prompts=["object", "container", "cup", "bottle", "can", "box"],
        detector_model_name="yoloe-11s-seg.pt",
        detector_conf=0.25,
        detector_iou=0.8,
        detector_max_det=20,
        detector_device="cuda",
        detector_tracking_enabled=False,
        history_path=xarm6_worldbelief_history_path(),
        compute_embeddings=True,
        embedding_model_id="openai/clip-vit-base-patch32",
        embedding_device="cuda",
        compute_visual_embeddings=True,
        visual_embedding_model_id="facebook/dinov2-base",
        visual_embedding_device="cuda",
        distance_threshold=0.08,
        # WorldBelief engine: trust an object as "present" after 3 detections within
        # recent_window. (min_support is WorldBelief's gate; min_detections_for_permanent
        # is the ObjectDB equivalent and is unused here.)
        min_support=3,
        recent_window=8.0,
        max_distance=1.0,
        use_aabb=True,
        max_obstacle_width=0.06,
        frustum_eviction_exemption=True,
        camera_motion_marks_partial=True,
    ),
    RerunBridgeModule.blueprint(
        blueprint=_rerun_blueprint,
        topic_to_entity=_topic_to_entity,
        visual_override={
            "world/color_camera": _color_camera_info_to_rerun,
            "world/depth_camera": _depth_camera_info_to_rerun,
        },
        max_hz={
            "world/color_camera/annotated_image": 10.0,
            "world/color_camera/color_image": 10.0,
            "world/depth_camera/depth_image": 5.0,
            "world/pointcloud": 5.0,
            "world/detections_2d": 10.0,
            "world/detections_3d": 10.0,
        },
    ),
    XArm6WorldBeliefRecorder.blueprint(),
    coordinator(
        hardware=[_xarm6_worldbelief_hw],
        tasks=[trajectory_task(_xarm6_worldbelief_hw, name="traj_xarm")],
    ),
).global_config(n_workers=7)
