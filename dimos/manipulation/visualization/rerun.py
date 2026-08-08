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

"""Rerun configuration for the pick-and-place workflow."""

from functools import partial
from typing import Any, cast

import rerun.blueprint as rrb

from dimos.robot.manipulators.xarm.grasp_config import XARM_TCP_TO_GRASP_FRAME


def picknplace_rerun_config() -> dict[str, Any]:
    """Return the Rerun layout and message conversions for pick and place."""
    return {
        "blueprint": _blueprint,
        "topic_to_entity": _topic_to_entity,
        "visual_override": {
            "world/color_camera": partial(
                _camera_info_to_rerun,
                image_topic="world/color_camera/color_image",
            ),
            "world/pointcloud": _pointcloud_to_rerun,
            "world/grasp_candidates": _grasp_candidates_to_rerun,
            "world/detections_3d": None,
            "world/depth_camera": None,
            "world/depth_camera/depth_image": None,
        },
    }


def _blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/annotated_image", name="Object Segmentation"),
                rrb.Spatial2DView(origin="world/color_camera/color_image", name="RGB"),
            ),
            rrb.Spatial3DView(origin="world", name="Filtered Objects"),
        )
    )


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    entities = {
        "/color_image": "world/color_camera/color_image",
        "/camera_info": "world/color_camera",
        "/depth_image": "world/depth_camera/depth_image",
        "/depth_camera_info": "world/depth_camera",
        "/grasp_candidates": "world/grasp_candidates",
        "/detections_3d": "world/detections_3d",
        "/pointcloud": "world/pointcloud",
    }
    for suffix, entity in entities.items():
        if topic_name == suffix or topic_name.endswith(suffix):
            return entity
    return f"world/{topic_name.lstrip('/')}"


def _camera_info_to_rerun(msg: Any, image_topic: str) -> list[tuple[str, Any]]:
    return cast(
        "list[tuple[str, Any]]",
        msg.to_rerun(image_topic=image_topic, optical_frame=getattr(msg, "frame_id", None)),
    )


def _pointcloud_to_rerun(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.001, mode="points")


def _grasp_candidates_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    """Render calibrated xArm TCP grasp candidates and their gripper geometry."""
    import rerun as rr

    root = "world/grasp_candidates"
    data: list[tuple[str, Any]] = [(root, rr.Clear(recursive=True))]
    frame_id = msg.header.frame_id
    if frame_id:
        data.append((root, rr.Transform3D(parent_frame=f"tf#/{frame_id}")))
    for rank, candidate in enumerate(msg.candidates[:10]):
        pose = candidate.pose
        path = f"{root}/{rank:02d}"
        selected = rank == msg.selected_index
        gripper_color = [255, 255, 0] if selected else [100, 190, 255]
        data.extend(
            [
                (
                    path,
                    rr.Transform3D(
                        translation=pose.position.as_tuple,
                        rotation=rr.Quaternion(xyzw=pose.orientation.to_tuple()),
                    ),
                ),
                (
                    f"{path}/tcp_axes",
                    rr.Arrows3D(
                        origins=[[0.0, 0.0, 0.0]] * 3,
                        vectors=[
                            [0.04, 0.0, 0.0],
                            [0.0, 0.04, 0.0],
                            [0.0, 0.0, 0.04],
                        ],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 128, 255]],
                        radii=[0.0015] * 3,
                    ),
                ),
                (
                    f"{path}/gripper_base",
                    rr.Transform3D(
                        translation=[0.0, 0.0, XARM_TCP_TO_GRASP_FRAME[2][3]],
                        rotation=rr.Quaternion(xyzw=[0.0, 0.0, -0.70710678, 0.70710678]),
                    ),
                ),
                (
                    f"{path}/gripper_base/jaws",
                    # The model sweep geometry is in the gripper-base frame:
                    # local X closes the jaws and local +Z approaches the object.
                    rr.LineStrips3D(
                        strips=[
                            [[-0.0425, 0.0, 0.095], [-0.0425, 0.0, 0.162]],
                            [[0.0425, 0.0, 0.095], [0.0425, 0.0, 0.162]],
                            [[-0.0425, 0.0, 0.095], [0.0425, 0.0, 0.095]],
                        ],
                        colors=[gripper_color] * 3,
                        radii=[0.0015] * 3,
                    ),
                ),
            ]
        )
        if selected:
            data.append(
                (
                    f"{path}/selected",
                    rr.Points3D(
                        positions=[[0.0, 0.0, 0.0]],
                        labels=[f"SELECTED #{rank} score={candidate.score:.3f}"],
                        colors=[[255, 255, 0]],
                        radii=[0.008],
                    ),
                )
            )
    return data
