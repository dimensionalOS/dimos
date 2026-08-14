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

"""Rerun stream adapters for the canonical pick-and-place workflow."""

from __future__ import annotations

from typing import Any

from dimos.manipulation.pick_and_place import PickAndPlaceState


def pick_and_place_rerun_config() -> dict[str, Any]:
    """Return Rerun entity mappings and conversions for pick-and-place streams."""
    return {
        "blueprint": _blueprint,
        "topic_to_entity": _topic_to_entity,
        "visual_override": {
            "world/pick-and-place/objects": _objects_to_rerun,
            "world/pick-and-place/selection": _state_to_rerun,
        },
    }


def _blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(rrb.Spatial3DView(origin="world", name="Pick and Place"))


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    if topic_name == "/objects" or topic_name.endswith("/objects"):
        return "world/pick-and-place/objects"
    if topic_name == "/pick_and_place_state" or topic_name.endswith("/pick_and_place_state"):
        return "world/pick-and-place/selection"
    return f"world/{topic_name.lstrip('/')}"


def _objects_to_rerun(objects: list[Any]) -> list[tuple[str, Any]]:
    """Render each detected object's measured point cloud in the planning frame."""
    import rerun as rr

    root = "world/pick-and-place/objects"
    data: list[tuple[str, Any]] = [(root, rr.Clear(recursive=True))]
    colors = ([80, 180, 255], [130, 230, 130], [230, 150, 230])
    for index, obj in enumerate(objects):
        points = obj.pointcloud.points_f32()
        if len(points):
            data.append(
                (
                    f"{root}/object-{index}",
                    rr.Points3D(positions=points, colors=colors[index % len(colors)], radii=0.003),
                )
            )
    return data


def _state_to_rerun(state: PickAndPlaceState) -> list[tuple[str, Any]]:
    """Render ranked grasp candidates and selected grasp targets."""
    import rerun as rr

    root = "world/pick-and-place/selection"
    data: list[tuple[str, Any]] = [(root, rr.Clear(recursive=True))]
    for rank, candidate in enumerate(state.candidates.candidates[:10]):
        pose = candidate.pose
        selected = rank == state.candidates.selected_index
        data.append(
            (
                f"{root}/candidates/rank-{rank}",
                _pose_axes(pose, [255, 220, 70] if selected else [100, 190, 255]),
            )
        )
    for name, pose, color in (
        ("grasp", state.selected_grasp, [255, 70, 70]),
        ("pregrasp", state.selected_pregrasp, [70, 255, 120]),
    ):
        if pose is not None:
            data.append((f"{root}/targets/{name}", _pose_axes(pose, color)))
    return data


def _pose_axes(pose: Any, color: list[int]) -> Any:
    import rerun as rr

    axes = pose.orientation.to_rotation_matrix() * 0.06
    return rr.Arrows3D(
        origins=[list(pose.position.as_tuple)] * 3,
        vectors=[list(axis) for axis in axes.T],
        colors=[color] * 3,
        radii=[0.0015] * 3,
    )
