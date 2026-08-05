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

"""Display-only Viser layers for the live pick grasp pipeline."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.msgs.geometry_msgs.Pose import Pose

CLOUD_LAYER = "pick/object-cloud"
CANDIDATES_LAYER = "pick/candidates"
ATTEMPT_LAYER = "pick/attempt"
PATH_LAYER = "pick/approach-path"

_GRASP_COLOR = (0, 220, 255)
_PRE_GRASP_COLOR = (255, 0, 200)
_LINK_COLOR = (150, 150, 150)
_PATH_COLOR = (255, 255, 255)
_MAX_CLOUD_POINTS = 20000


def _rotation(q: Any) -> np.ndarray:
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _fork_strips_local(gripper: Any) -> tuple[np.ndarray, ...]:
    open_extents = np.asarray(gripper.extents_open, dtype=float)
    half_extents = np.asarray(gripper.extents_half_open, dtype=float)
    open_offset = np.asarray(gripper.offset_open, dtype=float)
    half_offset = np.asarray(gripper.offset_half_open, dtype=float)

    rear_center = np.asarray([half_offset[0], 0.0, half_offset[2] - half_extents[2] / 2.0])
    mouth_center = np.asarray([open_offset[0], 0.0, open_offset[2] + open_extents[2] / 2.0])
    if mouth_center[2] <= rear_center[2]:
        raise ValueError("configured sweep profiles must open toward increasing local +Z")

    rear_half_width = max(float(half_extents[0]) / 2.0, 1e-6)
    mouth_half_width = max(float(open_extents[0]) / 2.0, rear_half_width)
    rear_left = rear_center + np.asarray([-rear_half_width, 0.0, 0.0])
    rear_right = rear_center + np.asarray([rear_half_width, 0.0, 0.0])
    mouth_left = mouth_center + np.asarray([-mouth_half_width, 0.0, 0.0])
    mouth_right = mouth_center + np.asarray([mouth_half_width, 0.0, 0.0])
    return (
        np.asarray([rear_center, [0.0, 0.0, 0.0]]),
        np.asarray([rear_left, rear_right]),
        np.asarray([rear_left, mouth_left]),
        np.asarray([rear_right, mouth_right]),
    )


def _wireframe(pose: Pose, gripper: Any, grasp_frame_to_tcp: Any) -> tuple[np.ndarray, np.ndarray]:
    world_to_tcp = np.eye(4, dtype=float)
    world_to_tcp[:3, :3] = _rotation(pose.orientation)
    world_to_tcp[:3, 3] = np.asarray(
        [pose.position.x, pose.position.y, pose.position.z], dtype=float
    )
    grasp_to_tcp = np.asarray(grasp_frame_to_tcp, dtype=float)
    if grasp_to_tcp.shape != (4, 4):
        raise ValueError("grasp_frame_to_tcp must have shape (4, 4)")
    world_to_grasp = world_to_tcp @ np.linalg.inv(grasp_to_tcp)
    rotation = world_to_grasp[:3, :3]
    translation = world_to_grasp[:3, 3]
    strips = tuple((rotation @ strip.T).T + translation for strip in _fork_strips_local(gripper))
    vertices = np.vstack(strips).astype(np.float32)
    edges = np.arange(len(vertices), dtype=np.int32).reshape((-1, 2))
    return vertices, edges


def _rank_color(index: int, total: int) -> tuple[int, int, int]:
    """Best rank green, worst orange."""
    t = 0.0 if total <= 1 else index / (total - 1)
    return (int(60 + 195 * t), int(220 - 60 * t), 60)


def _segment(a: Pose, b: Pose) -> tuple[np.ndarray, np.ndarray]:
    vertices = np.asarray(
        [[a.position.x, a.position.y, a.position.z], [b.position.x, b.position.y, b.position.z]],
        dtype=np.float32,
    )
    return vertices, np.asarray([[0, 1]], dtype=np.int32)


def cloud_layer(points: np.ndarray, frame_id: str) -> VisualizationLayer:
    """The object cloud the grasp generator consumed."""
    array = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if len(array) > _MAX_CLOUD_POINTS:
        array = array[:: int(np.ceil(len(array) / _MAX_CLOUD_POINTS))]
    return VisualizationLayer(
        CLOUD_LAYER, frame_id, (PointCloudElement("object", array, None, 0.004),)
    )


def candidates_layer(
    candidates: Sequence[Any], gripper: Any, grasp_frame_to_tcp: Any, frame_id: str
) -> VisualizationLayer:
    """Every ranked proposal as a gripper wireframe, best green through worst orange."""
    elements = []
    total = len(candidates)
    for index, candidate in enumerate(candidates):
        vertices, edges = _wireframe(candidate.pose, gripper, grasp_frame_to_tcp)
        elements.append(
            LineSetElement(
                f"rank-{index + 1}",
                vertices,
                edges,
                colors=np.asarray(_rank_color(index, total), dtype=np.uint8),
                line_width=1.5,
            )
        )
    return VisualizationLayer(CANDIDATES_LAYER, frame_id, tuple(elements))


def attempt_layer(
    grasp: Pose, pre_grasp: Pose, gripper: Any, grasp_frame_to_tcp: Any, frame_id: str
) -> VisualizationLayer:
    """The candidate currently being feasibility-checked, and its pregrasp."""
    grasp_vertices, grasp_edges = _wireframe(grasp, gripper, grasp_frame_to_tcp)
    pre_vertices, pre_edges = _wireframe(pre_grasp, gripper, grasp_frame_to_tcp)
    link_vertices, link_edges = _segment(pre_grasp, grasp)
    return VisualizationLayer(
        ATTEMPT_LAYER,
        frame_id,
        (
            LineSetElement(
                "grasp",
                grasp_vertices,
                grasp_edges,
                colors=np.asarray(_GRASP_COLOR, dtype=np.uint8),
                line_width=3.0,
            ),
            LineSetElement(
                "pre-grasp",
                pre_vertices,
                pre_edges,
                colors=np.asarray(_PRE_GRASP_COLOR, dtype=np.uint8),
                line_width=3.0,
            ),
            LineSetElement(
                "link",
                link_vertices,
                link_edges,
                colors=np.asarray(_LINK_COLOR, dtype=np.uint8),
                line_width=1.5,
            ),
        ),
    )


def path_layer(positions: Sequence[Sequence[float]], frame_id: str) -> VisualizationLayer:
    """Planned end-effector polyline for the approach."""
    vertices = np.asarray(positions, dtype=np.float32).reshape((-1, 3))
    edges = np.column_stack(
        [np.arange(len(vertices) - 1, dtype=np.int32), np.arange(1, len(vertices), dtype=np.int32)]
    )
    return VisualizationLayer(
        PATH_LAYER,
        frame_id,
        (
            LineSetElement(
                "approach",
                vertices,
                edges,
                colors=np.asarray(_PATH_COLOR, dtype=np.uint8),
                line_width=2.0,
            ),
        ),
    )
