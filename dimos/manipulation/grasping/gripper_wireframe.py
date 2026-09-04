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


"""Sweep-volume gripper wireframes, shared by the viser grasp layers.

Lifted from the demo_graspgenx renderer so a live viewer can draw the same
geometry without pulling in matplotlib.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, Protocol

import numpy as np


class SweepVolumeLike(Protocol):
    extents_open: tuple[float, float, float]
    offset_open: tuple[float, float, float]
    extents_half_open: tuple[float, float, float]
    offset_half_open: tuple[float, float, float]


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


def _fork_strips_local(gripper: SweepVolumeLike) -> tuple[np.ndarray, ...]:
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


def gripper_wireframe_strips(
    candidate: Any,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[np.ndarray, ...]:
    """Convert one TCP proposal into world-frame sweep-volume wireframe strips."""
    p, q = candidate.pose.position, candidate.pose.orientation
    world_to_tcp = np.eye(4, dtype=float)
    world_to_tcp[:3, :3] = _rotation(q)
    world_to_tcp[:3, 3] = np.asarray([p.x, p.y, p.z], dtype=float)
    grasp_to_tcp = np.asarray(grasp_frame_to_tcp, dtype=float)
    if grasp_to_tcp.shape != (4, 4):
        raise ValueError("grasp_frame_to_tcp must have shape (4, 4)")
    world_to_grasp = world_to_tcp @ np.linalg.inv(grasp_to_tcp)
    rotation = world_to_grasp[:3, :3]
    translation = world_to_grasp[:3, 3]
    return tuple((rotation @ strip.T).T + translation for strip in _fork_strips_local(gripper))


def gripper_wireframe_geometry(
    candidate: Any,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return indexed world-frame vertices and edges for one grasp proposal."""
    strips = gripper_wireframe_strips(candidate, gripper, grasp_frame_to_tcp)
    vertices = np.vstack(strips).astype(np.float32)
    edges = np.arange(len(vertices), dtype=np.int32).reshape((-1, 2))
    return vertices, edges
