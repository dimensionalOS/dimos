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

"""Recording-specific stream resolution + lidar-scan world registration."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.memory2.tf import StreamTF
    from dimos.memory2.type.observation import Observation

# (odom stream, lidar-fallback candidates). First pair whose odom stream exists wins, so a
# recording never mixes rigs (e.g. fastlio + pointlio). Ordered mid360 rig -> go2 -> generic.
STREAM_PAIRS: list[tuple[str, list[str]]] = [
    ("pointlio_odometry", ["pointlio_lidar"]),
    ("fastlio_odometry", ["fastlio_lidar"]),
    ("go2_odom", ["go2_lidar", "l1_lidar", "lidar"]),
    ("odom", ["lidar"]),
]


def resolve_streams(
    available: set[str] | list[str], odom: str = "", lidar: str = ""
) -> tuple[str, str]:
    """``(odom_stream, lidar_stream)`` defaults from what a recording actually has."""
    if not odom:
        odom = next((name for name, _ in STREAM_PAIRS if name in available), "odom")
    if not lidar:
        candidates = next((ls for name, ls in STREAM_PAIRS if name == odom), ["lidar"])
        lidar = next((name for name in candidates if name in available), candidates[0])
    return odom, lidar


def default_odom_edge(store: Any, odom_stream: str) -> str:
    """``"parent:child"`` from the odom stream's own header, or ``""`` if it has no child frame
    (e.g. ``PoseStamped`` odometry)."""
    observation = next(iter(store.stream(odom_stream)), None)
    if observation is None:
        return ""
    child_frame = getattr(observation.data, "child_frame_id", "")
    if not child_frame:
        return ""
    return f"{observation.data.frame_id}:{child_frame}"


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5 or 1.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        float,
    )


def world_register(
    observation: Observation[Any],
    store_tf: StreamTF | None,
    world_frame: str,
    fallback_frame: str,
    origin_lookup: Callable[[float], tuple[float, float, float]] | None = None,
) -> tuple[np.ndarray, tuple[float, float, float] | None]:
    """``(points_world_f32, sensor_origin_world)`` for one lidar observation.

    The scan's own ``frame_id`` decides everything: a scan already in
    ``world_frame`` is returned untouched; otherwise it is brought into the world
    via tf (``world_frame <- scan_frame``) and the tf translation is the ray
    origin. Falls back to ``fallback_frame`` when the scan carries no frame, then
    to the stored pose, then to assuming it is already world. ``origin_lookup``
    supplies the ray origin for already-world scans with no per-scan pose.
    Origin is ``None`` when none can be resolved (the caller skips that scan).
    """
    points = np.asarray(observation.data.points_f32())
    if not len(points):
        return points, None
    pose = observation.pose_tuple
    scan_frame = getattr(observation.data, "frame_id", "") or fallback_frame
    if scan_frame == world_frame:
        if pose is not None:
            origin = (float(pose[0]), float(pose[1]), float(pose[2]))
        elif origin_lookup is not None:
            origin = origin_lookup(float(observation.ts))
        else:
            origin = None
        return points.astype(np.float32), origin
    if store_tf is not None:
        transform = store_tf.get(world_frame, scan_frame, float(observation.ts), None)
        if transform is not None:
            rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z], float
            )
            world = points @ rotation.T + translation
            return world.astype(np.float32), (
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            )
    if pose is not None:
        rotation = _quat_to_matrix(pose[3], pose[4], pose[5], pose[6])
        world = points @ rotation.T + np.array(pose[:3], float)
        return world.astype(np.float32), (float(pose[0]), float(pose[1]), float(pose[2]))
    return points.astype(np.float32), None
