# Copyright 2025-2026 Dimensional Inc.
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

"""Shared utilities for depth perception modules."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np
import open3d as o3d
import open3d.core as o3c

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.utils.logging_config import Logger


def make_colored_cloud(points: np.ndarray, colors: np.ndarray, frame_id: str, ts: float) -> PointCloud2:
    """Pack numpy points + colors into a PointCloud2."""
    pcd_t = o3d.t.geometry.PointCloud()
    pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)
    if len(colors) == len(points):
        pcd_t.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)
    return PointCloud2(pcd_t, frame_id=frame_id, ts=ts)


def to_world(
    points_cam: np.ndarray,
    ts: float,
    tf_bus: object,
    world_frame: str,
    camera_frame: str,
    tf_timeout: float,
    last_tf: object | None,
    tf_last_attempt: float,
    logger: object,
) -> tuple[np.ndarray, str, object | None, float]:
    """Transform camera-frame points to world frame via TF.

    Returns (points_world, frame_id, updated_last_tf, updated_tf_last_attempt).
    Falls back to camera=world when TF is unavailable (trial / no-odometry mode).
    Throttles TF retries to 5 s when no transform has ever been found, to avoid
    per-frame warning spam.
    """
    if len(points_cam) == 0:
        return points_cam, world_frame, last_tf, tf_last_attempt

    now = time.monotonic()
    if last_tf is not None or now - tf_last_attempt >= 5.0:
        tf_last_attempt = now
        tf = tf_bus.get(world_frame, camera_frame, ts, tf_timeout)
        if tf is not None:
            last_tf = tf

    if last_tf is None:
        # No odometry: treat camera as world origin.
        # Use world frame_id so the Rerun bridge attaches the same TF anchor
        # to frame_cloud and global_map, keeping them co-located in the viewer.
        return points_cam, world_frame, last_tf, tf_last_attempt

    R = last_tf.rotation.to_rotation_matrix()
    t = np.array(
        [last_tf.translation.x, last_tf.translation.y, last_tf.translation.z],
        dtype=np.float32,
    )
    pts_world = (points_cam @ R.T).astype(np.float32) + t
    return pts_world, world_frame, last_tf, tf_last_attempt
