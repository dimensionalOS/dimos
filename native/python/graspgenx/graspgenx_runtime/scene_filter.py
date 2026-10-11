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

"""Check grasps against the scene with GraspGenX's own collision filter.

Importing graspgenx runs its setup hook, which clones repositories unless
GRASPGENX_CHECKPOINT_DIR and GRASPGENX_GRIPPER_CFG_DIR name existing
directories, so import this module only after setting them (backend.py does).
"""

from __future__ import annotations

from graspgenx.utils.collision_filter import filter_colliding_grasps
import numpy as np

# Most entries (4 bytes each) in one batch's gripper-to-scene distance table: 256 MB.
_MAX_DISTANCE_ENTRIES = 64_000_000


def collision_free(
    scene: np.ndarray,
    poses: np.ndarray,
    gripper_points: np.ndarray,
    collision_threshold: float,
    device: str | None = None,
) -> np.ndarray:
    """Which grasps keep the open gripper clear of the scene.

    Args:
        scene: (M, 3) scene points, metres.
        poses: (N, 4, 4) grasp poses in GraspGenX's grasp frame (not the TCP), in
            the scene's frame.
        gripper_points: (K, 3) points on the gripper body in the grasp frame,
            metres, from ``gripper_body_points``.
        collision_threshold: A gripper point closer than this (metres) to a scene
            point is a collision.
        device: Torch device for the distances; None uses the GPU when there is one.

    Returns an (N,) bool array, True where the grasp is clear.
    """
    batch = max(1, _MAX_DISTANCE_ENTRIES // max(1, len(gripper_points) * len(scene)))
    clear: np.ndarray = filter_colliding_grasps(
        scene,
        poses,
        collision_threshold=collision_threshold,
        batch_size=batch,
        gripper_surface_points=gripper_points,
        device=device,
    )
    return clear
