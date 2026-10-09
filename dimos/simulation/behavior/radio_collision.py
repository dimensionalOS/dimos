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

"""Task-local carried geometry for CPU planning; never a physics attachment."""

from collections.abc import Generator, Mapping
from contextlib import contextmanager
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld


@contextmanager
def temporary_carried_radio_geometry(
    world: "RoboPlanWorld",
    parent_link: str,
    link_from_geometry: Mapping[str, NDArray[np.float64]],
) -> Generator[None, None, None]:
    """Reparent registered radio parts for one locked planning/check operation.

    Caller supplies EVERY radio collision part and measured/calibrated rigid
    transforms from the holding link. Collision pairs are preserved unchanged:
    support/contact allowances must be independently and explicitly justified.
    This does not grasp an object, authorize execution or validate retention.
    No execution or wait for runtime feedback belongs inside this context.

    Native geometry is restored even after a partial setup failure. Failed
    restoration invalidates the world instead of permitting later planning
    against an unknown scene. This deliberately uses RoboPlan adapter internals;
    it is a bounded development helper, not a general SDK attachment contract.
    """
    if not parent_link or not link_from_geometry:
        raise ValueError("Holding link and complete radio collision geometry are required")
    if parent_link != "right_gripper_link":
        raise ValueError("This development adapter supports only the R1 Pro right holding gripper")
    matrices = {}
    for name, transform in link_from_geometry.items():
        matrix = np.asarray(transform, dtype=np.float64).copy()
        if (
            not name
            or matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-9)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-9)
        ):
            raise ValueError("Carried geometry requires named finite rigid transforms")
        matrices[name] = matrix
    # Keep all native queries and restoration under the existing scene lock.
    with world._lock:
        world._require_finalized()
        scene = world._require_scene()
        # Installed RoboPlan 0.6.0 omits a fixed frame's placement when updating
        # geometry. The wrist link is at its moving joint origin. Compose the
        # fixed gripper offset explicitly, from one unchanged FK snapshot.
        native_parent = "right_arm_link7"
        scene.getFrameId(native_parent)
        with world.scratch_context() as ctx:
            wrist = world.get_link_pose(ctx, native_parent)
            gripper = world.get_link_pose(ctx, parent_link)
        wrist_from_gripper = np.linalg.inv(wrist) @ gripper
        snapshots = {obstacle.name: obstacle for obstacle in world.get_obstacles()}
        if any(name not in snapshots for name in matrices):
            raise ValueError("Every carried radio part must already be registered")
        attempted = []
        try:
            for name, matrix in matrices.items():
                attempted.append(name)
                scene.updateGeometryPlacement(name, native_parent, wrist_from_gripper @ matrix)
            yield
        finally:
            errors = []
            for name in reversed(attempted):
                try:
                    scene.updateGeometryPlacement(
                        name, "dimos_world", pose_to_matrix(snapshots[name].pose)
                    )
                except Exception as error:
                    errors.append(error)
            if errors:
                world._usable = False
                raise RuntimeError(
                    "Carried radio geometry restoration failed; world unusable"
                ) from errors[0]
