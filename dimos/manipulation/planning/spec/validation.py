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

"""Validation for shared manipulation planning specifications."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.spec.enums import ObstacleType

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.models import Obstacle

_EXPECTED_DIMENSIONS = {
    ObstacleType.BOX: 3,
    ObstacleType.SPHERE: 1,
    ObstacleType.CYLINDER: 2,
}


def validate_obstacle(
    obstacle: Obstacle,
    pose_matrix: NDArray[np.float64],
    *,
    allow_empty_name: bool = False,
) -> None:
    """Validate obstacle name, dimensions, color and pose. Raises ValueError.

    The pose matrix is supplied by the caller because each backend builds it
    from its own transform utilities.
    """
    if not obstacle.name and not allow_empty_name:
        raise ValueError("Obstacle name must be non-empty")
    if obstacle.obstacle_type in _EXPECTED_DIMENSIONS:
        expected = _EXPECTED_DIMENSIONS[obstacle.obstacle_type]
        if len(obstacle.dimensions) != expected:
            raise ValueError(
                f"{obstacle.obstacle_type.name} obstacle requires {expected} dimensions, "
                f"got {len(obstacle.dimensions)}"
            )
        dimensions = np.asarray(obstacle.dimensions, dtype=np.float64)
        if not np.isfinite(dimensions).all() or np.any(dimensions <= 0.0):
            raise ValueError("Obstacle dimensions must be finite and positive")
    elif obstacle.obstacle_type == ObstacleType.MESH:
        if not obstacle.mesh_path:
            raise ValueError("MESH obstacle requires mesh_path")
    else:
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")
    color = np.asarray(obstacle.color, dtype=np.float64)
    if color.shape != (4,) or not np.isfinite(color).all():
        raise ValueError("Obstacle color must contain four finite values")
    if not np.isfinite(pose_matrix).all():
        raise ValueError("Obstacle pose must contain only finite values")
