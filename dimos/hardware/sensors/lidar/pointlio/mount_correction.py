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

"""Compute the rigid transform that makes a physically-tilted sensor look like a
differently-mounted one.

``mount_correction_matrix(original_static_tf, new_static_tf)`` returns the
row-major 4x4 (flattened to 16 floats) to hand to ``PointLio``'s ``transform``
arg. PointLio then moves every cloud point ``p' = C @ p`` and conjugates the body
pose ``T' = C @ T @ inv(C)`` before publishing, so the whole stack downstream sees
a sensor mounted per ``new_static_tf`` even though it is physically mounted per
``original_static_tf``. Usage::

    PointLio.blueprint(
        transform=mount_correction_matrix(rotated_urdf, normal_urdf),
        config="no_gravity_align.yaml",
    )

``C = inv(new_base_to_sensor) @ original_base_to_sensor`` at the shared
``sensor_frame``.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.urdf_loader import UrdfLoader


def _transform_to_matrix(transform: Transform) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = transform.rotation.to_rotation_matrix()
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


def base_to_frame_matrix(loader: UrdfLoader, leaf_frame: str) -> np.ndarray:
    """Compose the fixed-joint chain from the model root down to ``leaf_frame``."""
    static_transforms = loader.static_transforms
    chain: list[Transform] = []
    frame = leaf_frame
    while frame in static_transforms:
        transform = static_transforms[frame]
        chain.append(transform)
        frame = transform.frame_id
    matrix = np.eye(4)
    for transform in reversed(chain):
        matrix = matrix @ _transform_to_matrix(transform)
    return matrix


def mount_correction_matrix(
    original_static_tf: Path,
    new_static_tf: Path,
    sensor_frame: str = "mid360_link",
) -> list[float]:
    """Row-major 4x4 (16 floats) mapping the ``original`` mount to the ``new`` one."""
    original = base_to_frame_matrix(
        UrdfLoader(name="original", model_path=original_static_tf), sensor_frame
    )
    new = base_to_frame_matrix(UrdfLoader(name="new", model_path=new_static_tf), sensor_frame)
    correction = np.linalg.inv(new) @ original
    return [float(value) for value in correction.flatten()]
