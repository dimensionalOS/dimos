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

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import usd_frame_pose


class MountOdometry:
    """Odometry from the mount a body started on, so the body keeps its offset (its standing height) above it.

    The mount holds odom by an identity edge, so odom -> body must carry where the body sits on the mount.
    """

    def __init__(self, body_on_mount: NDArray[np.float64]) -> None:
        self._body_on_mount = body_on_mount
        self._mount_inverse: NDArray[np.float64] | None = None

    @classmethod
    def of(cls, usd: UsdModel, frame: str) -> MountOdometry:
        root = usd.source_path.parent if usd.source_path is not None else None
        return cls(usd_frame_pose(usd.usd, frame, root))

    def __call__(self, pose: Sequence[float] | NDArray[np.float64]) -> Transform:
        """The body's odometry, given its world pose as x, y, z, qx, qy, qz, qw."""
        here = Transform(
            translation=Vector3(*pose[:3]), rotation=Quaternion(*pose[3:7])
        ).to_matrix()
        if self._mount_inverse is None:
            self._mount_inverse = np.linalg.inv(here @ np.linalg.inv(self._body_on_mount))
        return Transform.from_matrix(self._mount_inverse @ here)
