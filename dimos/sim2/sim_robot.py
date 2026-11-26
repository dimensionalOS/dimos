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

"""A robot in a simulated world: what moves its body is its odometry, from the mount it started on."""

from collections.abc import Sequence
from functools import cached_property

import numpy as np
from numpy.typing import NDArray

from dimos.core.hardware_module import HardwareModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.sim2.mount_odometry import MountOdometry


class SimRobot(HardwareModule):
    """Hardware the world moves as a robot; its odometry holds its body, which no static edge then places."""

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The USD's static edges below its body, and the mount: odometry holds the body, so the prims above it go out on no edge."""
        body = f"{self.tf_frames[self.config.usd.root_link]}/"
        return [t for t in super()._static_edges if not body.startswith(f"{t.child_frame_id}/")]

    @cached_property
    def _mount_odometry(self) -> MountOdometry:
        return MountOdometry.of(self.config.usd, self.config.usd.root_link)

    def odometry(
        self, pose: Sequence[float] | NDArray[np.float64], stamp: float, frame_id: str
    ) -> PoseStamped:
        """The body's odometry in ``frame_id``, given its world pose as x, y, z, qx, qy, qz, qw."""
        relative = self._mount_odometry(pose)
        return PoseStamped(
            ts=stamp,
            frame_id=frame_id,
            position=relative.translation,
            orientation=relative.rotation,
        )
