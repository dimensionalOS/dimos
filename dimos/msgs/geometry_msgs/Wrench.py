# Copyright 2025 Dimensional Inc.
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

from dataclasses import dataclass

from dimos.msgs.geometry_msgs.Vector3 import Vector3


@dataclass
class Wrench:
    """
    Represents a force and torque in 3D space.

    This is equivalent to ROS geometry_msgs/Wrench.
    """

    force: Vector3 = None  # Force vector (N)
    torque: Vector3 = None  # Torque vector (Nm)

    def __post_init__(self) -> None:
        if self.force is None:
            self.force = Vector3(0.0, 0.0, 0.0)
        if self.torque is None:
            self.torque = Vector3(0.0, 0.0, 0.0)

    def __repr__(self) -> str:
        return f"Wrench(force={self.force}, torque={self.torque})"
