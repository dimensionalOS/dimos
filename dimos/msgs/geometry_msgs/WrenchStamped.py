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
import time

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Wrench import Wrench
from dimos.types.timestamped import Timestamped


@dataclass
class WrenchStamped(Timestamped):
    """
    Represents a stamped force/torque measurement.

    This is equivalent to ROS geometry_msgs/WrenchStamped.
    """

    msg_name = "geometry_msgs.WrenchStamped"
    ts: float = 0.0
    frame_id: str = ""
    wrench: Wrench = None

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()
        if self.wrench is None:
            self.wrench = Wrench()

    @classmethod
    def from_force_torque_array(
        cls, ft_data: list, frame_id: str = "ft_sensor", ts: float | None = None
    ):
        """
        Create WrenchStamped from a 6-element force/torque array.

        Args:
            ft_data: [fx, fy, fz, tx, ty, tz]
            frame_id: Reference frame
            ts: Timestamp (defaults to current time)

        Returns:
            WrenchStamped instance
        """
        if len(ft_data) != 6:
            raise ValueError(f"Expected 6 elements, got {len(ft_data)}")

        return cls(
            ts=ts if ts is not None else time.time(),
            frame_id=frame_id,
            wrench=Wrench(
                force=Vector3(x=ft_data[0], y=ft_data[1], z=ft_data[2]),
                torque=Vector3(x=ft_data[3], y=ft_data[4], z=ft_data[5]),
            ),
        )

    def __repr__(self) -> str:
        return f"WrenchStamped(ts={self.ts}, frame_id='{self.frame_id}', wrench={self.wrench})"
