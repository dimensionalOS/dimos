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

"""MuJoCo-simulated G1 connection.

A thin subclass of `MujocoConnectionBase` that sets the G1's camera mounting
offset and publishes an additional `map -> world` transform.
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.connection_registry import connection
from dimos.robot.unitree.mujoco_connection import MujocoConnectionBase


class G1SimConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


@connection(robot="g1", backend="mujoco")
class G1MujocoConnection(MujocoConnectionBase):
    """MuJoCo-simulated G1 connection."""

    config: G1SimConfig

    _camera_link_offset: Vector3 = Vector3(0.05, 0.0, 0.6)

    def _extra_transforms(self, msg: PoseStamped) -> list[Transform]:
        return [
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="map",
                child_frame_id="world",
                ts=msg.ts,
            ),
        ]
