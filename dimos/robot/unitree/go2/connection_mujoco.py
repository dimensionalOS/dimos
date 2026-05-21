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

"""MuJoCo-simulated Go2 connection.

A subclass of `MujocoConnectionBase` with Go2-specific camera mounting,
start/stop sequencing, robot RPC stubs, the perception protocols, and the
`observe()` skill.
"""

from __future__ import annotations

import time

import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.connection_registry import connection
from dimos.robot.unitree.go2.config import ConnectionConfig, Go2Mode
from dimos.robot.unitree.mujoco_connection import MujocoConnectionBase
from dimos.spec.perception import Camera, Pointcloud


@connection(robot="go2", backend="mujoco")
class Go2MujocoConnection(MujocoConnectionBase, Camera, Pointcloud):
    """MuJoCo-simulated Go2 connection."""

    config: ConnectionConfig
    pointcloud: Out[PointCloud2]

    _camera_link_offset: Vector3 = Vector3(0.3, 0.0, 0.0)

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    def _on_start(self) -> None:
        self.standup()
        time.sleep(3)
        self.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.enable_rage_mode()

        self.set_obstacle_avoidance(self.config.g.obstacle_avoidance)

    def _on_stop(self) -> None:
        self.liedown()

    @rpc
    def standup(self) -> bool:
        return True

    @rpc
    def liedown(self) -> bool:
        return True

    @rpc
    def balance_stand(self) -> bool:
        return True

    @rpc
    def enable_rage_mode(self) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        pass

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
