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

"""An IMU at its frame, read from a world of its own as the observations move it.

It describes itself to nobody: a manager mounts its `get_usd` -- the frame it reads at, which is
all an IMU is -- onto whatever it is bolted to, so the composed scene has it. This then asks the
manager for that scene, builds its own world out of it, and differences the pose it is given.
"""

import asyncio
from collections.abc import AsyncIterator

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor_usd import frame_usd
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror


class SimImuConfig(SceneConfig):
    # The IMU's frame, as a robot's description or the scene names it: where it sits.
    frame_id: str = "imu_link"
    rate_hz: float = 200.0


class SimImu(Module):
    config: SimImuConfig
    imu: Out[Imu]
    world_observations: In[WorldObservations]

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: the frame this IMU reads at. An IMU is a pose, not a shape."""
        return frame_usd(self.frame_id, name="sim_imu")

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._reading = asyncio.create_task(self._read())
        yield
        self._reading.cancel()

    def _open(self) -> WorldView:
        """This IMU's own world: the composed scene, with a site at its own frame.

        Opened on the first frame rather than at start, because the robots in the world are what
        the world says they are -- the observations announce them -- and the view has to name its
        bodies the way the observations do.
        """
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to read."
                f" Give this IMU the scene_manager that composes the world."
            )
        return open_view(
            usd_layer(scene),
            robot_ids(self._world),
            SensorMount("imu", self.frame_id, self.frame_id),
        )

    async def _read(self) -> None:
        # rate_hz of sim time: its readings difference velocities over sim time, so they hold at any speed.
        view: WorldView | None = None
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                view = self._open()
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(self.frame_id)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.frame_id,
                    ts=self._world.stamp(),
                )
            )
