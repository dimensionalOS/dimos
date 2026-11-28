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

"""A simulated sensor that reads a view of the world of its own, opened on its first tick."""

import asyncio
from collections.abc import AsyncIterator, Callable, Coroutine
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.sim import SimSensor


class ViewSensor(SimSensor):
    """Runs its ``loops()`` while it runs, each reading a view ``open_view`` opens."""

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        running = [asyncio.create_task(loop) for loop in self.loops()]
        yield
        for task in running:
            task.cancel()

    def loops(self) -> list[Coroutine[Any, Any, None]]:
        raise NotImplementedError

    def open_view(self, mount: SensorMount, quality: RenderQuality = RenderQuality()) -> WorldView:
        """A view of the composed world with this sensor mounted in it; it blocks, so open it in a thread."""
        return open_view(self.world_layer(), robot_ids(self._world), mount, quality)

    async def film(
        self,
        idle: list[WorldView],
        camera: str,
        width: int,
        height: int,
        rate_hz: float,
        stamp_tolerance: float | None,
        publish: Callable[[NDArray[np.uint8], float], None],
    ) -> None:
        """A frame of ``camera`` each tick on whichever of ``idle`` is free, then ``publish``ed with its stamp; with none free, the frame is skipped, as a real camera drops one."""
        drawing: set[asyncio.Task[None]] = set()
        published = [-math.inf]

        async def photograph(view: WorldView) -> None:
            try:
                stamp = view.time
                rgb = await asyncio.to_thread(view.render, camera, width, height)
            finally:
                idle.append(view)
            # Unless a newer frame went first.
            if stamp > published[0]:
                published[0] = stamp
                publish(rgb, stamp)

        try:
            async for _ in self._world.ticks(rate_hz):
                if not idle:
                    continue
                view = idle.pop()
                view.update(self._world)
                # Checked when it is taken, not once drawn: the world being off wall time is what drops a frame.
                if not self.on_time("color_image", view.time, 1 / rate_hz, stamp_tolerance):
                    idle.append(view)
                    continue
                task = asyncio.create_task(photograph(view))
                drawing.add(task)
                task.add_done_callback(drawing.discard)
        finally:
            for task in drawing:
                task.cancel()


def imu_message(view: WorldView, site: str) -> Imu:
    """What the IMU at ``site`` reads in ``view``, stamped with the view's time."""
    gyro, accelerometer, (w, x, y, z) = view.imu(site)
    return Imu(
        angular_velocity=Vector3(*gyro),
        linear_acceleration=Vector3(*accelerometer),
        orientation=Quaternion(x, y, z, w),
        frame_id=site,
        ts=view.time,
    )
