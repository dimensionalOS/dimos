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

"""An RGB camera: its own copy of the scene, posed from world_observations and rendered.

It describes itself to nobody. A manager mounts its `get_usd` onto the frame it is bolted to, so
the composed scene carries the pose; the camera then asks that manager for the scene, builds a
world of its own out of it, puts a camera at its own frame with its own field of view, and draws
from the observations the simulation publishes.
"""

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor_usd import camera_usd
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror


class SimCameraConfig(SceneConfig):
    # Its optical frame (z forward, x right, y down), as a robot's description or the scene
    # names it: where it sits.
    frame_id: str = "camera_optical"
    width: int = 640
    height: int = 480
    fovy: float = 60.0
    rate_hz: float = 10.0


class SimCamera(Module):
    config: SimCameraConfig
    color_image: Out[Image]
    world_observations: In[WorldObservations]

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: this camera's optical frame, and what it can see from there."""
        return camera_usd(self.frame_id, name="sim_camera", fovy=self.config.fovy)

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._capturing = asyncio.create_task(self._capture())
        yield
        self._capturing.cancel()

    async def _capture(self) -> None:
        """Open a world when the first frame says what is in one, then draw every tick."""
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to"
                f" render. Give this camera the scene_manager that composes the world."
            )
        layer, view = usd_layer(scene), None
        try:
            # rate_hz of sim time, so a world slower or faster than real gets as many frames per
            # sim second.
            async for _ in self._world.ticks(self.config.rate_hz):
                if view is None:
                    view = self._open(layer)
                view.update(self._world)
                rgb = view.render(self._name, self.config.width, self.config.height)
                self.color_image.publish(
                    Image(
                        data=rgb,
                        format=ImageFormat.RGB,
                        frame_id=self.frame_id,
                        ts=self._world.stamp(),
                    )
                )
        finally:
            if view is not None:
                view.close()

    def _open(self, layer: Path) -> WorldView:
        """This camera's world: the scene, with a camera of its own at its own frame.

        Opened on the first frame rather than at start, because the robots in the world are what
        the world says they are -- the observations announce them -- and the view has to name its
        bodies the way the observations do.
        """
        return open_view(
            layer,
            robot_ids(self._world),
            SensorMount("camera", self.frame_id, self._name, self.config.fovy),
        )

    @property
    def _name(self) -> str:
        """What this camera is called in its own world. Its frame will do: there is one of it.

        `self.frame_id`, not `config.frame_id`: in a namespace the frame carries that prefix, and
        the prefixed name is what the composed scene and the observations both use. Three robots
        each with a camera is three cameras called `camera_optical` otherwise, mounted at one
        frame, two of which belong to somebody else.
        """
        return self.frame_id
