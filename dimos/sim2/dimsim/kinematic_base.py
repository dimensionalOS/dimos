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

"""A robot the world moves as a box on cmd_vel: DimSim's kinematic base, wearing the robot's own USD."""

import asyncio
from collections.abc import AsyncIterator, Sequence
from functools import cached_property
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.dimsim.backend import BASE_JOINTS, MOUNTED_AT
from dimos.sim2.sim import SimulatedWorldSpec, until_it_stops
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import WorldMirror


class SimKinematicBaseConfig(HardwareModuleConfig):
    # Seconds a cmd_vel holds before the base stops, as a real base's command watchdog does.
    cmd_vel_timeout: float = 0.5
    odom_rate_hz: float = 50.0
    # The frame its odometry is in, which its mount holds; an OdomTf beside it publishes odom -> body.
    odom_frame: str = "odom"


class SimKinematicBase(HardwareModule):
    """Mount it on a frame of the scene, which is where it starts; its robot id is its namespace.

    Its odometry is an estimate, which a HardwareModule never puts on tf: it publishes ``odom``,
    and an ``OdomTf`` beside it turns that into odom -> body (its USD's root link).
    """

    config: SimKinematicBaseConfig
    world: SimulatedWorldSpec
    cmd_vel: In[Twist]
    world_observations: In[WorldObservations]
    world_actions: Out[WorldAction]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._expires: float | None = None
        self._start: np.ndarray | None = None

    @property
    def attach_frame(self) -> str:
        """Its odometry frame: the mount it started on holds odom, and odom holds its body."""
        return self.config.odom_frame

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The USD's static edges below its body, and the mount: odometry holds the body, so the prims above it go out on no edge."""
        body = f"{self.tf_frames[self.config.usd.root_link]}/"
        return [t for t in super()._static_edges if not body.startswith(f"{t.child_frame_id}/")]

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(self.namespace, self.config.usd, odom_frame=self.config.odom_frame)

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations, self.world.snapshot)
        reporting = asyncio.create_task(until_it_stops("the base's odometry", self._report()))
        yield
        reporting.cancel()

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self._drive((twist.linear.x, twist.linear.y, twist.angular.z))
        self._expires = time.time() + self.config.cmd_vel_timeout

    def _drive(self, velocity: tuple[float, float, float]) -> None:
        command = JointCommand(self.namespace, BASE_JOINTS, velocity=velocity)
        self.world_actions.publish(world_action(self._world, command))

    async def _report(self) -> None:
        async for stamp in self._world.ticks(self.config.odom_rate_hz):
            if self._expires is not None and stamp > self._expires:
                self._expires = None
                self._drive((0.0, 0.0, 0.0))
            odom = self._odom(stamp)
            if odom is None:
                continue
            self.odom.publish(odom)
            transforms = self.moving_transforms(ts=stamp)
            if transforms:
                self.tfbuffer.publish(*transforms)

    def _odom(self, stamp: float) -> PoseStamped | None:
        base = self._world.entity(self.namespace)
        if base is None or base.pose is None:
            return None
        if self._start is None:
            self._start = np.linalg.inv(_matrix(base.cbor[MOUNTED_AT]))
        # Odometry is from where the base started, the frame it is mounted on (which it stands above).
        relative = Transform.from_matrix(self._start @ _matrix(base.pose))
        translation, rotation = relative.translation, relative.rotation
        return PoseStamped(
            ts=stamp,
            frame_id=self.config.odom_frame,
            position=[translation.x, translation.y, translation.z],
            orientation=[rotation.x, rotation.y, rotation.z, rotation.w],
        )


def _matrix(pose: Sequence[float] | np.ndarray) -> np.ndarray:
    return Transform(translation=Vector3(*pose[:3]), rotation=Quaternion(*pose[3:7])).to_matrix()
