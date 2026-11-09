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

"""LivoxSim: the sim twin of the Mid360 module, read from a world of its own making.

The lidar casts the fitted Mid-360 scan pattern and return model; the IMU is the one inside the
unit, at the same mount. It describes itself to nobody: a manager mounts its `get_usd` -- the
same layer the real driver answers with -- onto the frame it is bolted to, so the composed scene
has the sensor in it. This then asks that manager for the scene, builds its own world out of it,
and ranges against the furniture that is really there.
"""

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path

from dimos_lcm.world_msgs import WorldObservations
import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.livox import module as mid360_driver
from dimos.hardware.sensors.lidar.livox.module import mid360_usd
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids, rotation_matrix
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import OcclusionMap, SimLidar
from dimos.simulation.sensors.mid360.lidar import sim_mid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.utils.data import get_data


class LivoxSimConfig(SceneConfig):
    # The lidar's frame, as a robot's description or the scene names it: where it sits.
    frame_id: str = "mid360_link"
    imu_frame_id: str = "imu_link"
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    # A lidar sees whatever a body would walk into, so these are the groups MuJoCo's own
    # convention puts collision geometry in: 0, where a hand-written scene's planes and boxes
    # go, and 3, where a converted one's collision meshes and every robot's do. Group 1 is left
    # out because it holds visual-only meshes (contype 0) -- drawing a desk is not the same as
    # being able to hit it. Seeing only group 0 was the old default: in office_555 that is 354
    # geoms out of 4138, so the beam passed straight through every desk, chair and robot and
    # returned the wall behind them.
    geom_groups: tuple[int, ...] = (0, 3)
    # Further frames the beam passes through, beyond the one the sensor is mounted on (which it
    # never sees, the way a real one cannot see its own housing). For a radome, a mast, or a
    # wall a test wants out of the way.
    blind_to: tuple[str, ...] = ()
    # The Go2 rig's mount blocks some directions; only meaningful when mounted on a Go2.
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(Module):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    world_observations: In[WorldObservations]

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: the MID-360, in the frames this instance stamps.

        The same layer the real Mid360 driver answers with, not a copy of it: a sim twin that
        described itself differently from the sensor it stands in for would put a different box
        on the screen depending on which one was running. The root is that driver's directory,
        for the reason it gives: the one path that is right wherever it is installed.
        """
        return [
            str(Path(mid360_driver.__file__ or "").parent),
            mid360_usd(self.frame_id, self.config.imu_frame_id),
        ]

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._scanning = asyncio.create_task(self._scan())
        self._reading = asyncio.create_task(self._read_imu())
        yield
        self._scanning.cancel()
        self._reading.cancel()

    def _layer(self) -> Path:
        """The composed scene, as a file. Asked for once; both tasks build their own world from it."""
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to"
                f" range against. Give this lidar the scene_manager that composes the world."
            )
        return usd_layer(scene)

    def _open(self, layer: Path) -> WorldView:
        """This sensor's own world: the composed scene, with a site at its own mount.

        Opened on the first frame rather than at start, because the robots in the world are what
        the world says they are -- the observations announce them -- and the view has to name its
        bodies the way the observations do.
        """
        return open_view(
            layer,
            robot_ids(self._world),
            SensorMount("lidar", self.frame_id, self.frame_id),
        )

    async def _scan(self) -> None:
        layer, site = None, self.frame_id
        view: WorldView | None = None
        sensor: SimLidar | None = None
        points_per_scan = int(POINT_RATE / self.config.rate_hz)
        # Scans and IMU readings pace on sim time, so both keep their rates per sim second at any speed.
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                layer = layer or self._layer()
                view = self._open(layer)
                sensor = sim_mid360(
                    view.raycaster(
                        self.config.geom_groups,
                        mounted_at=site,
                        blind_to=self.config.blind_to,
                    ),
                    self.config.seed,
                    OcclusionMap.load(get_data("go2_sim") / "go2_mid360_occlusion.npz")
                    if self.config.go2_occlusion
                    else None,
                )
            assert sensor is not None
            view.update(self._world)
            position, orientation = view.pose(site)
            # Off the loop, so the IMU beside it keeps its rate while a scan is cast.
            points = await asyncio.to_thread(
                sensor.cast, position, rotation_matrix(orientation), points_per_scan
            )
            # The Mid360's own layout; each scan is cast from one pose, so every point is at offset 0.
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points,
                    self.frame_id,
                    self._world.stamp(),
                    offset_times=np.zeros(len(points), np.uint32),
                )
            )

    async def _read_imu(self) -> None:
        """A world of its own, so a reading is not waiting on a scan being cast."""
        site, view = self.frame_id, None
        async for _ in self._world.ticks(self.config.imu_rate_hz):
            if view is None:
                view = self._open(self._layer())
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.config.imu_frame_id,
                    ts=self._world.stamp(),
                )
            )
