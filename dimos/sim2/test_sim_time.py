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

"""Sensors keep their rates per sim second whether the world runs slower or faster than real."""

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path
import time
from typing import Any
import uuid

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.camera import SimCamera
from dimos.sim2.imu import SimImu
from dimos.sim2.l1 import SimL1
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.utils.mjcf_frames import mjcf_with_frames
from dimos.sim2.world_observations import ACT, WorldMirror

L1_PERIOD = 0.13
RATES = {
    "color_image": 10.0,
    "imu": 50.0,
    "lidar": 10.0,
    "livox_imu": 50.0,
    "l1_lidar": 1 / L1_PERIOD,
}


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <geom type="box" pos="2 0 .5" size=".2 1 .5"/>
  </worldbody>
</mujoco>
""")
    return path


@pytest.fixture
def robot_model(tmp_path: Path) -> Path:
    path = tmp_path / "robot.xml"
    path.write_text("""
<mujoco model="dog">
  <worldbody>
    <body name="base" pos="0 0 .1">
      <freejoint/>
      <geom type="box" size=".2 .1 .1" mass="1"/>
      <body name="head" pos=".2 0 .1"><joint name="neck" type="hinge" axis="0 0 1"/><geom type="sphere" size=".05" mass=".1"/></body>
    </body>
  </worldbody>
  <actuator><motor name="neck" joint="neck"/></actuator>
</mujoco>
""")
    return path


class _DogConfig(ModuleConfig):
    model: Path
    # With a control period, the world waits this long (wall seconds) for each of its commands.
    control_period: float | None = None
    think: float = 0.0


class _Dog(Module):
    """A robot that, in lockstep, is slow to answer: the world runs slower than real waiting for it."""

    config: _DogConfig
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config = self.config
        robot = SimulatedRobot("dog", config.model, "base", control_period=config.control_period)
        return {"simulated": robot}

    async def main(self) -> AsyncIterator[None]:
        world, loop = WorldMirror(), asyncio.get_running_loop()

        def act() -> None:
            self.actions.publish(world_action(world, JointCommand("dog", ("neck",), effort=(0.0,))))

        def on_frame(seen: WorldMirror) -> None:
            if seen.got("dog", ACT):
                loop.call_soon_threadsafe(loop.call_later, self.config.think, act)

        world.on_frame(on_frame)
        world.subscribe(self.world_observations)
        yield


class _Readings(Module):
    color_image: In[Image]
    imu: In[Imu]
    lidar: In[PointCloud2]
    livox_imu: In[Imu]
    l1_lidar: In[PointCloud2]

    async def main(self) -> AsyncIterator[None]:
        self._stamps: dict[str, list[float]] = {name: [] for name in RATES}
        self._accelerations: list[float] = []
        self._started = time.monotonic()
        for name in RATES:
            getattr(self, name).subscribe(
                lambda message, n=name: self._stamps[n].append(message.ts)
            )
        self.imu.subscribe(
            lambda m: self._accelerations.append(
                float(np.linalg.norm(m.linear_acceleration.to_numpy()))
            )
        )
        yield

    @rpc
    def readings(self) -> dict[str, Any]:
        return {
            "stamps": self._stamps,
            "accelerations": self._accelerations,
            "wall": time.monotonic() - self._started,
        }


def _run(
    scene: Path, robot_model: Path, seconds: float, speed: float = 1.0, **dog: Any
) -> dict[str, Any]:
    # Every sensor at a frame of the dog's model.
    frames = {"eye": "head", "imu_link": "base", "livox": "head", "l1": "base"}
    model = mjcf_with_frames(
        robot_model,
        {frame: (body, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) for frame, body in frames.items()},
        out_dir=robot_model.parent,
    )
    blueprint = autoconnect(
        Simulation.blueprint(scene=scene, speed=speed),
        _Dog.blueprint(model=model, **dog),
        SimCamera.blueprint(frame_id="eye", width=64, height=48, rate_hz=RATES["color_image"]),
        SimImu.blueprint(frame_id="imu_link", rate_hz=RATES["imu"]),
        LivoxSim.blueprint(
            frame_id="livox", rate_hz=RATES["lidar"], imu_rate_hz=RATES["livox_imu"]
        ).remappings([(LivoxSim, "imu", "livox_imu")]),
        SimL1.blueprint(frame_id="l1", period=L1_PERIOD).remappings([(SimL1, "lidar", "l1_lidar")]),
        _Readings.blueprint(),
        # A worker each, as a robot's blueprint gives its sensors: one sharing a busy loop falls behind at 4x.
    ).global_config(n_workers=8)
    # Its own zenoh domain: another world running on this machine would otherwise share its topics.
    overrides = {"g": {"viewer": "none", "zenoh_domain": f"sim2-test-{uuid.uuid4().hex[:8]}"}}
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        readings = coordinator.get_instance(_Readings)
        # At least `seconds` of wall time, then until every sensor has steady stamps: on a loaded machine
        # a sensor's first point cloud (it imports open3d) can take most of that.
        deadline = time.monotonic() + seconds + MAX_EXTRA_WALL
        time.sleep(seconds)
        while time.monotonic() < deadline and not _steady(readings.readings()["stamps"]):
            time.sleep(1.0)
        return readings.readings()
    finally:
        coordinator.stop()


# Wall seconds a run may go on past its own, waiting for every sensor to be steady.
MAX_EXTRA_WALL = 60.0
STEADY_STAMPS = 8


def _steady(stamps: dict[str, list[float]]) -> bool:
    if not all(stamps[name] for name in RATES):
        return False
    start = min(stamps[name][0] for name in RATES)
    return all(
        sum(stamp >= start + WARM_UP for stamp in stamps[name]) >= STEADY_STAMPS for name in RATES
    )


# Sim seconds at the start in which a sensor may stall once, warming up (its first point cloud imports open3d).
WARM_UP = 1.0


def _per_sim_second(stamps: list[float], start: float) -> float:
    # Stamps are the wall time the sim started at plus its sim time, so their spacing is sim time.
    steady = [stamp for stamp in stamps if stamp >= start + WARM_UP]
    assert len(steady) > 5, f"only {len(steady)} after warming up"
    return (len(steady) - 1) / (steady[-1] - steady[0])


@pytest.mark.parametrize(
    ("speed", "dog"),
    [
        (0.5, {}),
        (4.0, {}),
        # Lockstep with a robot that takes 50 ms (wall) to answer each 20 ms (sim) tick.
        (1.0, {"control_period": 0.02, "think": 0.05}),
    ],
    ids=["half_speed", "four_times_speed", "slower_than_real_lockstep"],
)
def test_sensors_keep_their_rates_per_sim_second(scene, robot_model, speed, dog):
    readings = _run(scene, robot_model, seconds=8.0, speed=speed, **dog)
    stamps = readings["stamps"]
    counts = {name: len(stamps[name]) for name in RATES}
    assert all(count > 5 for count in counts.values()), counts
    start = min(stamps[name][0] for name in RATES)
    sim_seconds = stamps["imu"][-1] - stamps["imu"][0]
    for name, rate in RATES.items():
        measured = _per_sim_second(stamps[name], start)
        assert measured == pytest.approx(rate, rel=0.1), (name, measured)
    if dog:
        # The world waited on the robot: well slower than real.
        assert sim_seconds < 0.7 * readings["wall"]
    # At rest on the floor (after it settles), the IMU feels the floor push up against gravity.
    settled = readings["accelerations"][len(readings["accelerations"]) // 2 :]
    assert np.mean(settled) == pytest.approx(9.81, abs=0.3)
