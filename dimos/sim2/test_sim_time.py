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


import asyncio
from collections.abc import AsyncIterator
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
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import graft_usd, prefix_usd_frames, usd_with_drives
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.camera import SimCamera
from dimos.sim2.imu import SimImu
from dimos.sim2.l1 import SimL1
from dimos.sim2.livox import LivoxSim
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import ACT, WorldMirror

# Every scene here is read out of a USD layer, and a runner without usd-core cannot read one:
# linux-aarch64 has no wheel for it (pyproject's marker says so), so without this the tests
# fail there on dimos's own "Reading or writing USD needs usd-core (pxr)". Their neighbours
# in this directory skip the same way.
pytest.importorskip("pxr")

L1_PERIOD = 0.13
RATES = {
    "color_image": 10.0,
    "imu": 50.0,
    "lidar": 10.0,
    "livox_imu": 50.0,
    "l1_lidar": 1 / L1_PERIOD,
}


SCENE_MANAGER = f"scene/{UsdManager.name}"
FRAMES = {"eye": "head", "imu_link": "base", "livox": "head", "l1": "base"}


@pytest.fixture
def robot_model() -> UsdModel:
    model = toy_robot().with_frames(
        {frame: (body, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) for frame, body in FRAMES.items()}
    )
    return UsdModel.from_layer(usd_with_drives(model.usd, {"neck": (0.01, 0.001, 0.0)}))


@pytest.fixture
def composed(robot_model: UsdModel) -> UsdModel:
    room = UsdModel.from_file(LOGISTICS).with_frames(
        {"dog": ("logistics", (-9.0, -9.0, -8.7), (0.0, 0.0, 0.0))}
    )
    return UsdModel.from_layer(
        graft_usd(room.usd, {"dog": prefix_usd_frames(robot_model.usd, "dog")})
    )


class _DogConfig(ModuleConfig):
    control_period: float | None = None
    think: float = 0.0


class _Dog(Module):
    config: _DogConfig
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]

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
    composed: UsdModel,
    robot_model: UsdModel,
    seconds: float,
    speed: float = 1.0,
    **dog: Any,
) -> dict[str, Any]:
    blueprint = autoconnect(
        sim_module(
            scene_manager=SCENE_MANAGER,
            speed=speed,
            robots=[SimulatedRobot("dog", robot_model, control_period=dog.get("control_period"))],
        ),
        _Dog.blueprint(**dog),
        UsdManager.blueprint(
            model=composed, frame_id_prefix="", tf_renames={"logistics": "world"}
        ).namespace("scene", expose={"render_file_request", "file_info", "tf"}),
        SimCamera.blueprint(
            frame_id="eye",
            width=64,
            height=48,
            rate_hz=RATES["color_image"],
            scene_manager=SCENE_MANAGER,
        ),
        SimImu.blueprint(frame_id="imu_link", rate_hz=RATES["imu"], scene_manager=SCENE_MANAGER),
        LivoxSim.blueprint(
            frame_id="livox",
            rate_hz=RATES["lidar"],
            imu_rate_hz=RATES["livox_imu"],
            scene_manager=SCENE_MANAGER,
        ).remappings([(LivoxSim, "imu", "livox_imu")]),
        SimL1.blueprint(frame_id="l1", period=L1_PERIOD, scene_manager=SCENE_MANAGER).remappings(
            [(SimL1, "lidar", "l1_lidar")]
        ),
        _Readings.blueprint(),
    ).global_config(n_workers=9)
    overrides = {"g": {"viewer": "none", "zenoh_domain": f"sim2-test-{uuid.uuid4().hex[:8]}"}}
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        readings = coordinator.get_instance(_Readings)
        deadline = time.monotonic() + seconds + MAX_EXTRA_WALL
        time.sleep(seconds)
        while time.monotonic() < deadline and not _steady(readings.readings()["stamps"]):
            time.sleep(1.0)
        return readings.readings()
    finally:
        coordinator.stop()


MAX_EXTRA_WALL = 60.0
STEADY_STAMPS = 8


def _steady(stamps: dict[str, list[float]]) -> bool:
    if not all(stamps[name] for name in RATES):
        return False
    start = min(stamps[name][0] for name in RATES)
    return all(
        sum(stamp >= start + WARM_UP for stamp in stamps[name]) >= STEADY_STAMPS for name in RATES
    )


WARM_UP = 1.0


def _per_sim_second(stamps: list[float], start: float) -> float:
    steady = [stamp for stamp in stamps if stamp >= start + WARM_UP]
    assert len(steady) > 5, f"only {len(steady)} after warming up"
    return (len(steady) - 1) / (steady[-1] - steady[0])


@pytest.mark.parametrize(
    ("speed", "dog"),
    [
        (0.5, {}),
        (4.0, {}),
        (1.0, {"control_period": 0.02, "think": 0.05}),
    ],
    ids=["half_speed", "four_times_speed", "slower_than_real_lockstep"],
)
def test_sensors_keep_their_rates_per_sim_second(composed, robot_model, speed, dog):
    readings = _run(composed, robot_model, seconds=8.0, speed=speed, **dog)
    stamps = readings["stamps"]
    counts = {name: len(stamps[name]) for name in RATES}
    assert all(count > 5 for count in counts.values()), counts
    start = min(stamps[name][0] for name in RATES)
    sim_seconds = stamps["imu"][-1] - stamps["imu"][0]
    for name, rate in RATES.items():
        measured = _per_sim_second(stamps[name], start)
        assert measured == pytest.approx(rate, rel=0.1), (name, measured)
    if dog:
        assert sim_seconds < 0.7 * readings["wall"]
    settled = readings["accelerations"][len(readings["accelerations"]) // 2 :]
    assert np.mean(settled) == pytest.approx(9.81, abs=0.3)
