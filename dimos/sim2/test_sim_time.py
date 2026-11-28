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

from collections.abc import AsyncIterator
import time
from typing import Any
import uuid

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import UsdModel
from dimos.sim2.assets.logistics import Logistics
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.imu import SimImu
from dimos.sim2.l1 import SimL1
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.sim import on_mount, sim
from dimos.sim2.spec import SimulatedRobot

pytest.importorskip("pxr")

L1_PERIOD = 0.13
RATES = {
    "imu": 50.0,
    "l1_lidar": 1 / L1_PERIOD,
}
_BASE = "toy/Geometry/base"
# Each sensor's mount on the toy dog, and the frame it hangs off.
MOUNTS = {"imu": _BASE, "l1": _BASE}


class _DogConfig(HardwareModuleConfig):
    # Its base where its spawn point stands, not 9 m off it as the toy's own USD puts it.
    usd: UsdModel = UsdModel.from_layer(
        toy_robot().usd.replace("(9, 9, 9)", "(0, 0, 0.3)"), toy_robot().source_path
    ).with_frames(
        {frame: (body, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) for frame, body in MOUNTS.items()}
    )


class _Dog(HardwareModule):
    """A toy robot nothing drives, so the world is not held for it."""

    config: _DogConfig
    mounts = Mounts(*(f"{body}/{frame}" for frame, body in MOUNTS.items()))

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(self.namespace, self.config.usd, odom_frame=self.attach_frame)


class _Readings(Module):
    imu: In[Imu]
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


SENSOR_STREAMS = {"imu", "l1_lidar"}


def _run(seconds: float, speed: float) -> dict[str, Any]:
    dog = autoconnect(
        _Dog.blueprint(attached_to=Logistics.mounts.go2.name),
        # A mount namespaces its sensor's streams; these take them back out to _Readings.
        SimImu.attach(_Dog.mounts.imu, rate_hz=RATES["imu"]).remappings([(SimImu, "imu", "imu")]),
        SimL1.attach(_Dog.mounts.l1, period=L1_PERIOD).remappings([(SimL1, "lidar", "l1_lidar")]),
    )
    blueprint = autoconnect(
        sim({"mujoco": MujocoSim.blueprint(speed=speed)}, default="mujoco"),
        Logistics.blueprint(),
        on_mount(dog, Logistics.mounts.go2, shared=SENSOR_STREAMS),
        _Readings.blueprint(),
    ).solidify()
    overrides = {"g": {"viewer": "none", "zenoh_domain": f"sim2-test-{uuid.uuid4().hex[:8]}"}}
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        readings = coordinator.get_instance(_Readings)
        deadline = time.monotonic() + seconds + MAX_EXTRA_WALL
        time.sleep(seconds)
        while time.monotonic() < deadline and not _steady(readings.readings()["stamps"]):
            time.sleep(1.0)
        result: dict[str, Any] = readings.readings()
        return result
    finally:
        coordinator.stop()


MAX_EXTRA_WALL = 60.0
STEADY_STAMPS = 8
WARM_UP = 1.0


def _steady(stamps: dict[str, list[float]]) -> bool:
    if not all(stamps[name] for name in RATES):
        return False
    start = min(stamps[name][0] for name in RATES)
    return all(
        sum(stamp >= start + WARM_UP for stamp in stamps[name]) >= STEADY_STAMPS for name in RATES
    )


def _per_second(stamps: list[float], start: float) -> float:
    steady = [stamp for stamp in stamps if stamp >= start + WARM_UP]
    assert len(steady) > 5, f"only {len(steady)} after warming up"
    return (len(steady) - 1) / (steady[-1] - steady[0])


def test_sensors_keep_their_rates_in_real_time() -> None:
    readings = _run(seconds=8.0, speed=1.0)
    stamps = readings["stamps"]
    counts = {name: len(stamps[name]) for name in RATES}
    assert all(count > 5 for count in counts.values()), counts
    start = min(stamps[name][0] for name in RATES)
    for name, rate in RATES.items():
        measured = _per_second(stamps[name], start)
        assert measured == pytest.approx(rate, rel=0.1), (name, measured)
    settled = readings["accelerations"][len(readings["accelerations"]) // 2 :]
    assert np.mean(settled) == pytest.approx(9.81, abs=0.3)
