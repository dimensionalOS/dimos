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

"""What each sensor's loop reads from its view, with a world that moves on while it works."""

import asyncio
from collections.abc import AsyncIterator, Iterator, Sequence
from typing import Any

import numpy as np
import pytest

pytest.importorskip("pxr")

TICKS = 3


class _World:
    """A world that is a second further on every time something casts into it."""

    def __init__(self) -> None:
        self.time = 1.0

    def stamp(self) -> float:
        return self.time

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        for _ in range(TICKS):
            yield self.time


class _Raycaster:
    def __init__(self, world: _World) -> None:
        self.world = world

    def cast(self, origin: Any, directions: Any, max_range: float) -> tuple[Any, Any]:
        self.world.time += 1.0
        count = len(directions)
        return np.ones(count), np.tile([0.0, 0.0, 1.0], (count, 1))


class _View:
    def __init__(self, world: _World) -> None:
        self.world, self.time = world, 0.0
        self.mounted_at: list[str | None] = []
        self.imu_frames: list[str] = []

    def update(self, world: _World) -> None:
        self.time = world.time

    def pose(self, frame: str) -> tuple[Any, Any]:
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])

    def owner(self, frame: str) -> str | None:
        return None

    def raycaster(
        self, groups: Sequence[int], mounted_at: str | None = None, blind_to: Sequence[str] = ()
    ) -> _Raycaster:
        self.mounted_at.append(mounted_at)
        return _Raycaster(self.world)

    def imu(self, sensor: str) -> tuple[Any, Any, Any]:
        self.imu_frames.append(sensor)
        return np.zeros(3), np.array([0.0, 0.0, 9.81]), np.array([1.0, 0.0, 0.0, 0.0])


def _run(sensor: Any, loop: str, stream: str, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    world = _World()
    view = _View(world)
    published: list[Any] = []
    transforms: list[float | None] = []
    sensor._world = world
    monkeypatch.setattr(sensor, "_open", lambda: view)
    monkeypatch.setattr(getattr(sensor, stream), "publish", published.append)
    monkeypatch.setattr(sensor, "publish_usd_tf", lambda ts=None: transforms.append(ts))
    asyncio.run(getattr(sensor, loop)())
    return {"view": view, "published": published, "transforms": transforms}


@pytest.fixture
def livox() -> Iterator[Any]:
    from dimos.sim2.livox import LivoxSim

    sensor = LivoxSim(instance_name="left/livox")
    yield sensor
    sensor.stop()


@pytest.fixture
def l1() -> Iterator[Any]:
    from dimos.sim2.l1 import SimL1

    sensor = SimL1(instance_name="left/l1")
    yield sensor
    sensor.stop()


@pytest.mark.self_hosted  # the Mid-360 scan pattern is Git LFS data
def test_a_livox_cloud_is_stamped_with_the_world_it_was_cast_in(
    livox: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    run = _run(livox, "_scan", "lidar", monkeypatch)
    cast_at = [1.0 + tick for tick in range(TICKS)]
    assert [cloud.ts for cloud in run["published"]] == cast_at
    assert run["transforms"] == cast_at


def test_a_livox_imu_is_read_at_the_frame_it_is_labelled_with(
    livox: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    run = _run(livox, "_read_imu", "imu", monkeypatch)
    assert set(run["view"].imu_frames) == {livox.imu_frame_id}
    assert {reading.frame_id for reading in run["published"]} == {livox.imu_frame_id}


def test_an_l1_does_not_map_the_body_it_is_mounted_on(
    l1: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    run = _run(l1, "_map", "lidar", monkeypatch)
    assert run["view"].mounted_at == [l1.frame_id]


def test_an_l1_map_is_stamped_with_the_world_it_was_cast_in(
    l1: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    run = _run(l1, "_map", "lidar", monkeypatch)
    cast_at = [1.0 + tick for tick in range(TICKS)]
    assert [cloud.ts for cloud in run["published"]] == cast_at
    assert run["transforms"] == cast_at
