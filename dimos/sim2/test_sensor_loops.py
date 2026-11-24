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
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
        blind_to_robots: Sequence[str] = (),
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
def l1(monkeypatch: pytest.MonkeyPatch) -> Iterator[Any]:
    from dimos.sim2.l1 import SimL1

    sensor = SimL1(instance_name="left/l1")
    monkeypatch.setattr(sensor, "_rider_odometry", lambda rider: ("left/odom", np.eye(4)))
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


SPOT_XY, SPOT_YAW = (2.0, 1.0), np.pi / 2


def _toy_world(sensor: Any, mount: str) -> Any:
    """A real MuJoCo view of the toy dog standing in a scene, with ``sensor`` grafted on at ``mount``."""
    from pathlib import Path

    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.frames import prefix_usd_frames
    from dimos.sim2.assets.toy import toy_robot
    from dimos.sim2.sim import compose_world
    from dimos.sim2.spec import ODOM_FRAME
    from dimos.sim2.usd_layer import layer_file
    from dimos.sim2.world_observations import WorldMirror, WorldWriter

    class _Part:
        def __init__(self, usd: str, mounted_on: str | None) -> None:
            self.usd, self.on = usd, mounted_on

        def get_usd(self) -> str:
            return self.usd

        def mounted_on(self) -> str | None:
            return self.on

    scene = Path(__file__).parent / "mujoco" / "assets" / "scene.usda"
    room = UsdModel.from_file(scene).with_frames(
        {"dog": ("scene", (*SPOT_XY, 0.0), (0.0, 0.0, SPOT_YAW))}
    )
    dog = toy_robot().with_frames(
        {
            "lidar": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "l1": ("base", (0.0, 0.0, 0.0), (np.pi, 0.0, 0.0)),
            "l1_up": ("base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    world_usd = compose_world(
        [
            _Part(room.usd, None),
            _Part(prefix_usd_frames(dog.usd, "dog"), "dog"),
            _Part(sensor.get_usd(), mount),
        ]
    ).usd
    writer, mirror = WorldWriter(), WorldMirror()
    writer.start_ns = 0
    writer.add("world")
    writer.add("dog", model="dog.usda", **{ODOM_FRAME: "dog/odom"})
    writer.position("dog", [*SPOT_XY, 0.3, 0, 0, np.sin(SPOT_YAW / 2), np.cos(SPOT_YAW / 2)])
    mirror.apply(writer.end_frame(1.0))

    async def ticks(rate_hz: float) -> AsyncIterator[float]:
        for _ in range(TICKS):
            yield mirror.time

    mirror.ticks = ticks
    sensor._world = mirror
    sensor.world_layer = lambda: layer_file(world_usd, "toy-world")
    return mirror


@pytest.mark.self_hosted  # the Mid-360 scan pattern is Git LFS data
def test_a_livox_in_its_housing_sees_past_the_housing() -> None:
    from dimos.sim2.livox import LivoxSim

    livox = LivoxSim(instance_name="dog/lidar/livox", mount="dog/lidar")
    published: list[Any] = []
    try:
        _toy_world(livox, "dog/lidar")
        livox.lidar.publish = published.append
        livox.publish_usd_tf = lambda ts=None: None
        asyncio.run(livox._scan())
    finally:
        livox.stop()
    assert published and all(len(cloud.as_numpy()[0]) > 0 for cloud in published)


def test_an_l1_maps_in_the_odometry_of_the_robot_it_rides() -> None:
    from dimos.sim2.l1 import SimL1

    l1 = SimL1(instance_name="dog/l1/l1", mount="dog/l1")
    published: list[Any] = []
    try:
        _toy_world(l1, "dog/l1")
        l1.lidar.publish = published.append
        l1.publish_usd_tf = lambda ts=None: None
        asyncio.run(l1._map())
    finally:
        l1.stop()
    cloud = published[-1]
    points, _ = cloud.as_numpy()
    floor = points[points[:, 2] < l1.config.cell]
    assert cloud.frame_id == "dog/odom"
    assert len(floor) > 0 and np.abs(floor[:, 2]).max() <= l1.config.cell
    assert np.abs(floor[:, :2].mean(axis=0)).max() < 0.5, (
        "the floor is mapped around the odom origin"
    )


def test_an_l1_maps_none_of_the_robot_it_rides() -> None:
    from dimos.sim2.l1 import SimL1

    l1 = SimL1(instance_name="dog/l1_up/l1", mount="dog/l1_up")
    published: list[Any] = []
    try:
        _toy_world(l1, "dog/l1_up")
        l1.lidar.publish = published.append
        l1.publish_usd_tf = lambda ts=None: None
        asyncio.run(l1._map())
    finally:
        l1.stop()
    points, _ = published[-1].as_numpy()
    # The head sits on the neck joint 0.3 m ahead of and 0.2 m above the base, which stands 0.3 m up in odom.
    head = np.array([0.3, 0.0, 0.5])
    near_the_head = np.linalg.norm(points - head, axis=1) < 2 * l1.config.cell
    assert len(points) > 0 and not near_the_head.any(), "the L1 mapped the dog's own head"
