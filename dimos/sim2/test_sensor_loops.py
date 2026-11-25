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
        self.ticked: list[float] = []

    def stamp(self) -> float:
        return self.time

    def entity(self, name: str) -> None:
        # No world entity, so its geometry never changes and the views are built once.
        return None

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        for _ in range(TICKS):
            self.ticked.append(self.time)
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

    def twin(self) -> "_View":
        return self

    def close(self) -> None:
        pass

    def imu(self, sensor: str) -> tuple[Any, Any, Any]:
        self.imu_frames.append(sensor)
        return np.zeros(3), np.array([0.0, 0.0, 9.81]), np.array([1.0, 0.0, 0.0, 0.0])


def _run(
    sensor: Any, loop: str, stream: str, monkeypatch: pytest.MonkeyPatch, opening: float = 0.0
) -> dict[str, Any]:
    """Run ``sensor``'s ``loop`` in a _World, its view taking ``opening`` sim seconds to open."""
    world = _World()
    view = _View(world)
    published: list[Any] = []
    transforms: list[float | None] = []
    sensor._world = world

    def open_view() -> _View:
        world.time += opening
        return view

    monkeypatch.setattr(sensor, "_open", open_view)
    monkeypatch.setattr(getattr(sensor, stream), "publish", published.append)
    monkeypatch.setattr(sensor, "publish_usd_tf", lambda ts=None: transforms.append(ts))
    asyncio.run(getattr(sensor, loop)())
    return {"view": view, "published": published, "transforms": transforms, "ticked": world.ticked}


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
    # The first tick opens the view, and each after it casts.
    cast_at = [1.0 + tick for tick in range(TICKS - 1)]
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
    # The first tick opens the view, and each after it casts.
    cast_at = [1.0 + tick for tick in range(TICKS - 1)]
    assert [cloud.ts for cloud in run["published"]] == cast_at
    assert run["transforms"] == cast_at


@pytest.mark.parametrize(
    ("sensor", "loop", "stream"),
    [
        ("l1", "_map", "lidar"),
        # the Mid-360 scan pattern is Git LFS data
        pytest.param("livox", "_scan", "lidar", marks=pytest.mark.self_hosted),
        ("livox", "_read_imu", "imu"),
    ],
)
def test_a_sensor_slow_to_open_still_stamps_each_reading_with_its_own_tick(
    sensor: str,
    loop: str,
    stream: str,
    request: pytest.FixtureRequest,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run = _run(request.getfixturevalue(sensor), loop, stream, monkeypatch, opening=10.0)
    stamps = [reading.ts for reading in run["published"]]
    # One reading per tick after the one spent opening, each stamped with its tick's time.
    assert stamps == run["ticked"][1:]


class _Clock(_World):
    """A world whose ticks come at their rate in sim time, each loop's interleaved with the others'."""

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        for tick in range(int(SECONDS * rate_hz)):
            # Another loop runs between any two of this one's ticks.
            await asyncio.sleep(0)
            self.time = 1.0 + tick / rate_hz
            yield self.time


SECONDS = 1.0


@pytest.mark.self_hosted  # the Mid-360 scan pattern is Git LFS data
def test_a_livox_opens_one_view_for_its_lidar_and_imu_and_reads_both_at_their_rates(
    livox: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _Clock()
    opened: list[_View] = []
    published: dict[str, list[float]] = {"lidar": [], "imu": []}
    livox._world = world

    def open_view() -> _View:
        opened.append(_View(world))
        return opened[-1]

    monkeypatch.setattr(livox, "_open", open_view)
    for stream, stamps in published.items():
        monkeypatch.setattr(getattr(livox, stream), "publish", lambda m, s=stamps: s.append(m.ts))
    monkeypatch.setattr(livox, "publish_usd_tf", lambda ts=None: None)

    async def both() -> None:
        await asyncio.gather(livox._scan(), livox._read_imu())

    asyncio.run(both())
    assert len(opened) == 1
    # Both loops' first tick is spent opening it, and each reads every tick after.
    for stamps, rate in (
        (published["lidar"], livox.config.rate_hz),
        (published["imu"], livox.config.imu_rate_hz),
    ):
        assert stamps == pytest.approx(
            [1.0 + tick / rate for tick in range(1, int(SECONDS * rate))]
        )


class _Reshaped(_Clock):
    """A clock whose world changes its shapes once, half way through."""

    def entity(self, name: str) -> Any:
        from types import SimpleNamespace

        from dimos.sim2.world_observations import GEOMETRY

        return SimpleNamespace(cbor={GEOMETRY: int(self.time >= 1.0 + SECONDS / 2)})


@pytest.mark.self_hosted  # the Mid-360 scan pattern is Git LFS data
def test_a_livox_lidar_and_imu_rebuild_one_view_when_the_world_changes_shape(
    livox: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _Reshaped()
    opened: list[_View] = []
    published: dict[str, list[float]] = {"lidar": [], "imu": []}
    livox._world = world

    def open_view() -> _View:
        opened.append(_View(world))
        return opened[-1]

    monkeypatch.setattr(livox, "_open", open_view)
    for stream, stamps in published.items():
        monkeypatch.setattr(getattr(livox, stream), "publish", lambda m, s=stamps: s.append(m.ts))
    monkeypatch.setattr(livox, "publish_usd_tf", lambda ts=None: None)

    async def both() -> None:
        await asyncio.gather(livox._scan(), livox._read_imu())

    asyncio.run(both())
    assert len(opened) == 2
    reshaped = 1.0 + SECONDS / 2
    assert all(any(stamp > reshaped for stamp in stamps) for stamps in published.values())


@pytest.mark.self_hosted  # the Mid-360 scan pattern is Git LFS data
def test_a_livox_lidar_and_imu_share_one_real_view(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.sim2.livox import LivoxSim

    livox = LivoxSim(instance_name="dog/lidar/livox", mount="dog/lidar")
    opened: list[Any] = []
    clouds: list[Any] = []
    readings: list[Any] = []
    try:
        _toy_world(livox, "dog/lidar")
        open_view = livox._open

        def counted_open() -> Any:
            opened.append(open_view())
            return opened[-1]

        monkeypatch.setattr(livox, "_open", counted_open)
        monkeypatch.setattr(livox.lidar, "publish", clouds.append)
        monkeypatch.setattr(livox.imu, "publish", readings.append)
        monkeypatch.setattr(livox, "publish_usd_tf", lambda ts=None: None)

        async def both() -> None:
            await asyncio.gather(livox._scan(), livox._read_imu())

        asyncio.run(both())
    finally:
        livox.stop()
    assert len(opened) == 1
    assert clouds and all(len(cloud.as_numpy()[0]) > 0 for cloud in clouds)
    # The dog stands still, so its IMU reads gravity alone.
    assert readings and all(
        np.linalg.norm(r.linear_acceleration.to_numpy()) == pytest.approx(9.81, abs=0.01)
        for r in readings
    )


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
