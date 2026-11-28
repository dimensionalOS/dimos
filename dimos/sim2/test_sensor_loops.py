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
import math
import time
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
        blind_to_robots: Sequence[str] = (),
    ) -> _Raycaster:
        self.mounted_at.append(mounted_at)
        return _Raycaster(self.world)

    def twin(self) -> "_View":
        return self


def _run(
    sensor: Any, loop: str, stream: str, monkeypatch: pytest.MonkeyPatch, opening: float = 0.0
) -> dict[str, Any]:
    """Run ``sensor``'s ``loop`` in a _World, its view taking ``opening`` sim seconds to open."""
    world = _World()
    view = _View(world)
    published: list[Any] = []
    sensor._world = world

    def open_view(*_mount: Any) -> _View:
        world.time += opening
        return view

    monkeypatch.setattr(sensor, "open_view", open_view)
    monkeypatch.setattr(getattr(sensor, stream), "publish", published.append)
    asyncio.run(getattr(sensor, loop)())
    return {"view": view, "published": published, "ticked": world.ticked}


@pytest.fixture
def l1(monkeypatch: pytest.MonkeyPatch) -> Iterator[Any]:
    from dimos.sim2.l1 import SimL1

    sensor = SimL1(instance_name="left/l1", stamp_tolerance=math.inf)
    monkeypatch.setattr(sensor, "_rider_odometry", lambda rider: ("left/odom", np.eye(4)))
    yield sensor
    sensor.stop()


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


@pytest.mark.parametrize(
    ("sensor", "loop", "stream"),
    [
        ("l1", "_map", "lidar"),
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


SPOT_XY, SPOT_YAW = (2.0, 1.0), np.pi / 2
# The scene's frame the dog mounts on, and so the dog's namespace.
DOG = "scene/dog"
# The dog's frames a sensor mounts on (its raw USD names them by prim path), each the namespace of what mounts there.
L1_MOUNT = f"{DOG}/toy/Geometry/base/l1"
L1_UP_MOUNT = f"{DOG}/toy/Geometry/base/l1_up"


def _providing_hardware_files(sensor: Any) -> str:
    """Wire ``sensor``'s hardware_file streams to the topics a blueprint gives them and answer on them; its namespace."""
    from dimos.core.hardware_files import hardware_file_topics
    from dimos.core.transport_factory import make_transport
    from dimos.msgs.files.FileInfo import FileInfo

    namespace: str = sensor.namespace
    request_topic, response_topic = hardware_file_topics(namespace)
    sensor.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    sensor.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    sensor.provide_hardware_files()
    return namespace


def _toy_world(sensor: Any, mount: str, monkeypatch: pytest.MonkeyPatch) -> Any:
    """A real MuJoCo view of the toy dog standing in a scene, with ``sensor`` grafted on at ``mount``."""
    from pathlib import Path

    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.layers import usd_under_namespace
    from dimos.sim2.scene import layer_file, toy_robot
    from dimos.sim2.sim import (
        ODOM_FRAME,
        ROBOT,
        WorldMirror,
        WorldWriter,
        compose_usds,
        fetched_usd,
    )

    scene = Path(__file__).parent / "mujoco" / "assets" / "scene.usda"
    room = UsdModel.from_file(scene).with_frames(
        {"dog": ("scene", (*SPOT_XY, 0.0), (0.0, 0.0, SPOT_YAW))}
    )
    dog = toy_robot().with_frames(
        {
            "l1": ("toy/Geometry/base", (0.0, 0.0, 0.0), (np.pi, 0.0, 0.0)),
            "l1_up": ("toy/Geometry/base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    world_usd = compose_usds(
        [
            (room.usd, None),
            (usd_under_namespace(dog.usd, DOG), DOG),
            # The sensor's own files come over its hardware_file streams, as in a running blueprint.
            (fetched_usd(_providing_hardware_files(sensor)), mount),
        ]
    ).usd
    writer, mirror = WorldWriter(), WorldMirror()
    writer.start_ns = 0
    writer.add("world")
    writer.add(DOG, **{ROBOT: True, ODOM_FRAME: f"{DOG}/odom"})
    still = [0.0] * 6
    writer.body_state(
        DOG, [*SPOT_XY, 0.3, 0, 0, np.sin(SPOT_YAW / 2), np.cos(SPOT_YAW / 2)], still, still
    )
    mirror.apply(writer.end_frame(1.0))

    async def ticks(rate_hz: float) -> AsyncIterator[float]:
        for _ in range(TICKS):
            yield mirror.time

    monkeypatch.setattr(mirror, "ticks", ticks)
    sensor._world = mirror
    sensor.world_layer = lambda: layer_file(world_usd, "toy-world")
    return mirror


def test_an_l1_maps_in_the_odometry_of_the_robot_it_rides(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.sim2.l1 import SimL1

    l1 = SimL1(instance_name=f"{L1_MOUNT}/l1", attached_to="l1", stamp_tolerance=math.inf)
    published: list[Any] = []
    try:
        _toy_world(l1, L1_MOUNT, monkeypatch)
        monkeypatch.setattr(l1.lidar, "publish", published.append)
        asyncio.run(l1._map())
    finally:
        l1.stop()
    cloud = published[-1]
    points, _ = cloud.as_numpy()
    floor = points[points[:, 2] < l1.config.cell]
    assert cloud.frame_id == f"{DOG}/odom"
    assert len(floor) > 0 and np.abs(floor[:, 2]).max() <= l1.config.cell
    assert np.abs(floor[:, :2].mean(axis=0)).max() < 0.5, (
        "the floor is mapped around the odom origin"
    )


def test_an_l1_maps_none_of_the_robot_it_rides(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.sim2.l1 import SimL1

    l1 = SimL1(instance_name=f"{L1_UP_MOUNT}/l1", attached_to="l1_up", stamp_tolerance=math.inf)
    published: list[Any] = []
    try:
        _toy_world(l1, L1_UP_MOUNT, monkeypatch)
        monkeypatch.setattr(l1.lidar, "publish", published.append)
        asyncio.run(l1._map())
    finally:
        l1.stop()
    points, _ = published[-1].as_numpy()
    # The head sits on the neck joint 0.3 m ahead of and 0.2 m above the base, which stands 0.3 m up in odom.
    head = np.array([0.3, 0.0, 0.5])
    near_the_head = np.linalg.norm(points - head, axis=1) < 2 * l1.config.cell
    assert len(points) > 0 and not near_the_head.any(), "the L1 mapped the dog's own head"


def test_an_l1_in_a_world_far_off_wall_time_sends_nothing(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.sim2.l1 import SimL1

    l1 = SimL1(instance_name="left/l1")
    monkeypatch.setattr(l1, "_rider_odometry", lambda rider: ("left/odom", np.eye(4)))
    try:
        run = _run(l1, "_map", "lidar", monkeypatch)
    finally:
        l1.stop()
    assert run["published"] == []


def test_a_reading_off_wall_time_by_more_than_its_tolerance_is_dropped_and_warned_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos.sim2 import sim
    from dimos.sim2.imu import SimImu

    imu = SimImu(instance_name="dog/imu/simimu", attached_to="imu")
    warned: list[str] = []
    monkeypatch.setattr(sim.logger, "warning", warned.append)
    period = 1 / imu.config.rate_hz
    try:
        now = time.time()
        sent = [imu.on_time("imu", stamp, period, None) for stamp in (now, now - 1, now - 1)]
        sent.append(imu.on_time("imu", time.time(), period, None))
        sent.append(imu.on_time("imu", time.time() - 1, period, None))
        tolerated = imu.on_time("imu", time.time() - 1, period, 2.0)
    finally:
        imu.stop()

    assert sent == [True, False, False, True, False]
    # Once when it fell behind, and again only after it had caught up.
    assert len(warned) == 2
    assert tolerated
