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
import math
from pathlib import Path
from typing import Any, cast

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.String import String
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
)
from dimos.navigation.named_locations.module import NamedLocations, load_locations
from dimos.protocol.tf.tf import TF

# world -> map of a relocalized run: the old map sits 10 m along x and turned a quarter turn
FIX = Transform(
    translation=Vector3(10.0, 0.0, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
    frame_id="world",
    child_frame_id="map",
)


@pytest.fixture
def make(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    """A NamedLocations on an in-process tf, recording what it publishes."""
    built: list[NamedLocations] = []

    def build(
        **config: Any,
    ) -> tuple[NamedLocations, list[InteractiveMarkerUpdate], list[PointStamped]]:
        config.setdefault("save_to", str(tmp_path / "named_locations.db"))
        module = NamedLocations(**config)
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        snapshots: list[InteractiveMarkerUpdate] = []
        goals: list[PointStamped] = []
        monkeypatch.setattr(module.named_locations, "publish", snapshots.append)
        monkeypatch.setattr(module.goal, "publish", goals.append)
        built.append(module)
        return module, snapshots, goals

    yield build
    for module in built:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def begin(module: NamedLocations) -> None:
    """What start() does once its ports are bound."""
    module.open()


def at(x: float, y: float, frame_id: str, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=1.0,
        frame_id=frame_id,
        position=[x, y, 0.0],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def listing(poses: dict[str, PoseStamped]) -> InteractiveMarkerUpdate:
    """A whole list of named locations, as a viewer publishes it."""
    return InteractiveMarkerUpdate([named_pose(name, pose) for name, pose in poses.items()])


def publish(module: NamedLocations, poses: dict[str, PoseStamped]) -> None:
    asyncio.run(module.handle_named_locations(listing(poses)))


def names(snapshot: InteractiveMarkerUpdate) -> list[str]:
    return [p.name for p in snapshot.poses]


def positions(snapshot: InteractiveMarkerUpdate) -> dict[str, tuple[float, float]]:
    return {
        p.name: (round(p.pose.position.x, 6), round(p.pose.position.y, 6)) for p in snapshot.poses
    }


def saved(path: str) -> list[list[str]]:
    """Every list appended to a save file, oldest first."""
    from dimos.memory.store.sqlite import SqliteStore

    with SqliteStore(path=path, must_exist=True) as store:
        return [names(obs.data) for obs in store.stream("named_locations").order_by("ts")]


def test_a_published_list_replaces_the_locations_without_a_reply(make, tmp_path):
    module, published, _ = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map"), "desk": at(3, 4, "map")})
    # rename dock, move desk
    publish(module, {"charger": at(1, 2, "map"), "desk": at(5, 6, "map")})
    assert positions(module.snapshot()) == {"charger": (1, 2), "desk": (5, 6)}
    # delete desk
    publish(module, {"charger": at(1, 2, "map")})
    assert names(module.snapshot()) == ["charger"]

    # the module never answers a list: the publisher already has what it sent
    assert published == []
    assert saved(str(tmp_path / "named_locations.db")) == [
        ["dock", "desk"],
        ["charger", "desk"],
        ["charger"],
    ]


def test_its_own_list_coming_back_changes_nothing(make, tmp_path):
    module, published, _ = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map", yaw=0.3)})
    asyncio.run(module.handle_named_locations_request(String("")))

    asyncio.run(module.handle_named_locations(published[-1]))
    assert saved(str(tmp_path / "named_locations.db")) == [["dock"]]


def test_a_request_resends_every_location(make):
    """A viewer that starts after the module asks once and gets them all."""
    module, published, _ = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map")})

    asyncio.run(module.handle_named_locations_request(String("")))
    assert len(published) == 1
    assert positions(published[-1]) == {"dock": (1, 2)}
    assert all(p.header.frame_id == "map" for p in published[-1].poses)


def test_locations_in_the_world_frame_are_kept_in_the_map_frame(make):
    module, _, _ = make(load_from="none_yet.db")
    module.tfbuffer.receive_transform(FIX)

    # 1 m in front of the old map's origin: map x points along world y
    publish(module, {"door": at(10, 1, "world")})
    assert positions(module.snapshot()) == {"door": (1, 0)}


def test_a_list_without_a_map_transform_is_ignored(make):
    module, _, _ = make(load_from="none_yet.db")
    publish(module, {"door": at(10, 1, "world")})
    assert names(module.snapshot()) == []


def test_go_to_sends_the_goal_in_the_world_frame_through_the_fix(make):
    module, _, goals = make(load_from="none_yet.db")
    module.tfbuffer.receive_transform(FIX)
    publish(module, {"door": at(1, 0, "map")})

    asyncio.run(module.handle_go_to_named_location(String("door")))
    asyncio.run(module.handle_go_to_named_location(String("nowhere")))

    assert len(goals) == 1
    assert goals[0].frame_id == "world"
    assert (round(goals[0].x, 6), round(goals[0].y, 6)) == (10, 1)


def test_go_to_waits_for_relocalization(make):
    module, _, goals = make(load_from="none_yet.db")
    module._locations = {"door": at(1, 0, "map")}
    asyncio.run(module.handle_go_to_named_location(String("door")))
    assert goals == []


def test_a_new_map_is_the_world_frame(make):
    """Nothing to load: this run's world frame is the map, so locations and goals coincide."""
    module, _, goals = make()
    begin(module)
    publish(module, {"door": at(4, 5, "world")})
    asyncio.run(module.handle_go_to_named_location(String("door")))
    assert (goals[0].x, goals[0].y) == (4, 5)


def test_locations_are_loaded_from_the_saved_file_at_startup(make, tmp_path):
    run1 = str(tmp_path / "run1.db")
    first, _, _ = make(save_to=run1)
    begin(first)
    publish(first, {"dock": at(1, 2, "map", yaw=0.5), "desk": at(3, 4, "map")})
    publish(first, {"dock": at(1, 2, "map", yaw=0.5)})
    first.dispose()

    assert list(load_locations(run1, "named_locations")) == ["dock"]

    second, published, _ = make(load_from=run1, save_to=str(tmp_path / "run2.db"))
    begin(second)
    assert published == []
    asyncio.run(second.handle_named_locations_request(String("")))
    assert positions(published[0]) == {"dock": (1, 2)}
    loaded = published[0].poses[0]
    assert loaded.header.frame_id == "map"
    assert (
        round(Quaternion(*[getattr(loaded.pose.orientation, a) for a in "xyzw"]).to_euler().z, 6)
        == 0.5
    )


def test_a_missing_file_loads_nothing(tmp_path):
    assert load_locations(str(tmp_path / "absent.db"), "named_locations") == {}
