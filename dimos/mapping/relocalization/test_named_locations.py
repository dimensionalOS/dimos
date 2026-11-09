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
from typing import Any, cast

import pytest

from dimos.mapping.relocalization.module import PremapModule
from dimos.mapping.relocalization.named_locations import load_locations, location_frame
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
)
from dimos.protocol.tf.tf import TF

# world -> map of a relocalized run: the old map sits 10 m along x and turned a quarter turn
FIX = Transform(
    translation=Vector3(10.0, 0.0, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
    frame_id="world",
    child_frame_id="map",
)


@pytest.fixture
def make(monkeypatch: pytest.MonkeyPatch):
    """A PremapModule on an in-process tf, recording what it publishes."""
    built: list[PremapModule] = []

    def build(
        **config: Any,
    ) -> tuple[PremapModule, list[InteractiveMarkerUpdate]]:
        module = PremapModule(**config)
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        snapshots: list[InteractiveMarkerUpdate] = []
        monkeypatch.setattr(module.named_locations, "publish", snapshots.append)
        monkeypatch.setattr(module.tf, "publish", module.__dict__.setdefault("sent_tf", []).append)
        built.append(module)
        return module, snapshots

    yield build
    for module in built:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def begin(module: PremapModule) -> None:
    """What start() does for named locations once its ports are bound."""
    module.open_locations()
    module.named_locations.publish(module.locations_snapshot())
    if module.config.new_map:
        module.submit(module._identity(), "new map")


def at(x: float, y: float, frame_id: str, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=1.0,
        frame_id=frame_id,
        position=[x, y, 0.0],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def listing(
    poses: dict[str, PoseStamped], heard: InteractiveMarkerUpdate
) -> InteractiveMarkerUpdate:
    """A whole list of named locations, as a publisher sends it carrying the revision it last *heard*."""
    return InteractiveMarkerUpdate(
        [named_pose(name, pose) for name, pose in poses.items()],
        server_id=heard.server_id,
        seq_num=heard.seq_num,
    )


def publish(module: PremapModule, poses: dict[str, PoseStamped]) -> None:
    """A publisher that is up to date sends a new whole list."""
    asyncio.run(module.handle_named_locations(listing(poses, module.locations_snapshot())))


def names(snapshot: InteractiveMarkerUpdate) -> list[str]:
    return [p.name for p in snapshot.poses]


def positions(snapshot: InteractiveMarkerUpdate) -> dict[str, tuple[float, float]]:
    return {
        p.name: (round(p.pose.position.x, 6), round(p.pose.position.y, 6)) for p in snapshot.poses
    }


def test_a_published_list_replaces_the_locations_and_goes_out_as_the_next_revision(make):
    module, published = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map"), "desk": at(3, 4, "map")})
    # rename dock, move desk
    publish(module, {"charger": at(1, 2, "map"), "desk": at(5, 6, "map")})
    assert positions(module.locations_snapshot()) == {"charger": (1, 2), "desk": (5, 6)}
    # delete desk
    publish(module, {"charger": at(1, 2, "map")})
    assert names(module.locations_snapshot()) == ["charger"]

    # start() says what the list is, then every applied list goes out one revision on, so every
    # listener's cache follows
    assert [names(p) for p in published] == [[], ["dock", "desk"], ["charger", "desk"], ["charger"]]
    assert [p.seq_num for p in published] == [0, 1, 2, 3]


def test_its_own_list_coming_back_changes_nothing(make):
    module, published = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map", yaw=0.3)})

    asyncio.run(module.handle_named_locations(published[-1]))
    assert len(published) == 2, "start()'s snapshot and the one edit, and nothing for the echo"


def test_the_last_writer_wins_whatever_revision_it_saw(make):
    """No gate: a publisher that missed a rename still has its list applied, and hears the result."""
    module, published = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map")})
    both_saw = published[-1]

    asyncio.run(module.handle_named_locations(listing({"charger": at(1, 2, "map")}, both_saw)))
    stale = listing({"dock": at(1, 2, "map"), "desk": at(3, 4, "map")}, both_saw)
    asyncio.run(module.handle_named_locations(stale))

    # the second publisher wrote last, so its list is the list -- "dock" is back, by design
    assert names(module.locations_snapshot()) == ["dock", "desk"]
    # and it goes out, so the one that renamed hears it and agrees
    assert names(published[-1]) == ["dock", "desk"]
    assert published[-1].seq_num == 3


def test_a_list_from_another_run_is_applied_too(make):
    """A publisher that outlived a restart carries the old run's server_id; it is not a gate."""
    old_run, _ = make()
    begin(old_run)
    publish(old_run, {"dock": at(1, 2, "map")})

    module, published = make()
    begin(module)
    cached = listing(
        {"dock": at(1, 2, "map"), "desk": at(3, 4, "map")}, old_run.locations_snapshot()
    )
    asyncio.run(module.handle_named_locations(cached))
    assert names(module.locations_snapshot()) == ["dock", "desk"]
    assert names(published[-1]) == ["dock", "desk"]
    assert published[-1].server_id == module.locations_snapshot().server_id


def test_a_request_resends_every_location(make):
    """A viewer that starts after the module asks once and gets them all."""
    module, published = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map")})

    published.clear()
    asyncio.run(module.handle_named_locations_request(String("")))
    assert len(published) == 1
    assert positions(published[-1]) == {"dock": (1, 2)}
    assert all(p.header.frame_id == "map" for p in published[-1].poses)


def test_locations_in_the_world_frame_are_kept_in_the_map_frame(make):
    module, _ = make()
    module.submit(FIX)

    # 1 m in front of the old map's origin: map x points along world y
    publish(module, {"door": at(10, 1, "world")})
    assert positions(module.locations_snapshot()) == {"door": (1, 0)}


def test_a_list_without_a_map_transform_is_ignored_and_the_truth_resent(make):
    module, published = make()
    publish(module, {"door": at(10, 1, "world")})
    assert names(module.locations_snapshot()) == []
    # the publisher is waiting on an echo: it gets the current list, not silence
    assert len(published) == 1 and names(published[-1]) == []


def test_a_new_map_is_the_world_frame(make):
    """Nothing to load: this run's world frame is the map, so a location's pose is the same in both."""
    module, _ = make(new_map=True)
    begin(module)
    publish(module, {"door": at(4, 5, "world")})
    assert positions(module.locations_snapshot()) == {"door": (4, 5)}


def test_locations_are_loaded_from_the_map_recording_at_startup(make, tmp_path):
    from dimos.memory.store.sqlite import SqliteStore

    run1 = str(tmp_path / "run1.db")
    first, published_by_first = make()
    begin(first)
    publish(first, {"dock": at(1, 2, "map", yaw=0.5), "desk": at(3, 4, "map")})
    publish(first, {"dock": at(1, 2, "map", yaw=0.5)})
    # what a recording of named_locations holds
    with SqliteStore(path=run1) as store:
        lists = store.stream("named_locations", InteractiveMarkerUpdate)
        for ts, snapshot in enumerate(published_by_first):
            lists.append(snapshot, ts=float(ts))

    assert list(load_locations(run1, "named_locations")) == ["dock"]

    second, published = make(map_file=run1)
    begin(second)
    # start() publishes the seeded list, so a client that connected before this module hears it
    # without having to ask
    assert positions(published[0]) == {"dock": (1, 2)}
    asyncio.run(second.handle_named_locations_request(String("")))
    assert positions(published[-1]) == {"dock": (1, 2)}
    loaded = published[0].poses[0]
    assert loaded.header.frame_id == "map"
    assert (
        round(Quaternion(*[getattr(loaded.pose.orientation, a) for a in "xyzw"]).to_euler().z, 6)
        == 0.5
    )

    relabelled, published = make(map_file=run1, map_frame="premap")
    begin(relabelled)
    asyncio.run(relabelled.handle_named_locations_request(String("")))
    assert published[0].poses[0].header.frame_id == "premap"
    assert positions(published[0]) == {"dock": (1, 2)}


def test_a_missing_file_loads_nothing(tmp_path):
    assert load_locations(str(tmp_path / "absent.db"), "named_locations") == {}


def test_locations_are_loaded_from_an_mcap(tmp_path):
    mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")
    path = tmp_path / "walk.mcap"
    snapshot = InteractiveMarkerUpdate([named_pose("dock", at(1, 2, "map"))])
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        channel = writer.register_channel(
            topic="named_locations",
            message_encoding="lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.visualization_msgs.InteractiveMarkerUpdate.InteractiveMarkerUpdate"
            },
        )
        writer.add_message(
            channel_id=channel, log_time=1, publish_time=1, data=snapshot.lcm_encode()
        )
        writer.finish()

    loaded = load_locations(str(path), "named_locations")
    assert list(loaded) == ["dock"]
    assert (loaded["dock"].position.x, loaded["dock"].position.y) == (1, 2)


def test_a_list_in_another_frame_goes_through_tf(make):
    """Only world/map come from the module's own fix; any other frame is looked up in tf."""
    module, _ = make()
    module.tfbuffer.receive_transform(
        Transform(translation=Vector3(0.0, 2.0, 0.0), frame_id="map", child_frame_id="desk_frame")
    )
    publish(module, {"lamp": at(1, 0, "desk_frame")})
    assert positions(module.locations_snapshot()) == {"lamp": (1, 2)}


def test_locations_hang_off_the_map_frame_and_follow_every_fix(make):
    """Each location is a tf frame under map, so a new world -> map moves it with the old map."""
    module, _ = make(relocalize_once=False)
    module.submit(FIX)
    publish(module, {"door": at(1, 0, "map")})
    sent: list[TFMessage] = module.__dict__["sent_tf"]
    door = [t for t in sent[-1].transforms if t.child_frame_id == location_frame("door")]
    assert len(door) == 1 and door[0].frame_id == "map"

    def door_in_world() -> tuple[float, float]:
        goal = module.reexpress(module._locations["door"], "world")
        return (round(goal.x, 6), round(goal.y, 6))

    assert door_in_world() == (10, 1)
    module.submit(
        Transform(translation=Vector3(-3.0, 0.0, 0.0), frame_id="world", child_frame_id="map")
    )
    assert door_in_world() == (-2, 0)


def test_a_premap_recording_also_seeds_its_named_locations(make, tmp_path):
    """map_file a recording: the premap is its map_cloud, the locations its last snapshot."""
    import numpy as np

    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    recording = str(tmp_path / "walk.db")
    with SqliteStore(path=recording) as store:
        store.stream("map_cloud", PointCloud2).append(
            PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=1.0), ts=1.0
        )
        store.stream("map_cloud", PointCloud2).append(
            PointCloud2.from_numpy(np.ones((7, 3), dtype=np.float32), timestamp=2.0), ts=2.0
        )
        store.stream("named_locations", InteractiveMarkerUpdate).append(
            InteractiveMarkerUpdate([named_pose("dock", at(1, 2, "map"))]), ts=2.0
        )
    module, _ = make(map_file=recording)
    module.register_disposable = lambda d: d
    module.open_locations()
    module._load_premap(recording)
    assert module.premap is not None and len(module.premap) == 7
    assert module.premap.frame_id == "map"
    assert list(module._locations) == ["dock"]
