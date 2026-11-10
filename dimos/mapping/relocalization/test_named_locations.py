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

from dimos.mapping.relocalization.module import PremapModule, map_names, resolve_map
from dimos.mapping.relocalization.named_locations import load_locations
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.String import String
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
    if module.config.map_file:
        module._resolve_map(module.config.map_file)
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
    # rename dock, move desk, then delete desk
    publish(module, {"charger": at(1, 2, "map"), "desk": at(5, 6, "map")})
    assert positions(module.locations_snapshot()) == {"charger": (1, 2), "desk": (5, 6)}
    publish(module, {"charger": at(1, 2, "map")})
    assert names(module.locations_snapshot()) == ["charger"]

    # start() says what the list is, then every applied list goes out one revision on
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

    # the second publisher wrote last, so "dock" is back and the one that renamed hears it
    assert names(module.locations_snapshot()) == ["dock", "desk"]
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


def test_an_empty_request_on_the_shared_topic_resends_the_list(make):
    """A recording that starts later asks for the list, so its snapshot ends up in the recording."""
    module, published = make()
    begin(module)
    publish(module, {"dock": at(1, 2, "map")})

    published.clear()
    asyncio.run(module.handle_render_file_request(String("")))
    assert len(published) == 1
    assert positions(published[-1]) == {"dock": (1, 2)}
    assert all(p.header.frame_id == "map" for p in published[-1].poses)

    # another module's file: this one stays silent
    asyncio.run(module.handle_render_file_request(String("/some/robot.urdf")))
    assert len(published) == 1


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
    assert len(published) == 1 and names(published[-1]) == []


def test_a_new_map_is_the_world_frame(make):
    """Nothing to load: this run's world frame is the map, so a location's pose is the same in both."""
    module, _ = make(new_map=True)
    begin(module)
    publish(module, {"door": at(4, 5, "world")})
    assert positions(module.locations_snapshot()) == {"door": (4, 5)}


def test_locations_are_loaded_from_the_map_recording_at_startup(make, tmp_path):
    import numpy as np

    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    run1 = str(tmp_path / "run1.db")
    first, published_by_first = make()
    begin(first)
    publish(first, {"dock": at(1, 2, "map", yaw=0.5), "desk": at(3, 4, "map")})
    publish(first, {"dock": at(1, 2, "map", yaw=0.5)})
    # what a recording of the map's named_locations holds
    with SqliteStore(path=run1) as store:
        store.stream("map_v1_cloud", PointCloud2).append(
            PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=0.0), ts=0.0
        )
        lists = store.stream("map_v1_named_locations", InteractiveMarkerUpdate)
        for ts, snapshot in enumerate(published_by_first):
            lists.append(snapshot, ts=float(ts))

    assert list(load_locations(run1, "map_v1_named_locations")) == ["dock"]

    second, published = make(map_file=run1)
    begin(second)
    # start() publishes the seeded list, so a client that connected first hears it unasked
    assert positions(published[0]) == {"dock": (1, 2)}
    assert all(p.header.frame_id == "map/v1" for p in published[0].poses)
    loaded = published[0].poses[0]
    assert loaded.header.frame_id == "map/v1"
    assert (
        round(Quaternion(*[getattr(loaded.pose.orientation, a) for a in "xyzw"]).to_euler().z, 6)
        == 0.5
    )

    # the map names its own frame, so a configured map_frame does not relabel a loaded one
    relabelled, published = make(map_file=run1, map_frame="premap")
    begin(relabelled)
    assert published[0].poses[0].header.frame_id == "map/v1"
    assert positions(published[0]) == {"dock": (1, 2)}


def write_mcap(path, channels):
    """An .mcap holding one message per (topic, type, message), the way a recorder writes them."""
    mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        for topic, kind, message in channels:
            channel = writer.register_channel(
                topic=topic,
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": f"{kind.__module__}.{kind.__name__}",
                },
            )
            writer.add_message(
                channel_id=channel, log_time=1, publish_time=1, data=message.lcm_encode()
            )
        writer.finish()


def test_a_named_map_is_found_and_names_the_frame(make, tmp_path):
    """One `map/<name>/cloud`: it is the map, `map/<name>` is the frame, its own locations load."""
    import numpy as np

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    path = tmp_path / "walk.mcap"
    cloud = PointCloud2.from_numpy(np.ones((7, 3), dtype=np.float32), timestamp=2.0)
    write_mcap(
        path,
        [
            ("map/v1/cloud", PointCloud2, cloud),
            (
                "map/v1/named_locations",
                InteractiveMarkerUpdate,
                InteractiveMarkerUpdate([named_pose("dock", at(1, 2, "map/v1"))]),
            ),
            # the flat streams an older build left behind are not what gets picked
            (
                "map_cloud",
                PointCloud2,
                PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=1.0),
            ),
            (
                "named_locations",
                InteractiveMarkerUpdate,
                InteractiveMarkerUpdate([named_pose("desk", at(3, 4, "map"))]),
            ),
        ],
    )

    module, _ = make(map_file=str(path))
    module.register_disposable = lambda d: d
    module._resolve_map(str(path))
    assert module.map_frame == "map/v1"
    module.open_locations()
    module._load_premap(str(path))
    assert module.premap is not None and len(module.premap) == 7
    assert module.premap.frame_id == "map/v1"
    assert list(module._locations) == ["dock"]


def test_which_map_to_load():
    """resolve_map: a lone named map, a picked one, several, and a recording from before names."""
    one = ["/tf", "map/v1/cloud", "map/v1/named_locations"]
    assert map_names(one) == {"v1": "map/v1/cloud"}
    assert resolve_map(one, None) == ("map/v1/cloud", "map/v1", "map/v1/named_locations")
    # a map with no locations of its own: the stream is still its own, there is just nothing on it
    assert resolve_map(["map/v1/cloud"], None) == (
        "map/v1/cloud",
        "map/v1",
        "map/v1/named_locations",
    )
    # a recording with no named map has no map at all: say so rather than guess at a flat stream
    with pytest.raises(ValueError, match="no map in the recording"):
        resolve_map(["map_cloud", "/tf"], None)
    # several maps: neither is "the" map, so say so instead of guessing
    two = ["map/v1/cloud", "map/v2/cloud"]
    assert sorted(map_names(two)) == ["v1", "v2"]
    with pytest.raises(ValueError, match="2 maps in the recording"):
        resolve_map(two, None)
    # ... unless one is asked for, which also names the frame
    assert resolve_map(two, "map/v2/cloud") == ("map/v2/cloud", "map/v2", "map/v2/named_locations")
    # asked for something that is not one of its maps
    with pytest.raises(ValueError, match="is not a map in this recording"):
        resolve_map(two, "map_cloud")
    # an .mcap topic keeps its leading slash
    assert resolve_map(["/map/v3/cloud"], None) == (
        "/map/v3/cloud",
        "map/v3",
        "map/v3/named_locations",
    )


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


def test_locations_are_kept_in_the_map_frame_and_follow_every_fix(make):
    """A location is held in the old map's frame, so a new world -> map moves it with that map."""
    module, _ = make(relocalize_once=False)
    module.submit(FIX)
    publish(module, {"door": at(1, 0, "map")})

    def door_in_world() -> tuple[float, float]:
        goal = module.reexpress(module._locations["door"], "world")
        return (round(goal.x, 6), round(goal.y, 6))

    assert door_in_world() == (10, 1)
    module.submit(
        Transform(translation=Vector3(-3.0, 0.0, 0.0), frame_id="world", child_frame_id="map")
    )
    assert door_in_world() == (-2, 0)


def test_a_premap_recording_also_seeds_its_named_locations(make, tmp_path):
    """map_file a .db: the premap is the map's last cloud, the locations its last snapshot."""
    import numpy as np

    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    recording = str(tmp_path / "walk.db")
    with SqliteStore(path=recording) as store:
        clouds = store.stream("map_v1_cloud", PointCloud2)
        clouds.append(
            PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=1.0), ts=1.0
        )
        clouds.append(
            PointCloud2.from_numpy(np.ones((7, 3), dtype=np.float32), timestamp=2.0), ts=2.0
        )
        store.stream("map_v1_named_locations", InteractiveMarkerUpdate).append(
            InteractiveMarkerUpdate([named_pose("dock", at(1, 2, "map/v1"))]), ts=2.0
        )
    module, _ = make(map_file=recording)
    module.register_disposable = lambda d: d
    module._resolve_map(recording)
    module.open_locations()
    module._load_premap(recording)
    assert module.premap is not None and len(module.premap) == 7
    assert module.premap.frame_id == "map/v1"
    assert list(module._locations) == ["dock"]
