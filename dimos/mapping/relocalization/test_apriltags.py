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

import numpy as np
import pytest

from dimos.mapping.relocalization.apriltags import (
    TagClustering,
    TagDetection,
    TagGates,
    cluster_by_time,
    cluster_tags,
    marker_poses,
    recording_tags,
    robust_cluster_pose,
    tags_message,
)
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.protocol.tf.tf import TF

FACING = [0.0, 0.0, 0.0, 1.0]
GOOD = {"sharpness": 500.0, "reproj_px": 0.3, "tag_px": 60.0, "lin_speed": 0.1, "ang_speed": 5.0}


def glimpse(ts: float, marker_id: int, x: float, y: float = 0.0, z: float = 0.0) -> TagDetection:
    return TagDetection(ts, marker_id, [x, y, z, *FACING])


def test_huber_medoid_ignores_a_few_wild_glimpses():
    """A plain mean would be dragged 0.33 m by the outliers; the robust pose stays put."""
    rng = np.random.default_rng(0)
    cluster = [glimpse(i * 0.1, 7, 1.0 + rng.normal(0, 0.005)) for i in range(10)]
    cluster += [glimpse(1.1, 7, 3.0), glimpse(1.2, 7, 3.0)]
    pose = robust_cluster_pose(cluster, TagClustering())
    assert abs(pose.pose[0] - 1.0) < 0.02
    assert abs(float(np.mean([d.pose[0] for d in cluster])) - 1.0) > 0.3


def test_glimpses_split_into_visits_and_thin_visits_are_dropped():
    visits = [glimpse(t, 1, 2.0) for t in (0.0, 0.5, 1.0)]  # a visit
    visits += [glimpse(t, 1, 2.0) for t in (20.0, 20.5)]  # too thin to trust
    visits += [glimpse(t, 2, 5.0) for t in (0.0, 1.0, 2.0)]
    assert sorted(len(c) for c in cluster_by_time(visits, gap_sec=5.0)) == [2, 3, 3]
    clustered = cluster_tags(visits, TagClustering(gap_sec=5.0, min_observations=3))
    assert sorted(clustered) == [1, 2]
    assert round(clustered[2].pose[0], 6) == 5.0


def write_recording(path: Path, glimpses: list[tuple[float, dict[str, Any], float]]) -> None:
    """A recording whose camera sits 2 m along x of its map frame, looking at tags 0.5 m ahead."""
    with SqliteStore(path=str(path)) as store:
        store.stream("map_cloud", PointCloud2).append(
            PointCloud2.from_numpy(
                np.zeros((4, 3), dtype=np.float32), timestamp=0.0, frame_id="odom"
            ),
            ts=0.0,
        )
        tf = store.stream("tf", TFMessage)
        for ts in np.arange(0.0, 10.0, 0.5):
            tf.append(
                TFMessage(
                    Transform(
                        translation=Vector3(2.0, 0.0, 0.0),
                        frame_id="odom",
                        child_frame_id="camera_optical",
                        ts=float(ts),
                    )
                ),
                ts=float(ts),
            )
        tags = store.stream("april_tags_raw", PoseStamped)
        for ts, quality, x in glimpses:
            tags.append(
                PoseStamped(
                    ts=ts, frame_id="camera_optical", position=[x, 0.0, 0.5], orientation=FACING
                ),
                ts=ts,
                tags={"marker_id": 4, **quality},
            )


def test_tags_from_a_recording_land_in_its_map_frame(tmp_path):
    """Placed through the recording's own tf at each glimpse's time; blurry glimpses gated."""
    path = tmp_path / "walk.db"
    write_recording(
        path,
        [
            (1.0, GOOD, 0.0),
            (1.5, GOOD, 0.01),
            (2.0, GOOD, -0.01),
            (2.5, {**GOOD, "sharpness": 1.0}, 0.4),  # blurred, and wrong
        ],
    )
    tags = recording_tags(str(path), ("april_tags_raw",), "odom", TagGates(), TagClustering())
    assert list(tags) == [4]
    assert np.allclose(tags[4].pose[:3], [2.0, 0.0, 0.5], atol=0.011)


def test_the_tag_message_round_trips_ids_and_poses():
    message = tags_message({4: glimpse(1.0, 4, 1.5, 2.5)}, "map", 1.0)
    decoded = Detection3DArray.lcm_decode(message.lcm_encode())
    [(marker_id, pose)] = marker_poses(decoded)
    assert (marker_id, pose.frame_id, pose.position.x, pose.position.y) == (4, "map", 1.5, 2.5)


@pytest.fixture
def relocalizer(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    """A relocalizer on the recording above, on an in-process tf, recording what it publishes."""
    path = tmp_path / "walk.db"
    write_recording(path, [(1.0, GOOD, 0.0), (1.5, GOOD, 0.0), (2.0, GOOD, 0.0)])
    module = RelocalizationModule(map_file=str(path), world_frame="world", relocalize_once=False)
    cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
    sent: dict[str, list[Any]] = {"tf": [], "old": [], "new": []}
    monkeypatch.setattr(module.tf, "publish", sent["tf"].append)
    monkeypatch.setattr(module.old_april_tags, "publish", sent["old"].append)
    monkeypatch.setattr(module.new_april_tags, "publish", sent["new"].append)
    monkeypatch.setattr(module.loaded_map, "publish", lambda _: None)
    module.register_disposable = lambda d: d  # type: ignore[method-assign,assignment,return-value]
    module._load_premap(str(path))
    yield module, sent
    cast("dict[str, Any]", module.__dict__)["_tf"] = None
    module.dispose()


def fix(x: float, yaw: float = 0.0) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        frame_id="world",
        child_frame_id="map",
    )


def frames(message: TFMessage) -> dict[str, Transform]:
    return {t.child_frame_id: t for t in message.transforms}


def in_world(module: RelocalizationModule, frame: str, sent: TFMessage) -> tuple[float, float]:
    """Where a published frame sits in the world, chaining it up to world as tf would."""
    transforms = frames(sent)
    chain = transforms[frame]
    while chain.frame_id != "world":
        chain = transforms[chain.frame_id] + chain
    return (round(chain.translation.x, 6), round(chain.translation.y, 6))


def test_old_tags_ride_on_every_fix_and_new_tags_stay_put(relocalizer):
    """Old tags hang off the old map, so each new world -> map moves them; live tags don't move."""
    module, sent = relocalizer
    assert list(module.old_tags) == [4]

    live = tags_message({4: glimpse(3.0, 4, 7.0, 1.0)}, "world", 3.0)
    for _ in range(3):  # enough glimpses for a cluster
        asyncio.run(module.handle_marker_detections(live))
    assert list(module.new_tags) == [4]
    assert frames(sent["tf"][-1])["tag_4"].frame_id == "world"

    module.submit(fix(10.0))
    module.publish_tf()
    first = sent["tf"][-1]
    assert frames(first)["tag_old_4"].frame_id == "map"
    assert in_world(module, "tag_old_4", first) == (12.0, 0.0)
    assert in_world(module, "tag_4", first) == (7.0, 1.0)

    module.submit(fix(-1.0, yaw=math.pi / 2))
    module.publish_tf()
    second = sent["tf"][-1]
    assert in_world(module, "tag_old_4", second) == (-1.0, 2.0)
    assert in_world(module, "tag_4", second) == (7.0, 1.0)

    # two separate outputs: the old set in map, the live set in world
    # (the old set goes out with the premap on each fix)
    assert [p.frame_id for _, p in marker_poses(sent["old"][-1])] == ["map"]
    assert [p.frame_id for _, p in marker_poses(sent["new"][-1])] == ["world"]


def write_mcap(path: Path, topics: dict[str, list[tuple[float, Any]]]) -> None:
    """A dimos .mcap as a recording writes it: ``dimos/<port>/<msg_name>`` topics, lcm-encoded."""
    from mcap.writer import Writer

    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos", library="test")
        for topic, messages in topics.items():
            kind = type(messages[0][1])
            channel = writer.register_channel(
                topic=f"dimos/{topic.strip('/')}/{kind.msg_name}",
                message_encoding="lcm",
                schema_id=0,
            )
            for ts, message in messages:
                nanoseconds = int(ts * 1e9)
                writer.add_message(
                    channel_id=channel,
                    log_time=nanoseconds,
                    publish_time=nanoseconds,
                    data=message.lcm_encode(),
                )
        writer.finish()


def test_an_mcap_recording_gives_the_premap_its_tags_and_its_locations(tmp_path, monkeypatch):
    """/map_cloud, /tf, a marker detector's /marker_detections and /named_locations of an .mcap."""
    from dimos.mapping.relocalization.named_locations import load_locations
    from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
        InteractiveMarkerUpdate,
        named_pose,
    )

    path = tmp_path / "walk.mcap"
    camera = Transform(
        translation=Vector3(2.0, 0.0, 0.0), frame_id="odom", child_frame_id="camera_optical"
    )
    seen = tags_message({4: glimpse(0.0, 4, 0.0, 0.0, 0.5)}, "camera_optical", 0.0)
    write_mcap(
        path,
        {
            "/map_cloud": [
                (
                    0.5,
                    PointCloud2.from_numpy(
                        np.zeros((9, 3), dtype=np.float32), timestamp=0.5, frame_id="odom"
                    ),
                )
            ],
            "/tf": [
                (ts, TFMessage(Transform(**{**vars(camera), "ts": ts})))
                for ts in np.arange(0.0, 2.0, 0.25)
            ],
            "/marker_detections": [(ts, seen) for ts in (0.5, 1.0, 1.5)],
            "/named_locations": [
                (
                    1.0,
                    InteractiveMarkerUpdate(
                        [
                            named_pose(
                                "dock", PoseStamped(ts=1.0, frame_id="map", position=[1, 2, 0])
                            )
                        ]
                    ),
                )
            ],
        },
    )
    module = RelocalizationModule(map_file=str(path))
    module.register_disposable = lambda d: d  # type: ignore[method-assign,assignment,return-value]
    module._load_premap(str(path))
    assert module.premap is not None and len(module.premap) == 9
    assert list(module.old_tags) == [4]
    assert np.allclose(module.old_tags[4].pose[:3], [2.0, 0.0, 0.5], atol=1e-6)
    assert list(load_locations(str(path), "named_locations")) == ["dock"]
    module.dispose()
