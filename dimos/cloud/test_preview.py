# Copyright 2025-2026 Dimensional Inc.
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

import base64
from collections.abc import Iterator, Sequence
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from dimos.cloud import preview
from dimos.cloud.constants import PREVIEW_SCALE
from dimos.memory.store.base import Store
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Stream:
    """The reads preview makes of a memory stream, over real dimos payloads."""

    def __init__(self, name: str, items: Sequence[tuple[float, Any, Pose | None]]) -> None:
        self.name, self.obs = name, [SimpleNamespace(ts=t, data=d, pose=p) for t, d, p in items]

    def __iter__(self) -> Iterator[SimpleNamespace]:
        return iter(self.obs)

    def count(self) -> int:
        return len(self.obs)

    def first(self) -> SimpleNamespace:
        return self.obs[0]


def store(**streams: Sequence[tuple[float, Any, Pose | None]]) -> Store:
    built = {name: Stream(name, items) for name, items in streams.items()}
    return cast("Store", SimpleNamespace(list_streams=lambda: list(built), streams=built))


def points(doc: dict[str, Any], b64: str) -> np.ndarray:
    q = np.frombuffer(base64.b64decode(b64), dtype="<i2").reshape(-1, 3)
    return np.asarray(q * PREVIEW_SCALE + np.array(doc["origin"]))


def cloud(t: float, *xyz: tuple[float, float, float], frame: str = "world") -> PointCloud2:
    return PointCloud2.from_numpy(np.array(xyz, dtype=np.float32), frame_id=frame, timestamp=t)


def test_world_frame_recording() -> None:
    lidar = [
        (100.0 + i, cloud(100.0 + i, (i, 1, 0.5), (i, -1, 0.5), (i, 0, 9.0)), Pose(i * 0.5, 0, 0.3))
        for i in range(10)
    ]
    frames = [
        (100.0 + i, Image.from_numpy(np.full((48, 64, 3), 25 * i, np.uint8)), None)
        for i in range(10)
    ]
    mapper = [(99.0, cloud(99.0, (50, 50, 0.5)), None)]  # listed first, not a scan
    doc = preview.build(store(global_map=mapper, lidar=lidar, color_image=frames))
    assert doc is not None and doc["duration_s"] == 9 and doc["streams"]["lidar"]["name"] == "lidar"
    assert doc["trajectory"][3][:2] == [3.0, 1.5] and len(doc["trajectory"]) == 10
    m = points(doc, doc["map"])
    assert np.abs(m[:, 2] - 0.5).max() < 0.02  # the 9 m "ceiling" is cut, the rest round-trips
    assert {round(x) for x in m[:, 0]} == set(range(10))
    assert doc["thumb"] == len(doc["camera"]) - 1  # the brightest frame


def test_sensor_frame_scans_use_their_own_pose() -> None:
    mount = Pose(
        5.0, 0.0, 0.3, 0.0, 0.0, float(np.sin(np.pi / 4)), float(np.cos(np.pi / 4))
    )  # 90 deg yaw
    lidar = [
        (0.5, cloud(0.5, (9, 9, 0), frame="mid360_link"), None),  # saved before odometry: skipped
        (1.0, cloud(1.0, (1, 0, 0), frame="mid360_link"), mount),
    ]
    doc = preview.build(store(pointlio_lidar=lidar))
    assert doc is not None
    assert np.allclose(points(doc, doc["map"]), [[5.0, 1.0, 0.3]], atol=0.03)
    assert doc["trajectory"][0][1:3] == [5.0, 0.0]


def test_timelapse(tmp_path: Path) -> None:
    frames = [
        (100.0 + i, Image.from_numpy(np.full((48, 64, 3), 8 * i, np.uint8)), None)
        for i in range(30)
    ]
    depth = [(100.0, Image.from_numpy(np.zeros((48, 64), np.uint16)), None)]  # listed first
    meta = preview.timelapse(store(depth_image=depth, color_image=frames), tmp_path / "t.mp4")
    assert (
        meta is not None and meta["speed"] == 1.0 and meta["duration_s"] == 29
    )  # under a minute: real time
    assert (tmp_path / "t.mp4").read_bytes()[4:8] == b"ftyp"  # MP4


def test_path_from_denser_odometry() -> None:
    odom = [
        (100.0 + i, PoseStamped(position=[i, 2, 0.3], frame_id="world"), None) for i in range(5)
    ]
    frames = [(101.0, Image.from_numpy(np.zeros((48, 64, 3), np.uint8)), None)]
    lidar = [(102.0, cloud(102.0, (0, 0, 0.3)), Pose(9, 9, 0.3))]  # one posed scan: odom is denser
    doc = preview.build(store(color_image=frames, odom=odom, lidar=lidar))
    assert doc is not None and len(doc["scans"]) == 1 and doc["duration_s"] == 4
    assert [r[1] for r in doc["trajectory"]] == [0, 1, 2, 3, 4] and len(doc["camera"]) == 1


def test_joystick_samples_ride_along() -> None:
    odom = [
        (100.0 + i, PoseStamped(position=[i, 0, 0.3], frame_id="world"), None) for i in range(3)
    ]
    sticks = [
        (100.5, Joy(axes=[0.0, 0.4567, -1.0, 0.0], buttons=[1, 0, 0]), None),
        (101.5, Joy(axes=[0.0, 0.0, 0.0, 0.0], buttons=[0, 0, 1]), None),
    ]
    doc = preview.build(store(odom=odom, joystick=sticks))
    assert doc is not None and doc["streams"]["joystick"] == {"name": "joystick", "count": 2}
    assert doc["joy"] == [
        [0.5, [0.0, 0.46, -1.0, 0.0], [1, 0, 0]],
        [1.5, [0.0, 0.0, 0.0, 0.0], [0, 0, 1]],
    ]
    bare = preview.build(store(odom=odom))
    assert bare is not None and bare["joy"] == [] and "joystick" not in bare["streams"]
