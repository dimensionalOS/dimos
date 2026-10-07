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
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.cloud import preview

if TYPE_CHECKING:
    from dimos.memory.store.base import Store as RealStore


class PointCloud2:
    def __init__(self, pts: list[list[float]], frame_id: str = "world") -> None:
        self.pts, self.frame_id = np.array(pts, dtype=np.float32), frame_id

    def points_f32(self) -> np.ndarray:
        return self.pts


class PoseStamped:
    def __init__(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.position = SimpleNamespace(x=x, y=y, z=0.3)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=np.sin(yaw / 2), w=np.cos(yaw / 2))


class Image:
    width = 64

    def __init__(self, level: int) -> None:
        self.level = level

    def as_numpy(self) -> np.ndarray:
        return np.full((4, 4, 3), self.level, dtype=np.uint8)

    def to_jpeg_bytes(self, quality: int = 75) -> bytes:
        return b"\xff\xd8" + bytes([self.level])


@dataclass
class Obs:
    ts: float
    data: Any
    pose: Any = None


@dataclass
class Stream:
    name: str
    items: list[Obs] = field(default_factory=list)

    def first(self) -> Obs:
        if not self.items:
            raise LookupError("No matching observation")
        return self.items[0]

    def to_list(self) -> list[Obs]:
        return list(self.items)

    def __iter__(self) -> Any:
        return iter(self.items)

    def count(self) -> int:
        return len(self.items)


class Store:
    def __init__(self, *streams: Stream) -> None:
        self.streams = {s.name: s for s in streams}

    def list_streams(self) -> list[str]:
        return list(self.streams)


def unpack(doc: dict[str, Any], b64: str) -> np.ndarray:
    q = np.frombuffer(base64.b64decode(b64), dtype="<i2").reshape(-1, 3)
    return np.asarray(q * float(doc["scale"]) + np.array(doc["origin"], dtype=np.float64))


def build(store: Store, **kw: Any) -> dict[str, Any] | None:
    return preview.build(cast("RealStore", store), **kw)


def test_world_frame_recording() -> None:
    odom = Stream("odom", [Obs(100 + i, PoseStamped(i * 0.5, 0.0, 0.1 * i)) for i in range(10)])
    lidar = Stream(
        "lidar",
        [
            Obs(100 + i, PointCloud2([[i, 1.0, 0.5], [i, -1.0, 0.5], [i, 0, 9.0]]))
            for i in range(10)
        ],
    )
    camera = Stream(
        "color_image", [Obs(100 + i, Image(level=10 + i * 20 % 200)) for i in range(10)]
    )
    doc = build(Store(odom, lidar, camera, Stream("empty")), frames=4)
    assert doc is not None
    assert doc["format"] == preview.FORMAT and doc["duration_s"] == 9
    assert doc["trajectory"][0][:3] == [0.0, 0.0, 0.0] and len(doc["trajectory"]) == 10
    assert len(doc["scans"]) == 4 and len(doc["camera"]) == 4
    m = unpack(doc, doc["map"])
    assert (
        np.abs(m[:, 2] - 0.5).max() < 0.02
    )  # the 9 m "ceiling" point is cut, the rest round-trips
    assert {round(x) for x in m[:, 0]} == set(range(10))
    levels = [base64.b64decode(c["jpeg"])[2] for c in doc["camera"]]
    assert doc["thumb"] == int(np.argmax(levels))
    assert set(doc["streams"]) == {"pose", "lidar", "camera"}


def test_sensor_frame_clouds_use_their_own_pose() -> None:
    mount = SimpleNamespace(
        position=SimpleNamespace(x=5.0, y=0.0, z=0.3),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=np.sin(np.pi / 4), w=np.cos(np.pi / 4)),
    )  # 90 deg yaw
    lidar = Stream(
        "pointlio_lidar", [Obs(1.0, PointCloud2([[1.0, 0.0, 0.0]], "mid360_link"), pose=mount)]
    )
    other_odom = Stream("go2_odom", [Obs(1.0, PoseStamped(-50.0, -50.0))])  # a different frame
    doc = build(Store(lidar, other_odom))
    assert doc is not None
    p = unpack(doc, doc["map"])[0]
    assert np.allclose(p, [5.0, 1.0, 0.3], atol=0.03)  # rotated by the mount yaw, then translated
    assert doc["trajectory"][0][1:3] == [5.0, 0.0]  # trajectory from the same poses, not go2_odom


def test_nothing_to_preview() -> None:
    assert build(Store(Stream("empty"))) is None
