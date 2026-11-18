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

"""What the headset feed sends for the wrist camera's pictures and clouds."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import math
from typing import Any, cast

import cv2
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.teleop.webxr import headset_feed
from dimos.teleop.webxr.headset_feed import HeadsetFeedModule


@pytest.fixture
def feed() -> Iterator[HeadsetFeedModule]:
    module = HeadsetFeedModule(tf_forward_tolerance_s=0.0)
    # A local transform buffer stands in for the tf stream.
    cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
    yield module
    cast("dict[str, Any]", module.__dict__)["_tf"] = None
    module.dispose()


def _mount_camera(feed: HeadsetFeedModule, at: Vector3, yaw: float, ts: float) -> None:
    feed.tfbuffer.receive_transform(
        Transform(
            translation=at,
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            frame_id="world",
            child_frame_id="camera",
            ts=ts,
        )
    )


def _coloured_cloud(
    points: np.ndarray,
    frame_id: str = "camera",
    ts: float = 1.0,
    colour: tuple[float, float, float] = (1.0, 0.0, 0.0),
) -> PointCloud2:
    """Points of one colour, red unless told."""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    cloud.colors = o3d.utility.Vector3dVector(np.tile(colour, (len(points), 1)))
    return PointCloud2(pointcloud=cloud, frame_id=frame_id, ts=ts)


def _sent_clouds(feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch) -> list[PointCloud2]:
    """What goes out on vr_cloud; each arrival is one second after the last, so every one is due."""
    clock = [100.0]

    def tick() -> float:
        clock[0] += 1.0
        return clock[0]

    monkeypatch.setattr(headset_feed.time, "monotonic", tick)
    sent: list[PointCloud2] = []
    feed.vr_cloud.subscribe(sent.append)
    return sent


def _arrive(feed: HeadsetFeedModule, cloud: PointCloud2) -> None:
    asyncio.run(feed.handle_pointcloud(cloud))


def _sorted_points(cloud: PointCloud2) -> np.ndarray:
    points = cloud.points_f32()
    return points[np.lexsort(points.T[::-1])]


def test_a_cloud_is_placed_in_the_world_at_its_capture_time(feed: HeadsetFeedModule) -> None:
    # The camera stands 1 m along world x, turned 90 degrees: 1 m ahead of it is world (1, 1, 0).
    _mount_camera(feed, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    # Where it went next does not move a cloud captured before.
    _mount_camera(feed, Vector3(5.0, 0.0, 0.0), 0.0, ts=1.5)

    light = feed.light_cloud(_coloured_cloud(np.array([[1.0, 0.0, 0.0]]), ts=1.0))

    assert light is not None
    assert light.frame_id == "world"
    np.testing.assert_allclose(light.points_f32(), [[1.0, 1.0, 0.0]], atol=1e-5)


def test_a_cloud_keeps_one_point_per_voxel_and_its_colours(feed: HeadsetFeedModule) -> None:
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)
    # 1000 points inside one 1.5 cm cell, and one point 10 cm away.
    rng = np.random.default_rng(0)
    clump = rng.uniform(0.001, 0.014, size=(1000, 3))
    cloud = _coloured_cloud(np.vstack([clump, [[0.1, 0.0, 0.0]]]))

    light = feed.light_cloud(cloud)

    assert light is not None
    assert len(light) == 2
    colours = light.pointcloud_tensor.point["colors"].numpy()
    np.testing.assert_allclose(colours, [[1.0, 0.0, 0.0]] * 2, atol=1e-5)


def test_a_cloud_without_a_camera_pose_is_not_sent(feed: HeadsetFeedModule) -> None:
    sent: list[PointCloud2] = []
    feed.vr_cloud.subscribe(sent.append)

    asyncio.run(feed.handle_pointcloud(_coloured_cloud(np.array([[1.0, 0.0, 0.0]]))))

    assert sent == []


def test_clouds_and_pictures_are_sent_at_most_at_their_rates(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock = [100.0]
    monkeypatch.setattr(headset_feed.time, "monotonic", lambda: clock[0])
    # Each cloud sent as it is, not merged into the map, so every one taken goes out.
    monkeypatch.setattr(feed.config, "accumulate", False)
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)
    clouds: list[PointCloud2] = []
    pictures: list[CompressedImage] = []
    feed.vr_cloud.subscribe(clouds.append)
    feed.vr_camera.subscribe(pictures.append)
    image = Image(np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=1.0)

    def arrive() -> None:
        asyncio.run(feed.handle_pointcloud(_coloured_cloud(np.array([[1.0, 0.0, 0.0]]))))
        asyncio.run(feed.handle_color_image(image))

    # Five arrivals at once: only the first of each goes out.
    for _ in range(5):
        arrive()
    assert (len(clouds), len(pictures)) == (1, 1)

    # 0.2 s on, a picture is due (8 a second) but a cloud is not (4 a second).
    clock[0] += 0.2
    arrive()
    assert (len(clouds), len(pictures)) == (1, 2)

    # 0.35 s from the start, both are due.
    clock[0] += 0.15
    arrive()
    assert (len(clouds), len(pictures)) == (2, 3)


def test_a_faster_camera_is_thinned_to_the_set_rate(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock = [100.0]
    monkeypatch.setattr(headset_feed.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(feed.config, "accumulate", False)
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)
    clouds: list[PointCloud2] = []
    feed.vr_cloud.subscribe(clouds.append)

    # 10 s of a 5 Hz camera, each cloud a little late or early.
    for k in range(50):
        clock[0] = 100.0 + 0.2 * k + (0.003 if k % 2 else -0.003)
        asyncio.run(feed.handle_pointcloud(_coloured_cloud(np.array([[1.0, 0.0, 0.0]]))))

    # 4 a second (cloud_hz), not every second cloud (2.5 a second).
    assert 39 <= len(clouds) <= 41


def test_a_picture_goes_out_as_a_small_jpeg(feed: HeadsetFeedModule) -> None:
    image = Image(
        np.full((720, 1280, 3), 128, dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam", ts=2.0
    )

    picture = feed.light_picture(image)

    assert picture.format == "jpeg"
    assert (picture.frame_id, picture.ts) == ("cam", 2.0)
    decoded = cv2.imdecode(np.frombuffer(picture.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (360, 640, 3)


def test_clouds_from_two_viewpoints_grow_into_one(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent = _sent_clouds(feed, monkeypatch)
    ahead = np.array([[1.0, 0.0, 0.0]])
    # The same spot 1 m ahead of the camera, seen facing world +x and then world +y.
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)
    _arrive(feed, _coloured_cloud(ahead, ts=1.0))
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), math.pi / 2, ts=2.0)
    _arrive(feed, _coloured_cloud(ahead, ts=2.0))

    assert len(sent) == 2
    assert sent[-1].frame_id == "world"
    np.testing.assert_allclose(
        _sorted_points(sent[-1]), [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]], atol=1e-5
    )


def test_a_newer_point_replaces_the_one_in_its_cell(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent = _sent_clouds(feed, monkeypatch)
    _mount_camera(feed, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)
    # Two points 5 mm apart, inside one 2 cm cell of the map.
    _arrive(feed, _coloured_cloud(np.array([[0.302, 0.302, 0.302]]), colour=(1.0, 0.0, 0.0)))
    _arrive(feed, _coloured_cloud(np.array([[0.307, 0.307, 0.307]]), colour=(0.0, 0.0, 1.0)))

    assert len(sent[-1]) == 1
    np.testing.assert_allclose(sent[-1].points_f32(), [[0.307, 0.307, 0.307]], atol=1e-5)
    colours = sent[-1].pointcloud_tensor.point["colors"].numpy()
    np.testing.assert_allclose(colours, [[0.0, 0.0, 1.0]], atol=1e-5)


def test_a_full_map_keeps_its_cells_and_drops_new_ones(monkeypatch: pytest.MonkeyPatch) -> None:
    feed = HeadsetFeedModule(accumulate_max_points=3)
    try:
        sent = _sent_clouds(feed, monkeypatch)
        # Already in the world frame, so no transform is needed. Each x is mid-cell.
        for x in (0.01, 0.11, 0.21, 0.31):
            _arrive(feed, _coloured_cloud(np.array([[x, 0.0, 0.0]]), frame_id="world"))
        np.testing.assert_allclose(_sorted_points(sent[-1])[:, 0], [0.01, 0.11, 0.21], atol=1e-5)

        # Full: a blue point in the cell of 0.01 replaces it; the one at 0.41 is dropped.
        both = np.array([[0.015, 0.0, 0.0], [0.41, 0.0, 0.0]])
        _arrive(feed, _coloured_cloud(both, frame_id="world", colour=(0.0, 0.0, 1.0)))

        points = sent[-1].points_f32()
        np.testing.assert_allclose(np.sort(points[:, 0]), [0.015, 0.11, 0.21], atol=1e-5)
        colours = sent[-1].pointcloud_tensor.point["colors"].numpy()
        updated = np.isclose(points[:, 0], 0.015, atol=1e-5)
        np.testing.assert_allclose(colours[updated], [[0.0, 0.0, 1.0]], atol=1e-5)
        np.testing.assert_allclose(colours[~updated], [[1.0, 0.0, 0.0]] * 2, atol=1e-5)
    finally:
        feed.dispose()


def test_a_cloud_bigger_than_the_room_left_fills_the_map_and_no_more(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    feed = HeadsetFeedModule(accumulate_max_points=3)
    try:
        sent = _sent_clouds(feed, monkeypatch)
        five = np.array([[0.01 + 0.1 * k, 0.0, 0.0] for k in range(5)])
        _arrive(feed, _coloured_cloud(five, frame_id="world"))

        assert len(sent[-1]) == 3
    finally:
        feed.dispose()


def test_the_map_has_its_own_cell_size(monkeypatch: pytest.MonkeyPatch) -> None:
    feed = HeadsetFeedModule(cloud_voxel_m=0.01, map_voxel_m=0.1)
    try:
        sent = _sent_clouds(feed, monkeypatch)
        # 5 cm apart: two cells of the 1 cm cloud grid, one cell of the 10 cm map grid.
        pair = _coloured_cloud(np.array([[0.02, 0.02, 0.02], [0.07, 0.02, 0.02]]), frame_id="world")

        light = feed.light_cloud(pair)
        _arrive(feed, pair)

        assert light is not None and len(light) == 2
        assert len(sent[-1]) == 1
    finally:
        feed.dispose()


def test_clear_cloud_starts_the_map_again(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent = _sent_clouds(feed, monkeypatch)
    _arrive(feed, _coloured_cloud(np.array([[0.0, 0.0, 0.0]]), frame_id="world"))

    feed.clear_cloud()
    _arrive(feed, _coloured_cloud(np.array([[0.5, 0.0, 0.0]]), frame_id="world"))

    np.testing.assert_allclose(sent[-1].points_f32(), [[0.5, 0.0, 0.0]], atol=1e-5)


def test_a_cloud_without_a_camera_pose_is_not_added(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent = _sent_clouds(feed, monkeypatch)
    _arrive(feed, _coloured_cloud(np.array([[0.0, 0.0, 0.0]]), frame_id="world"))
    # No transform for "camera" yet: dropped, not merged where it sits in the camera frame.
    _arrive(feed, _coloured_cloud(np.array([[0.5, 0.0, 0.0]])))
    _arrive(feed, _coloured_cloud(np.array([[0.0, 0.5, 0.0]]), frame_id="world"))

    np.testing.assert_allclose(
        _sorted_points(sent[-1]), [[0.0, 0.0, 0.0], [0.0, 0.5, 0.0]], atol=1e-5
    )


def test_the_map_goes_out_at_its_own_rate(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock = [100.0]
    monkeypatch.setattr(headset_feed.time, "monotonic", lambda: clock[0])
    sent: list[PointCloud2] = []
    feed.vr_cloud.subscribe(sent.append)

    # 4 s of clouds at 4 a second, each a new point: all are added, the map goes out every 2 s.
    for k in range(16):
        clock[0] = 100.0 + 0.25 * k
        _arrive(feed, _coloured_cloud(np.array([[0.1 * k, 0.0, 0.0]]), frame_id="world"))

    assert [len(cloud) for cloud in sent] == [1, 9]


def test_without_accumulate_only_the_newest_cloud_goes_out(monkeypatch: pytest.MonkeyPatch) -> None:
    feed = HeadsetFeedModule(accumulate=False)
    try:
        sent = _sent_clouds(feed, monkeypatch)
        _arrive(feed, _coloured_cloud(np.array([[0.0, 0.0, 0.0]]), frame_id="world"))
        _arrive(feed, _coloured_cloud(np.array([[0.5, 0.0, 0.0]]), frame_id="world"))

        assert len(sent) == 2
        np.testing.assert_allclose(sent[-1].points_f32(), [[0.5, 0.0, 0.0]], atol=1e-5)
    finally:
        feed.dispose()


def test_the_map_survives_the_wire_at_16_bytes_a_point(
    feed: HeadsetFeedModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent = _sent_clouds(feed, monkeypatch)
    _arrive(
        feed, _coloured_cloud(np.array([[0.2, 0.0, 0.0]]), frame_id="world", colour=(0.0, 1.0, 0.0))
    )
    one_point = sent[-1].lcm_encode()
    _arrive(feed, _coloured_cloud(np.array([[0.4, 0.0, 0.0]]), frame_id="world"))
    two_points = sent[-1].lcm_encode()

    decoded = PointCloud2.lcm_decode(one_point)
    np.testing.assert_allclose(decoded.points_f32(), [[0.2, 0.0, 0.0]], atol=1e-5)
    colours = decoded.pointcloud_tensor.point["colors"].numpy()
    np.testing.assert_allclose(colours, [[0.0, 1.0, 0.0]], atol=1e-2)
    # x, y, z and packed rgb, float32 each: the size accumulate_max_points is set from.
    assert len(two_points) - len(one_point) == 16
