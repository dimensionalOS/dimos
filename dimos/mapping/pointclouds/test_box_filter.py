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

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import math
import time
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.mapping.pointclouds import box_filter
from dimos.mapping.pointclouds.box_filter import (
    PointCloudBoxFilter,
    inside_boxes,
    rasterize_voxels,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.transform_utils import pose_to_matrix


def _module(**config: Any) -> Iterator[PointCloudBoxFilter]:
    instance = PointCloudBoxFilter(**config)
    # A local transform buffer stands in for the tf stream.
    cast("dict[str, Any]", instance.__dict__)["_tf"] = MultiTBuffer()
    yield instance
    cast("dict[str, Any]", instance.__dict__)["_tf"] = None
    instance.dispose()


@pytest.fixture
def module() -> Iterator[PointCloudBoxFilter]:
    yield from _module()


@pytest.fixture
def re_expressing() -> Iterator[PointCloudBoxFilter]:
    """A filter that sends its clouds in ``base``, without waiting for a late transform."""
    yield from _module(target_frame="base", tf_forward_tolerance_s=0.0)


@pytest.fixture
def sent(module: PointCloudBoxFilter) -> list[PointCloud2]:
    clouds: list[PointCloud2] = []
    module.filtered_cloud.subscribe(clouds.append)
    return clouds


def _cloud(points: list[list[float]], frame_id: str = "world", ts: float = 1.0) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32), frame_id=frame_id, timestamp=ts
    )


def _feed(module: PointCloudBoxFilter, cloud: PointCloud2) -> None:
    asyncio.run(module.handle_cloud(cloud))


def _place_tool(module: PointCloudBoxFilter, at: Vector3, yaw: float, ts: float) -> None:
    module.tfbuffer.receive_transform(
        Transform(
            translation=at,
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            frame_id="world",
            child_frame_id="tool",
            ts=ts,
        )
    )


def test_a_fixed_box_drops_the_points_inside(
    module: PointCloudBoxFilter, sent: list[PointCloud2]
) -> None:
    module.set_box("target", Pose(position=Vector3(1.0, 0.0, 0.0)), Vector3(0.2, 0.2, 0.2))

    _feed(module, _cloud([[1.0, 0.0, 0.0], [1.05, -0.05, 0.05], [2.0, 0.0, 0.0]]))

    np.testing.assert_allclose(sent[-1].points_f32(), [[2.0, 0.0, 0.0]])


def test_a_box_on_a_moving_frame_follows_it(
    module: PointCloudBoxFilter, sent: list[PointCloud2]
) -> None:
    # 20 cm along the tool's x axis; the tool is turned 90 degrees, so in the
    # world the box sits 20 cm along y from the tool.
    module.set_box("held", Pose(position=Vector3(0.2, 0.0, 0.0)), Vector3(0.1, 0.1, 0.1), "tool")
    cloud = _cloud([[1.0, 0.2, 0.0], [1.2, 0.0, 0.0], [2.0, 0.2, 0.0]])

    _place_tool(module, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    _feed(module, cloud)
    np.testing.assert_allclose(sent[-1].points_f32(), [[1.2, 0.0, 0.0], [2.0, 0.2, 0.0]])

    _place_tool(module, Vector3(2.0, 0.0, 0.0), math.pi / 2, ts=2.0)
    _feed(module, cloud)
    np.testing.assert_allclose(sent[-1].points_f32(), [[1.0, 0.2, 0.0], [1.2, 0.0, 0.0]])


def _place_base(module: PointCloudBoxFilter, at: Vector3, yaw: float, ts: float) -> None:
    """Where the base stands in odom; the filter looks the inverse up."""
    module.tfbuffer.receive_transform(
        Transform(
            translation=at,
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            frame_id="odom",
            child_frame_id="base",
            ts=ts,
        )
    )


def test_a_target_frame_re_expresses_the_cloud_at_its_stamp(
    re_expressing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    re_expressing.filtered_cloud.subscribe(sent.append)
    # The base stands 1 m along odom x, turned 90 degrees: odom (1, 1, 0) is 1 m ahead of it.
    _place_base(re_expressing, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    # Where the base went next does not move a cloud stamped before it.
    _place_base(re_expressing, Vector3(5.0, 0.0, 0.0), 0.0, ts=1.5)
    cloud = _cloud([[1.0, 1.0, 0.0], [2.0, 0.0, 0.0]], frame_id="odom", ts=1.0)
    cloud.seq = 7

    _feed(re_expressing, cloud)

    (out,) = sent
    assert (out.frame_id, out.ts, out.seq) == ("base", 1.0, 7)
    np.testing.assert_allclose(out.points_f32(), [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], atol=1e-6)


def test_a_box_without_a_frame_is_in_the_target_frame(
    re_expressing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    re_expressing.filtered_cloud.subscribe(sent.append)
    _place_base(re_expressing, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    # 1 m ahead of the base: odom (1, 1, 0), not odom (1, 0, 0).
    re_expressing.set_box("target", Pose(position=Vector3(1.0, 0.0, 0.0)), Vector3(0.2, 0.2, 0.2))

    _feed(re_expressing, _cloud([[1.0, 1.0, 0.0], [1.0, 2.0, 0.0]], frame_id="odom"))

    np.testing.assert_allclose(sent[-1].points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_a_cloud_with_no_transform_is_dropped_until_one_arrives(
    re_expressing: PointCloudBoxFilter, monkeypatch: pytest.MonkeyPatch
) -> None:
    warnings: list[str] = []
    monkeypatch.setattr(
        box_filter.logger, "warning", lambda msg, *args: warnings.append(msg % args)
    )
    sent: list[PointCloud2] = []
    re_expressing.filtered_cloud.subscribe(sent.append)
    cloud = _cloud([[1.0, 1.0, 0.0]], frame_id="odom", ts=1.0)

    for _ in range(3):
        _feed(re_expressing, cloud)
    assert sent == []
    assert len(warnings) == 1, "one warning for a run of dropped clouds, not one per cloud"
    assert warnings[0].startswith("No transform from base to odom")

    # The transform lands, and a box change re-sends the last cloud, now placed.
    _place_base(re_expressing, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    re_expressing.set_box("held", Pose(position=Vector3(9.0, 0.0, 0.0)), Vector3(0.1, 0.1, 0.1))
    (out,) = sent
    assert out.frame_id == "base"
    np.testing.assert_allclose(out.points_f32(), [[1.0, 0.0, 0.0]], atol=1e-6)


def test_a_cloud_too_far_from_any_transform_is_dropped(
    re_expressing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    re_expressing.filtered_cloud.subscribe(sent.append)
    _place_base(re_expressing, Vector3(1.0, 0.0, 0.0), 0.0, ts=1.0)

    _feed(re_expressing, _cloud([[1.0, 1.0, 0.0]], frame_id="odom", ts=1.0 + 0.5))

    assert sent == [], "0.5 s from the only edge, outside the 0.1 s tolerance"


def test_an_empty_cloud_is_relabelled_without_a_transform(
    re_expressing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    re_expressing.filtered_cloud.subscribe(sent.append)

    _feed(re_expressing, PointCloud2(frame_id="odom", ts=1.0))

    (out,) = sent
    assert (out.frame_id, len(out.points_f32())) == ("base", 0)


VOXEL_M = 0.02
# Where a whole-body pick may leave the base in odom: turned 42 degrees, and off the voxel grid
# by a fraction of a voxel on every axis.
TURNED_AT = Vector3(0.3137, -0.1171, -0.3 * VOXEL_M)
TURNED_YAW = math.radians(42.0)
PLANE_SIDE = 40


@pytest.fixture
def rasterizing() -> Iterator[PointCloudBoxFilter]:
    """A filter that re-rasterizes a voxel map in ``odom`` onto ``base``'s grid."""
    yield from _module(target_frame="base", tf_forward_tolerance_s=0.0, target_voxel_m=VOXEL_M)


def _plane(side: int = PLANE_SIDE) -> NDArray[np.int64]:
    """The keys of a one-voxel-thick square of voxels, on odom's bottom layer."""
    i, j = np.meshgrid(np.arange(side), np.arange(side), indexing="ij")
    return np.stack((i.ravel(), j.ravel(), np.zeros(i.size, dtype=np.int64)), axis=1)


def _voxels(keys: NDArray[np.int64]) -> PointCloud2:
    """A voxel map as its mapper sends it: the centres of these voxels, in odom."""
    centres = ((keys + 0.5) * VOXEL_M).astype(np.float32)
    return PointCloud2.from_numpy(centres, frame_id="odom", timestamp=1.0)


def _cells(points: NDArray[np.float32]) -> set[tuple[int, ...]]:
    """The cells the planner's octree puts points in: cell k spans [k, k + 1) voxels an axis."""
    keys = np.floor(points.astype(np.float64) / VOXEL_M).astype(np.int64)
    return set(map(tuple, keys.tolist()))


def _odom_from_base(at: Vector3, yaw: float) -> NDArray[np.float64]:
    turned = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    return Transform(
        translation=at, rotation=turned, frame_id="odom", child_frame_id="base"
    ).to_matrix()


def test_a_turned_voxel_plane_reaches_the_base_grid_without_holes(
    rasterizing: PointCloudBoxFilter, re_expressing: PointCloudBoxFilter
) -> None:
    """Every base cell whose centre lies on the plane, a voxel in from its edges, is sent."""
    sent: list[PointCloud2] = []
    moved: list[PointCloud2] = []
    rasterizing.filtered_cloud.subscribe(sent.append)
    re_expressing.filtered_cloud.subscribe(moved.append)
    for module in (rasterizing, re_expressing):
        _place_base(module, TURNED_AT, TURNED_YAW, ts=1.0)
        _feed(module, _voxels(_plane()))

    # Every base cell over the plane, two layers either side of it, placed back in odom.
    odom_from_base = _odom_from_base(TURNED_AT, TURNED_YAW)
    i, j, k = np.meshgrid(np.arange(-60, 60), np.arange(-60, 60), np.arange(-2, 3), indexing="ij")
    candidates = np.stack((i.ravel(), j.ravel(), k.ravel()), axis=1)
    in_odom = (candidates + 0.5) * VOXEL_M @ odom_from_base[:3, :3].T + odom_from_base[:3, 3]
    inner = (in_odom[:, :2] >= VOXEL_M) & (in_odom[:, :2] < (PLANE_SIDE - 1) * VOXEL_M)
    on_plane = inner.all(axis=1) & (in_odom[:, 2] >= 0.0) & (in_odom[:, 2] < VOXEL_M)
    expected = set(map(tuple, candidates[on_plane].tolist()))
    assert len(expected) > (PLANE_SIDE - 3) ** 2
    out = sent[-1].points_f32()
    assert expected <= _cells(out), "a hole in the re-rasterized plane"
    assert len(expected - _cells(moved[-1].points_f32())) > 0.1 * len(expected), (
        "one point per voxel, a plane turned 42 degrees leaves holes"
    )
    # One point per cell, at its centre, and no cell further than one from the plane.
    assert len(_cells(out)) == len(out)
    np.testing.assert_allclose(out, (np.floor(out / VOXEL_M) + 0.5) * VOXEL_M, atol=1e-6)
    back = out @ odom_from_base[:3, :3].T + odom_from_base[:3, 3]
    assert np.all(
        (back > -VOXEL_M) & (back < np.array([PLANE_SIDE + 1, PLANE_SIDE + 1, 2]) * VOXEL_M)
    )


def test_re_rasterizing_through_an_identity_returns_the_same_cells(
    rasterizing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    rasterizing.filtered_cloud.subscribe(sent.append)
    keys = np.unique(np.random.default_rng(0).integers(-30, 30, size=(800, 3)), axis=0)
    _place_base(rasterizing, Vector3(0.0, 0.0, 0.0), 0.0, ts=1.0)

    _feed(rasterizing, _voxels(keys))

    (out,) = sent
    assert out.frame_id == "base"
    np.testing.assert_allclose(out.points_f32(), (keys + 0.5) * VOXEL_M, atol=1e-7)


def test_a_box_leaves_its_cells_out_of_the_re_rasterized_map(
    rasterizing: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    rasterizing.filtered_cloud.subscribe(sent.append)
    _place_base(rasterizing, TURNED_AT, TURNED_YAW, ts=1.0)
    # 10 cm across, in base, over the middle of the plane.
    base_from_odom = np.linalg.inv(_odom_from_base(TURNED_AT, TURNED_YAW))
    middle = base_from_odom[:3, :3] @ [0.4, 0.4, 0.01] + base_from_odom[:3, 3]
    box = Pose(position=Vector3(*middle))
    rasterizing.set_box("held", box, Vector3(0.1, 0.1, 0.1))

    _feed(rasterizing, _voxels(_plane()))
    boxed = sent[-1].points_f32()
    rasterizing.clear_box("held")
    whole = sent[-1].points_f32()

    inside = inside_boxes(whole, [(pose_to_matrix(box), np.full(3, 0.1))])
    assert inside.sum() >= 16, "the box covers some of the plane's cells"
    np.testing.assert_array_equal(boxed, whole[~inside])


def test_re_rasterizing_a_bench_sized_map_takes_milliseconds() -> None:
    """The arm's map is ~3600 voxels once a second; the Orin re-rasterizes one in ~4 ms."""
    centres = ((_plane(64) + 0.5) * VOXEL_M).astype(np.float32)
    base_from_odom = np.linalg.inv(_odom_from_base(TURNED_AT, TURNED_YAW))
    took: list[float] = []
    for _ in range(5):
        start = time.perf_counter()
        rasterize_voxels(centres, base_from_odom, VOXEL_M)
        took.append(time.perf_counter() - start)
    assert len(centres) == 4096
    assert min(took) < 0.020


# A crop in base around the plane's middle, a third of it each way.
CROP_MIN = (0.2, 0.1, -0.05)
CROP_MAX = (0.6, 0.5, 0.05)


@pytest.fixture
def cropping() -> Iterator[PointCloudBoxFilter]:
    """A filter that re-rasterizes a voxel map in ``odom`` onto ``base``'s grid, cropped there."""
    yield from _module(
        target_frame="base",
        tf_forward_tolerance_s=0.0,
        target_voxel_m=VOXEL_M,
        crop_min=CROP_MIN,
        crop_max=CROP_MAX,
    )


@pytest.fixture
def cropped_in_base() -> Iterator[PointCloudBoxFilter]:
    """A filter that sends its clouds in ``base``, cropped to a metre-wide box ahead of it."""
    yield from _module(
        target_frame="base",
        tf_forward_tolerance_s=0.0,
        crop_min=(0.5, -0.5, -0.5),
        crop_max=(1.5, 0.5, 0.5),
    )


@pytest.fixture
def cropped_in_place() -> Iterator[PointCloudBoxFilter]:
    """A filter that crops its clouds in their own frame to the unit cube."""
    yield from _module(crop_min=(0.0, 0.0, 0.0), crop_max=(1.0, 1.0, 1.0))


def test_a_crop_keeps_only_the_points_inside_it_in_the_target_frame(
    cropped_in_base: PointCloudBoxFilter,
) -> None:
    cropped = cropped_in_base
    sent: list[PointCloud2] = []
    cropped.filtered_cloud.subscribe(sent.append)
    # The base stands 1 m along odom x, turned 90 degrees: odom (1, 1, 0) is 1 m ahead of it, and
    # odom (2, 0, 0), which the crop would hold in odom, is 1 m to its right.
    _place_base(cropped, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)

    _feed(cropped, _cloud([[1.0, 1.0, 0.0], [2.0, 0.0, 0.0], [1.0, 3.0, 0.0]], frame_id="odom"))

    (out,) = sent
    assert out.frame_id == "base"
    np.testing.assert_allclose(out.points_f32(), [[1.0, 0.0, 0.0]], atol=1e-6)


def test_without_a_target_frame_the_crop_is_in_the_clouds_own_and_keeps_its_attributes(
    cropped_in_place: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    cropped_in_place.filtered_cloud.subscribe(sent.append)
    points = np.asarray([[0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [1.0, 1.0, 1.0]], dtype=np.float32)
    cloud = PointCloud2.from_numpy(
        points,
        frame_id="world",
        timestamp=1.0,
        intensities=np.asarray([10.0, 20.0, 30.0], dtype=np.float32),
    )

    _feed(cropped_in_place, cloud)

    (out,) = sent
    assert out is not cloud and out.frame_id == "world"
    np.testing.assert_allclose(out.points_f32(), points[[0, 2]], err_msg="the faces are inside")
    intensities = out.pointcloud_tensor.point["intensities"].numpy().ravel()
    np.testing.assert_allclose(intensities, [10.0, 30.0])


def test_a_crop_picks_voxels_by_their_centres_before_they_are_re_rasterized(
    cropping: PointCloudBoxFilter,
) -> None:
    sent: list[PointCloud2] = []
    cropping.filtered_cloud.subscribe(sent.append)
    _place_base(cropping, TURNED_AT, TURNED_YAW, ts=1.0)
    centres = ((_plane() + 0.5) * VOXEL_M).astype(np.float32)
    base_from_odom = np.linalg.inv(_odom_from_base(TURNED_AT, TURNED_YAW))

    _feed(cropping, _voxels(_plane()))

    out = sent[-1].points_f32()
    in_base = centres @ base_from_odom[:3, :3].T + base_from_odom[:3, 3]
    inside = box_filter.inside_crop(in_base, CROP_MIN, CROP_MAX)
    assert 0 < inside.sum() < len(centres) / 4, "the crop cuts the plane"
    np.testing.assert_array_equal(
        out, rasterize_voxels(centres[inside], base_from_odom, VOXEL_M), "the voxels inside, whole"
    )
    # Cropped after the re-rasterization, no cell's centre would lie outside the crop.
    outside = ~box_filter.inside_crop(out, CROP_MIN, CROP_MAX)
    assert outside.any(), "a voxel inside covers cells past the crop's faces"
    edge = np.full(3, VOXEL_M)
    assert np.all((out > np.subtract(CROP_MIN, edge)) & (out < np.add(CROP_MAX, edge))), (
        "but none more than one edge past them"
    )


def test_a_box_leaves_its_cells_out_of_a_cropped_map(cropping: PointCloudBoxFilter) -> None:
    sent: list[PointCloud2] = []
    cropping.filtered_cloud.subscribe(sent.append)
    _place_base(cropping, TURNED_AT, TURNED_YAW, ts=1.0)
    _feed(cropping, _voxels(_plane()))
    whole = sent[-1].points_f32()
    # 10 cm across in base, straddling the crop's +x face.
    box = Pose(position=Vector3(CROP_MAX[0], 0.3, 0.0))

    cropping.set_box("held", box, Vector3(0.1, 0.1, 0.1))

    inside = inside_boxes(whole, [(pose_to_matrix(box), np.full(3, 0.1))])
    assert inside.sum() >= 4, "the box covers some of the cropped map's cells"
    np.testing.assert_array_equal(sent[-1].points_f32(), whole[~inside])


@pytest.mark.parametrize(
    "crop",
    [
        {"crop_min": (0.0, 0.0, 0.0)},
        {"crop_max": (1.0, 1.0, 1.0)},
        {"crop_min": (0.0, 0.0, 1.0), "crop_max": (1.0, 1.0, 1.0)},
        {"crop_min": (0.0, math.nan, 0.0), "crop_max": (1.0, 1.0, 1.0)},
    ],
    ids=["min-only", "max-only", "flat", "nan"],
)
def test_a_crop_is_a_box_with_volume(crop: dict[str, Any]) -> None:
    with pytest.raises(ValueError, match="crop_m"):
        PointCloudBoxFilter(**crop)
