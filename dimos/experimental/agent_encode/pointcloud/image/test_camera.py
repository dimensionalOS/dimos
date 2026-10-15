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

from dataclasses import replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from PIL import Image as PILImage
import pytest

from dimos.experimental.agent_encode.pointcloud.image.base import selected_pixels
from dimos.experimental.agent_encode.pointcloud.image.camera import CameraView
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Line
from dimos.experimental.agent_encode.pointcloud.image.lib.splat import axes
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def pair() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[2, 0, 0], [4, 0, -1]], dtype=np.float32), frame_id="sensor", timestamp=7
    )


@pytest.fixture
def view(tmp_path: Path) -> CameraView:
    return CameraView((0, 0, 1, 0, 0), size=(100, 100), point_size_m=0.4, out_dir=tmp_path)


def depth(
    cloud: PointCloud2, yaw: float, out_dir: Path, max_depth: float | None = None
) -> NDArray[np.float32]:
    view = CameraView(
        (0.0, 0.0, 1.0, yaw, 0.0), size=(64, 40), max_depth=max_depth, out_dir=out_dir
    )
    return view.run(cloud).raster.depth


def test_depth_image_sees_the_wall_ahead(room: PointCloud2, tmp_path: Path) -> None:
    ahead = depth(room, 0.0, tmp_path, max_depth=10.0)
    centre = float(ahead[18:23, 30:35].min())  # sparse synthetic walls: nearest in a centre block
    assert centre == pytest.approx(3.0, abs=0.15), "the east wall is 3 m ahead"
    # Facing north the wall is 2 m ahead.
    assert float(depth(room, 90.0, tmp_path)[18:23, 30:35].min()) == pytest.approx(2.0, abs=0.15)
    # Facing west there is no wall within range: the top half sees nothing.
    assert np.isinf(depth(room, 180.0, tmp_path)[0, 32])


@pytest.mark.parametrize("uv,provenance", [((50, 75), 1), ((50, 80), 2)])
def test_drawn_pixels_return_the_original_point_through_splats(
    pair: PointCloud2, view: CameraView, uv: tuple[int, int], provenance: int
) -> None:
    image = view.run(pair)

    assert image.raster.provenance[uv[1], uv[0]] == provenance
    assert image.pick(uv=uv).points_f32().tolist() == [[2, 0, 0]]
    assert image.world(uv) == (2, 0, 0)
    assert image.raster.projected_uv[0].tolist() == [50, 75]


def test_filled_pixels_show_a_neighbour_but_pick_nothing(
    pair: PointCloud2, view: CameraView
) -> None:
    image = view.run(pair)

    assert image.raster.provenance[81, 50] == 3
    assert image.raster.point_ids[81, 50] == 0
    assert len(image.pick(uv=(50, 81))) == 0
    assert image.world((50, 81)) is None


@pytest.mark.parametrize("yaw,pitch", [(0, 30), (90, -30), (180, 0)])
def test_projection_uses_camera_pitch_heading_and_cloud_frame(
    yaw: float, pitch: float, tmp_path: Path
) -> None:
    pose = (1.0, -3.0, 2.0, float(yaw), float(pitch))
    forward, right, up = axes(pose)
    point = np.array([1, -3, 2]) + 3 * forward + 0.2 * right + 0.1 * up
    cloud = PointCloud2.from_numpy(
        np.array([point], dtype=np.float32), frame_id="optical", timestamp=9
    )
    image = CameraView(pose, size=(100, 100), point_size_m=0, out_dir=tmp_path).run(cloud)

    picked = image.pick(uv=(53, 48))

    assert picked.points_f32()[0] == pytest.approx(point)
    assert float(image.raster.depth[48, 53]) == pytest.approx(3)
    assert image.pixel(tuple(point)) == pytest.approx((53 + 1 / 3, 48 + 1 / 3), abs=0.01)


def test_occlusion_ties_missing_and_clipped_neighbourhood(
    pair: PointCloud2, view: CameraView
) -> None:
    image = view.run(pair)
    assert image.pick(uv=(50, 75), radius_px=5).points_f32().tolist() == [[2, 0, 0]]
    assert len(selected_pixels((0, 0), 2, None, None, image.size)) == 9
    assert len(image.pick(uv=(0, 0))) == 0
    assert len(image.pick(uv=(100, 0))) == 0
    duplicate = PointCloud2.from_numpy(
        np.array([[2, 0, 0], [2, 0, 0]], dtype=np.float32), frame_id="sensor"
    )
    doubled = view.run(duplicate)
    assert doubled.raster.point_ids[75, 50] == 0
    assert len(doubled.pick(uv=(50, 75))) == 1


def test_distinct_region_hits_keep_all_returns(tmp_path: Path) -> None:
    cloud = PointCloud2.from_numpy(np.array([[2, -0.5, 0], [4, 0.5, 0]], dtype=np.float32))
    image = CameraView((0, 0, 0, 0, 0), size=(100, 100), point_size_m=0, out_dir=tmp_path).run(
        cloud
    )
    picked = image.pick(rect=(40, 45, 30, 10))
    assert sorted(picked.points_f32().tolist()) == [[2, -0.5, 0], [4, 0.5, 0]]


def test_polygon_and_rectangle_deduplicate_and_limits_are_contained(
    pair: PointCloud2, view: CameraView
) -> None:
    image = view.run(pair)
    rectangle = image.pick(rect=(45, 70, 11, 11))
    polygon = image.pick(polygon=((45, 70), (56, 70), (56, 81), (45, 81)))
    assert len(rectangle) == len(polygon) == 1
    assert len(selected_pixels(None, 0, (45, 70, 11, 11), None, image.size)) == 121
    square = ((45.0, 70.0), (56.0, 70.0), (56.0, 81.0), (45.0, 81.0))
    assert len(selected_pixels(None, 0, None, square, image.size)) == 121
    with pytest.raises(ValueError, match="exactly one"):
        image.pick(uv=(1, 1), rect=(0, 0, 2, 2))
    with pytest.raises(ValueError, match="integer"):
        image.pick(uv=(1.5, 2))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="radius_px"):
        image.pick(uv=(1, 1), radius_px=65)
    with pytest.raises(ValueError, match="inside the image"):
        image.pick(rect=(-1, 0, 3, 3))
    with pytest.raises(ValueError, match="3..32"):
        image.pick(polygon=((0, 0), (1, 1)))
    large = replace(view, size=(300, 300)).run(pair)
    with pytest.raises(ValueError, match="65536"):
        large.pick(rect=(0, 0, 300, 300))


def test_drawn_items_change_the_image_but_not_the_measurement(
    pair: PointCloud2, view: CameraView
) -> None:
    plain = view.run(pair)
    line = Line(((2.0, -0.3, 0.0), (2.0, 0.3, 0.0)))
    drawn = replace(view, draw=(line, pair)).run(pair)
    assert [(d.label, d.colour) for d in drawn.drawn] == [
        ("Line", "#ff3bcc"),
        ("PointCloud2", "#00cfef"),
    ]
    assert drawn.path != plain.path
    with PILImage.open(plain.path) as a, PILImage.open(drawn.path) as b:
        assert a.getpixel((44, 75)) != b.getpixel((44, 75))
    assert (
        drawn.pick(uv=(50, 80)).points_f32().tolist()
        == plain.pick(uv=(50, 80)).points_f32().tolist()
    )
