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

from collections.abc import Generator

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception import point_cloud_self_filter
from dimos.perception.point_cloud_self_filter import (
    PointCloudSelfFilter,
    SelfFilterRegion,
)
from dimos.protocol.tf.tf import MultiTBuffer


class _FakeTF(MultiTBuffer):
    def dispose(self) -> None:
        pass


@pytest.fixture(autouse=True)
def _stop_created_modules(mocker: MockerFixture) -> Generator[None, None, None]:
    modules: list[PointCloudSelfFilter] = []
    original_init = PointCloudSelfFilter.__init__

    def tracked_init(self: PointCloudSelfFilter, **kwargs: object) -> None:
        original_init(self, **kwargs)
        modules.append(self)

    mocker.patch.object(PointCloudSelfFilter, "__init__", tracked_init)
    yield
    for module in modules:
        module.stop()


def _cloud(
    points: list[tuple[float, float, float]], intensities: list[float] | None = None
) -> PointCloud2:
    intensity_array = None if intensities is None else np.asarray(intensities, dtype=np.float32)
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32),
        frame_id="cloud",
        timestamp=12.5,
        intensities=intensity_array,
    )


def _filter(regions: list[SelfFilterRegion], *, drop_missing: bool = False) -> PointCloudSelfFilter:
    module = PointCloudSelfFilter(
        regions=regions,
        drop_cloud_on_missing_tf=drop_missing,
        tf_tolerance_s=100.0,
    )
    module._tf = _FakeTF()  # type: ignore[assignment]
    return module


def test_sphere_filtering_removes_points_inside_tf_anchored_region() -> None:
    module = _filter([SelfFilterRegion(shape="sphere", frame_id="tool", radius=1.0)])
    module.__dict__["_tf"].receive_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="cloud",
            child_frame_id="tool",
            ts=12.5,
        )
    )

    filtered = module.filter_cloud(_cloud([(1.0, 0.0, 0.0), (1.9, 0.0, 0.0), (3.0, 0.0, 0.0)]))

    assert filtered is not None
    np.testing.assert_allclose(
        filtered.points_f32(), np.asarray([[3.0, 0.0, 0.0]], dtype=np.float32)
    )


def test_box_filtering_removes_points_inside_tf_anchored_region() -> None:
    module = _filter([SelfFilterRegion(shape="box", frame_id="box", size=(2.0, 2.0, 2.0))])
    module.__dict__["_tf"].receive_transform(
        Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="cloud",
            child_frame_id="box",
            ts=12.5,
        )
    )

    filtered = module.filter_cloud(_cloud([(2.5, 0.5, 0.5), (3.2, 0.0, 0.0), (0.0, 0.0, 0.0)]))

    assert filtered is not None
    np.testing.assert_allclose(
        filtered.points_f32(), np.asarray([[3.2, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
    )


def test_filter_preserves_input_frame_and_timestamp() -> None:
    module = _filter([SelfFilterRegion(shape="sphere", frame_id="tool", radius=0.5)])
    module.__dict__["_tf"].receive_transform(
        Transform(frame_id="cloud", child_frame_id="tool", ts=12.5)
    )
    cloud = _cloud([(0.6, 0.0, 0.0)])

    filtered = module.filter_cloud(cloud)

    assert filtered is not None
    assert filtered.frame_id == cloud.frame_id
    assert filtered.ts == cloud.ts


def test_early_return_preserves_intensities_without_regions() -> None:
    module = _filter([])
    cloud = _cloud([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], [0.25, 0.75])

    filtered = module.filter_cloud(cloud)

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), cloud.points_f32())
    intensities = filtered.intensities_f32()
    assert intensities is not None
    np.testing.assert_allclose(intensities, np.asarray([0.25, 0.75], dtype=np.float32))


def test_missing_tf_skips_region_or_drops_cloud_with_warning(mocker: MockerFixture) -> None:
    region = SelfFilterRegion(shape="sphere", frame_id="missing", radius=1.0)
    skip_module = _filter([region])
    warning = mocker.patch.object(point_cloud_self_filter.logger, "warning")

    skipped = skip_module.filter_cloud(_cloud([(0.0, 0.0, 0.0)]))

    assert skipped is not None
    assert len(skipped) == 1
    warning.assert_called_with(
        "Missing TF for PointCloudSelfFilter region %s -> %s", "cloud", "missing"
    )

    drop_module = _filter([region], drop_missing=True)
    assert drop_module.filter_cloud(_cloud([(0.0, 0.0, 0.0)])) is None
