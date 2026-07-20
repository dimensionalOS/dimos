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

from __future__ import annotations

from collections.abc import Iterator
from typing import Any, cast
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.tf.tf import TFSpec


class _FakeTF:
    def __init__(self, result: Any) -> None:
        self.result = result
        self.calls: list[tuple[Any, ...]] = []

    def get(self, *args: Any, **kwargs: Any) -> Any:
        self.calls.append((args, kwargs))
        return self.result

    def stop(self) -> None:
        pass


def _image(timestamp: float) -> Image:
    return Image(
        data=np.ones((2, 2), dtype=np.float32),
        format=ImageFormat.DEPTH,
        frame_id="camera",
        ts=timestamp,
    )


@pytest.fixture
def module() -> Iterator[ObjectSceneRegistrationModule]:
    module = ObjectSceneRegistrationModule(target_frame="map")
    module._camera_info = MagicMock(K=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    module._latest_scene_snapshot = None
    yield module
    module.stop()


def test_temporal_tf_lookup_uses_bounded_image_timestamp(
    monkeypatch: Any, module: ObjectSceneRegistrationModule
) -> None:
    tf = _FakeTF(MagicMock())
    module._tf = cast("TFSpec", tf)
    monkeypatch.setattr(
        "dimos.perception.object_scene_registration.Object.from_2d_to_list",
        lambda **_: [],
    )

    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        _image(12.5),
        _image(12.5),
    )

    assert tf.calls == [(("map", "camera", 12.5, 0.1), {"forward_tolerance": 0.2})]


def test_failed_lookup_does_not_retry_without_time_or_replace_coherent_cache(
    monkeypatch: Any, module: ObjectSceneRegistrationModule
) -> None:
    old_transform = MagicMock(name="old_transform")
    tf = _FakeTF(old_transform)
    module._tf = cast("TFSpec", tf)
    monkeypatch.setattr(
        "dimos.perception.object_scene_registration.Object.from_2d_to_list",
        lambda **_: [],
    )

    old_depth = _image(1.0)
    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        old_depth,
        old_depth,
    )
    tf.result = None
    new_depth = _image(2.0)
    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        new_depth,
        new_depth,
    )

    assert len(tf.calls) == 2
    assert tf.calls[1] == (("map", "camera", 2.0, 0.1), {"forward_tolerance": 0.2})
    assert module._latest_scene_snapshot == (old_depth, old_transform)


def test_full_scene_pointcloud_uses_one_coherent_scene_snapshot(
    monkeypatch: Any, module: ObjectSceneRegistrationModule
) -> None:
    depth = _image(3.0)
    transform = MagicMock(name="transform")
    module._tf = cast("TFSpec", _FakeTF(transform))
    module._latest_scene_snapshot = (depth, transform)

    class _PointCloud:
        points = list(range(100))

        def voxel_down_sample(self, voxel_size: float) -> _PointCloud:
            return self

    pointcloud = _PointCloud()
    fake_o3d = MagicMock()
    fake_o3d.camera.PinholeCameraIntrinsic.return_value = MagicMock()
    fake_o3d.geometry.Image.return_value = MagicMock()
    fake_o3d.geometry.PointCloud.create_from_depth_image.return_value = pointcloud
    monkeypatch.setattr("dimos.perception.object_scene_registration.o3d", fake_o3d)

    result = MagicMock()
    result.transform.side_effect = lambda used_transform: (
        result if used_transform is transform else pytest.fail("mixed scene snapshot")
    )
    monkeypatch.setattr(
        "dimos.perception.object_scene_registration.PointCloud2",
        lambda *_args, **_kwargs: result,
    )

    module.get_full_scene_pointcloud()
    result.transform.assert_called_once_with(transform)
