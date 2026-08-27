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
import sys
from typing import Any
from unittest.mock import ANY, MagicMock

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.spec.utils import spec_annotation_compliance


class _FakeTF:
    def __init__(self, result: Any) -> None:
        self.result = result
        self.calls: list[tuple[Any, ...]] = []

    def get(self, *args: Any, **kwargs: Any) -> Any:
        self.calls.append((args, kwargs))
        return self.result

    def dispose(self) -> None:
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
    module._tf = tf  # type: ignore[assignment]
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
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
    module._tf = tf  # type: ignore[assignment]
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
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
    module._tf = _FakeTF(transform)  # type: ignore[assignment]
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
    # open3d is imported inside the method under test, so swap the module itself
    monkeypatch.setitem(sys.modules, "open3d", fake_o3d)

    result = MagicMock()
    result.transform.side_effect = lambda used_transform: (
        result if used_transform is transform else pytest.fail("mixed scene snapshot")
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.PointCloud2",
        lambda *_args, **_kwargs: result,
    )

    module.get_full_scene_pointcloud()
    result.transform.assert_called_once_with(transform)


def test_osr_implements_request_driven_scan_spec(module: ObjectSceneRegistrationModule) -> None:
    assert spec_annotation_compliance(module, ObjectSceneRegistrationSpec)


def test_owlv2_prompts_are_text_only() -> None:
    module = ObjectSceneRegistrationModule(detector_backend="owlv2")
    try:
        module.set_prompts(text=["mug"])
        assert module._owlv2_prompts == ["mug"]
        with pytest.raises(ValueError, match="text prompts"):
            module.set_prompts(bboxes=np.zeros((1, 4)))
    finally:
        module.stop()


def test_owlv2_queries_configured_prompts(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(detector_backend="owlv2")
    module._camera_info = MagicMock()
    module._detector = MagicMock()
    module._owlv2_prompts = ["mug"]
    module.detections_2d = MagicMock()
    color = Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=4.0,
    )
    detections = ImageDetections2D(color, [])
    module._detector.query_detections.return_value = detections
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(4.0))

    module._detector.query_detections.assert_called_once_with(color, ["mug"], threshold=0.6)
    process_3d.assert_called_once_with(detections, color, ANY)
    module.stop()


def test_edgetam_refines_detector_output(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(segmentation_backend="edgetam")
    module._camera_info = MagicMock()
    color = Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=4.0,
    )
    raw_detections = ImageDetections2D(color, [])
    segmented_detections = ImageDetections2D(color, [])
    module._segmenter = MagicMock()
    module._segmenter.segment.return_value = segmented_detections
    module._detector = MagicMock()
    module._detector.process_image.return_value = raw_detections
    module.detections_2d = MagicMock()
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(4.0))

    module._segmenter.segment.assert_called_once_with(raw_detections)
    process_3d.assert_called_once_with(segmented_detections, color, ANY)
    module.stop()


def test_request_driven_scan_processes_latest_aligned_frame(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(target_frame="camera", detect_on_request=True)
    color = Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=4.0,
    )
    depth = _image(4.0)
    module._on_aligned_frames((color, depth))
    output = MagicMock()
    result = MagicMock(spec=Detection3DArray)

    def process_images(got_color: Image, got_depth: Image) -> None:
        assert (got_color, got_depth) == (color, depth)
        module._latest_output_objects = (output,)

    monkeypatch.setattr(module, "_process_images", process_images)
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.to_detection3d_array",
        lambda objects, **kwargs: result,
    )

    assert module.scan_scene() is result
    module.stop()


def test_request_driven_detect_triggers_scan(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(detector_backend="owlv2", detect_on_request=True)
    module._detector = MagicMock()
    scan_scene = MagicMock()
    monkeypatch.setattr(module, "scan_scene", scan_scene)
    monkeypatch.setattr(module, "get_detected_objects", lambda: [])

    assert module.detect("mug") == "No objects detected."
    assert module._owlv2_prompts == ["mug"]
    scan_scene.assert_called_once_with()
    module.stop()


def test_scan_output_includes_pending_objects(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(target_frame="camera")
    module._camera_info = MagicMock()
    module._object_db = MagicMock()
    pending = MagicMock()
    permanent = MagicMock()
    module._object_db.get_all_objects.return_value = [pending, permanent]
    module._object_db.get_objects.return_value = [permanent]
    module.detections_3d = MagicMock()
    module.objects = MagicMock()
    module.pointcloud = MagicMock()
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
        lambda **_: [pending],
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.to_detection3d_array",
        MagicMock(),
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.aggregate_pointclouds",
        MagicMock(),
    )

    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        _image(4.0),
        _image(4.0),
    )

    assert module._latest_output_objects == (pending, permanent)
    module.stop()
