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
from threading import Event, Thread
from typing import Any
from unittest.mock import ANY, MagicMock, call

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.experimental.objectDB import ObjectDB


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


def test_owlv2_queries_configured_prompts(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(detector_backend="owlv2")
    module._camera_info = MagicMock()
    module._detector = MagicMock()
    module._text_prompts = ["mug"]
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


def test_moondream_queries_each_configured_prompt(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(detector_backend="moondream")
    module._camera_info = MagicMock()
    module._detector = MagicMock()
    module._text_prompts = ["cup", "bottle"]
    module.detections_2d = MagicMock()
    color = Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=4.0,
    )
    cup = MagicMock(track_id=0, class_id=-1)
    bottle = MagicMock(track_id=0, class_id=-1)
    module._detector.query_detections.side_effect = [
        ImageDetections2D(color, [cup]),
        ImageDetections2D(color, [bottle]),
    ]
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(4.0))

    assert module._detector.query_detections.call_args_list == [
        call(color, "cup"),
        call(color, "bottle"),
    ]
    combined = process_3d.call_args.args[0]
    assert combined.detections == [cup, bottle]
    assert (cup.track_id, cup.class_id) == (-1, 0)
    assert (bottle.track_id, bottle.class_id) == (-1, 1)
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
    module = ObjectSceneRegistrationModule(
        target_frame="camera", detector_backend="owlv2", detect_on_request=True
    )
    module._object_db = MagicMock()
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

    def process_images(got_color: Image, got_depth: Image) -> list[MagicMock]:
        assert (got_color, got_depth) == (color, depth)
        return [output]

    monkeypatch.setattr(module, "_process_images", process_images)
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.to_detection3d_array",
        lambda objects, **kwargs: result,
    )

    assert module.scan_scene(text=["mug"]) is result
    assert module._text_prompts == ["mug"]
    module.stop()


def test_request_driven_detect_triggers_scan(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(detector_backend="owlv2", detect_on_request=True)
    module._detector = MagicMock()
    current_object = MagicMock()
    current_object.agent_encode.return_value = {"name": "mug", "object_id": "current"}
    scan_objects = MagicMock(return_value=[current_object])
    monkeypatch.setattr(module, "_scan_scene_objects", scan_objects)
    monkeypatch.setattr(module, "get_detected_objects", lambda: pytest.fail("read stale database"))

    assert module.detect(["mug"]) == "Detected 1 object(s):\n  - mug (object_id='current')"
    assert module._text_prompts == ["mug"]
    scan_objects.assert_called_once_with()
    module.stop()


def test_scan_output_includes_pending_objects(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(target_frame="camera")
    module._camera_info = MagicMock()
    module._object_db = MagicMock()
    pending = MagicMock()
    permanent = MagicMock()
    module._object_db.get_all_objects.return_value = [pending, permanent]
    module._object_db.get_objects.return_value = [permanent]
    module._object_db.add_objects.return_value = [pending]
    module.detections_3d = MagicMock()
    module.objects = MagicMock()
    module.pointcloud = MagicMock()
    converted_objects = [pending]
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
        lambda **_: converted_objects,
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.to_detection3d_array",
        MagicMock(),
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.aggregate_pointclouds",
        MagicMock(),
    )

    observed = ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        _image(4.0),
        _image(4.0),
    )

    assert observed == [pending]

    converted_objects.clear()
    module._object_db.add_objects.reset_mock()
    observed = ObjectSceneRegistrationModule._process_3d_detections(
        module,
        MagicMock(spec=ImageDetections2D),
        _image(5.0),
        _image(5.0),
    )

    assert observed == []
    module._object_db.add_objects.assert_called_once_with([])
    module.stop()


def test_concurrent_request_scans_do_not_overlap(monkeypatch: Any) -> None:
    module = ObjectSceneRegistrationModule(
        target_frame="camera", detector_backend="owlv2", detect_on_request=True
    )
    module._object_db = MagicMock()
    color = Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=4.0,
    )
    module._on_aligned_frames((color, _image(4.0)))
    first_started = Event()
    release_first = Event()
    second_started = Event()
    seen_prompts: list[list[str]] = []

    def process_images(_: Image, __: Image) -> list[MagicMock]:
        seen_prompts.append(list(module._text_prompts))
        if len(seen_prompts) == 1:
            first_started.set()
            assert release_first.wait(timeout=1.0)
        else:
            second_started.set()
        return []

    monkeypatch.setattr(module, "_process_images", process_images)
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.to_detection3d_array",
        MagicMock(),
    )
    first = Thread(target=lambda: module.scan_scene(text=["cup"]))
    second = Thread(target=lambda: module.scan_scene(text=["mug"]))
    first.start()
    assert first_started.wait(timeout=1.0)
    second.start()
    assert not second_started.wait(timeout=0.05)
    release_first.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert seen_prompts == [["cup"], ["mug"]]
    module.stop()


def test_object_db_uses_wall_clock_for_pending_ttl(monkeypatch: Any) -> None:
    object_db = ObjectDB(pending_ttl_s=5.0)
    detected = MagicMock()
    detected.object_id = "stable-id"
    detected.track_id = -1
    detected.ts = 4.0  # Hardware timestamps are relative to camera boot.
    detected.last_seen_ts = None
    detected.center = None
    detected.detections_count = 1
    now = [1000.0]
    monkeypatch.setattr("dimos.perception.experimental.objectDB.time.time", lambda: now[0])

    object_db.add_objects([detected])
    assert detected.last_seen_ts == 1000.0
    now[0] = 1004.0
    object_db.add_objects([])
    assert object_db.find_by_object_id("stable-id") is detected
    now[0] = 1006.0
    object_db.add_objects([])
    assert object_db.find_by_object_id("stable-id") is None


def test_object_db_counts_each_source_frame_once(monkeypatch: Any) -> None:
    object_db = ObjectDB(min_detections_for_permanent=10)
    now = [1000.0]
    monkeypatch.setattr("dimos.perception.experimental.objectDB.time.time", lambda: now[0])

    first = MagicMock(
        object_id="first-id",
        track_id=-1,
        ts=4.0,
        last_seen_ts=None,
        detections_count=1,
    )
    first.center = MagicMock()
    duplicate = MagicMock(object_id="duplicate-id", track_id=-1, ts=4.0)
    duplicate.center = MagicMock()
    duplicate.center.distance.return_value = 0.0

    observed = object_db.add_objects([first, duplicate])

    assert observed == [first]
    first.update_object.assert_not_called()
    assert first.last_seen_ts == 1000.0

    newer = MagicMock(object_id="newer-id", track_id=-1, ts=5.0)
    newer.center = MagicMock()
    newer.center.distance.return_value = 0.0
    first.update_object.side_effect = lambda _: setattr(first, "detections_count", 2)
    now[0] = 1001.0

    assert object_db.add_objects([newer]) == [first]
    first.update_object.assert_called_once_with(newer)
    assert first.last_seen_ts == 1001.0


def test_object_db_promotes_a_first_sighting_when_threshold_is_one() -> None:
    object_db = ObjectDB(min_detections_for_permanent=1)
    detected = MagicMock(
        object_id="first-id",
        track_id=-1,
        ts=1.0,
        last_seen_ts=None,
        detections_count=1,
        name="cup",
    )
    detected.center = None

    object_db.add_objects([detected])

    assert object_db.get_objects() == [detected]
    assert object_db.get_stats() == {
        "pending_count": 0,
        "permanent_count": 1,
        "total_count": 1,
    }


def test_object_db_keeps_first_sighting_pending_above_threshold() -> None:
    object_db = ObjectDB(min_detections_for_permanent=2)
    detected = MagicMock(
        object_id="first-id",
        track_id=7,
        ts=1.0,
        last_seen_ts=None,
        detections_count=1,
        name="cup",
    )
    detected.center = None
    newer = MagicMock(object_id="newer-id", track_id=7, ts=2.0)
    newer.center = None
    detected.update_object.side_effect = lambda _: setattr(detected, "detections_count", 2)

    object_db.add_objects([detected])

    assert object_db.get_objects() == []
    assert object_db.find_by_object_id("first-id") is detected

    object_db.add_objects([newer])

    assert object_db.get_objects() == [detected]
