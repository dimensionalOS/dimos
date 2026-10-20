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

from contextlib import ExitStack
from threading import Event, Thread
from typing import Any
from unittest.mock import ANY, MagicMock, call

import numpy as np
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.messages.vision_msgs.msg import Detection3DArray
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.msgs.time import time_from_seconds
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.experimental.objectDB import ObjectDB
from dimos.perception.experimental.test_object_generated import _object


def _image(timestamp: float) -> Image:
    return image_from_array(
        np.ones((2, 2), dtype=np.float32),
        encoding="32FC1",
        header=Header(frame_id="camera", stamp=time_from_seconds(timestamp)),
    )


@pytest.fixture
def module_factory():
    with ExitStack() as cleanup:

        def create(**kwargs):
            module = ObjectSceneRegistrationModule(**kwargs)
            cleanup.callback(module.stop)
            module._camera_info = camera_info_from_intrinsics(
                1, 1, 0, 0, 2, 2, header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0))
            )
            wire = CdrTransport("object_scene_tf", TFMessage, backend="lcm", url="memq://")
            cleanup.callback(wire.stop)
            module.tf.transport = wire
            return module

        yield create


@pytest.fixture
def module(module_factory):
    return module_factory(target_frame="map")


def test_transform_is_captured_before_slow_detector(
    monkeypatch: Any, module: ObjectSceneRegistrationModule, module_factory, mocker
) -> None:
    now = [0.0]
    transform = TransformStamped(
        header=Header(frame_id="map", stamp=time_from_seconds(12.5)),
        child_frame_id="camera",
        transform=Transform(
            translation=Vector3(x=1, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1)
        ),
    )
    lookup = mocker.patch.object(
        module.tfbuffer,
        "get",
        side_effect=lambda *args, **kwargs: transform if now[0] <= 10 else None,
    )
    module._detector_backend = "owlv2"
    module._detector = MagicMock()
    module._text_prompts = ["cup"]
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(12.5)),
    )
    detections = ImageDetections2D(color, [])

    def slow_detection(*_args: Any, **_kwargs: Any) -> ImageDetections2D:
        now[0] = 11.0
        return detections

    module._detector.query_detections.side_effect = slow_detection
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(12.5))

    lookup.assert_called_once_with("map", "camera", 12.5, 0.1, forward_tolerance=0.2)
    process_3d.assert_called_once_with(detections, color, ANY, transform)


def test_failed_lookup_does_not_retry_without_time_or_replace_coherent_cache(
    monkeypatch: Any, module: ObjectSceneRegistrationModule, module_factory, mocker
) -> None:
    old_transform = TransformStamped(
        header=Header(frame_id="map", stamp=time_from_seconds(1)),
        child_frame_id="camera",
        transform=Transform(
            translation=Vector3(x=0, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1)
        ),
    )
    warning = MagicMock()
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.logger.warning", warning
    )
    monkeypatch.setattr(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
        lambda **_: [],
    )

    old_depth = _image(1.0)
    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        ImageDetections2D(_image(1), []),
        old_depth,
        old_depth,
        old_transform,
    )
    new_depth = _image(2.0)
    ObjectSceneRegistrationModule._process_3d_detections(
        module,
        ImageDetections2D(_image(1), []),
        new_depth,
        new_depth,
        None,
    )

    assert module._latest_scene_snapshot == (old_depth, old_transform)
    warning.assert_called_once_with("Failed to lookup transform from camera frame to target frame")


def test_full_scene_pointcloud_uses_one_coherent_scene_snapshot(
    monkeypatch: Any, module: ObjectSceneRegistrationModule, module_factory, mocker
) -> None:
    depth = _image(3.0)
    transform = TransformStamped(
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="camera",
        transform=Transform(
            translation=Vector3(x=1, y=0.0, z=0.0), rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
    )
    depth = image_from_array(
        np.ones((20, 20), dtype=np.float32), encoding="32FC1", header=depth.header
    )
    module._camera_info = camera_info_from_intrinsics(100, 100, 10, 10, 20, 20, header=depth.header)
    module._latest_scene_snapshot = (depth, transform)
    lookup = mocker.spy(module.tfbuffer, "get")
    result = module.get_full_scene_pointcloud(voxel_size=0.001)
    assert result is not None
    assert result.header.frame_id == "map"
    assert result.header.stamp == depth.header.stamp
    points = pointcloud_xyz(result)
    assert len(points) == 400
    np.testing.assert_allclose(points[:, 2], 1)
    assert points[:, 0].min() == pytest.approx(0.9)
    assert points[:, 0].max() == pytest.approx(1.09)
    lookup.assert_not_called()


def test_owlv2_queries_configured_prompts(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(
        target_frame="camera", detector_backend="owlv2", detector_confidence=0.07
    )
    module._detector = MagicMock()
    module._text_prompts = ["mug"]
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(4.0)),
    )
    detections = ImageDetections2D(color, [])
    module._detector.query_detections.return_value = detections
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(4.0))

    module._detector.query_detections.assert_called_once_with(color, ["mug"], threshold=0.07)
    process_3d.assert_called_once_with(detections, color, ANY, None)
    module.stop()


def test_owlv2_yolo_constructs_box_prompt_segmenter(mocker: Any, module_factory) -> None:
    module = module_factory(
        detector_backend="owlv2",
        segmentation_backend="yolo",
        segmentation_confidence=0.04,
    )
    segmenter = MagicMock()
    segmenter_factory = mocker.patch(
        "dimos.perception.experimental.object_scene_registration.YoloeBoxSegmenter",
        return_value=segmenter,
    )

    try:
        created = module._create_segmenter()
    finally:
        module.stop()

    assert created is segmenter
    segmenter_factory.assert_called_once_with(confidence=0.04)


def test_moondream_queries_each_configured_prompt(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(target_frame="camera", detector_backend="moondream")
    detector = MagicMock()
    module._detector = detector
    module._text_prompts = ["cup", "bottle"]
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(4.0)),
    )
    cup = Detection2DBBox((0, 0, 1, 1), 0, -1, 0.9, "cup", 4.0, color)
    bottle = Detection2DBBox((1, 1, 2, 2), 0, -1, 0.8, "bottle", 4.0, color)
    detector.query_detections.side_effect = [
        ImageDetections2D(color, [cup]),
        ImageDetections2D(color, [bottle]),
    ]
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    try:
        module._process_images(color, _image(4.0))
    finally:
        module.stop()

    assert detector.query_detections.call_args_list == [
        call(color, "cup"),
        call(color, "bottle"),
    ]
    combined = process_3d.call_args.args[0]
    assert combined.detections == [cup, bottle]
    assert (cup.track_id, cup.class_id) == (-1, 0)
    assert (bottle.track_id, bottle.class_id) == (-1, 1)


def test_edgetam_refines_detector_output(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(target_frame="camera", segmentation_backend="edgetam")
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(4.0)),
    )
    raw_detections = ImageDetections2D(color, [])
    segmented_detections = ImageDetections2D(color, [])
    module._segmenter = MagicMock()
    module._segmenter.segment.return_value = segmented_detections
    module._detector = MagicMock()
    module._detector.process_image.return_value = raw_detections
    process_3d = MagicMock()
    monkeypatch.setattr(module, "_process_3d_detections", process_3d)

    module._process_images(color, _image(4.0))

    module._segmenter.segment.assert_called_once_with(raw_detections)
    process_3d.assert_called_once_with(segmented_detections, color, ANY, None)
    module.stop()


def test_request_driven_scan_processes_latest_aligned_frame(
    monkeypatch: Any, module_factory, mocker
) -> None:
    module = module_factory(target_frame="camera", detector_backend="owlv2", detect_on_request=True)
    module._object_db = MagicMock()
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(4.0)),
    )
    depth = _image(4.0)
    module._on_aligned_frames((color, depth))
    output = MagicMock()
    result = Detection3DArray(
        header=Header(frame_id="camera", stamp=time_from_seconds(4)), detections=[]
    )

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


def test_request_driven_detect_triggers_scan(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(detector_backend="owlv2", detect_on_request=True)
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


def test_scan_output_includes_pending_objects(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(target_frame="camera")
    module._object_db = MagicMock()
    pending = MagicMock()
    permanent = MagicMock()
    module._object_db.get_all_objects.return_value = [pending, permanent]
    module._object_db.get_objects.return_value = [permanent]
    module._object_db.add_objects.return_value = [pending]
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
        ImageDetections2D(_image(1), []),
        _image(4.0),
        _image(4.0),
        None,
    )

    assert observed == [pending]

    converted_objects.clear()
    module._object_db.add_objects.reset_mock()
    observed = ObjectSceneRegistrationModule._process_3d_detections(
        module,
        ImageDetections2D(_image(1), []),
        _image(5.0),
        _image(5.0),
        None,
    )

    assert observed == []
    module._object_db.add_objects.assert_called_once_with([])
    module.stop()


def test_concurrent_request_scans_do_not_overlap(monkeypatch: Any, module_factory, mocker) -> None:
    module = module_factory(target_frame="camera", detector_backend="owlv2", detect_on_request=True)
    module._object_db = MagicMock()
    color = image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=time_from_seconds(4.0)),
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
    try:
        assert first_started.wait(timeout=1.0)
        second.start()
        assert not second_started.wait(timeout=0.05)
    finally:
        release_first.set()
        first.join(timeout=1.0)
        if second.ident is not None:
            second.join(timeout=1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert seen_prompts == [["cup"], ["mug"]]
    module.stop()


def test_object_db_uses_wall_clock_for_pending_ttl(monkeypatch):
    database = ObjectDB(pending_ttl_s=5)
    detected = _object("stable-id", 0, 4)
    now = [1000]
    monkeypatch.setattr("dimos.perception.experimental.objectDB.time.time", lambda: now[0])
    database.add_objects([detected])
    assert detected.last_seen_ts == 1000
    now[0] = 1004
    database.add_objects([])
    assert database.find_by_object_id("stable-id") is detected
    now[0] = 1006
    database.add_objects([])
    assert database.find_by_object_id("stable-id") is None


def test_object_db_counts_each_source_frame_once(monkeypatch):
    database = ObjectDB(min_detections_for_permanent=10)
    now = [1000]
    monkeypatch.setattr("dimos.perception.experimental.objectDB.time.time", lambda: now[0])
    first = _object("first-id", 0, 4)
    duplicate = _object("duplicate-id", 0, 4)
    assert database.add_objects([first, duplicate]) == [first]
    assert first.detections_count == 1
    assert first.last_seen_ts == 1000
    newer = _object("newer-id", 0, 5)
    now[0] = 1001
    assert database.add_objects([newer]) == [first]
    assert first.detections_count == 2
    assert first.last_seen_ts == 1001


def test_object_db_promotes_a_first_sighting_when_threshold_is_one():
    database = ObjectDB(min_detections_for_permanent=1)
    detected = _object("first-id", 0, 1)
    database.add_objects([detected])
    assert database.get_objects() == [detected]
    assert database.get_stats() == {"pending_count": 0, "permanent_count": 1, "total_count": 1}


def test_object_db_keeps_first_sighting_pending_above_threshold():
    database = ObjectDB(min_detections_for_permanent=2)
    detected = _object("first-id", 0, 1)
    detected.track_id = 7
    newer = _object("newer-id", 0, 2)
    newer.track_id = 7
    database.add_objects([detected])
    assert database.get_objects() == []
    assert database.find_by_object_id("first-id") is detected
    database.add_objects([newer])
    assert database.get_objects() == [detected]
