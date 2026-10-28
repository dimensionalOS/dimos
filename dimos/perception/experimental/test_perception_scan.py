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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.experimental.object import Object
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule


@pytest.fixture
def scene(mocker):
    module = ObjectSceneRegistrationModule(
        target_frame="camera",
        max_input_age_s=0.5,
        max_result_age_s=15,
        max_pair_skew_s=0.002,
        snapshot_only=True,
        require_segmentation=True,
        min_valid_depth_pixels=100,
        min_valid_depth_fraction=0.3,
    )
    color = Image(np.zeros((32, 32, 3), np.uint8), ImageFormat.RGB, frame_id="camera", ts=100.0)
    depth = Image(np.ones((32, 32), np.float32), ImageFormat.DEPTH, frame_id="camera", ts=100.0)
    module._camera_info = CameraInfo(
        width=32, height=32, K=[100.0, 0, 16, 0, 100.0, 16, 0, 0, 1], frame_id="camera", ts=100.0
    )
    module._latest_aligned_frames = (color, depth)
    now = [100.1]
    mocker.patch(
        "dimos.perception.experimental.object_scene_registration.time.time",
        side_effect=lambda: now[0],
    )
    detector = mocker.MagicMock()
    mask = np.ones((32, 32), np.uint8) * 255
    detection = Detection2DSeg(
        bbox=(0, 0, 31, 31),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name="cup",
        ts=100.0,
        image=color,
        mask=mask,
    )
    detector.process_image.return_value = ImageDetections2D(color, [detection])
    module._detector = detector
    module._segmenter = None
    module.detections_2d = mocker.MagicMock()
    module.objects = mocker.MagicMock()
    module.pointcloud = mocker.MagicMock()
    module.detections_3d = mocker.MagicMock()
    projection = mocker.patch(
        "dimos.perception.experimental.object_scene_registration.Object.from_2d_to_list",
        return_value=[],
    )
    try:
        yield module, detector, projection, now
    finally:
        module.stop()


def test_scan_api_filters_depth_and_reports_quality(scene):
    module, _, projection, _ = scene
    result = module.scan_scene(["cup"])
    assert isinstance(result, Detection3DArray)
    assert len(projection.call_args.kwargs["detections_2d"].detections) == 1
    status = module.get_scan_status()
    assert status["available"] and status["frame_id"] == "camera"
    assert status["quality"][0]["valid_depth_fraction"] == 1.0
    assert status["quality"][0]["accepted"] is True


def test_insufficient_depth_remains_2d_only(scene):
    module, _, projection, _ = scene
    module._latest_aligned_frames[1].data[:] = 0
    result = module.scan_scene(["cup"])
    assert result.detections_length == 0
    assert projection.call_args.kwargs["detections_2d"].detections == []
    assert module.get_scan_status()["quality"][0]["reason"] == "insufficient valid foreground depth"


def test_scan_routes_reconstructed_geometry_through_standard_3d_api(scene, mocker):
    module, _, projection, _ = scene
    detection = module._detector.process_image.return_value.detections[0]
    cloud = mocker.MagicMock()
    obj = Object(
        bbox=detection.bbox,
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name="cup",
        ts=100.0,
        image=detection.image,
        center=Vector3(0, 0, 1),
        size=Vector3(0.1, 0.1, 0.2),
        pose=PoseStamped(position=Vector3(0, 0, 1), frame_id="camera", ts=100.0),
        pointcloud=cloud,
        frame_id="camera",
    )
    projection.return_value = [obj]
    mocker.patch(
        "dimos.perception.experimental.object_scene_registration.aggregate_pointclouds",
        return_value=cloud,
    )
    result = module.scan_scene(["cup"])
    assert result.detections_length == 1
    assert result.detections[0].header.frame_id == "camera"
    assert result.detections[0].bbox.center.position.z == 1.0
    assert result.detections[0].bbox.size.x == 0.1
    assert module.get_detected_objects()[0]["object_id"] == obj.object_id
    assert module.get_object_pointcloud_by_object_id(obj.object_id) is cloud


def test_stale_input_never_calls_detector_and_old_geometry_is_unavailable(scene):
    module, detector, _, now = scene
    now[0] = 101.0
    with pytest.raises(ValueError, match="stale"):
        module.scan_scene(["cup"])
    detector.process_image.assert_not_called()
    assert module.get_scan_visualization() == {"status": module.get_scan_status()}
    assert module.get_detected_objects() == []
    assert module.get_object_pointcloud_by_name("cup") is None
    assert module.get_full_scene_pointcloud() is None


def test_slow_inference_cannot_publish_expired_geometry(scene):
    module, detector, _, now = scene
    output = detector.process_image.return_value

    def slow(image):
        now[0] = 116.0
        return output

    detector.process_image.side_effect = slow
    with pytest.raises(ValueError, match="freshness"):
        module.scan_scene(["cup"])
    module.objects.publish.assert_not_called()
    module.pointcloud.publish.assert_not_called()
    assert not module.get_scan_status()["available"]


def test_disconnect_expiry_hides_snapshot_and_object_queries(scene):
    module, _, _, now = scene
    module.scan_scene(["cup"])
    now[0] = 116.0
    assert not module.get_scan_status()["available"]
    assert module.get_scan_visualization().keys() == {"status"}
    assert module.get_object_track_ids() == []
    assert module.select_object(0) is None
    assert module.get_object_pointcloud_by_object_id("old") is None


def test_no_frame_scan_clears_previous_snapshot(scene):
    module, _, _, _ = scene
    module.scan_scene(["cup"])
    module._latest_aligned_frames = None
    with pytest.raises(ValueError, match="No RGB-D"):
        module.scan_scene(["cup"])
    assert not module.get_scan_status()["available"]
    assert module.get_scan_status()["object_count"] == 0


def test_snapshot_scans_clear_camera_frame_history(scene, mocker):
    module, _, _, _ = scene
    clear = mocker.spy(module._object_db, "clear")
    module.scan_scene(["cup"])
    module.scan_scene(["cup"])
    assert clear.call_count == 2


def test_old_blueprint_defaults_leave_freshness_policy_disabled():
    module = ObjectSceneRegistrationModule()
    try:
        assert module.config.max_input_age_s is None
        assert module.config.max_result_age_s is None
        assert module._scene_is_fresh()
    finally:
        module.stop()


def test_legacy_scan_error_preserves_previous_scene_cache(mocker):
    module = ObjectSceneRegistrationModule()
    image = Image(np.ones((2, 2), np.float32), ImageFormat.DEPTH, frame_id="camera", ts=100.0)
    old = (image, None, CameraInfo())
    module._latest_scene_snapshot = old
    module._latest_aligned_frames = (image, image)
    mocker.patch.object(module, "_scan_scene_objects", side_effect=RuntimeError("model failed"))
    clear = mocker.spy(module._object_db, "clear")
    try:
        with pytest.raises(RuntimeError, match="model failed"):
            module.scan_scene()
        assert module._latest_scene_snapshot is old
        clear.assert_not_called()
    finally:
        module.stop()


def test_real_open3d_reconstructs_metric_masked_foreground():
    pytest.importorskip("open3d")
    color = Image(np.zeros((64, 64, 3), np.uint8), ImageFormat.RGB, frame_id="camera", ts=100.0)
    depth = Image(np.ones((64, 64), np.float32), ImageFormat.DEPTH, frame_id="camera", ts=100.0)
    mask = np.zeros((64, 64), np.uint8)
    mask[16:48, 16:48] = 255
    detection = Detection2DSeg(
        bbox=(16, 16, 48, 48),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name="cup",
        ts=100.0,
        image=color,
        mask=mask,
    )
    info = CameraInfo(
        width=64, height=64, K=[100.0, 0, 32, 0, 100.0, 32, 0, 0, 1], frame_id="camera", ts=100.0
    )
    objects = Object.from_2d_to_list(
        ImageDetections2D(color, [detection]), color, depth, info, use_aabb=True
    )
    assert len(objects) == 1
    obj = objects[0]
    assert obj.frame_id == "camera" and obj.ts == 100.0
    assert obj.center.z == pytest.approx(1.0)
    assert obj.center.x == pytest.approx(-0.005, abs=0.02)
    assert 0.1 < obj.size.x < 0.32
    assert len(obj.pointcloud.pointcloud.points) > 100
