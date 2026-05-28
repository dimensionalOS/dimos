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

import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_detect import detect_markers_in_image
from dimos.perception.fiducial.marker_transformer import DetectMarkers
from dimos.perception.fiducial.test_helpers import (
    camera_info,
    synthetic_marker_image,
    world_T_optical,
)


def test_detect_markers_in_image_builds_rich_marker_detection() -> None:
    marker_id = 7
    marker_length_m = 0.18
    image = synthetic_marker_image(marker_id)
    info = camera_info(image.ts)

    detections = detect_markers_in_image(
        image,
        camera_info=info,
        world_T_optical=world_T_optical(image.ts),
        marker_length_m=marker_length_m,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    assert len(detections) == 1
    det = detections[0]
    assert det.marker_id == marker_id
    assert det.track_id == -1
    assert det.name == "DICT_APRILTAG_36h11:7"
    assert det.image is image
    assert det.frame_id == "world"
    assert det.size.x == pytest.approx(marker_length_m)
    assert det.size.y == pytest.approx(marker_length_m)
    assert det.size.z == pytest.approx(0.0)
    assert det.confidence == pytest.approx(1.0)
    assert det.reprojection_error < 0.1
    assert det.bbox == pytest.approx((210.0, 130.0, 429.0, 349.0), abs=2.0)
    assert det.center.x == pytest.approx(1.0, abs=0.02)
    assert det.center.y == pytest.approx(2.0, abs=0.02)
    assert det.center.z > 3.3

    msg = det.to_detection3d_msg()
    assert msg.id == str(marker_id)
    assert msg.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"


def test_detect_markers_in_image_returns_empty_for_no_marker_frame() -> None:
    ts = 11.0
    image = Image(
        data=np.full((480, 640, 3), 255, dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )

    detections = detect_markers_in_image(
        image,
        camera_info=camera_info(ts),
        world_T_optical=world_T_optical(ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    assert detections == []


def test_detect_markers_transformer_preserves_observation_context_and_tags() -> None:
    marker_id = 7
    image = synthetic_marker_image(marker_id, ts=12.0)
    obs = Observation[Image](
        id=42,
        ts=image.ts,
        data_type=Image,
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    transformer = DetectMarkers(
        camera_info=camera_info(image.ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    results = list(transformer(iter([obs])))

    assert len(results) == 1
    out = results[0]
    assert out.id == obs.id
    assert out.ts == obs.ts
    assert out.data.marker_id == marker_id
    assert out.data.image is image
    assert out.pose is not None
    assert out.tags["marker_id"] == marker_id
    assert out.tags["track_id"] == -1
    assert out.data.track_id == -1


def test_detect_markers_transformer_can_emit_empty_frame_sentinel() -> None:
    image = Image(
        data=np.full((480, 640, 3), 255, dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=13.0,
    )
    obs = Observation[Image](
        id=43,
        ts=image.ts,
        data_type=Image,
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    transformer = DetectMarkers(
        camera_info=camera_info(image.ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
        emit_empty_frames=True,
    )

    results = list(transformer(iter([obs])))

    assert len(results) == 1
    out = results[0]
    assert out.id == obs.id
    assert out.ts == obs.ts
    assert out.data is None
    assert out.tags["marker_frame_image"] is image
    assert out.tags["marker_frame_count"] == 0


def test_detect_markers_transformer_uses_callablecamera_info_source() -> None:
    image = synthetic_marker_image(marker_id=7, ts=14.0)
    obs = Observation[Image](
        id=44,
        ts=image.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    latest_info: CameraInfo | None = None
    transformer = DetectMarkers(
        camera_info=lambda: latest_info,
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
        emit_empty_frames=True,
    )

    assert list(transformer(iter([obs]))) == []

    latest_info = camera_info(image.ts)
    results = list(transformer(iter([obs])))

    assert len(results) == 1
    assert results[0].data.marker_id == 7
    assert results[0].tags["marker_frame_count"] == 1


def test_detect_markers_rebuilds_intrinsics_without_resetting_smoothing_track() -> None:
    marker_id = 7
    marker_length_m = 0.18
    image_a = synthetic_marker_image(marker_id=marker_id, ts=15.0)
    image_b = synthetic_marker_image(marker_id=marker_id, ts=15.2)
    info_a = camera_info(image_a.ts)
    info_b = camera_info(image_b.ts)
    info_b.K = info_b.K.copy()
    info_b.P = info_b.P.copy()
    info_b.K[0] = info_b.K[4] = 900.0
    info_b.P[0] = info_b.P[5] = 900.0
    latest_info: CameraInfo | None = info_a

    obs_a = Observation[Image](
        id=45,
        ts=image_a.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image_a,
    )
    obs_b = Observation[Image](
        id=46,
        ts=image_b.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image_b,
    )
    transformer = DetectMarkers(
        camera_info=lambda: latest_info,
        marker_length_m=marker_length_m,
        aruco_dictionary="DICT_APRILTAG_36h11",
        smoothing_window=1.0,
    )

    first = next(transformer(iter([obs_a])))
    latest_info = info_b
    second = next(transformer(iter([obs_b])))
    raw_after_k_change = next(
        DetectMarkers(
            camera_info=info_b,
            marker_length_m=marker_length_m,
            aruco_dictionary="DICT_APRILTAG_36h11",
        )(iter([obs_b]))
    )

    assert first.data.track_id == second.data.track_id
    assert first.data.track_id > 0
    assert raw_after_k_change.data.center.z != pytest.approx(first.data.center.z, abs=0.05)
    assert second.data.center.z == pytest.approx(
        (first.data.center.z + raw_after_k_change.data.center.z) / 2.0,
        abs=0.02,
    )
