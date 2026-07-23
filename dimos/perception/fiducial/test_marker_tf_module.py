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

from unittest.mock import patch

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule


def _detection_array(
    *,
    ts: float,
    marker_id: str = "0",
    class_id: str = "DICT_APRILTAG_36h11:0",
    center: Vector3 | None = None,
    orientation: Quaternion | None = None,
) -> Detection3DArray:
    if center is None:
        center = Vector3(1.2, -0.3, 0.8)
    if orientation is None:
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    det = Detection3D()
    det.header = Header(ts, "world")
    det.id = marker_id
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id=class_id,
                score=1.0,
            )
        )
    ]
    det.results_length = len(det.results)
    det.bbox = BoundingBox3D(
        center=Pose(
            position=center,
            orientation=orientation,
        ),
        size=Vector3(0.18, 0.18, 0.0),
    )
    return Detection3DArray(
        header=Header(ts, "world"),
        detections=[det],
        detections_length=1,
    )


def test_marker_tf_module_publishes_world_markers_chain() -> None:
    ts = 1_000_000.0
    center = Vector3(1.3, -0.2, 0.4)
    orientation = Quaternion(0.1, 0.2, 0.3, 0.9)

    mod = MarkerTfModule()
    try:
        mod._process_detections(
            _detection_array(
                ts=ts,
                marker_id="7",
                class_id="DICT_APRILTAG_36h11:99",
                center=center,
                orientation=orientation,
            )
        )

        wm = mod.tf.get("world", "markers", ts, 1.0)
        assert wm is not None
        assert abs(wm.translation.x) < 1e-6
        assert abs(wm.translation.y) < 1e-6
        assert abs(wm.translation.z) < 1e-6

        w_m7 = mod.tf.get("world", "marker_7", ts, 1.0)
        assert w_m7 is not None
        assert w_m7.translation.x == pytest.approx(center.x)
        assert w_m7.translation.y == pytest.approx(center.y)
        assert w_m7.translation.z == pytest.approx(center.z)
        assert w_m7.rotation.x == pytest.approx(orientation.x)
        assert w_m7.rotation.y == pytest.approx(orientation.y)
        assert w_m7.rotation.z == pytest.approx(orientation.z)
        assert w_m7.rotation.w == pytest.approx(orientation.w)
        assert mod.tf.get("world", "marker_99", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_parses_class_id_when_detection_id_empty() -> None:
    ts = 700_000.0

    mod = MarkerTfModule()
    try:
        mod._process_detections(
            _detection_array(ts=ts, marker_id="", class_id="DICT_APRILTAG_36h11:42")
        )

        assert mod.tf.get("world", "marker_42", ts, 1.0) is not None
    finally:
        mod.stop()


def test_marker_tf_empty_array_skips_publication() -> None:
    ts = 600_000.0
    mod = MarkerTfModule()
    try:
        mod._process_detections(
            Detection3DArray(
                header=Header(ts, "world"),
                detections=[],
                detections_length=0,
            )
        )

        assert mod.tf.get("world", "markers", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_non_empty_array_without_marker_id_skips_publication() -> None:
    ts = 650_000.0
    mod = MarkerTfModule()
    try:
        mod._process_detections(_detection_array(ts=ts, marker_id="", class_id="marker"))

        assert mod.tf.get("world", "markers", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_does_not_recompute_marker_pose() -> None:
    ts = 800_000.0
    mod = MarkerTfModule()
    try:
        with patch("dimos.perception.fiducial.marker_pose.estimate_marker_pose") as mock_estimate:
            mod._process_detections(_detection_array(ts=ts, marker_id="4"))

        mock_estimate.assert_not_called()
        assert mod.tf.get("world", "marker_4", ts, 1.0) is not None
    finally:
        mod.stop()


def test_marker_namespace_prefix_child_frames() -> None:
    ts = 500_000.0

    mod = MarkerTfModule(marker_namespace_prefix="r1")
    try:
        mod._process_detections(_detection_array(ts=ts))

        assert mod.tf.get("world", "r1/markers", ts, 1.0) is not None
        assert mod.tf.get("world", "r1/marker_0", ts, 1.0) is not None
    finally:
        mod.stop()
