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

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.time import time_from_seconds
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.protocol.tf.tf import TF


def _detection_array(
    *,
    ts: float,
    marker_id: str = "0",
    class_id: str = "DICT_APRILTAG_36h11:0",
    center: Point | None = None,
    orientation: Quaternion | None = None,
) -> Detection3DArray:
    if center is None:
        center = Point(x=1.2, y=-0.3, z=0.8)
    if orientation is None:
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    det = Detection3D()
    det.header = Header(stamp=time_from_seconds(ts), frame_id="world")
    det.id = marker_id
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id=class_id,
                score=1.0,
            )
        )
    ]
    det.bbox = BoundingBox3D(
        center=Pose(
            position=center,
            orientation=orientation,
        ),
        size=Vector3(x=0.18, y=0.18, z=0.0),
    )
    return Detection3DArray(
        header=Header(stamp=time_from_seconds(ts), frame_id="world"),
        detections=[det],
    )


def test_marker_tf_module_publishes_world_markers_chain() -> None:
    ts = 1_000_000.0
    center = Point(x=1.3, y=-0.2, z=0.4)
    orientation = quaternion_from_euler(0.1, 0.2, 0.3)

    mod = MarkerTfModule()
    view = TF(mod.tf)
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

        wm = view.get("world", "markers", ts, 1.0)
        assert wm is not None
        assert abs(wm.transform.translation.x) < 1e-6
        assert abs(wm.transform.translation.y) < 1e-6
        assert abs(wm.transform.translation.z) < 1e-6

        w_m7 = view.get("world", "marker_7", ts, 1.0)
        assert w_m7 is not None
        assert w_m7.transform.translation.x == pytest.approx(center.x)
        assert w_m7.transform.translation.y == pytest.approx(center.y)
        assert w_m7.transform.translation.z == pytest.approx(center.z)
        assert w_m7.transform.rotation.x == pytest.approx(orientation.x)
        assert w_m7.transform.rotation.y == pytest.approx(orientation.y)
        assert w_m7.transform.rotation.z == pytest.approx(orientation.z)
        assert w_m7.transform.rotation.w == pytest.approx(orientation.w)
        assert view.get("world", "marker_99", ts, 0.1) is None
    finally:
        view.dispose()
        mod.stop()


def test_marker_tf_parses_class_id_when_detection_id_empty() -> None:
    ts = 700_000.0

    mod = MarkerTfModule()
    view = TF(mod.tf)
    try:
        mod._process_detections(
            _detection_array(ts=ts, marker_id="", class_id="DICT_APRILTAG_36h11:42")
        )

        assert view.get("world", "marker_42", ts, 1.0) is not None
    finally:
        view.dispose()
        mod.stop()


def test_marker_tf_empty_array_skips_publication() -> None:
    ts = 600_000.0
    mod = MarkerTfModule()
    view = TF(mod.tf)
    try:
        mod._process_detections(
            Detection3DArray(
                header=Header(stamp=time_from_seconds(ts), frame_id="world"),
                detections=[],
            )
        )

        assert view.get("world", "markers", ts, 0.1) is None
    finally:
        view.dispose()
        mod.stop()


def test_marker_tf_non_empty_array_without_marker_id_skips_publication() -> None:
    ts = 650_000.0
    mod = MarkerTfModule()
    view = TF(mod.tf)
    try:
        mod._process_detections(_detection_array(ts=ts, marker_id="", class_id="marker"))

        assert view.get("world", "markers", ts, 0.1) is None
    finally:
        view.dispose()
        mod.stop()


def test_marker_tf_does_not_recompute_marker_pose() -> None:
    ts = 800_000.0
    mod = MarkerTfModule()
    view = TF(mod.tf)
    try:
        with patch("dimos.perception.fiducial.marker_pose.estimate_marker_pose") as mock_estimate:
            mod._process_detections(_detection_array(ts=ts, marker_id="4"))

        mock_estimate.assert_not_called()
        assert view.get("world", "marker_4", ts, 1.0) is not None
    finally:
        view.dispose()
        mod.stop()


def test_marker_namespace_prefix_child_frames() -> None:
    ts = 500_000.0

    mod = MarkerTfModule(marker_namespace_prefix="r1")
    view = TF(mod.tf)
    try:
        mod._process_detections(_detection_array(ts=ts))

        assert view.get("world", "r1/markers", ts, 1.0) is not None
        assert view.get("world", "r1/marker_0", ts, 1.0) is not None
    finally:
        view.dispose()
        mod.stop()


def test_marker_tf_wire_preserves_exact_timestamp():
    module = MarkerTfModule()
    outputs = []
    unsubscribe = module.tf.subscribe(lambda msg: outputs.append(TFMessage.decode(msg.encode())))
    detections = _detection_array(ts=0)
    detections.header.stamp.sec = 1700000000
    detections.header.stamp.nanosec = 123456789
    try:
        module._process_detections(detections)
        assert len(outputs) == 1
        assert len(outputs[0].transforms) == 2
        assert all(t.header.stamp == detections.header.stamp for t in outputs[0].transforms)
    finally:
        unsubscribe()
        module.stop()
