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

"""Integration test for ``MarkerTfModule`` consuming Detection3DArray over LCM."""

from __future__ import annotations

import time
import uuid

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.protocol.tf.tf import LCMTF


def _marker_detection_array(ts: float) -> Detection3DArray:
    det = Detection3D()
    det.header = Header(ts, "world")
    det.id = "11"
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id="DICT_APRILTAG_36h11:11",
                score=1.0,
            )
        )
    ]
    det.results_length = len(det.results)
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Vector3(0.5, -0.2, 1.25),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        size=Vector3(0.18, 0.18, 0.0),
    )
    return Detection3DArray(
        header=Header(ts, "world"),
        detections=[det],
        detections_length=1,
    )


def test_marker_tf_consumes_detection_array_over_lcm() -> None:
    ts = time.time()
    module = MarkerTfModule(marker_namespace_prefix="marker_tf")
    detections_transport = LCMTransport(
        f"/mtf/{uuid.uuid4().hex[:8]}",
        Detection3DArray,
    )
    module.detections.transport = detections_transport
    host_tf = LCMTF()

    try:
        module.start()
        msg = _marker_detection_array(ts)
        w_markers = None
        w_marker = None
        for _ in range(40):
            detections_transport.publish(msg)
            time.sleep(0.05)
            w_markers = host_tf.get("world", "marker_tf/markers", ts, 1.0)
            w_marker = host_tf.get("world", "marker_tf/marker_11", ts, 1.0)
            if w_markers is not None and w_marker is not None:
                break

        assert w_markers is not None, "Timed out waiting for world -> marker_tf/markers"
        assert w_marker is not None, "Timed out waiting for world -> marker_tf/marker_11"
        assert w_markers.frame_id == "world"
        assert w_markers.child_frame_id == "marker_tf/markers"
        assert w_marker.frame_id == "world"
        assert w_marker.child_frame_id == "marker_tf/marker_11"
        assert w_marker.translation.x == pytest.approx(0.5)
        assert w_marker.translation.y == pytest.approx(-0.2)
        assert w_marker.translation.z == pytest.approx(1.25)
    finally:
        host_tf.stop()
        module.stop()
