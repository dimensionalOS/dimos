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

from dimos.core.transport import LCMTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.time import time_from_seconds
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.protocol.tf.tf import TF


def _marker_detection_array(ts: float) -> Detection3DArray:
    det = Detection3D()
    det.header = Header(stamp=time_from_seconds(ts), frame_id="world")
    det.id = "11"
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id="DICT_APRILTAG_36h11:11",
                score=1.0,
            )
        )
    ]
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Point(x=0.5, y=-0.2, z=1.25),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        size=Vector3(x=0.18, y=0.18, z=0.0),
    )
    return Detection3DArray(
        header=Header(stamp=time_from_seconds(ts), frame_id="world"),
        detections=[det],
    )


def test_marker_tf_consumes_detection_array_over_lcm() -> None:
    ts = time.time()
    module = MarkerTfModule(marker_namespace_prefix="marker_tf")
    detections_transport = LCMTransport(
        f"/mtf/{uuid.uuid4().hex[:8]}",
        Detection3DArray,
    )
    module.detections.transport = detections_transport
    module.tf.transport = make_transport("/tf", TFMessage)
    host_transport = make_transport("/tf", TFMessage)
    host_tf = TF(host_transport)

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
        assert w_markers.header.frame_id == "world"
        assert w_markers.child_frame_id == "marker_tf/markers"
        assert w_marker.header.frame_id == "world"
        assert w_marker.child_frame_id == "marker_tf/marker_11"
        assert w_marker.transform.translation.x == pytest.approx(0.5)
        assert w_marker.transform.translation.y == pytest.approx(-0.2)
        assert w_marker.transform.translation.z == pytest.approx(1.25)
    finally:
        host_tf.dispose()
        host_transport.stop()
        module.stop()
