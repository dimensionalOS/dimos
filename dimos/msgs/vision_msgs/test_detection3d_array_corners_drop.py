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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker


def test_marker_corners_px_are_dropped_from_detection3d_array_wire() -> None:
    """CHARACTERIZATION: pin the dormant-ambiguity-gate root cause.

    ``corners_px`` (the four detected marker corner pixels the IPPE mirror-
    ambiguity gate needs) live on the ``Detection3DMarker`` dataclass but the
    wire message it serializes into — ``Detection3D`` / ``Detection3DArray`` —
    has no corners field (LCM ``__slots__`` = results_length, header, results,
    bbox, id). So corners never cross the wire and any consumer reading the
    published ``Detection3DArray`` cannot run the live ambiguity gate: it is
    dormant. Everything else (marker id, class id, pose) survives, isolating
    corners as the specific thing dropped.

    When corners-on-wire lands (corners added to the LCM message), FLIP this
    test: assert the decoded wire detection carries corners and that they
    round-trip equal to the source ``corners_px``.
    """
    rng = np.random.default_rng(0)
    # Non-trivial corner pixels so a "drop" cannot be masked by the (0,0) default.
    corners_px = rng.uniform(0.0, 128.0, size=(4, 2)).astype(np.float32)

    image = Image(
        data=np.zeros((100, 120, 3), dtype=np.uint8),
        frame_id="camera_optical",
        ts=42.0,
    )
    marker = Detection3DMarker(
        bbox=(10.0, 20.0, 50.0, 80.0),
        track_id=7,
        class_id=999,
        confidence=1.0,
        name="marker_42",
        ts=image.ts,
        image=image,
        center=Vector3(1.0, 2.0, 3.0),
        size=Vector3(0.16, 0.16, 0.0),
        frame_id="world",
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        marker_id=42,
        corners_px=corners_px,
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=0.5,
    )
    # Precondition: the source dataclass really carries the corners.
    assert marker.corners_px.shape == (4, 2)
    assert np.any(marker.corners_px != 0.0)

    # The dataclass -> wire message boundary: corners have nowhere to go.
    detection = marker.to_detection3d_msg()
    assert "corners_px" not in detection.__slots__
    assert not hasattr(detection, "corners_px")

    array = Detection3DArray()
    array.header = Header(image.ts, "world")
    array.detections = [detection]
    array.detections_length = 1

    decoded = Detection3DArray.lcm_decode(array.lcm_encode())

    assert decoded.detections_length == 1
    decoded_detection = decoded.detections[0]

    # Marker identity survives the wire — proves the round-trip works and that
    # corners are the specific field lost, not a broken encode.
    assert decoded_detection.id == "42"
    assert decoded_detection.results_length == 1
    assert decoded_detection.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:42"

    # The drop, pinned: no corners field exists on the decoded wire detection.
    assert "corners_px" not in decoded_detection.__slots__
    assert not hasattr(decoded_detection, "corners_px")
