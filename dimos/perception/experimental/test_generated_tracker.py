# Copyright 2026 Dimensional Inc.
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

from unittest.mock import MagicMock

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Detection3DArray,
    Point2D,
    Pose2D,
)
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.image import image_view
from dimos.msgs.time import time_from_seconds
from dimos.perception.experimental.object_tracker_2d import ObjectTracker2D
from dimos.perception.experimental.object_tracker_3d import ObjectTracker3D


@pytest.mark.parametrize("succeeded", [True, False])
def test_tracker_publishes_generated_detection_and_clears_failed_track(succeeded: bool) -> None:
    module = ObjectTracker2D(frame_id="camera")
    module.tracker = MagicMock()
    module.tracker.update.return_value = (succeeded, (10, 12, 20, 16))
    module.tracking_initialized = True
    module._latest_rgb_frame = np.zeros((64, 64, 3), dtype=np.uint8)
    module.detection2darray = MagicMock()
    module.tracked_overlay = MagicMock()
    try:
        module._process_tracking()
        result = cdr_decode(
            cdr_encode(module.detection2darray.publish.call_args.args[0]), Detection2DArray
        )
        assert result.header.frame_id == "camera"
        if not succeeded:
            assert not result.detections
            assert not module.tracking_initialized
            module.tracked_overlay.publish.assert_not_called()
            return
        (detection,) = result.detections
        assert (detection.bbox.center.position.x, detection.bbox.center.position.y) == (20, 20)
        assert (detection.bbox.size_x, detection.bbox.size_y) == (20, 16)
        assert detection.results[0].hypothesis.class_id == "tracked_object"
        overlay = cdr_decode(cdr_encode(module.tracked_overlay.publish.call_args.args[0]), Image)
        assert overlay.header == result.header
        assert image_view(overlay).shape == (64, 64, 3)
        assert overlay.encoding == "rgb8"
    finally:
        module.stop()


@pytest.mark.parametrize("depth", [2.0, 0.0, float("nan")])
def test_tracker3d_generated_projection_preserves_depth_filter_and_source_stamp(
    depth: float,
) -> None:
    module = ObjectTracker3D(frame_id="camera")
    module.camera_intrinsics = [100.0, 100.0, 32.0, 32.0]
    module._latest_depth_frame = np.full((64, 64), depth, dtype=np.float32)
    module.tf = MagicMock()
    module.detection2darray = MagicMock()
    header = Header(stamp=time_from_seconds(12.25), frame_id="camera")
    detection = Detection2DArray(
        header=header,
        detections=[
            Detection2D(
                header=header,
                bbox=BoundingBox2D(
                    center=Pose2D(position=Point2D(x=32, y=32), theta=0.0), size_x=20, size_y=16
                ),
                results=[],
                id="",
            )
        ],
    )
    try:
        output = module._create_detection3d_from_2d(detection)
        if not np.isfinite(depth) or depth <= 0:
            assert output is None
            module.tf.publish.assert_not_called()
            return
        result = cdr_decode(cdr_encode(output), Detection3DArray)
        assert result.header == header
        box = result.detections[0].bbox
        assert (box.center.position.x, box.center.position.y, box.center.position.z) == (2, 0, 0)
        assert np.allclose([box.size.x, box.size.y, box.size.z], [0.4, 0.32, 0.1])
        transform = cdr_decode(
            cdr_encode(module.tf.publish.call_args.args[0]), TFMessage
        ).transforms[0]
        assert transform.header == header
        assert transform.child_frame_id == "tracked_object"
        assert transform.transform.translation.x == 2.0
        assert transform.transform.rotation.w == 1.0
    finally:
        module.stop()
