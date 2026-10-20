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

from contextlib import ExitStack
from threading import Event

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Point2D,
    Pose2D,
)
from dimos.msgs.image import image_view
from dimos.perception.experimental.object_tracker_2d import ObjectTracker2D
from dimos.perception.experimental.object_tracker_3d import ObjectTracker3D


@pytest.fixture
def tracker_factory():
    with ExitStack() as cleanup:

        def create(cls):
            transports = cleanup.enter_context(ExitStack())
            module = cls(frame_id="camera")
            cleanup.callback(module.stop)
            for name, output in module.outputs.items():
                wire = CdrTransport("tracker_" + name, output.type, backend="lcm", url="memq://")
                transports.callback(wire.stop)
                output.transport = wire
            return module

        yield create, cleanup


@pytest.mark.parametrize("succeeded", [True, False])
def test_tracker_publishes_generated_detection_and_clears_failed_track(
    tracker_factory, mocker, succeeded
):
    create, cleanup = tracker_factory
    module = create(ObjectTracker2D)
    tracker = mocker.Mock()
    tracker.update.return_value = (succeeded, (10, 12, 20, 16))
    mocker.patch.object(module, "tracker", tracker)
    module.tracking_initialized = True
    module._latest_rgb_frame = np.zeros((64, 64, 3), dtype=np.uint8)
    detections, overlays = [], []
    detection_ready, overlay_ready = Event(), Event()

    def receive_detection(value):
        detections.append(value)
        detection_ready.set()

    def receive_overlay(value):
        overlays.append(value)
        overlay_ready.set()

    cleanup.callback(module.detection2darray.transport.subscribe(receive_detection))
    cleanup.callback(module.tracked_overlay.transport.subscribe(receive_overlay))
    module._process_tracking()
    assert detection_ready.wait(2)
    assert len(detections) == 1
    result = detections[0]
    assert result.header.frame_id == "camera"
    if not succeeded:
        assert not result.detections
        assert not module.tracking_initialized
        assert not overlays
        return
    assert overlay_ready.wait(2)
    (detection,) = result.detections
    assert (detection.bbox.center.position.x, detection.bbox.center.position.y) == (20, 20)
    assert (detection.bbox.size_x, detection.bbox.size_y) == (20, 16)
    assert detection.results[0].hypothesis.class_id == "tracked_object"
    assert len(overlays) == 1
    assert overlays[0].header == result.header
    assert image_view(overlays[0]).shape == (64, 64, 3)
    assert overlays[0].encoding == "rgb8"


@pytest.mark.parametrize("depth", [2.0, 0.0, float("nan")])
def test_tracker3d_projection_preserves_depth_filter_and_source_stamp(tracker_factory, depth):
    create, cleanup = tracker_factory
    module = create(ObjectTracker3D)
    module.camera_intrinsics = [100.0, 100.0, 32.0, 32.0]
    module._latest_depth_frame = np.full((64, 64), depth, dtype=np.float32)
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="camera")
    detection = Detection2DArray(
        header=header,
        detections=[
            Detection2D(
                header=header,
                bbox=BoundingBox2D(
                    center=Pose2D(position=Point2D(x=32, y=32), theta=0), size_x=20, size_y=16
                ),
                results=[],
                id="",
            )
        ],
    )
    received = []
    ready = Event()

    def receive(value):
        received.append(value)
        ready.set()

    cleanup.callback(module.tf.transport.subscribe(receive))
    output = module._create_detection3d_from_2d(detection)
    if not np.isfinite(depth) or depth <= 0:
        assert output is None
        assert not received
        return
    result = decode(encode(output), type(output))
    assert result.header == header
    box = result.detections[0].bbox
    assert (box.center.position.x, box.center.position.y, box.center.position.z) == (2, 0, 0)
    np.testing.assert_allclose([box.size.x, box.size.y, box.size.z], [0.4, 0.32, 0.1])
    assert ready.wait(2)
    assert len(received) == 1
    (transform,) = received[0].transforms
    assert transform.header == header
    assert transform.child_frame_id == "tracked_object"
    assert transform.transform.translation.x == 2
    assert transform.transform.rotation.w == 1
