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

from contextlib import ExitStack
from copy import deepcopy
from unittest.mock import PropertyMock

import numpy as np
import pytest
from reactivex.subject import Subject

from dimos.message_utils import camera_info, image as image_utils
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import Detection2DArray
from dimos.perception.detection.person_tracker import (
    PersonTracker,
    _align_images_and_detections as align_person,
)
from dimos.perception.detection.reid.module import (
    _align_images_and_detections as align_reid,
)
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms


@pytest.mark.parametrize("align", [align_person, align_reid], ids=["person", "reid"])
@pytest.mark.parametrize("seconds", [0, 1700000000])
def test_alignment_retains_float_matching_and_filters_empty_arrays(align, seconds):
    image = image_utils.from_array(
        np.zeros((32, 32, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(frame_id="camera", stamp=Time(sec=seconds, nanosec=123456789)),
    )
    second = deepcopy(image)
    second.header.stamp.nanosec += 1
    box = Detection2DBBox(
        image=second,
        bbox=(12, 12, 20, 20),
        track_id=7,
        class_id=0,
        confidence=0.9,
        name="person",
        ts=0.0,
    )
    packet = ImageDetections2D(second, [box]).to_ros_detection2d_array()
    received = []
    with ExitStack() as cleanup:
        images, detections = Subject(), Subject()
        cleanup.callback(images.dispose)
        cleanup.callback(detections.dispose)
        subscription = align(images, detections).subscribe(received.append)
        cleanup.callback(subscription.dispose)
        detections.on_next(Detection2DArray(header=second.header, detections=[]))
        images.on_next(second)
        assert received == []
        detections.on_next(packet)
        assert len(received) == 1
        assert received[0].image is second
        assert received[0][0].track_id == 7
        assert received[0][0].bbox == (12, 12, 20, 20)
        images.on_next(image)
        assert len(received) == (1 if seconds == 0 else 2)


def test_person_target_composition_keeps_detection_source_stamp(mocker):
    header = Header(frame_id="camera_optical", stamp=Time(sec=1700000000, nanosec=123456789))
    image = image_utils.from_array(
        np.zeros((32, 32, 3), dtype=np.uint8), encoding="rgb8", header=header
    )
    box = Detection2DBBox(
        image=image,
        bbox=(12, 12, 20, 20),
        track_id=7,
        class_id=0,
        confidence=1.0,
        name="person",
        ts=1700000000.123456789,
    )
    camera = camera_info.from_intrinsics(10, 10, 16, 16, 32, 32, header=header)
    with ExitStack() as cleanup:
        module = PersonTracker(cameraInfo=camera)
        cleanup.callback(module.stop)
        publish = mocker.patch.object(module.target, "publish")
        mount = frames_to_edge_transforms([("camera_link", "world", (1, 2, 3), (0, 0, 0))])[0]
        mount.header.stamp = Time(sec=1699999999, nanosec=123456789)
        mocker.patch.object(
            PersonTracker,
            "tfbuffer",
            new_callable=PropertyMock,
            return_value=mocker.Mock(get=mocker.Mock(return_value=mount)),
        )
        module.track(ImageDetections2D(image, [box]))
        target = publish.call_args.args[0]
        assert target.header.frame_id == "world"
        assert target.header.stamp == image.header.stamp
        position = target.pose.position
        assert (position.x, position.y, position.z) == (3, 2, 3)
