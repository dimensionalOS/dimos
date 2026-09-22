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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection2D, Detection2DArray
import numpy as np

from dimos.msgs.image import image_from_array, image_view
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint


def test_generated_bbox_image_and_wire_round_trip():
    header = Header(frame_id="camera_optical", stamp=Time(sec=1700000000, nanosec=123456789))
    pixels = np.zeros((40, 60, 3), dtype=np.uint8)
    pixels[:, :, 0] = 250
    image = image_from_array(pixels, encoding="rgb8", header=header)
    det = Detection2DBBox(
        bbox=(10, 10, 30, 30),
        track_id=7,
        class_id=2,
        confidence=0.9,
        name="target",
        ts=1.0,
        image=image,
    )
    assert det.is_valid()
    crop = det.cropped_image(padding=2)
    assert (crop.width, crop.height) == (24, 24)
    assert crop.header == header
    np.testing.assert_array_equal(image_view(crop), pixels[8:32, 8:32])
    annotated = det.annotated_image()
    assert annotated.encoding == "bgr8"
    assert annotated.header == header
    np.testing.assert_array_equal(image_view(annotated)[39, 59], [0, 0, 250])
    np.testing.assert_array_equal(image_view(image), pixels)
    wire = Detection2D.decode(det.to_ros_detection2d().encode())
    assert wire.header == header
    assert wire.results[0].hypothesis.class_id == "2"
    restored = Detection2DBBox.from_ros_detection2d(wire, image=image)
    assert restored.bbox == det.bbox
    assert restored.track_id == 7
    assert restored.class_id == 2


def test_generated_detection_collection_round_trip():
    pixels = np.zeros((40, 60, 3), dtype=np.uint8)
    image = image_from_array(
        pixels, encoding="rgb8", header=Header(frame_id="camera", stamp=Time(nanosec=123))
    )
    det = Detection2DBBox(
        bbox=(10, 10, 30, 30),
        track_id=7,
        class_id=2,
        confidence=0.9,
        name="target",
        ts=0,
        image=image,
    )
    collection = ImageDetections2D(image=image, detections=[det])
    wire = Detection2DArray.decode(collection.to_ros_detection2d_array().encode())
    assert wire.header == image.header
    assert len(wire.detections) == 1
    restored = ImageDetections2D.from_ros_detection2d_array(image, wire)
    assert len(restored) == 1
    assert restored[0].bbox == det.bbox
    annotated = collection.annotated_image()
    assert annotated.header == image.header
    assert annotated.encoding == "bgr8"
    assert np.any(image_view(annotated))
    assert not np.any(image_view(image))


def test_generated_point_detection_crop_and_cdr():
    pixels = np.arange(100, dtype=np.uint8).reshape(10, 10)
    image = image_from_array(
        pixels, encoding="mono8", header=Header(frame_id="camera", stamp=Time(nanosec=789))
    )
    point = Detection2DPoint(x=1, y=1, name="point", ts=0, image=image)
    assert point.is_valid()
    crop = point.cropped_image(padding=2)
    np.testing.assert_array_equal(image_view(crop), pixels[:3, :3])
    assert crop.header == image.header
    wire = Detection2D.decode(point.to_ros_detection2d().encode())
    assert wire.header == image.header
    assert wire.bbox.center.position.x == wire.bbox.center.position.y == 1
    assert wire.bbox.size_x == wire.bbox.size_y == 0
    assert wire.results[0].hypothesis.class_id == "-1"
