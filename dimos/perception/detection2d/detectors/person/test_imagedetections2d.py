# Copyright 2025 Dimensional Inc.
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

"""Test ImageDetections2D with pose detections."""

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection2d.type import ImageDetections2D
from dimos.utils.data import get_data


def test_image_detections_2d_with_person():
    """Test creating ImageDetections2D from person detector."""
    # Load image and detect people
    image = Image.from_file(get_data("cafe.jpg"))
    detector = YoloPersonDetector()
    people = detector.detect_people(image)

    # Create ImageDetections2D using from_pose_detector
    image_detections = ImageDetections2D.from_pose_detector(image, people)

    # Verify structure
    assert image_detections.image is image
    assert len(image_detections.detections) == len(people)
    assert all(det in people for det in image_detections.detections)

    # Test image annotations (includes pose keypoints)
    annotations = image_detections.to_foxglove_annotations()
    print(f"\nImageDetections2D created with {len(people)} people")
    print(f"Total text annotations: {annotations.texts_length}")
    print(f"Total points annotations: {annotations.points_length}")

    # Points should include: bounding boxes + keypoints + skeleton lines
    # At least 3 annotations per person (bbox, keypoints, skeleton)
    assert annotations.points_length >= len(people) * 3

    # Text annotations should include confidence, name/id, and keypoint count
    assert annotations.texts_length >= len(people) * 3

    print("\n✓ ImageDetections2D.from_pose_detector working correctly!")


if __name__ == "__main__":
    test_image_detections_2d_with_person()
