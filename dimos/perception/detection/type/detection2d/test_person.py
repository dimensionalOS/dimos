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
import pytest


def test_person_ros_confidence() -> None:
    """Test that Detection2DPerson preserves confidence when converting to ROS format."""

    from dimos.msgs.sensor_msgs import Image
    from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
    from dimos.perception.detection.type.detection2d.person import Detection2DPerson
    from dimos.utils.data import get_data

    # Load test image
    image_path = get_data("cafe.jpg")
    image = Image.from_file(image_path)

    # Run pose detection
    detector = YoloPersonDetector(device="cpu")
    detections = detector.process_image(image)

    # Find a Detection2DPerson (should have at least one person in cafe.jpg)
    person_detections = [d for d in detections.detections if isinstance(d, Detection2DPerson)]
    assert len(person_detections) > 0, "No person detections found in cafe.jpg"

    # Test each person detection
    for person_det in person_detections:
        original_confidence = person_det.confidence
        assert 0.0 <= original_confidence <= 1.0, "Confidence should be between 0 and 1"

        # Convert to ROS format
        ros_det = person_det.to_ros_detection2d()

        # Extract confidence from ROS message
        assert len(ros_det.results) > 0, "ROS detection should have results"
        ros_confidence = ros_det.results[0].hypothesis.score

        # Verify confidence is preserved (allow small floating point tolerance)
        assert original_confidence == pytest.approx(ros_confidence, abs=0.001), (
            f"Confidence mismatch: {original_confidence} != {ros_confidence}"
        )

        print("\nSuccessfully preserved confidence in ROS conversion for Detection2DPerson:")
        print(f"  Original confidence: {original_confidence:.3f}")
        print(f"  ROS confidence: {ros_confidence:.3f}")
        print(f"  Track ID: {person_det.track_id}")
        print(f"  Visible keypoints: {len(person_det.get_visible_keypoints(threshold=0.3))}/17")


def test_person_from_ros_raises() -> None:
    """Test that Detection2DPerson.from_ros_detection2d() raises NotImplementedError."""
    from dimos.perception.detection.type.detection2d.person import Detection2DPerson

    with pytest.raises(NotImplementedError) as exc_info:
        Detection2DPerson.from_ros_detection2d()

    # Verify the error message is informative
    error_msg = str(exc_info.value)
    assert "keypoint data" in error_msg.lower()
    assert "Detection2DBBox" in error_msg
