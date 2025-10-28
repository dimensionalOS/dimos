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

from dimos.perception.detection.type import ImageDetections2D


def test_from_ros_detection2d_array(get_moment_2d) -> None:
    moment = get_moment_2d()

    detections2d = moment["detections2d"]

    test_image = detections2d.image

    # Convert to ROS detection array
    ros_array = detections2d.to_ros_detection2d_array()

    # Convert back to ImageDetections2D
    recovered = ImageDetections2D.from_ros_detection2d_array(test_image, ros_array)

    # Verify we got the same number of detections
    assert len(recovered.detections) == len(detections2d.detections)

    # Verify the detection matches
    original_det = detections2d.detections[0]
    recovered_det = recovered.detections[0]

    # Check bbox is approximately the same (allow 1 pixel tolerance due to float conversion)
    for orig_val, rec_val in zip(original_det.bbox, recovered_det.bbox, strict=False):
        assert orig_val == pytest.approx(rec_val, abs=1.0)

    # Check other properties
    assert recovered_det.track_id == original_det.track_id
    assert recovered_det.class_id == original_det.class_id
    assert recovered_det.confidence == pytest.approx(original_det.confidence, abs=0.01)

    print("\nSuccessfully round-tripped detection through ROS format:")
    print(f"  Original bbox: {original_det.bbox}")
    print(f"  Recovered bbox: {recovered_det.bbox}")
    print(f"  Track ID: {recovered_det.track_id}")
    print(f"  Confidence: {recovered_det.confidence:.3f}")
