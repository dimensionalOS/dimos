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
import pytest

from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

pytestmark = pytest.mark.slow


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


def test_filter(imageDetections2d: ImageDetections2D) -> None:
    dets = imageDetections2d.detections
    assert len(dets) >= 1, "fixture should provide at least one detection"

    # No predicates → keep everything.
    assert imageDetections2d.filter().detections == dets

    # Single predicate that always fails → empty.
    result = imageDetections2d.filter(lambda d: False)
    assert result.detections == []
    assert isinstance(result, ImageDetections2D)  # subclass preserved

    # Single predicate that always passes → same detections, new instance.
    result = imageDetections2d.filter(lambda d: True)
    assert result.detections == dets
    assert result is not imageDetections2d

    # Multi-predicate cascade: only keep detections with confidence > 0.5 AND
    # class_id == first detection's class_id. Cascade must AND all predicates.
    target_cls = dets[0].class_id
    expected = [d for d in dets if d.confidence > 0.5 and d.class_id == target_cls]
    result = imageDetections2d.filter(
        lambda d: d.confidence > 0.5,
        lambda d: d.class_id == target_cls,
    )
    assert result.detections == expected

    # Preserves image.
    assert result.image is imageDetections2d.image
