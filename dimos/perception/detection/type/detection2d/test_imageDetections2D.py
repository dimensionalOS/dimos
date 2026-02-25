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
import time
from unittest.mock import MagicMock

from dimos_lcm.vision_msgs import Detection2DArray as LCMArray
import pytest

from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


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


def _make_detection(
    class_id: int = 0,
    confidence: float = 0.9,
    track_id: int = 1,
    bbox: tuple[float, float, float, float] = (10.0, 20.0, 100.0, 200.0),
) -> Detection2DBBox:
    """Create a Detection2DBBox with given attributes, using a mock Image."""
    img = MagicMock()
    img.ts = time.time()
    img.width = 640
    img.height = 480
    img.shape = (480, 640, 3)
    img.crop.return_value = img

    return Detection2DBBox(
        bbox=bbox,
        track_id=track_id,
        class_id=class_id,
        confidence=confidence,
        name=f"class_{class_id}",
        ts=img.ts,
        image=img,
    )


@pytest.mark.parametrize("class_id", [0, 1, 15, 79])
def test_to_ros_detection2d_class_id_is_str_in_lcm(class_id: int) -> None:
    """
    LCM ObjectHypothesis.class_id type is string, so
    to_ros_detection2d() must encode class_id to string.
    If int is passed, AttributeError: 'int' object has no attribute 'encode' will occur.
    """
    det = _make_detection(class_id=class_id)
    ros_det = det.to_ros_detection2d()

    assert ros_det.results_length == 1, "results_length must equal len(results)"
    assert len(ros_det.results) == 1

    lcm_class_id = ros_det.results[0].hypothesis.class_id
    assert isinstance(lcm_class_id, str), (
        f"LCM class_id must be str, got {type(lcm_class_id).__name__}. "
        "Passing int causes AttributeError in dimos_lcm _encode_one."
    )
    assert lcm_class_id == str(class_id)


@pytest.mark.parametrize("class_id", [0, 1, 15, 79])
def test_to_ros_detection2d_lcm_encode_does_not_crash(class_id: int) -> None:
    """
    to_ros_detection2d() → Detection2DArray → lcm_encode() entire pipeline must not crash.
    """
    det = _make_detection(class_id=class_id)

    ros_det = det.to_ros_detection2d()
    array = LCMArray(
        detections_length=1,
        header=ros_det.header,
        detections=[ros_det],
    )

    encoded = array.lcm_encode()
    assert isinstance(encoded, bytes)
    assert len(encoded) > 0


@pytest.mark.parametrize("class_id", [0, 1, 15, 79])
def test_lcm_roundtrip_class_id_preserved_as_int(class_id: int) -> None:
    """
    Detection2DBBox → LCM serialization → LCM deserialization → from_ros_detection2d() restoration
    class_id must be restored as the original int value.

    After LCM decoding, hypothesis.class_id is str,
    so it must be converted to int() inside from_ros_detection2d().
    """
    det = _make_detection(class_id=class_id, confidence=0.87, track_id=42)
    img = det.image

    # Encode
    ros_det = det.to_ros_detection2d()
    array = LCMArray(
        detections_length=1,
        header=ros_det.header,
        detections=[ros_det],
    )
    encoded = array.lcm_encode()

    # Decode
    decoded_array = LCMArray.lcm_decode(encoded)
    assert decoded_array.detections_length == 1
    decoded_det = decoded_array.detections[0]

    # After LCM decoding, class_id is str
    assert isinstance(decoded_det.results[0].hypothesis.class_id, str)

    # from_ros_detection2d restoration → class_id must be int
    recovered = Detection2DBBox.from_ros_detection2d(decoded_det, image=img)
    assert isinstance(recovered.class_id, int), (
        "Recovered class_id must be int (Detection2DBBox.class_id: int). "
        "from_ros_detection2d must convert str -> int."
    )
    assert recovered.class_id == class_id
    assert recovered.confidence == pytest.approx(0.87, abs=0.01)
    assert recovered.track_id == 42
