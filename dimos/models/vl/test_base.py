import os
from unittest.mock import MagicMock

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
import pytest

from dimos.core import LCMTransport
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data

# Captured actual response from Qwen API for cafe.jpg with query "humans"
# Added garbage around JSON to ensure we are robustly extracting it
MOCK_QWEN_RESPONSE = """
   Locating humans for you 😊😊

   [
    ["humans", 76, 368, 219, 580],
    ["humans", 354, 372, 512, 525],
    ["humans", 409, 370, 615, 748],
    ["humans", 628, 350, 762, 528],
    ["humans", 785, 323, 960, 650]
   ]

   Here is some trash at the end of the response :)
   Let me know if you need anything else 😀😊
   """


def test_query_detections_mocked() -> None:
    """Test query_detections with mocked API response (no API key required)."""
    # Load test image
    image = Image.from_file(get_data("cafe.jpg"))

    # Create model and mock the query method
    model = QwenVlModel()
    model.query = MagicMock(return_value=MOCK_QWEN_RESPONSE)  # type: ignore[method-assign]

    # Query for humans in the image
    query = "humans"
    detections = model.query_detections(image, query)

    # Verify the return type
    assert isinstance(detections, ImageDetections2D)

    # Should have 5 detections based on our mock data
    assert len(detections.detections) == 5, (
        f"Expected 5 detections, got {len(detections.detections)}"
    )

    # Verify each detection
    img_height, img_width = image.shape[:2]

    for i, detection in enumerate(detections.detections):
        # Verify attributes
        assert detection.name == "humans"
        assert detection.confidence == 1.0
        assert detection.class_id == -1  # VLM detections use -1 for class_id
        assert detection.track_id == i
        assert len(detection.bbox) == 4

        assert detection.is_valid()

        # Verify bbox coordinates are valid (out-of-bounds detections are discarded)
        x1, y1, x2, y2 = detection.bbox
        assert x2 > x1, f"Detection {i}: Invalid x coordinates: x1={x1}, x2={x2}"
        assert y2 > y1, f"Detection {i}: Invalid y coordinates: y1={y1}, y2={y2}"

        # Check bounds (out-of-bounds detections would have been discarded)
        assert 0 <= x1 <= img_width, f"Detection {i}: x1={x1} out of bounds"
        assert 0 <= x2 <= img_width, f"Detection {i}: x2={x2} out of bounds"
        assert 0 <= y1 <= img_height, f"Detection {i}: y1={y1} out of bounds"
        assert 0 <= y2 <= img_height, f"Detection {i}: y2={y2} out of bounds"

    print(f"✓ Successfully processed {len(detections.detections)} mocked detections")


@pytest.mark.tool
@pytest.mark.skipif(not os.getenv("ALIBABA_API_KEY"), reason="ALIBABA_API_KEY not set")
def test_query_detections_real() -> None:
    """Test query_detections with real API calls (requires API key)."""
    # Load test image
    image = Image.from_file(get_data("cafe.jpg"))

    # Initialize the model (will use real API)
    model = QwenVlModel()

    # Query for humans in the image
    query = "humans"
    detections = model.query_detections(image, query)

    assert isinstance(detections, ImageDetections2D)
    print(detections)

    # Check that detections were found
    if detections.detections:
        for detection in detections.detections:
            # Verify each detection has expected attributes
            assert detection.bbox is not None
            assert len(detection.bbox) == 4
            assert detection.name
            assert detection.confidence == 1.0
            assert detection.class_id == -1  # VLM detections use -1 for class_id
            assert detection.is_valid()

    print(f"Found {len(detections.detections)} detections for query '{query}'")


@pytest.mark.tool
def test_query_points() -> None:
    """Test query_points with real API calls (requires API key)."""
    # Load test image
    image = Image.from_file(get_data("cafe.jpg"), format=ImageFormat.RGB).to_rgb()

    # Initialize the model (will use real API)
    model = MoondreamVlModel()

    # Query for points in the image
    query = "center of each person's head"
    detections = model.query_points(image, query)

    assert isinstance(detections, ImageDetections2D)
    print(detections)

    # Check that detections were found
    if detections.detections:
        for point in detections.detections:
            # Verify each point has expected attributes
            assert hasattr(point, "x")
            assert hasattr(point, "y")
            assert point.name
            assert point.confidence == 1.0
            assert point.class_id == -1  # VLM detections use -1 for class_id
            assert point.is_valid()

    print(f"Found {len(detections.detections)} points for query '{query}'")

    image_topic: LCMTransport[Image] = LCMTransport("/image", Image)
    image_topic.publish(image)
    image_topic.lcm.stop()

    annotations: LCMTransport[ImageAnnotations] = LCMTransport("/annotations", ImageAnnotations)
    annotations.publish(detections.to_foxglove_annotations())
    annotations.lcm.stop()
