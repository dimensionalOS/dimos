import json
import os
from unittest.mock import MagicMock

import pytest

from dimos.models.vl.base import extract_json
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.type import ImageDetections2D
from dimos.utils.data import get_data

# Captured actual response from Qwen API for cafe.jpg with query "humans"
MOCK_QWEN_RESPONSE = """
   Here you go bro:

   [
    ["humans", 76, 368, 219, 580],
    ["humans", 354, 372, 512, 525],
    ["humans", 409, 370, 615, 748],
    ["humans", 628, 350, 762, 528],
    ["humans", 785, 323, 960, 650]
   ]

   Hope this helps!😀😊 :)"""


def test_extract_json_clean_response():
    """Test extract_json with clean JSON response."""
    clean_json = '[["object", 1, 2, 3, 4]]'
    result = extract_json(clean_json)
    assert result == [["object", 1, 2, 3, 4]]


def test_extract_json_with_text_before_after():
    """Test extract_json with text before and after JSON."""
    messy = """Here's what I found:
    [
        ["person", 10, 20, 30, 40],
        ["car", 50, 60, 70, 80]
    ]
    Hope this helps!"""
    result = extract_json(messy)
    assert result == [["person", 10, 20, 30, 40], ["car", 50, 60, 70, 80]]


def test_extract_json_with_emojis():
    """Test extract_json with emojis and markdown code blocks."""
    messy = """Sure! 😊 Here are the detections:

    ```json
    [["human", 100, 200, 300, 400]]
    ```

    Let me know if you need anything else! 👍"""
    result = extract_json(messy)
    assert result == [["human", 100, 200, 300, 400]]


def test_extract_json_multiple_json_blocks():
    """Test extract_json when there are multiple JSON blocks."""
    messy = """First attempt (wrong format):
    {"error": "not what we want"}

    Correct format:
    [
        ["cat", 10, 10, 50, 50],
        ["dog", 60, 60, 100, 100]
    ]

    Another block: {"also": "not needed"}"""
    result = extract_json(messy)
    # Should return the first valid array
    assert result == [["cat", 10, 10, 50, 50], ["dog", 60, 60, 100, 100]]


def test_extract_json_object():
    """Test extract_json with JSON object instead of array."""
    response = 'The result is: {"status": "success", "count": 5}'
    result = extract_json(response)
    assert result == {"status": "success", "count": 5}


def test_extract_json_nested_structures():
    """Test extract_json with nested arrays and objects."""
    response = """Processing complete:
    [
        ["label1", 1, 2, 3, 4],
        {"nested": {"value": 10}},
        ["label2", 5, 6, 7, 8]
    ]"""
    result = extract_json(response)
    assert result[0] == ["label1", 1, 2, 3, 4]
    assert result[1] == {"nested": {"value": 10}}
    assert result[2] == ["label2", 5, 6, 7, 8]


def test_extract_json_invalid():
    """Test extract_json raises error when no valid JSON found."""
    response = "This response has no valid JSON at all!"
    with pytest.raises(json.JSONDecodeError) as exc_info:
        extract_json(response)
    assert "Could not extract valid JSON" in str(exc_info.value)


def test_extract_json_with_real_llm_response():
    """Test extract_json with the actual messy response."""
    result = extract_json(MOCK_QWEN_RESPONSE)
    assert isinstance(result, list)
    assert len(result) == 5
    assert result[0] == ["humans", 76, 368, 219, 580]
    assert result[-1] == ["humans", 785, 323, 960, 650]


def test_query_detections_mocked():
    """Test query_detections with mocked API response (no API key required)."""
    # Load test image
    image = Image.from_file(get_data("cafe.jpg"))

    # Create model and mock the query method
    model = QwenVlModel()
    model.query = MagicMock(return_value=MOCK_QWEN_RESPONSE)

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
        assert detection.class_id == -100
        assert detection.track_id == i
        assert len(detection.bbox) == 4

        # Verify bbox coordinates are valid and clamped
        x1, y1, x2, y2 = detection.bbox
        assert x2 > x1, f"Detection {i}: Invalid x coordinates: x1={x1}, x2={x2}"
        assert y2 > y1, f"Detection {i}: Invalid y coordinates: y1={y1}, y2={y2}"

        # Check bounds
        assert 0 <= x1 <= img_width, f"Detection {i}: x1={x1} out of bounds"
        assert 0 <= x2 <= img_width, f"Detection {i}: x2={x2} out of bounds"
        assert 0 <= y1 <= img_height, f"Detection {i}: y1={y1} out of bounds"
        assert 0 <= y2 <= img_height, f"Detection {i}: y2={y2} out of bounds"

        # Verify clamping worked (the 3rd detection has y2=748 which exceeds image height of 771)
        if i == 2:  # Third detection
            assert y2 <= img_height, f"Detection {i}: y2={y2} should be clamped to {img_height}"

    print(f"✓ Successfully processed {len(detections.detections)} mocked detections")


@pytest.mark.tool
@pytest.mark.skipif(not os.getenv("ALIBABA_API_KEY"), reason="ALIBABA_API_KEY not set")
def test_query_detections_real():
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
            assert detection.class_id == -100

            # Verify bbox coordinates are valid
            x1, y1, x2, y2 = detection.bbox
            assert x2 > x1, f"Invalid x coordinates: x1={x1}, x2={x2}"
            assert y2 > y1, f"Invalid y coordinates: y1={y1}, y2={y2}"

            # Verify coordinates are within image bounds
            img_height, img_width = image.shape[:2]
            assert 0 <= x1 <= img_width
            assert 0 <= x2 <= img_width
            assert 0 <= y1 <= img_height
            assert 0 <= y2 <= img_height

    print(f"Found {len(detections.detections)} detections for query '{query}'")
