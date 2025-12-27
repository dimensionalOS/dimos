import os
import time

import pytest

from dimos.models.vl.moondream_hosted import MoondreamHostedVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import ImageDetections2D

# Skip all tests in this module if API key is missing
pytestmark = pytest.mark.skipif(
    not os.getenv("MOONDREAM_API_KEY"),
    reason="MOONDREAM_API_KEY not set"
)

@pytest.fixture
def model() -> MoondreamHostedVlModel:
    return MoondreamHostedVlModel()

@pytest.fixture
def test_image() -> Image:
    image_path = os.path.join(os.getcwd(), "assets/test.png")
    if not os.path.exists(image_path):
        pytest.skip(f"Test image not found at {image_path}")
    return Image.from_file(image_path)

def test_caption(model: MoondreamHostedVlModel, test_image: Image) -> None:
    """Test generating a caption."""
    print("\n--- Testing Caption ---")
    caption = model.caption(test_image)
    print(f"Caption: {caption}")
    assert isinstance(caption, str)
    assert len(caption) > 0

def test_query(model: MoondreamHostedVlModel, test_image: Image) -> None:
    """Test querying the image."""
    print("\n--- Testing Query ---")
    question = "Is there an xbox controller in the image?"
    answer = model.query(test_image, question)
    print(f"Question: {question}")
    print(f"Answer: {answer}")
    assert isinstance(answer, str)
    assert len(answer) > 0
    # The answer should likely be positive given the user's prompt
    assert "yes" in answer.lower() or "controller" in answer.lower()

def test_query_latency(model: MoondreamHostedVlModel, test_image: Image) -> None:
    """Test that a simple query returns in under 1 second."""
    print("\n--- Testing Query Latency ---")
    question = "What is this?"

    # Warmup (optional, but good practice if first call establishes connection)
    # model.query(test_image, "warmup")

    start_time = time.perf_counter()
    model.query(test_image, question)
    end_time = time.perf_counter()

    duration = end_time - start_time
    print(f"Query took {duration:.4f} seconds")

    assert duration < 1.0, f"Query took too long: {duration:.4f}s > 1.0s"

@pytest.mark.parametrize("subject", ["xbox controller", "lip balm"])
def test_detect(model: MoondreamHostedVlModel, test_image: Image, subject: str) -> None:
    """Test detecting objects."""
    print(f"\n--- Testing Detect: {subject} ---")
    detections = model.query_detections(test_image, subject)

    assert isinstance(detections, ImageDetections2D)
    print(f"Found {len(detections.detections)} detections for {subject}")

    # We expect to find at least one of each in the provided test image
    assert len(detections.detections) > 0

    for det in detections.detections:
        assert det.is_valid()
        assert det.name == subject
        # Check if bbox coordinates are within image dimensions
        x1, y1, x2, y2 = det.bbox
        assert 0 <= x1 < x2 <= test_image.width
        assert 0 <= y1 < y2 <= test_image.height

@pytest.mark.parametrize("subject", ["xbox controller", "lip balm"])
def test_point(model: MoondreamHostedVlModel, test_image: Image, subject: str) -> None:
    """Test pointing at objects."""
    print(f"\n--- Testing Point: {subject} ---")
    points = model.point(test_image, subject)

    print(f"Found {len(points)} points for {subject}: {points}")
    assert isinstance(points, list)
    assert len(points) > 0

    for x, y in points:
        assert isinstance(x, (int, float))
        assert isinstance(y, (int, float))
        assert 0 <= x <= test_image.width
        assert 0 <= y <= test_image.height
