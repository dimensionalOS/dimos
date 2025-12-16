import time
from typing import Protocol

import pytest

from dimos.models.vl.florence import Florence2Model
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


@pytest.fixture(scope="module")
def test_image() -> Image:
    return Image.from_file(get_data("cafe.jpg")).to_rgb()


class CaptionerModel(Protocol):
    """Intersection of Captioner and Resource for testing."""

    def caption(self, image: Image) -> str: ...
    def caption_batch(self, *images: Image) -> list[str]: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (Florence2Model, "Florence-2"),
        (MoondreamVlModel, "Moondream"),
    ],
    ids=["florence2", "moondream"],
)
@pytest.mark.gpu
def test_captioner(
    model_class: type[CaptionerModel], model_name: str, test_image: Image
) -> None:
    """Test captioning functionality across different model types."""
    image = test_image

    print(f"\nTesting {model_name} captioning")

    # Initialize model
    print(f"Loading {model_name} model...")
    model = model_class()
    model.start()

    # Test single caption
    print("Generating caption...")
    start_time = time.time()
    caption = model.caption(image)
    caption_time = time.time() - start_time

    print(f"  Caption: {caption}")
    print(f"  Time: {caption_time:.3f}s")

    assert isinstance(caption, str)
    assert len(caption) > 0

    # Test batch captioning
    print("\nTesting batch captioning (3 images)...")
    start_time = time.time()
    captions = model.caption_batch(image, image, image)
    batch_time = time.time() - start_time

    print(f"  Captions: {captions}")
    print(f"  Total time: {batch_time:.3f}s")
    print(f"  Per image: {batch_time / 3:.3f}s")

    assert len(captions) == 3
    assert all(isinstance(c, str) and len(c) > 0 for c in captions)

    print(f"\n{model_name} captioning test passed!")

    model.stop()


@pytest.mark.gpu
def test_florence2_detail_levels(test_image: Image) -> None:
    """Test Florence-2 different detail levels."""
    image = test_image

    model = Florence2Model()
    model.start()

    detail_levels = ["brief", "normal", "detailed", "more_detailed"]

    for detail in detail_levels:
        print(f"\nDetail level: {detail}")
        start_time = time.time()
        caption = model.caption(image, detail=detail)
        caption_time = time.time() - start_time

        print(f"  Caption ({len(caption)} chars): {caption[:100]}...")
        print(f"  Time: {caption_time:.3f}s")

        assert isinstance(caption, str)
        assert len(caption) > 0
