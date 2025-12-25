from collections.abc import Generator
import time
from typing import Protocol, TypeVar

import pytest

from dimos.models.vl.florence import Florence2Model
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


class CaptionerModel(Protocol):
    """Intersection of Captioner and Resource for testing."""

    def caption(self, image: Image) -> str: ...
    def caption_batch(self, *images: Image) -> list[str]: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...


M = TypeVar("M", bound=CaptionerModel)


@pytest.fixture(scope="module")
def test_image() -> Image:
    return Image.from_file(get_data("cafe.jpg")).to_rgb()


def generic_model_fixture(model_type: type[M]) -> Generator[M, None, None]:
    model_instance = model_type()
    model_instance.start()
    yield model_instance
    model_instance.stop()


@pytest.fixture(params=[Florence2Model, MoondreamVlModel])
def captioner_model(request: pytest.FixtureRequest) -> Generator[CaptionerModel, None, None]:
    yield from generic_model_fixture(request.param)


@pytest.fixture(params=[Florence2Model])
def florence2_model(request: pytest.FixtureRequest) -> Generator[Florence2Model, None, None]:
    yield from generic_model_fixture(request.param)


@pytest.mark.gpu
def test_captioner(captioner_model: CaptionerModel, test_image: Image) -> None:
    """Test captioning functionality across different model types."""
    # Test single caption
    start_time = time.time()
    caption = captioner_model.caption(test_image)
    caption_time = time.time() - start_time

    print(f"  Caption: {caption}")
    print(f"  Time: {caption_time:.3f}s")

    assert isinstance(caption, str)
    assert len(caption) > 0

    # Test batch captioning
    print("\nTesting batch captioning (3 images)...")
    start_time = time.time()
    captions = captioner_model.caption_batch(test_image, test_image, test_image)
    batch_time = time.time() - start_time

    print(f"  Captions: {captions}")
    print(f"  Total time: {batch_time:.3f}s")
    print(f"  Per image: {batch_time / 3:.3f}s")

    assert len(captions) == 3
    assert all(isinstance(c, str) and len(c) > 0 for c in captions)


@pytest.mark.gpu
def test_florence2_detail_levels(florence2_model: Florence2Model, test_image: Image) -> None:
    """Test Florence-2 different detail levels."""
    detail_levels = ["brief", "normal", "detailed", "more_detailed"]

    for detail in detail_levels:
        print(f"\nDetail level: {detail}")
        start_time = time.time()
        caption = florence2_model.caption(test_image, detail=detail)
        caption_time = time.time() - start_time

        print(f"  Caption ({len(caption)} chars): {caption[:100]}...")
        print(f"  Time: {caption_time:.3f}s")

        assert isinstance(caption, str)
        assert len(caption) > 0
