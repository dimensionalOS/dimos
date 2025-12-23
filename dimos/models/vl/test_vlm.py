import time
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,  # type: ignore[import-untyped]
)
import pytest

from dimos.core import LCMTransport
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.cli.plot import bar
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel


# For these tests you can run foxglove-bridge to visualize results
# You can also run lcm-spy to confirm that messages are being published


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
        (QwenVlModel, "Qwen"),
    ],
)
@pytest.mark.gpu
def test_vlm_bbox_detections(model_class: "type[VlModel]", model_name: str) -> None:
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"Testing {model_name}")

    # Initialize model
    print(f"Loading {model_name} model...")
    model: VlModel = model_class()
    model.start()

    queries = [
        "glasses",
        "blue shirt",
        "bulb",
        "cigarette",
        "reflection of a car",
        "knee",
        "flowers on the left table",
        "shoes",
        "leftmost persons ear",
        "rightmost arm",
    ]

    all_detections = ImageDetections2D(image)
    query_times = []

    # Publish to LCM with model-specific channel names
    annotations_transport: LCMTransport[ImageAnnotations] = LCMTransport(
        "/annotations", ImageAnnotations
    )

    image_transport: LCMTransport[Image] = LCMTransport("/image", Image)

    image_transport.publish(image)

    # Then run VLM queries
    for query in queries:
        print(f"\nQuerying for: {query}")
        start_time = time.time()
        detections = model.query_detections(image, query, max_objects=5)
        query_time = time.time() - start_time
        query_times.append(query_time)

        print(f"  Found {len(detections)} detections in {query_time:.3f}s")
        all_detections.detections.extend(detections.detections)
        annotations_transport.publish(all_detections.to_foxglove_annotations())

    avg_time = sum(query_times) / len(query_times) if query_times else 0
    print(f"\n{model_name} Results:")
    print(f"  Average query time: {avg_time:.3f}s")
    print(f"  Total detections: {len(all_detections)}")
    print(all_detections)

    annotations_transport.publish(all_detections.to_foxglove_annotations())

    annotations_transport.lcm.stop()
    image_transport.lcm.stop()
    model.stop()


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
        (QwenVlModel, "Qwen"),
    ],
)
@pytest.mark.gpu
def test_vlm_point_detections(model_class: "type[VlModel]", model_name: str) -> None:
    """Test VLM point detection capabilities."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"Testing {model_name} point detection")

    # Initialize model
    print(f"Loading {model_name} model...")
    model: VlModel = model_class()
    model.start()

    queries = [
        "center of each person's head",
        "tip of the nose",
        "center of the glasses",
        "cigarette tip",
        "center of each light bulb",
        "center of each shoe",
    ]

    all_detections = ImageDetections2D(image)
    query_times = []

    # Publish to LCM with model-specific channel names
    annotations_transport: LCMTransport[ImageAnnotations] = LCMTransport(
        "/annotations", ImageAnnotations
    )

    image_transport: LCMTransport[Image] = LCMTransport("/image", Image)

    image_transport.publish(image)

    # Then run VLM queries
    for query in queries:
        print(f"\nQuerying for: {query}")
        start_time = time.time()
        detections = model.query_points(image, query)
        query_time = time.time() - start_time
        query_times.append(query_time)

        print(f"  Found {len(detections)} points in {query_time:.3f}s")
        all_detections.detections.extend(detections.detections)
        annotations_transport.publish(all_detections.to_foxglove_annotations())

    avg_time = sum(query_times) / len(query_times) if query_times else 0
    print(f"\n{model_name} Results:")
    print(f"  Average query time: {avg_time:.3f}s")
    print(f"  Total points: {len(all_detections)}")
    print(all_detections)

    annotations_transport.publish(all_detections.to_foxglove_annotations())

    annotations_transport.lcm.stop()
    image_transport.lcm.stop()
    model.stop()


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
    ],
)
@pytest.mark.gpu
def test_vlm_query_multi(model_class: "type[VlModel]", model_name: str) -> None:
    """Test query_multi optimization - single image, multiple queries."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"\nTesting {model_name} query_multi optimization")

    model: VlModel = model_class()
    model.start()

    queries = [
        "How many people are in this image?",
        "What color is the leftmost person's shirt?",
        "Are there any glasses visible?",
        "What's on the table?",
    ]

    # Sequential queries
    print("\nSequential queries:")
    start_time = time.time()
    sequential_results = [model.query(image, q) for q in queries]
    sequential_time = time.time() - start_time
    print(f"  Time: {sequential_time:.3f}s")

    # Batched queries (encode image once)
    print("\nBatched queries (query_multi):")
    start_time = time.time()
    batch_results = model.query_multi(image, queries)
    batch_time = time.time() - start_time
    print(f"  Time: {batch_time:.3f}s")

    speedup_pct = (sequential_time - batch_time) / sequential_time * 100
    print(f"\nSpeedup: {speedup_pct:.1f}%")

    # Print results
    for q, seq_r, batch_r in zip(queries, sequential_results, batch_results, strict=True):
        print(f"\nQ: {q}")
        print(f"  Sequential: {seq_r[:120]}...")
        print(f"  Batch:      {batch_r[:120]}...")

    model.stop()


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
    ],
)
@pytest.mark.tool
@pytest.mark.gpu
def test_vlm_query_batch(model_class: "type[VlModel]", model_name: str) -> None:
    """Test query_batch optimization - multiple images, same query."""
    from dimos.utils.testing import TimedSensorReplay

    # Load 5 frames at 1-second intervals using TimedSensorReplay
    replay = TimedSensorReplay[Image]("unitree_go2_office_walk2/video")
    images = [replay.find_closest_seek(i).to_rgb() for i in range(0, 10, 2)]  # type: ignore[union-attr]

    print(f"\nTesting {model_name} query_batch with {len(images)} images")

    model: VlModel = model_class()
    model.start()

    query = "Describe this image in a short sentence"

    # Sequential queries (print as they come in)
    print("\nSequential queries:")
    sequential_results = []
    start_time = time.time()
    for i, img in enumerate(images):
        result = model.query(img, query)
        sequential_results.append(result)
        print(f"  [{i}] {result[:120]}...")
    sequential_time = time.time() - start_time
    print(f"  Time: {sequential_time:.3f}s")

    # Batched queries (pre-encode all images)
    print("\nBatched queries (query_batch):")
    start_time = time.time()
    batch_results = model.query_batch(images, query)
    batch_time = time.time() - start_time
    for i, result in enumerate(batch_results):
        print(f"  [{i}] {result[:120]}...")
    print(f"  Time: {batch_time:.3f}s")

    speedup_pct = (sequential_time - batch_time) / sequential_time * 100
    print(f"\nSpeedup: {speedup_pct:.1f}%")

    # Verify results are valid strings
    assert len(batch_results) == len(images)
    assert all(isinstance(r, str) and len(r) > 0 for r in batch_results)

    model.stop()


@pytest.mark.parametrize(
    "model_class,sizes",
    [
        (MoondreamVlModel, [None, (512, 512), (256, 256)]),
        (QwenVlModel, [None, (512, 512), (256, 256)]),
    ],
)
@pytest.mark.gpu
def test_vlm_resize(
    model_class: "type[VlModel]",
    sizes: list[tuple[int, int] | None],
) -> None:
    """Test VLM auto_resize effect on performance."""
    from dimos.utils.testing import TimedSensorReplay

    replay = TimedSensorReplay[Image]("unitree_go2_office_walk2/video")
    image = replay.find_closest_seek(0).to_rgb()  # type: ignore[union-attr]

    labels: list[str] = []
    avg_times: list[float] = []

    for auto_resize in sizes:
        resize_str = f"{auto_resize[0]}x{auto_resize[1]}" if auto_resize else "full"
        print(f"\nOriginal image: {image.width}x{image.height}, auto_resize: {resize_str}")

        model: VlModel = model_class(auto_resize=auto_resize)
        model.start()

        times = []
        for i in range(3):
            start = time.time()
            result = model.query_detections(image, "box")
            elapsed = time.time() - start
            times.append(elapsed)
            print(f"  [{i}] ({elapsed:.2f}s)", result)

        avg = sum(times) / len(times)
        print(f"Avg time: {avg:.2f}s")
        labels.append(resize_str)
        avg_times.append(avg)

        # Free GPU memory before next model
        model.stop()

    # Plot results
    print(f"\n{model_class.__name__} resize performance:")
    bar(labels, avg_times, title=f"{model_class.__name__} Query Time", ylabel="seconds")
