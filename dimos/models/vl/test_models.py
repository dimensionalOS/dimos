import time

import pytest
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations

from dimos.core import LCMTransport
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
        (QwenVlModel, "Qwen"),
    ],
    ids=["moondream", "qwen"],
)
@pytest.mark.heavy
def test_vlm(model_class, model_name):
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"\n{'=' * 60}")
    print(f"Testing {model_name}")
    print(f"{'=' * 60}")

    # Initialize model
    print(f"Loading {model_name} model...")
    model: VlModel = model_class()

    queries = [
        "glasses",
        "blue shirt",
        "bulb",
        "dog",
        "flowers on the left table",
        "shoes",
        "leftmost persons ear",
        "rightmost arm",
    ]

    all_detections = ImageDetections2D(image)
    query_times = []

    for query in queries:
        print(f"\nQuerying for: {query}")
        start_time = time.time()
        detections = model.query_detections(image, query, max_objects=5)
        query_time = time.time() - start_time
        query_times.append(query_time)

        print(f"  Found {len(detections)} detections in {query_time:.3f}s")
        all_detections.detections.extend(detections.detections)

    avg_time = sum(query_times) / len(query_times) if query_times else 0
    print(f"\n{model_name} Results:")
    print(f"  Average query time: {avg_time:.3f}s")
    print(f"  Total detections: {len(all_detections)}")
    print(all_detections)

    # Publish to LCM with model-specific channel names
    annotations_transport: LCMTransport[ImageAnnotations] = LCMTransport(
        "/annotations", ImageAnnotations
    )
    annotations_transport.publish(all_detections.to_foxglove_annotations())

    image_transport: LCMTransport[Image] = LCMTransport("/image", Image)
    image_transport.publish(image)

    annotations_transport.lcm.stop()
    image_transport.lcm.stop()

    print(f"Published {model_name} annotations and image to LCM")
