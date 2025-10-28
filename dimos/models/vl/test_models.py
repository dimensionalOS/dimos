import time
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
import pytest

from dimos.core import LCMTransport
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
        (QwenVlModel, "Qwen"),
    ],
    ids=["moondream", "qwen"],
)
@pytest.mark.gpu
def test_vlm(model_class, model_name: str) -> None:
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"Testing {model_name}")

    # Initialize model
    print(f"Loading {model_name} model...")
    model: VlModel = model_class()
    model.warmup()

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

    # # First, run YOLO detection
    # print("\nRunning YOLO detection...")
    # yolo_detector = Yolo2DDetector()
    # yolo_detections = yolo_detector.process_image(image)
    # print(f"  YOLO found {len(yolo_detections.detections)} objects")
    # all_detections.detections.extend(yolo_detections.detections)
    # annotations_transport.publish(all_detections.to_foxglove_annotations())

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
