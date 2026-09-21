# Copyright 2026 Dimensional Inc.
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

from dimos.core.transport import LCMTransport
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.utils.data import get_data


@pytest.mark.skipif_no_alibaba
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
