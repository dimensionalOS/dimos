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

from threading import Event
from unittest.mock import MagicMock

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Point2D,
    Pose2D,
)
import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds
from dimos.perception.detection.reid.module import ReidModule
from dimos.perception.detection.reid.type import PassthroughIDSystem


def test_generated_detection_alignment_uses_headers_and_filters_empty_arrays(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scheduler = ThreadPoolScheduler(max_workers=2)
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    module = ReidModule(idsystem=PassthroughIDSystem())
    images: Subject[Image] = Subject()
    detections: Subject[Detection2DArray] = Subject()
    module.image = MagicMock()
    module.image.pure_observable.return_value = images
    module.detections = MagicMock()
    module.detections.pure_observable.return_value = detections
    received = []
    errors = []
    ready = Event()

    def receive(value: object) -> None:
        received.append(value)
        ready.set()

    subscription = module.detections_stream().subscribe(receive, errors.append)
    header = Header(stamp=time_from_seconds(1.25), frame_id="camera")
    image = image_from_array(np.zeros((32, 32, 3), dtype=np.uint8), encoding="rgb8", header=header)
    try:
        detections.on_next(Detection2DArray(header=header))
        detections.on_next(
            Detection2DArray(
                header=header,
                detections=[
                    Detection2D(
                        header=header,
                        id="7",
                        bbox=BoundingBox2D(
                            center=Pose2D(position=Point2D(x=16, y=16)), size_x=8, size_y=8
                        ),
                    )
                ],
            )
        )
        images.on_next(image)
        assert ready.wait(timeout=2.0)
        assert not errors
        (result,) = received
        assert result.image is image
        (detection,) = result.detections
        assert detection.track_id == 7
        assert detection.bbox == (12, 12, 20, 20)
        assert result.ts == 1.25
    finally:
        subscription.dispose()
        images.dispose()
        detections.dispose()
        module.stop()
        scheduler.executor.shutdown(wait=True)
