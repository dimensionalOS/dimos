# Copyright 2025 Dimensional Inc.
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

from collections.abc import Callable

from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D


class VLM2DDetector(Detector):
    def __init__(
        self,
        model: Callable[[], VlModel] | VlModel,
        query: str,
    ) -> None:
        self.model = model() if callable(model) else model
        self.query = query

    def process_image(self, image: Image) -> ImageDetections2D:
        return self.model.query_detections(image, self.query)
