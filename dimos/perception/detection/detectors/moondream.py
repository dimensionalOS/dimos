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

"""Text-prompted Moondream object detector adapter."""

from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class Moondream2DDetector(Detector):
    """Identify prompted objects with Moondream's native box detector."""

    def __init__(self, model: MoondreamVlModel | None = None, *, max_objects: int = 5) -> None:
        if max_objects < 1:
            raise ValueError("max_objects must be positive")
        self._model = model or MoondreamVlModel()
        self._model.start()
        self._max_objects = max_objects
        self._text_prompts: tuple[str, ...] = ()

    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: object | None = None,
    ) -> None:
        """Set one or more text queries; visual prompts are unsupported."""
        if bboxes is not None:
            raise ValueError("Moondream detector supports text prompts only")
        if text is None:
            raise ValueError("Moondream detector requires at least one text prompt")
        prompts = tuple(prompt.strip() for prompt in text if prompt.strip())
        if not prompts:
            raise ValueError("Moondream detector requires at least one nonempty text prompt")
        self._text_prompts = prompts

    def process_image(self, image: Image) -> ImageDetections2D:
        """Run prompted object identification on one image."""
        detections = ImageDetections2D(image)
        for prompt in self._text_prompts:
            result = self._model.query_detections(image, prompt, max_objects=self._max_objects)
            detections.detections.extend(result.detections)
        return detections

    def describe_image(self, image: Image, question: str) -> str:
        """Answer an open-ended question about an image with the loaded VLM."""
        return str(self._model.query(image, question))

    def stop(self) -> None:
        """Release the Moondream model and GPU memory."""
        self._model.stop()
