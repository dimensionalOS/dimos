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

"""Moondream implementation of object-detection evidence."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class MoondreamObjectDetector:
    """Adapt the existing Moondream VLM to the family-facing detector contract."""

    def __init__(self, model: MoondreamVlModel) -> None:
        self._model = model

    def detect(self, image: Image, object_name: str) -> ImageDetections2D:
        return self._model.query_detections(image, object_name)
