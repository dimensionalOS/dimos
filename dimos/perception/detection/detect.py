# Copyright 2025-2026 Dimensional Inc.
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

"""One-shot VL detection returning existing Detection2DBBox."""

from __future__ import annotations

from typing import Any

from dimos.models.vl.base import VlModel
from dimos.models.vl.create import create
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

_model_cache: dict[str, VlModel[Any]] = {}


def detect(query: str, image: Image, model: str = "qwen") -> Detection2DBBox | None:
    """Detect an object in an image by text description.

    Uses existing VL model infrastructure. Returns existing Detection2DBBox
    type so the result is chainable with .project() and the rest of the
    detection pipeline.

    Args:
        query: Text description (e.g. "person", "red chair").
        image: Camera image to search.
        model: VL model backend (default: "qwen").

    Returns:
        Detection2DBBox if found, None otherwise.
    """
    if model not in _model_cache:
        _model_cache[model] = create(model)

    bbox = get_object_bbox_from_image(_model_cache[model], image, query)
    if bbox is None:
        return None

    return Detection2DBBox(
        bbox=bbox,
        track_id=-1,
        class_id=-1,
        confidence=1.0,
        name=query,
        ts=image.ts,
        image=image,
    )
