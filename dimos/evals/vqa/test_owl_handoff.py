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

from __future__ import annotations

import numpy as np

from dimos.evals.vqa.primitives.cross_view_identity import SightingView
from dimos.evals.vqa.template_author import LocalizeSightingCatalog
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _image(ts: float = 1.0) -> Image:
    return Image(
        data=np.zeros((20, 20, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        ts=ts,
    )


def test_localize_sighting_catalog_names_and_boxes_near_timestamp() -> None:
    image = _image(1.0)
    far = _image(10.0)
    view = SightingView(
        query="green backpack",
        instance_id="g0",
        image=image,
        bbox=(2.0, 2.0, 8.0, 8.0),
        confidence=0.9,
        timestamp=1.0,
        n_views=2,
        semantic_score=0.5,
    )
    catalog = LocalizeSightingCatalog({"green backpack": {"g0": (view,)}}, ts_tolerance_s=0.25)

    assert catalog.names_for(image) == ("green backpack",)
    assert catalog.names_for(far) == ()
    near = catalog.query_detections(image, "green backpack")
    assert len(near.detections) == 1
    assert near.detections[0].bbox == (2.0, 2.0, 8.0, 8.0)
    assert catalog.query_detections(far, "green backpack").detections == []
