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

from dimos.evals.vqa.families import AVAILABLE_FAMILIES
from dimos.evals.vqa.template_author import names_from_detections, proposals_from_detected_names
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
import numpy as np


def _image() -> Image:
    return Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam", ts=1.0
    )


def test_names_from_detections_rank_by_frequency() -> None:
    image = _image()

    def box(track_id: int, name: str, conf: float) -> Detection2DBBox:
        return Detection2DBBox(
            bbox=(0.0, 0.0, 2.0, 2.0),
            track_id=track_id,
            class_id=0,
            confidence=conf,
            name=name,
            ts=image.ts,
            image=image,
        )

    dets = ImageDetections2D(
        image=image,
        detections=[box(0, "Chair", 0.9), box(1, "chair", 0.8), box(2, "plant", 0.7)],
    )
    assert names_from_detections(dets)[0] == "Chair"


def test_proposals_from_detected_names_fill_templates() -> None:
    families = tuple(
        family
        for family in AVAILABLE_FAMILIES
        if family.name
        in {
            "presence",
            "object_count",
            "horizontal_direction",
            "image_coverage",
            "largest_visible_area",
        }
    )
    proposals = proposals_from_detected_names(("chair", "plant"), families)
    families_hit = {p.family for p in proposals}
    assert "presence" in families_hit
    assert "object_count" in families_hit
    assert any(p.family == "largest_visible_area" and set(p.object_names) == {"chair", "plant"} for p in proposals)
