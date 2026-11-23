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

"""OmDet boxes → BoT-SORT ids → optional crop-embedding identity."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from dimos.models.embedding.base import EmbeddingModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.identity.mot import BotSortMot, Mot
from dimos.perception.identity.reid import AppearanceReId

DEFAULT_QUERIES: tuple[str, ...] = (
    "person",
    "car",
    "backpack",
    "handbag",
    "suitcase",
    "chair",
    "bench",
    "dog",
    "bicycle",
    "motorcycle",
)


class OpenVocabDetector(Protocol):
    def query_detections(
        self, image: Image, queries: list[str], threshold: float = 0.3
    ) -> ImageDetections2D: ...


class IdentityTracker:
    def __init__(
        self,
        detector: OpenVocabDetector,
        mot: Mot,
        reid: AppearanceReId | None = None,
        *,
        queries: Sequence[str] = DEFAULT_QUERIES,
        det_threshold: float = 0.3,
    ) -> None:
        if not queries:
            raise ValueError("queries must not be empty")
        self._detector = detector
        self._mot = mot
        self._reid = reid
        self._queries = tuple(queries)
        self._det_threshold = det_threshold

    def update(self, image: Image) -> list[Detection2DBBox]:
        boxes = self._detector.query_detections(
            image, list(self._queries), threshold=self._det_threshold
        )
        tracked = self._mot.update(list(boxes.detections), image.to_opencv())
        if self._reid is None:
            return tracked
        return self._reid.assign(tracked, image)

    def reset(self) -> None:
        reset = getattr(self._mot, "reset", None)
        if callable(reset):
            reset()
        if self._reid is not None:
            self._reid.reset()

    def stop(self) -> None:
        stop = getattr(self._detector, "stop", None)
        if callable(stop):
            stop()


def load_embedder(name: str) -> EmbeddingModel:
    if name == "clip":
        from dimos.models.embedding.clip import CLIPModel

        return CLIPModel()
    if name == "siglip":
        from dimos.models.embedding.siglip import SigLIPModel

        return SigLIPModel()
    if name == "dino":
        from dimos.models.embedding.dino import DINOModel

        return DINOModel()
    raise ValueError(f"unknown embedder {name!r}; use clip, siglip, or dino")


def build_identity_tracker(
    *,
    queries: Sequence[str] = DEFAULT_QUERIES,
    det_threshold: float = 0.3,
    reid: bool = True,
    embedder: str = "siglip",
    reid_threshold: float = 0.75,
) -> IdentityTracker:
    from dimos.perception.detection.detectors.omdet import OmDetDetector

    appearance = None
    if reid:
        appearance = AppearanceReId(load_embedder(embedder), threshold=reid_threshold)
    return IdentityTracker(
        OmDetDetector(),
        BotSortMot(),
        appearance,
        queries=queries,
        det_threshold=det_threshold,
    )
