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

"""The swappable models behind localize's stages, as one disposable resource.

``localize`` sees three interfaces only: the embedder is any dimos
``EmbeddingModel`` (index frames and queries), a ``Detector`` scores every
query against every box it proposes, a ``Segmenter`` turns those boxes into
masks. Each stage takes a registry name, a ``module:attr`` path to a class
or factory, or a built model. Enter once, query many times on warm weights;
``stop()`` releases whatever loaded, after the live embedding that uses them.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import importlib
from typing import TYPE_CHECKING, Any, Protocol

from dimos.core.resource import Resource
from dimos.memory.embed import EmbedImages
from dimos.perception.localize.localize import index_frames, localize
from dimos.perception.localize.types import LocalizePolicy

if TYPE_CHECKING:
    from collections.abc import Callable

    import numpy as np
    from reactivex.abc import DisposableBase

    from dimos.memory.stream import Stream
    from dimos.models.embedding.base import EmbeddingModel
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.localize.association import Groups
    from dimos.perception.localize.localize import LocalizeTrace
    from dimos.perception.localize.rig import Rig
    from dimos.perception.localize.types import Localization


class Detector(Protocol):
    def query_score_rows(
        self, image: Image, queries: list[str], threshold: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Pixel ``(x1, y1, x2, y2)`` boxes whose best query scores above
        ``threshold``, and their ``(n_boxes, n_queries)`` scores."""
        ...

    def stop(self) -> None: ...


class Segmenter(Protocol):
    def segment(self, detections: ImageDetections2D[Any]) -> ImageDetections2D[Any]:
        """The detections as masks, each keeping its ``track_id``; any may be dropped."""
        ...

    def stop(self) -> None: ...


class NoSegmenter:
    """Boxes lift as they are; the lift keeps the depth cluster at the box median."""

    def segment(self, detections: ImageDetections2D[Any]) -> ImageDetections2D[Any]:
        return detections

    def stop(self) -> None:
        pass


def _owlv2() -> Detector:
    import torch

    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

    return Owlv2Detector(dtype=torch.float16)


@dataclass(frozen=True)
class Option:
    """A registered model: how to build it, and the policy fields its score scale shifts."""

    build: str | Callable[[], Any]  # "module:attr" of a class or factory, or the callable
    policy: dict[str, Any] = field(default_factory=dict)


EMBEDDERS: dict[str, Option] = {
    "siglip": Option("dimos.models.embedding.siglip:SigLIPModel"),
    "clip": Option("dimos.models.embedding.clip:CLIPModel"),
}
DETECTORS: dict[str, Option] = {
    "owlv2": Option(_owlv2),
    "omdet": Option(
        "dimos.perception.detection.detectors.omdet:OmDetDetector",
        {"candidate_floor": 0.15, "accept_score": 0.30},
    ),
    "gdino": Option(
        "dimos.perception.detection.detectors.grounding_dino:GroundingDinoDetector",
        {"candidate_floor": 0.35, "accept_score": 0.55},
    ),
}
SEGMENTERS: dict[str, Option] = {
    "edgetam": Option("dimos.models.segmentation.edge_tam:EdgeTAMImageSegmenter"),
    "yoloe": Option("dimos.models.segmentation.yoloe:YoloeBoxSegmenter"),
    "none": Option(NoSegmenter),
}


def _option(registry: dict[str, Option], spec: Any) -> Option:
    if not isinstance(spec, str):
        return Option(lambda: spec)
    if spec in registry:
        return registry[spec]
    if ":" not in spec:
        raise ValueError(f"unknown model {spec!r}: one of {', '.join(registry)} or module:attr")
    return Option(spec)


def _build(option: Option) -> Any:
    if not isinstance(option.build, str):
        return option.build()
    module, _, attr = option.build.partition(":")
    return getattr(importlib.import_module(module), attr)()


class DanDetector(Resource):
    """One embedder, detector and segmenter; the HuggingFace models load on first use."""

    embedder: EmbeddingModel
    detector: Detector
    segmenter: Segmenter

    def __init__(
        self, embedder: Any = "siglip", detector: Any = "owlv2", segmenter: Any = "edgetam"
    ) -> None:
        specs = (embedder, detector, segmenter)
        self.names = tuple(s if isinstance(s, str) else type(s).__name__ for s in specs)
        self.options = tuple(
            _option(registry, spec)
            for registry, spec in zip((EMBEDDERS, DETECTORS, SEGMENTERS), specs, strict=True)
        )

    def start(self) -> None:
        self.embedder, self.detector, self.segmenter = (_build(o) for o in self.options)
        self._live: list[DisposableBase] = []

    def stop(self) -> None:
        for disposable in self._live:
            disposable.dispose()
        for model in (self.embedder, self.detector, self.segmenter):
            model.stop()
        del self.embedder, self.detector, self.segmenter

    def policy(self, overrides: dict[str, Any] | None = None) -> LocalizePolicy:
        """``LocalizePolicy`` shifted to these models' score scales, then ``overrides``."""
        shifts = {key: value for option in self.options for key, value in option.policy.items()}
        return replace(LocalizePolicy(), **{**shifts, **(overrides or {})})

    def embed_live(self, rig: Rig, source: Stream[Any, Any], index: Stream[Any, Any]) -> None:
        """Tail ``source`` into ``index`` on a background thread while the resource is open."""
        pipeline = (
            index_frames(rig, source.live())
            .transform(EmbedImages(self.embedder, batch_size=1))
            .save(index)
        )
        self._live.append(pipeline.drain_thread())

    def localize(
        self,
        queries: list[str],
        *,
        index: Stream[Any, Any],
        rig: Rig,
        policy: LocalizePolicy | None = None,
        traces: list[LocalizeTrace] | None = None,
        groups: dict[str, Groups] | None = None,
    ) -> list[list[Localization]]:
        """:func:`localize` on this resource's models, at :meth:`policy` unless given one."""
        return localize(
            queries,
            index=index,
            rig=rig,
            embedder=self.embedder,
            detector=self.detector,
            segmenter=self.segmenter,
            policy=policy or self.policy(),
            traces=traces,
            groups=groups,
        )
