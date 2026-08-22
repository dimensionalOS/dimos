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

"""One disposable resource wrapping the memory perception API.

``DanDetector`` owns the models behind :func:`embed_index`, :func:`localize`,
and :func:`inventory`: enter once, query many times on warm weights, and
``stop()`` (or leave the ``with`` block) releases whatever loaded.

Every entry point takes an optional :class:`~dimos.perception.memory.rig.Rig`
describing where poses and 3D geometry come from; without one the store's
shape decides.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal, cast, overload

from dimos.core.resource import Resource
from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow
from dimos.perception.memory.inventory import DEFAULT_VOCABULARY, NamingVocabulary, inventory
from dimos.perception.memory.localize import embed_index, localize
from dimos.perception.memory.rig import Rig

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.memory.stream import Stream
    from dimos.models.embedding.siglip import SigLIPModel
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    from dimos.perception.memory.types import Instance, Localization


class DanDetector(Resource):
    """The perception models as one resource.

    ``start()`` constructs SigLIP, OWLv2, and EdgeTAM. The two
    HuggingFace models load lazily on first use, so an inventory-only
    caller never pays for SigLIP; ``stop()`` releases whatever loaded.
    """

    siglip: SigLIPModel
    detector: Owlv2Detector
    segmenter: EdgeTAMImageSegmenter

    def start(self) -> None:
        import torch

        from dimos.models.embedding.siglip import SigLIPModel
        from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
        from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

        self.siglip = SigLIPModel()
        self.detector = Owlv2Detector(dtype=torch.float16)
        self.segmenter = EdgeTAMImageSegmenter()
        self._live: list[DisposableBase] = []

    def stop(self) -> None:
        for disposable in self._live:
            disposable.dispose()
        self.siglip.stop()
        self.detector.stop()
        del self.segmenter

    @overload
    def embed(
        self,
        store: Any,
        after: float,
        before: float,
        *,
        live: Literal[False] = False,
        rig: Rig | None = ...,
    ) -> Stream[Any, Any]: ...
    @overload
    def embed(
        self,
        store: Any,
        *,
        live: Literal[True],
        rig: Rig | None = ...,
    ) -> Stream[Any, Any]: ...
    def embed(
        self,
        store: Any,
        after: float | None = None,
        before: float | None = None,
        *,
        live: bool = False,
        rig: Rig | None = None,
    ) -> Stream[Any, Any]:
        """SigLIP-embedded, world-posed frame index for :meth:`localize`.

        Replay mode indexes ``[after, before]`` in memory and returns when
        done. ``live=True`` instead tails the rig's color stream and keeps
        saving into the store's named ``color_image_embedded`` stream on a
        background thread; the returned stream is that named stream.
        """
        rig = rig or Rig.from_store(store)
        if not live:
            return embed_index(
                store,
                self.siglip,
                cast("float", after),
                cast("float", before),
                rig=rig,
            )

        from dimos.msgs.sensor_msgs.Image import Image

        embedded: Stream[Any, Any] = store.stream("color_image_embedded", Image)
        pipeline = (
            rig.color.live()
            .filter(lambda obs: obs.data.brightness > 0.1)
            .transform(QualityWindow(lambda img: img.sharpness, window=1.0 / rig.embed_hz))
            .map(lambda obs: obs.derive(data=obs.data, pose=rig.index_pose(obs)))
            .filter(lambda obs: obs.pose is not None)
            .transform(EmbedImages(self.siglip, batch_size=1))
            .save(embedded)
        )
        self._live.append(pipeline.drain_thread())
        return embedded

    def localize(
        self,
        store: Any,
        query: str | list[str],
        *,
        index: Stream[Any, Any],
        **kwargs: Any,
    ) -> list[Localization] | list[list[Localization]]:
        """:func:`localize` on this resource's models."""
        return localize(
            store,
            query,
            index=index,
            siglip=self.siglip,
            detector=self.detector,
            segmenter=self.segmenter,
            **kwargs,
        )

    def inventory(
        self,
        store: Any,
        *,
        naming_vocabulary: NamingVocabulary = DEFAULT_VOCABULARY,
        **kwargs: Any,
    ) -> list[Instance]:
        """:func:`inventory` on this resource's models."""
        return inventory(
            store,
            segmenter=self.segmenter,
            detector=self.detector,
            naming_vocabulary=naming_vocabulary,
            **kwargs,
        )
