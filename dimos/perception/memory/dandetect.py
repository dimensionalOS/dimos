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

"""The perception models as one disposable resource.

Enter once, query many times on warm weights; ``stop()`` releases whatever
loaded. Every entry point takes an optional
:class:`~dimos.perception.memory.rig.Rig`; without one the store's shape
decides.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.resource import Resource
from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow
from dimos.perception.memory.localize import embed_index, localize
from dimos.perception.memory.rig import Rig

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.memory.stream import Stream
    from dimos.models.embedding.siglip import SigLIPModel
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    from dimos.perception.memory.types import Localization, LocalizePolicy


class DanDetector(Resource):
    """SigLIP, OWLv2 and EdgeTAM; the HuggingFace two load on first use."""

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

    def embed(
        self, store: Any, after: float, before: float, *, rig: Rig | None = None
    ) -> Stream[Any, Any]:
        """SigLIP-embedded, world-posed index over ``[after, before]``."""
        return embed_index(store, self.siglip, after, before, rig=rig or Rig.from_store(store))

    def embed_live(self, store: Any, *, rig: Rig | None = None) -> Stream[Any, Any]:
        """Tail the colour stream into ``color_image_embedded`` on a background thread.

        Returns that named stream, which :meth:`localize` reads like a replay
        index; it keeps filling for as long as the resource is open.
        """
        from dimos.msgs.sensor_msgs.Image import Image

        rig = rig or Rig.from_store(store)
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
        policy: LocalizePolicy | None = None,
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
            policy=policy,
            **kwargs,
        )
