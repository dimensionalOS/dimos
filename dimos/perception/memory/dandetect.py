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
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal, cast, overload

from dimos.core.resource import Resource
from dimos.memory.embed import EmbedImages
from dimos.memory.tf import StreamTF
from dimos.memory.transform import throttle
from dimos.perception.memory import gates
from dimos.perception.memory.gates import OPTICAL_FRAME, TF_TOLERANCE, WORLD_FRAME
from dimos.perception.memory.inventory import DEFAULT_VOCABULARY, NamingVocabulary, inventory
from dimos.perception.memory.localize import EMBED_HZ, embed_index, localize

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
        from dimos.models.embedding.siglip import SigLIPModel
        from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
        from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

        self.siglip = SigLIPModel()
        self.detector = Owlv2Detector()
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
        optical_frame: str = ...,
        world_frame: str = ...,
        tf_tolerance: float = ...,
    ) -> Stream[Any, Any]: ...
    @overload
    def embed(
        self,
        store: Any,
        *,
        live: Literal[True],
        optical_frame: str = ...,
        world_frame: str = ...,
        tf_tolerance: float = ...,
    ) -> Stream[Any, Any]: ...
    def embed(
        self,
        store: Any,
        after: float | None = None,
        before: float | None = None,
        *,
        live: bool = False,
        optical_frame: str = OPTICAL_FRAME,
        world_frame: str = WORLD_FRAME,
        tf_tolerance: float = TF_TOLERANCE,
    ) -> Stream[Any, Any]:
        """SigLIP-embedded, world-posed frame index for :meth:`localize`.

        Replay mode indexes ``[after, before]`` in memory and returns when
        done. ``live=True`` instead tails ``color_image`` and keeps saving
        into the store's named ``color_image_embedded`` stream on a
        background thread; the returned stream is that named stream.
        """
        if not live:
            return embed_index(
                store,
                self.siglip,
                cast("float", after),
                cast("float", before),
                optical_frame=optical_frame,
                world_frame=world_frame,
                tf_tolerance=tf_tolerance,
            )

        from dimos.msgs.sensor_msgs.Image import Image

        tf = StreamTF.from_store(store)
        if tf is None:
            raise ValueError("store has no tf stream")
        embedded: Stream[Any, Any] = store.stream("color_image_embedded", Image)
        pipeline = (
            store.streams.color_image.live()
            .transform(throttle(1.0 / EMBED_HZ))
            .map(
                lambda obs: obs.derive(
                    data=obs.data,
                    pose=gates.camera_pose(tf, obs.ts, optical_frame, world_frame, tf_tolerance),
                )
            )
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
    ) -> Localization | list[Localization | None] | None:
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
