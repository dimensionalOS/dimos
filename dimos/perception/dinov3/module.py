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

"""DINOv3 patch-embedding module: camera images in, dense feature grids out."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.embedding.base import PatchEmbeddings
from dimos.models.embedding.dinov3 import DINOv3Model
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from reactivex.observable import Observable


class DINOv3ModuleConfig(ModuleConfig):
    model_name: str = "vit_huge_plus_patch16_dinov3"
    normalize: bool = True
    #: Embed at most this many frames per second (sharpest frame per window).
    max_freq: float = 2.0
    #: Model device; None auto-selects cuda when available.
    device: str | None = None
    #: Inference resolution (width, height); patch grid is (h/16, w/16).
    input_size: tuple[int, int] = (848, 480)


class DINOv3Module(Module):
    """Publish a DINOv3 patch-feature grid for each processed camera image.

    Output vectors are (grid_h, grid_w, dim) numpy arrays carried in
    :class:`PatchEmbeddings`, timestamped from the source image. DINOv3 has
    no text tower, so downstream consumers must query by patch similarity or
    PCA rather than text.
    """

    config: DINOv3ModuleConfig
    model: DINOv3Model

    color_image: In[Image]
    patch_embeddings: Out[PatchEmbeddings]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        model_kwargs: dict[str, Any] = {
            "model_name": self.config.model_name,
            "normalize": self.config.normalize,
            "input_size": self.config.input_size,
        }
        if self.config.device is not None:
            model_kwargs["device"] = self.config.device
        self.model = DINOv3Model(**model_kwargs)

    def _process(self, image: Image) -> PatchEmbeddings:
        embeddings = self.model.embed_patches(image)
        # Publish CPU numpy so subscribers never need a matching GPU/torch device.
        embeddings.vector = embeddings.to_numpy()
        return embeddings

    @simple_mcache
    def patch_embedding_stream(self) -> Observable[PatchEmbeddings]:
        stream = self.color_image.pure_observable()
        if self.config.max_freq > 0:
            stream = stream.pipe(sharpness_barrier(self.config.max_freq))
        return backpressure(stream.pipe(ops.map(self._process)))

    @rpc
    def start(self) -> None:
        super().start()
        self.model.start()
        self.register_disposable(
            self.patch_embedding_stream().subscribe(self.patch_embeddings.publish)
        )

    @rpc
    def stop(self) -> None:
        self.model.stop()
        super().stop()
