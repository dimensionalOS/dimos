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

"""SigLIP 2 patch-embedding module: camera images in, embedding grids out."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from reactivex import operators as ops

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.models.embedding.base import PatchEmbeddings
from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from reactivex.observable import Observable


class SigLIP2ModuleConfig(ModuleConfig):
    #: so400m at a 144-token budget beat base-256, so400m-384 and naflex@64 on
    #: fridge/human/flag localization while costing about the same as base.
    model_name: str = "google/siglip2-so400m-patch16-naflex"
    normalize: bool = True
    #: Embed at most this many frames per second (sharpest frame per window).
    max_freq: float = 2.0
    #: Model device; None auto-selects cuda when available.
    device: str | None = None
    #: NaFlex token budget per image (576 -> 18x32 grid on a wide frame);
    #: None keeps the checkpoint default. Ignored by fixed-resolution models.
    #: 576 costs barely more than 144 (56ms vs 51ms on the 5070 laptop, 18fps
    #: single-frame) and won the fridge/crate A/B on grounding cleanliness.
    max_num_patches: int | None = 576


class SigLIP2Module(Module):
    """Publish a SigLIP 2 patch-embedding grid for each processed camera image.

    Output vectors are (grid_h, grid_w, dim) numpy arrays carried in
    :class:`PatchEmbeddings`, timestamped from the source image.
    """

    config: SigLIP2ModuleConfig
    model: SigLIP2Model

    color_image: In[Image]
    patch_embeddings: Out[PatchEmbeddings]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        model_kwargs: dict[str, Any] = {
            "model_name": self.config.model_name,
            "normalize": self.config.normalize,
        }
        if self.config.device is not None:
            model_kwargs["device"] = self.config.device
        if self.config.max_num_patches is not None:
            model_kwargs["max_num_patches"] = self.config.max_num_patches
        self.model = SigLIP2Model(**model_kwargs)

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


demo_siglip2 = autoconnect(
    CameraModule.blueprint(),
    SigLIP2Module.blueprint(),
)
