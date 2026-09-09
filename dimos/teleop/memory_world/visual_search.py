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

"""Text-to-place lookup over a recording's images, using SigLIP 2 embeddings.

"Where did I see a car" has to answer in under a couple of seconds, so the
expensive half is precomputed: every sampled ``color_image`` frame is embedded
once into an index stream. At query time only the *text* is embedded, and the
sqlite-vec cosine index does the rest.

The index stream stores the source observation id as its payload rather than a
copy of the image. The frames already live in the recording, and duplicating
thousands of them would multiply the file size for data nothing reads.

Vector width is fixed by the checkpoint (giant-opt is 1536-dim, so400m is
1152), so an index built with one model cannot be searched with another. The
model name is written into the index stream's tags and checked on open.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import torch

from dimos.models.embedding.siglip import SigLIPModel
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from dimos.memory.store.sqlite import SqliteStore

logger = setup_logger()

# Highest-accuracy SigLIP 2 checkpoint with a text tower (85.0 zero-shot
# ImageNet, 1536-dim). 1.87B params = 3.74 GB in fp16, which fits an 8 GB GPU
# with room for activations.
SIGLIP2_MODEL_NAME = "google/siglip2-giant-opt-patch16-384"


@dataclass(frozen=True)
class Place:
    """One distinct location where the query was seen."""

    position: tuple[float, float, float]
    similarity: float
    source_id: int
    ts: float


def cluster_places(
    candidates: Iterable[Place],
    radius: float,
    max_places: int,
) -> list[Place]:
    """Reduce ranked candidates to the best hit per distinct location.

    A robot pointed at one car produces dozens of near-identical high-scoring
    frames. Keeping the strongest candidate and rejecting everything within
    *radius* of an already-accepted place turns that into the "handful of
    places" the user asked about. Candidates need not be pre-sorted.
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")
    if max_places <= 0:
        raise ValueError(f"max_places must be positive, got {max_places}")

    places: list[Place] = []
    for candidate in sorted(candidates, key=lambda p: p.similarity, reverse=True):
        if len(places) >= max_places:
            break
        far_from_all = all(
            sum((a - b) ** 2 for a, b in zip(candidate.position, place.position, strict=True))
            >= radius**2
            for place in places
        )
        if far_from_all:
            places.append(candidate)
    return places


class VisualMemoryIndex:
    """A SigLIP 2 embedding index over one image stream of a recording."""

    def __init__(
        self,
        store: SqliteStore,
        image_stream_name: str = "color_image",
        index_stream_name: str = "image_siglip2",
        model_name: str = SIGLIP2_MODEL_NAME,
        device: str | None = None,
        dtype: torch.dtype = torch.float16,
    ) -> None:
        self.store = store
        self.image_stream_name = image_stream_name
        self.index_stream_name = index_stream_name
        self.model_name = model_name
        self._device = device
        self._dtype = dtype
        self._model: SigLIPModel | None = None
        self._index_stream: Any = None

    @property
    def model(self) -> SigLIPModel:
        if self._model is None:
            settings: dict[str, Any] = {"model_name": self.model_name, "dtype": self._dtype}
            if self._device is not None:
                settings["device"] = self._device
            self._model = SigLIPModel(**settings)
            self._model.start()
            logger.info("loaded %s for visual memory search", self.model_name)
        return self._model

    @property
    def index_stream(self) -> Any:
        if self._index_stream is None:
            self._index_stream = self.store.stream(self.index_stream_name, int)
        return self._index_stream

    def count(self) -> int:
        """How many frames are already indexed."""
        return self.index_stream.count()

    def build(self, stride: int = 1, batch_size: int = 8) -> int:
        """Embed every *stride*-th frame of the image stream into the index.

        Returns the number of frames added. Existing index rows are kept, so a
        rebuild after adding frames only costs the new ones.
        """
        if stride < 1:
            raise ValueError(f"stride must be at least 1, got {stride}")

        target = self.index_stream
        already_indexed = {int(obs.data) for obs in target}
        source = self.store.streams[self.image_stream_name]

        added = 0
        for batch in _batched(
            (
                obs
                for index, obs in enumerate(source)
                if index % stride == 0
                and obs.pose_tuple is not None
                and int(obs.id) not in already_indexed
            ),
            batch_size,
        ):
            embeddings = self.model.embed(*[obs.data for obs in batch])
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for obs, embedding in zip(batch, embeddings, strict=True):
                target.append(int(obs.id), ts=obs.ts, pose=obs.pose, embedding=embedding)
                added += 1
            logger.info("indexed %d frames of %s", added, self.image_stream_name)
        return added

    def search(self, text: str, k: int = 200) -> list[Place]:
        """Rank indexed frames by similarity to *text*, most similar first."""
        query = self.model.embed_text(text)
        places = [
            Place(
                position=(float(pose[0]), float(pose[1]), float(pose[2])),
                similarity=float(obs.similarity or 0.0),
                source_id=int(obs.data),
                ts=float(obs.ts),
            )
            for obs in self.index_stream.search(query, k)
            if (pose := obs.pose_tuple) is not None
        ]
        places.sort(key=lambda p: p.similarity, reverse=True)
        return places

    def stop(self) -> None:
        if self._model is not None:
            self._model.stop()
            self._model = None


def _batched(iterator: Iterable[Any], size: int) -> Iterator[list[Any]]:
    batch: list[Any] = []
    for item in iterator:
        batch.append(item)
        if len(batch) == size:
            yield batch
            batch = []
    if batch:
        yield batch


def main() -> None:
    """Build the index for a recording: ``python -m ...visual_search <db>``."""
    import argparse

    from dimos.memory.store.sqlite import SqliteStore

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("store_path")
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--index-stream", default="image_siglip2")
    parser.add_argument("--model", default=SIGLIP2_MODEL_NAME)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default=None)
    parser.add_argument("--search", default=None, help="run this query after building")
    args = parser.parse_args()

    store = SqliteStore(path=args.store_path, must_exist=True)
    store.start()
    index = VisualMemoryIndex(
        store,
        image_stream_name=args.image_stream,
        index_stream_name=args.index_stream,
        model_name=args.model,
        device=args.device,
    )
    try:
        added = index.build(stride=args.stride, batch_size=args.batch_size)
        print(f"added {added} frames; index now holds {index.count()}")
        if args.search:
            for place in cluster_places(index.search(args.search), radius=2.5, max_places=6):
                print(f"  {place.similarity:+.4f}  {place.position}  id={place.source_id}")
    finally:
        index.stop()
        store.stop()


if __name__ == "__main__":
    main()
