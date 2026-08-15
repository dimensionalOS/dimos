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

"""Text-query scoring over an embedding voxel map."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

#: Generic prompts a query is contrasted against by default.
DEFAULT_BACKGROUND_PROMPTS = (
    "the floor",
    "a wall",
    "the ceiling",
    "an office",
    "furniture",
    "a doorway",
    "clutter on a desk",
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.models.embedding.base import EmbeddingModel
    from dimos.perception.hyperspace.voxel_map import EmbeddingVoxelMap


def score_query(
    voxel_map: EmbeddingVoxelMap,
    text_model: EmbeddingModel,
    query: str,
    background_prompts: tuple[str, ...] | list[str] = DEFAULT_BACKGROUND_PROMPTS,
    smooth_iterations: int = 2,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    """(centers (N, 3), scores (N,)) of every voxel against a text query.

    Raw patch-token cosines barely separate a query from generic office
    surfaces, so each voxel scores as cos(query) minus the max cosine over
    the background prompts; near-duplicate backgrounds (querying "floor"
    against "the floor") are dropped. Scores are then averaged over voxel
    neighborhoods to suppress single-voxel speckle.
    """
    prompts = [query, *background_prompts]
    embeddings = text_model.embed_text(*prompts)
    if not isinstance(embeddings, list):
        embeddings = [embeddings]
    centers, scores = voxel_map.query(embeddings[0].to_numpy())
    distinct = [e for e in embeddings[1:] if embeddings[0] @ e < 0.85]
    if distinct:
        background = np.stack([voxel_map.query(e.to_numpy())[1] for e in distinct]).max(axis=0)
        scores = scores - background
    if smooth_iterations > 0:
        scores = voxel_map.smooth_scores(scores, iterations=smooth_iterations)
    return centers, scores
