# Copyright 2025-2026 Dimensional Inc.
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

"""Pure embedding helpers for object association."""

from __future__ import annotations

from collections.abc import MutableSequence, Sequence
from typing import Any

import numpy as np


def normalize_embedding(value: Any) -> np.ndarray | None:
    """Return a unit float32 vector from an embedding-like value, or None if unusable."""
    if value is None:
        return None
    vector = np.asarray(value, dtype=np.float32).reshape(-1)
    if vector.size == 0:
        return None
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 0.0 else None


def gallery_cos(embedding: Any, gallery: Sequence[Any]) -> float | None:
    """Best compatible cosine of an embedding against a stored view gallery."""
    emb = normalize_embedding(embedding)
    if emb is None or not gallery:
        return None
    values: list[float] = []
    for item in gallery:
        view = normalize_embedding(item)
        if view is not None and view.size == emb.size:
            values.append(float(np.dot(emb, view)))
    return max(values) if values else None


def add_diverse_embedding_view(
    gallery: MutableSequence[Any],
    embedding: Any,
    *,
    novelty: float,
    max_size: int,
) -> None:
    """Add an embedding view only when it increases gallery diversity."""
    emb = normalize_embedding(embedding)
    if emb is None or max_size <= 0:
        return
    if not gallery:
        gallery.append(emb)
        return
    cosine = gallery_cos(emb, gallery)
    if cosine is not None and cosine >= novelty:
        return
    if len(gallery) < max_size:
        gallery.append(emb)
        return
    gallery[int(np.argmax(_embedding_redundancy_scores(gallery)))] = emb


def _embedding_redundancy_scores(gallery: Sequence[Any]) -> list[float]:
    scores: list[float] = []
    for i, item in enumerate(gallery):
        score = 0.0
        for j, other in enumerate(gallery):
            if i != j:
                score += gallery_cos(item, [other]) or 0.0
        scores.append(score)
    return scores
