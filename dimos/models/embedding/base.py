# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Generic, TypeVar

import numpy as np
import torch

from dimos.models.base import HuggingFaceModelConfig, LocalModelConfig
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import Image


@dataclass
class EmbeddingModelConfig(LocalModelConfig):
    """Base config for embedding models."""

    normalize: bool = True


@dataclass
class HuggingFaceEmbeddingModelConfig(HuggingFaceModelConfig):
    """Base config for HuggingFace-based embedding models."""

    normalize: bool = True


class Embedding(Timestamped):
    """Base class for embeddings with vector data.

    Supports both torch.Tensor (for GPU-accelerated comparisons) and np.ndarray.
    Embeddings are kept as torch.Tensor on device by default for efficiency.
    """

    vector: torch.Tensor | np.ndarray  # type: ignore[type-arg]

    def __init__(self, vector: torch.Tensor | np.ndarray, timestamp: float | None = None) -> None:  # type: ignore[type-arg]
        self.vector = vector
        if timestamp:
            self.timestamp = timestamp
        else:
            self.timestamp = time.time()

    def __matmul__(self, other: Embedding) -> float:
        """Compute cosine similarity via @ operator."""
        if isinstance(self.vector, torch.Tensor):
            other_tensor = other.to_torch(self.vector.device)
            result = self.vector @ other_tensor
            return result.item()
        return float(self.vector @ other.to_numpy())

    def to_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
        """Convert to numpy array (moves to CPU if needed)."""
        if isinstance(self.vector, torch.Tensor):
            return self.vector.detach().cpu().numpy()
        return self.vector

    def to_torch(self, device: str | torch.device | None = None) -> torch.Tensor:
        """Convert to torch tensor on specified device."""
        if isinstance(self.vector, np.ndarray):
            tensor = torch.from_numpy(self.vector)
            return tensor.to(device) if device else tensor

        if device is not None and self.vector.device != torch.device(device):
            return self.vector.to(device)
        return self.vector

    def to_cpu(self) -> Embedding:
        """Move embedding to CPU, returning self for chaining."""
        if isinstance(self.vector, torch.Tensor):
            self.vector = self.vector.cpu()
        return self


E = TypeVar("E", bound="Embedding")


class EmbeddingModel(ABC, Generic[E]):
    """Abstract base class for embedding models supporting vision and language."""

    device: str

    @abstractmethod
    def embed(self, *images: Image) -> E | list[E]:
        """
        Embed one or more images.
        Returns single Embedding if one image, list if multiple.
        """
        pass

    @abstractmethod
    def embed_text(self, *texts: str) -> E | list[E]:
        """
        Embed one or more text strings.
        Returns single Embedding if one text, list if multiple.
        """
        pass

    def compare_one_to_many(self, query: E, candidates: list[E]) -> torch.Tensor:
        """
        Efficiently compare one query against many candidates on GPU.

        Args:
            query: Query embedding
            candidates: List of candidate embeddings

        Returns:
            torch.Tensor of similarities (N,)
        """
        query_tensor = query.to_torch(self.device)
        candidate_tensors = torch.stack([c.to_torch(self.device) for c in candidates])
        return query_tensor @ candidate_tensors.T

    def compare_many_to_many(self, queries: list[E], candidates: list[E]) -> torch.Tensor:
        """
        Efficiently compare all queries against all candidates on GPU.

        Args:
            queries: List of query embeddings
            candidates: List of candidate embeddings

        Returns:
            torch.Tensor of similarities (M, N) where M=len(queries), N=len(candidates)
        """
        query_tensors = torch.stack([q.to_torch(self.device) for q in queries])
        candidate_tensors = torch.stack([c.to_torch(self.device) for c in candidates])
        return query_tensors @ candidate_tensors.T

    def query(self, query_emb: E, candidates: list[E], top_k: int = 5) -> list[tuple[int, float]]:
        """
        Find top-k most similar candidates to query (GPU accelerated).

        Args:
            query_emb: Query embedding
            candidates: List of candidate embeddings
            top_k: Number of top results to return

        Returns:
            List of (index, similarity) tuples sorted by similarity (descending)
        """
        similarities = self.compare_one_to_many(query_emb, candidates)
        top_values, top_indices = similarities.topk(k=min(top_k, len(candidates)))
        return [(idx.item(), val.item()) for idx, val in zip(top_indices, top_values, strict=False)]

    def warmup(self) -> None:
        """Optional warmup method to pre-load model."""
        pass
