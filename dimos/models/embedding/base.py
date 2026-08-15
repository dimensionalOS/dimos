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

from __future__ import annotations

from abc import ABC, abstractmethod
import time
from typing import TYPE_CHECKING, overload

import numpy as np
import torch

from dimos.core.resource import Resource
from dimos.models.base import HuggingFaceModelConfig, LocalModelConfig
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


class EmbeddingModelConfig(LocalModelConfig):
    """Base config for embedding models."""

    normalize: bool = True


class HuggingFaceEmbeddingModelConfig(HuggingFaceModelConfig):
    """Base config for HuggingFace-based embedding models."""

    normalize: bool = True


class Embedding(Timestamped):
    """Base class for embeddings with vector data.

    Supports both torch.Tensor (for GPU-accelerated comparisons) and np.ndarray.
    Embeddings are kept as torch.Tensor on device by default for efficiency.
    """

    vector: torch.Tensor | np.ndarray

    def __init__(self, vector: torch.Tensor | np.ndarray, timestamp: float | None = None) -> None:
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

    def to_numpy(self) -> np.ndarray:
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


class PatchEmbeddings(Timestamped):
    """Grid of per-patch embeddings for a single image.

    ``vector`` has shape (grid_h, grid_w, dim) — one embedding per image patch,
    laid out in row-major image order. ``source_width``/``source_height`` are
    the pixel size of the image the grid was computed from: the model resizes
    to its own square input, so patch (i, j) covers the source-pixel region
    ``[j * source_width / grid_w, (j + 1) * source_width / grid_w) x
    [i * source_height / grid_h, (i + 1) * source_height / grid_h)``.
    """

    vector: torch.Tensor | np.ndarray
    frame_id: str
    source_width: int
    source_height: int

    def __init__(
        self,
        vector: torch.Tensor | np.ndarray,
        frame_id: str = "",
        ts: float | None = None,
        source_width: int = 0,
        source_height: int = 0,
    ) -> None:
        if vector.ndim != 3:
            raise ValueError(
                f"PatchEmbeddings requires a (grid_h, grid_w, dim) array, got shape {tuple(vector.shape)}"
            )
        self.vector = vector
        self.frame_id = frame_id
        self.source_width = source_width
        self.source_height = source_height
        self.ts = ts if ts else time.time()

    def __repr__(self) -> str:
        return (
            f"PatchEmbeddings(grid={self.grid_shape}, dim={self.dim}, "
            f"frame_id='{self.frame_id}', source={self.source_width}x{self.source_height}, "
            f"ts={self.ts})"
        )

    def patch_index_for_pixel(
        self, u: np.ndarray, v: np.ndarray
    ) -> np.ndarray:
        """Flat patch index for source-image pixel coords (u=x, v=y), vectorized."""
        if not self.source_width or not self.source_height:
            raise ValueError("PatchEmbeddings has no source image size")
        grid_h, grid_w = self.grid_shape
        col = np.clip((u * grid_w) // self.source_width, 0, grid_w - 1).astype(np.int64)
        row = np.clip((v * grid_h) // self.source_height, 0, grid_h - 1).astype(np.int64)
        return row * grid_w + col

    @property
    def grid_shape(self) -> tuple[int, int]:
        """(grid_h, grid_w) — the patch grid layout."""
        return int(self.vector.shape[0]), int(self.vector.shape[1])

    @property
    def dim(self) -> int:
        """Embedding dimension of each patch."""
        return int(self.vector.shape[2])

    def flat(self) -> torch.Tensor | np.ndarray:
        """Patch embeddings flattened to (grid_h * grid_w, dim)."""
        return self.vector.reshape(-1, self.vector.shape[2])

    def to_numpy(self) -> np.ndarray:
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

    def to_cpu(self) -> PatchEmbeddings:
        """Move embeddings to CPU, returning self for chaining."""
        if isinstance(self.vector, torch.Tensor):
            self.vector = self.vector.cpu()
        return self


class EmbeddingModel(Resource, ABC):
    """Abstract base class for embedding models supporting vision and language."""

    device: str

    @overload
    def embed(self, image: Image, /) -> Embedding: ...
    @overload
    def embed(self, *images: Image) -> list[Embedding]: ...
    @abstractmethod
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        """Embed one or more images.

        Returns single Embedding if one image, list if multiple.
        """
        ...

    @overload
    def embed_text(self, text: str, /) -> Embedding: ...
    @overload
    def embed_text(self, *texts: str) -> list[Embedding]: ...
    @abstractmethod
    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        """Embed one or more text strings.

        Returns single Embedding if one text, list if multiple.
        """
        ...

    def compare_one_to_many(self, query: Embedding, candidates: list[Embedding]) -> torch.Tensor:
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

    def compare_many_to_many(
        self, queries: list[Embedding], candidates: list[Embedding]
    ) -> torch.Tensor:
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

    def query(
        self, query_emb: Embedding, candidates: list[Embedding], top_k: int = 5
    ) -> list[tuple[int, float]]:
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

        ...
