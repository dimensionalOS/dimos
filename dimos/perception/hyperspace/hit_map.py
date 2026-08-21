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

"""Map-visibility hit store over a lidar premap, with multi-view query scoring.

Instead of aggregating embeddings per voxel (mean/median/medoid — which wash
out objects that are only clearly seen in a handful of views), this keeps full
fidelity: per-frame patch-embedding grids plus compact hit references
``(voxel, frame, patch, yaw)``. Geometry comes from the premap: each camera
frame z-buffers the seed voxels from its pose (no depth image), and every
visible voxel takes a hit from the patch covering its pixels.

Query scoring (benchmarked against a hand-verified trash-can ground truth on
the Alfred recording; see the LivelyFowl report):

    score(voxel) = LSE_t(hit scores) * sqrt(#yaw bins with hot evidence)

where ``LSE_t(s) = t * log(mean(exp(s / t)))`` is softmax pooling over the
voxel's per-hit contrasted scores and the yaw-bin factor demands the evidence
come from several distinct viewing directions — single-viewpoint false
positives (screens, wall art) rarely stay hot as the camera moves around an
object, real objects do.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

import numpy as np

from dimos.perception.hyperspace.query import DEFAULT_BACKGROUND_PROMPTS

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.models.embedding.base import Embedding


class TextEmbedder(Protocol):
    """The slice of EmbeddingModel that querying needs (structural, easy to stub)."""

    def embed_text(self, *texts: str) -> Embedding | list[Embedding]: ...


#: Softmax-pooling temperature over per-hit contrasted scores.
LSE_TEMPERATURE = 0.02
#: A per-(voxel, frame) max score above this counts as hot evidence.
HOT_SCORE_THRESHOLD = 0.04
#: Yaw is quantized into this many bins for the view-diversity factor.
N_YAW_BINS = 8


class HitVoxelMap:
    """Frame grids + per-voxel hit references over a fixed seed voxel set.

    The build is promptless: only embeddings and geometry are stored; text
    enters at query time. Storage is O(frames * grid + hits), far below
    storing per-voxel embedding lists at equal fidelity.
    """

    def __init__(
        self,
        seed_keys: NDArray[np.int64],
        seed_centers: NDArray[np.floating],
        voxel_size: float,
        zbuffer_size: tuple[int, int] = (212, 120),
        max_range_m: float = 25.0,
        min_range_m: float = 0.3,
    ) -> None:
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        order = np.argsort(seed_keys)
        self.keys: NDArray[np.int64] = seed_keys[order]
        self.centers: NDArray[np.float32] = seed_centers[order].astype(np.float32)
        self.voxel_size = voxel_size
        self.zbuffer_size = zbuffer_size
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m
        self._grids: list[NDArray[np.float16]] = []
        self._grid_shape: tuple[int, int] | None = None
        self._hit_voxel: list[NDArray[np.int32]] = []
        self._hit_frame: list[NDArray[np.int32]] = []
        self._hit_patch: list[NDArray[np.int16]] = []
        self._hit_yaw: list[NDArray[np.float16]] = []

    @property
    def frame_count(self) -> int:
        return len(self._grids)

    @property
    def hit_count(self) -> int:
        return int(sum(len(v) for v in self._hit_voxel))

    def add_frame(
        self,
        patch_grid: NDArray[np.floating],
        world_from_camera: NDArray[np.floating],
        intrinsics: NDArray[np.floating],
        image_size: tuple[int, int],
    ) -> int:
        """Add one camera frame; returns the number of new hits.

        ``patch_grid`` is the (gh, gw, dim) patch-embedding grid of the frame,
        ``world_from_camera`` the 4x4 optical-frame pose, ``intrinsics`` the
        3x3 (or flat 9) camera matrix for ``image_size`` = (width, height).
        """
        grid = np.asarray(patch_grid)
        if grid.ndim != 3:
            raise ValueError(f"patch_grid must be (gh, gw, dim), got {grid.shape}")
        if self._grid_shape is None:
            self._grid_shape = (grid.shape[0], grid.shape[1])
        elif self._grid_shape != (grid.shape[0], grid.shape[1]):
            raise ValueError("all patch grids must share one shape")
        gh, gw = self._grid_shape
        zw, zh = self.zbuffer_size
        k = np.asarray(intrinsics, dtype=np.float64).reshape(3, 3)
        width, height = image_size
        sx, sy = zw / width, zh / height
        fx, fy = k[0, 0] * sx, k[1, 1] * sy
        cx, cy = k[0, 2] * sx, k[1, 2] * sy

        m = np.asarray(world_from_camera, dtype=np.float64)
        rot, t = m[:3, :3], m[:3, 3]
        xc = (self.centers - t) @ rot
        z = xc[:, 2]
        vis = (z > self.min_range_m) & (z < self.max_range_m)
        safe_z = np.where(vis, z, 1.0)
        u = (xc[:, 0] / safe_z * fx + cx).astype(np.int32)
        v = (xc[:, 1] / safe_z * fy + cy).astype(np.int32)
        max_splat = 20
        vis &= (u >= -max_splat) & (u < zw + max_splat) & (v >= -max_splat) & (v < zh + max_splat)
        idx = np.flatnonzero(vis)
        if not len(idx):
            self._grids.append(grid.astype(np.float16))
            return 0
        idx = idx[np.argsort(-z[idx])]  # far -> near so later (nearer) writes win
        size = np.clip(np.round(fx * 1.2 * self.voxel_size / z[idx]).astype(np.int32), 1, max_splat)
        yy, xx = v[idx] - size // 2, u[idx] - size // 2
        zbuf = np.full((zh, zw), -1, dtype=np.int64)
        for dy in range(max_splat):
            for dx in range(max_splat):
                covers = (
                    (size > max(dy, dx))
                    & (yy + dy >= 0)
                    & (yy + dy < zh)
                    & (xx + dx >= 0)
                    & (xx + dx < zw)
                )
                zbuf[yy[covers] + dy, xx[covers] + dx] = idx[covers]

        px_row = (np.arange(zh) * gh // zh).clip(0, gh - 1)
        px_col = (np.arange(zw) * gw // zw).clip(0, gw - 1)
        px_patch = px_row[:, None] * gw + px_col[None, :]
        seen = zbuf >= 0
        pair = np.unique(zbuf[seen] * (gh * gw) + px_patch[seen])
        vox = (pair // (gh * gw)).astype(np.int32)
        patch = (pair % (gh * gw)).astype(np.int16)
        delta = self.centers[vox] - t.astype(np.float32)
        yaw = np.arctan2(delta[:, 1], delta[:, 0]).astype(np.float16)

        frame_id = len(self._grids)
        self._grids.append(grid.astype(np.float16))
        self._hit_voxel.append(vox)
        self._hit_frame.append(np.full(len(vox), frame_id, dtype=np.int32))
        self._hit_patch.append(patch)
        self._hit_yaw.append(yaw)
        return len(vox)

    def query(
        self,
        text_model: TextEmbedder,
        query: str,
        background_prompts: tuple[str, ...] | list[str] = DEFAULT_BACKGROUND_PROMPTS,
        lse_temperature: float = LSE_TEMPERATURE,
        hot_threshold: float = HOT_SCORE_THRESHOLD,
    ) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """(centers (N, 3), scores (N,)) of every seed voxel for a text query."""
        if not self._grids:
            return self.centers, np.zeros(len(self.centers), dtype=np.float32)
        prompts = [query, *background_prompts]
        embeddings = text_model.embed_text(*prompts)
        if not isinstance(embeddings, list):
            embeddings = [embeddings]
        qv = embeddings[0].to_numpy().astype(np.float32)
        distinct = [
            e.to_numpy().astype(np.float32) for e in embeddings[1:] if embeddings[0] @ e < 0.85
        ]

        grids = np.stack(self._grids).astype(np.float32)  # (F, gh, gw, dim)
        n_frames, gh, gw, _ = grids.shape
        flat = grids.reshape(n_frames * gh * gw, -1)
        scores_flat = flat @ qv
        if distinct:
            scores_flat = scores_flat - (flat @ np.stack(distinct).T).max(axis=1)

        h_vox = np.concatenate(self._hit_voxel).astype(np.int64)
        h_frame = np.concatenate(self._hit_frame).astype(np.int64)
        h_patch = np.concatenate(self._hit_patch).astype(np.int64)
        h_yaw = np.concatenate(self._hit_yaw).astype(np.float32)
        hit_scores = scores_flat[h_frame * (gh * gw) + h_patch]

        n_vox = len(self.centers)
        out = np.zeros(n_vox, dtype=np.float32)

        # softmax pooling (LSE) over each voxel's hit scores
        order = np.lexsort((hit_scores, h_vox))
        sv, ss = h_vox[order], hit_scores[order]
        starts = np.flatnonzero(np.diff(sv, prepend=-1))
        counts = np.diff(np.append(starts, len(sv)))
        t = lse_temperature
        lse = np.array(
            [
                t * np.log(np.exp(ss[s : s + c] / t).mean())
                for s, c in zip(starts, counts, strict=True)
            ],
            dtype=np.float32,
        )

        # view-diversity factor: distinct yaw bins whose per-(voxel, frame)
        # max score is hot
        order2 = np.lexsort((hit_scores, h_frame, h_vox))
        gv, gf, gs = h_vox[order2], h_frame[order2], hit_scores[order2]
        gy = h_yaw[order2]
        vf = gv * (n_frames + 1) + gf
        ends = np.flatnonzero(np.diff(vf, append=np.iinfo(np.int64).max))
        frame_vox, frame_max, frame_yaw = gv[ends], gs[ends], gy[ends]
        yaw_bin = ((frame_yaw + np.pi) / (2 * np.pi) * N_YAW_BINS).astype(np.int64) % N_YAW_BINS
        hot = frame_max > hot_threshold
        hot_bins = np.bincount(
            np.unique(frame_vox[hot] * N_YAW_BINS + yaw_bin[hot]) // N_YAW_BINS,
            minlength=n_vox,
        ).astype(np.float32)

        out[sv[starts]] = lse
        return self.centers, out * np.sqrt(hot_bins)

    def save(self, path: str) -> None:
        """Persist to an uncompressed npz."""
        if self._grid_shape is None:
            raise ValueError("cannot save an empty map")
        np.savez(
            path,
            keys=self.keys,
            centers=self.centers,
            voxel_size=np.float64(self.voxel_size),
            zbuffer_size=np.asarray(self.zbuffer_size, dtype=np.int64),
            max_range_m=np.float64(self.max_range_m),
            min_range_m=np.float64(self.min_range_m),
            grids=np.stack(self._grids),
            hit_voxel=np.concatenate(self._hit_voxel) if self._hit_voxel else np.empty(0, np.int32),
            hit_frame=np.concatenate(self._hit_frame) if self._hit_frame else np.empty(0, np.int32),
            hit_patch=np.concatenate(self._hit_patch) if self._hit_patch else np.empty(0, np.int16),
            hit_yaw=np.concatenate(self._hit_yaw) if self._hit_yaw else np.empty(0, np.float16),
        )

    @classmethod
    def load(cls, path: str) -> HitVoxelMap:
        data = np.load(path)
        out = cls(
            seed_keys=data["keys"],
            seed_centers=data["centers"],
            voxel_size=float(data["voxel_size"]),
            zbuffer_size=(int(data["zbuffer_size"][0]), int(data["zbuffer_size"][1])),
            max_range_m=float(data["max_range_m"]),
            min_range_m=float(data["min_range_m"]),
        )
        grids = data["grids"]
        out._grids = list(grids)
        out._grid_shape = (grids.shape[1], grids.shape[2])
        if len(data["hit_voxel"]):
            out._hit_voxel = [data["hit_voxel"]]
            out._hit_frame = [data["hit_frame"]]
            out._hit_patch = [data["hit_patch"]]
            out._hit_yaw = [data["hit_yaw"]]
        return out
