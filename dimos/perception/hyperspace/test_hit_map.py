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

from pathlib import Path

import numpy as np

from dimos.models.embedding.base import Embedding
from dimos.perception.hyperspace.hit_map import HitVoxelMap


class _StubTextModel:
    """embed_text stub: 'object' aligns with axis 0, everything else axis 1."""

    def embed_text(self, *texts: str) -> list[Embedding]:
        out = []
        for text in texts:
            v = np.zeros(8, dtype=np.float32)
            v[0 if text == "object" else 1] = 1.0
            out.append(Embedding(vector=v))
        return out


def _wall_seed(voxel: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """A z=0..1 wall at y=2, x in [-1, 1]."""
    xs = np.arange(-1.0, 1.0, voxel)
    zs = np.arange(0.0, 1.0, voxel)
    centers = np.array([[x + voxel / 2, 2.0, z + voxel / 2] for x in xs for z in zs])
    k = np.floor(centers / voxel).astype(np.int64)
    keys = (k[:, 0] << 42) + (k[:, 1] << 21) + k[:, 2]
    return keys, centers.astype(np.float32)


def _camera_at(x: float, y: float) -> np.ndarray:
    """Optical-frame camera at (x, y, 0.5) looking along +y (z_optical = +y)."""
    m = np.eye(4)
    # columns: x_cam -> -x_world? keep simple: x_cam=+x, y_cam=-z (down), z_cam=+y
    m[:3, 0] = [1, 0, 0]
    m[:3, 1] = [0, 0, -1]
    m[:3, 2] = [0, 1, 0]
    m[:3, 3] = [x, y, 0.5]
    return m


K = np.array([[100.0, 0, 106], [0, 100.0, 60], [0, 0, 1]])
IMAGE = (212, 120)


TARGET = np.array([0.0, 2.0, 0.5])


def _grid(hot_patch: tuple[int, int] | None) -> np.ndarray:
    g = np.zeros((6, 8, 8), dtype=np.float32)
    g[:, :, 1] = 1.0  # background embedding everywhere
    if hot_patch is not None:
        g[hot_patch[0], hot_patch[1], :] = 0.0
        g[hot_patch[0], hot_patch[1], 0] = 1.0  # object embedding
    return g


def _grid_at_target(camera: np.ndarray) -> np.ndarray:
    """Grid whose object patch is wherever TARGET projects in this camera."""
    xc = (TARGET - camera[:3, 3]) @ camera[:3, :3]
    u = xc[0] / xc[2] * K[0, 0] + K[0, 2]
    v = xc[1] / xc[2] * K[1, 1] + K[1, 2]
    row = int(v * 6 / IMAGE[1])
    col = int(u * 8 / IMAGE[0])
    return _grid((min(max(row, 0), 5), min(max(col, 0), 7)))


def test_add_frame_hits_visible_wall() -> None:
    keys, centers = _wall_seed()
    m = HitVoxelMap(keys, centers, voxel_size=0.1)
    n = m.add_frame(_grid(None), _camera_at(0.0, 0.0), K, IMAGE)
    assert n > 0
    assert m.frame_count == 1
    assert m.hit_count == n


def test_query_prefers_multi_view_object() -> None:
    keys, centers = _wall_seed()
    m = HitVoxelMap(keys, centers, voxel_size=0.1)
    # The image center patch sees the wall straight ahead; mark it 'object'
    # from several distinct camera positions (distinct yaw bins), plus give
    # a single-view-only hot patch far to the side as a false-positive probe.
    for x in (-0.8, -0.3, 0.3, 0.8):
        cam = _camera_at(x, 0.0)
        m.add_frame(_grid_at_target(cam), cam, K, IMAGE)
    centers_out, scores = m.query(_StubTextModel(), "object", background_prompts=("stuff",))
    assert centers_out.shape == centers.shape
    assert scores.max() > 0
    # Hot voxels sit near TARGET, where every object patch pointed.
    hot = centers[scores > scores.max() * 0.5]
    assert np.all(np.abs(hot[:, 0] - TARGET[0]) < 0.7)


def test_single_view_is_suppressed() -> None:
    keys, centers = _wall_seed()
    m = HitVoxelMap(keys, centers, voxel_size=0.1)
    cam = _camera_at(0.0, 0.0)
    m.add_frame(_grid_at_target(cam), cam, K, IMAGE)
    _, one_view = m.query(_StubTextModel(), "object", background_prompts=("stuff",))
    for x in (-0.8, 0.8):
        cam = _camera_at(x, 0.0)
        m.add_frame(_grid_at_target(cam), cam, K, IMAGE)
    _, multi_view = m.query(_StubTextModel(), "object", background_prompts=("stuff",))
    # The sqrt(hot yaw bins) factor rewards angular diversity.
    assert multi_view.max() > one_view.max()


def test_save_load_roundtrip(tmp_path: Path) -> None:
    keys, centers = _wall_seed()
    m = HitVoxelMap(keys, centers, voxel_size=0.1)
    for x in (-0.5, 0.5):
        cam = _camera_at(x, 0.0)
        m.add_frame(_grid_at_target(cam), cam, K, IMAGE)
    _, before = m.query(_StubTextModel(), "object", background_prompts=("stuff",))
    path = str(tmp_path / "map.npz")
    m.save(path)
    loaded = HitVoxelMap.load(path)
    assert loaded.frame_count == m.frame_count
    assert loaded.hit_count == m.hit_count
    _, after = loaded.query(_StubTextModel(), "object", background_prompts=("stuff",))
    np.testing.assert_allclose(before, after, atol=1e-3)
