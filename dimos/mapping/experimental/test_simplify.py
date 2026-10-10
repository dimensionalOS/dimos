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

import torch

from dimos.mapping.experimental.simplify import EdgeCollapse, PlaneSnap, boundary


def _grid(n: int, noise: float = 0.0) -> tuple[torch.Tensor, torch.Tensor]:
    i, j = torch.meshgrid(torch.arange(n), torch.arange(n), indexing="ij")
    v = torch.stack([i, j, torch.zeros_like(i)], -1).reshape(-1, 3).float()
    v[:, 2] += noise * torch.randn(len(v), generator=torch.Generator().manual_seed(0))
    q = (i[:-1, :-1] * n + j[:-1, :-1]).reshape(-1)
    f = torch.cat([torch.stack([q, q + n, q + 1], 1), torch.stack([q + 1, q + n, q + n + 1], 1)])
    return v, f


def test_flat_grid_collapses_and_keeps_its_border() -> None:
    v, f = _grid(20)
    border = boundary(f, len(v))
    v2, f2 = EdgeCollapse(tol=0.01)(v, f)
    assert len(f2) < len(f) / 4
    assert torch.equal(v2[border], v[border])
    assert v2[f2.unique()][:, 2].abs().max() < 1e-5


def test_noise_blocks_collapse_until_planes_snap() -> None:
    v, f = _grid(20, noise=0.05)
    _, f_raw = EdgeCollapse(tol=0.01)(v, f)
    v_s, f_s = EdgeCollapse(tol=0.01)(*PlaneSnap(tol=0.3, feather=0)(v, f))
    assert len(f_s) < len(f_raw) / 2
    interior = ~boundary(f, len(v))
    assert v_s[f_s.unique()][interior[f_s.unique()]][:, 2].std() < 0.05


def test_planes_face_their_faces() -> None:
    # the two sides of a thin slab must never fit the same plane
    v, f = _grid(10)
    up = PlaneSnap(tol=0.1).assign(v, f)[2]
    down = PlaneSnap(tol=0.1).assign(v, f[:, [0, 2, 1]])[2]
    assert (up[:, 2] > 0.99).all() and (down[:, 2] < -0.99).all()


def test_snap_fades_in_from_the_border() -> None:
    v, f = _grid(20, noise=0.05)
    border = boundary(f, len(v))
    moved = (PlaneSnap(tol=0.3, feather=3)(v, f)[0] - v).norm(dim=1)
    full = (PlaneSnap(tol=0.3, feather=0)(v, f)[0] - v).norm(dim=1)
    i, j = torch.meshgrid(torch.arange(20), torch.arange(20), indexing="ij")
    ring1 = ((i == 1) | (i == 18) | (j == 1) | (j == 18)).reshape(-1) & ~border
    assert moved[border].max() == 0
    assert torch.allclose(moved[ring1], full[ring1] / 3, atol=1e-5)
