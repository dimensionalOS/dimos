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

"""Live mesh of a voxel map: occupancy, blurred, marching cubes on the GPU, emitted per chunk."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np

from dimos.memory.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.shape_msgs.TriangleMesh import TriangleMesh

if TYPE_CHECKING:
    from collections.abc import Iterator

    import torch

    from dimos.mapping.experimental.simplify import Simplifier
    from dimos.memory.type.observation import Observation

C = 32  # chunk edge, voxels
Key = tuple[int, int, int]
Chunk = tuple[Key, np.ndarray, np.ndarray, np.ndarray]
Part = tuple["torch.Tensor", "torch.Tensor", "torch.Tensor"]


PAD = 2  # block overlap: the blur reaches 1 voxel, marching cubes 1 more
P = C + 2 * PAD
MAX_BATCH = 64  # blocks per marching cubes, bounds its GPU buffers
_OFFSETS = np.array([(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)])
_BIAS = 1 << 20


def _code(k: np.ndarray) -> np.ndarray:
    k = k.astype(np.int64) + _BIAS
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def _key(code: int) -> Key:
    m = (1 << 21) - 1
    return ((code >> 42) - _BIAS, ((code >> 21) & m) - _BIAS, (code & m) - _BIAS)


class OccupancyMesher:
    """Meshes voxel centers in padded C^3 blocks; only blocks whose contents changed."""

    def __init__(
        self, voxel_size: float, iso: float, device: str, simplify: Sequence[Simplifier] = ()
    ) -> None:
        import torch
        import warp as wp

        wp.init()  # type: ignore[no-untyped-call]
        self.vox = voxel_size
        self.iso = iso
        self.simplify = simplify
        self.dev = device
        self.mc = wp.MarchingCubes(nx=2, ny=2, nz=2, device=device)
        k = torch.tensor([0.25, 0.5, 0.25], device=device)
        self.kernels = [k.view(1, 1, 3, 1, 1), k.view(1, 1, 1, 3, 1), k.view(1, 1, 1, 1, 3)]
        self.hashes: dict[int, int] = {}

    def mesh(self, points: np.ndarray) -> list[Chunk]:
        """(chunk key, world vertices, normals, faces) for every chunk that changed."""
        ijk = np.floor(points / self.vox).astype(np.int64)
        # every (block, local) a voxel lands in, its own chunk's block and the neighbours' pads
        ck = ijk // C
        blocks, locals_ = [], []
        for off in _OFFSETS:
            bk = ck + off
            local = ijk - bk * C + PAD
            ok = ((local >= 0) & (local < P)).all(1)
            blocks.append(_code(bk[ok]))
            locals_.append(local[ok])
        code = np.concatenate(blocks)
        local = np.concatenate(locals_)
        h = ((local[:, 0] * P + local[:, 1]) * P + local[:, 2]) * 2654435761 % (1 << 31)
        codes, inv = np.unique(code, return_inverse=True)
        sums = np.bincount(inv, weights=h.astype(np.float64), minlength=len(codes))
        hashes = dict(zip(codes.tolist(), sums.tolist(), strict=True))
        changed = sorted(
            c for c in hashes.keys() | self.hashes.keys() if hashes.get(c) != self.hashes.get(c)
        )
        self.hashes = hashes
        sel = np.isin(code, changed)
        code, local = code[sel], local[sel]
        # marching cubes in bounded batches, then the simplifiers once over every batch
        parts = []
        for i in range(0, len(changed), MAX_BATCH):
            batch = np.array(changed[i : i + MAX_BATCH], np.int64)
            m = (code >= batch[0]) & (code <= batch[-1])
            b = np.searchsorted(batch, code[m])
            parts.append(self._surface(len(batch), b, local[m], i))
        return self._finish(np.array(changed, np.int64), parts)

    def _surface(self, B: int, b: np.ndarray, local: np.ndarray, first: int) -> Part:
        """Blocks laid side by side along x, one marching cubes over all of them.

        Vertices in block-local voxels, the faces each block owns, each vertex's block.
        """
        import torch
        import warp as wp

        occ = torch.zeros(B * P, P, P, device=self.dev)
        idx = torch.from_numpy(local + np.stack([b * P, 0 * b, 0 * b], 1)).to(self.dev)
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = 1.0
        vol = occ.view(B, 1, P, P, P)
        for k in self.kernels:
            vol = torch.nn.functional.conv3d(vol, k, padding="same")
        self.mc.resize(nx=B * P, ny=P, nz=P)
        self.mc.surface(wp.from_torch(vol.view(B * P, P, P).contiguous()), self.iso)
        assert self.mc.verts is not None and self.mc.indices is not None
        v = wp.to_torch(self.mc.verts)
        # flip the winding: warp's normals face up the gradient, into the occupied side
        f = wp.to_torch(self.mc.indices).view(-1, 3).long()[:, [0, 2, 1]]
        # a face belongs to the block whose own C^3 holds its centroid; drops pads and seams
        cen = v[f].mean(1)
        cen[:, 0] -= (cen[:, 0] // P) * P
        f = f[((cen >= PAD) & (cen < PAD + C)).all(1)]
        vb = (v[:, 0] // P).long().clamp(0, B - 1)
        v = v.clone()
        v[:, 0] -= vb * P
        return v, f, vb + first

    def _finish(self, changed: np.ndarray, parts: list[Part]) -> list[Chunk]:
        import torch

        if not parts:
            return []
        offs = np.cumsum([0] + [len(v) for v, _, _ in parts[:-1]])
        v = torch.cat([v for v, _, _ in parts])
        f = torch.cat([f + int(o) for (_, f, _), o in zip(parts, offs, strict=True)])
        vb = torch.cat([vb for _, _, vb in parts])
        if self.simplify:
            # block-local metres: small coordinates keep float32 quadrics exact
            m = v * self.vox
            for s in self.simplify:
                m, f = s(m, f)
            v = m / self.vox
        blk = vb[f[:, 0]]
        a, bb, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        n = torch.zeros_like(v).index_add_(
            0, f.reshape(-1), torch.cross(bb - a, c - a, dim=1).repeat_interleave(3, 0)
        )
        normals = torch.nn.functional.normalize(n, dim=1).cpu().numpy()
        keys = np.array([_key(int(c)) for c in changed]).reshape(-1, 3)
        origin = torch.from_numpy(keys * C - PAD).to(v)[vb]
        world = ((v + origin + 0.5) * self.vox).cpu().numpy()
        f_np, blk_np = f.cpu().numpy(), blk.cpu().numpy()
        order = np.argsort(blk_np, kind="stable")
        groups = np.split(f_np[order], np.cumsum(np.bincount(blk_np, minlength=len(keys)))[:-1])
        out: list[Chunk] = []
        for key, g in zip(keys, groups, strict=True):
            used, li = np.unique(g, return_inverse=True)
            f_k = li.reshape(-1, 3).astype(np.uint32)
            out.append(((int(key[0]), int(key[1]), int(key[2])), world[used], normals[used], f_k))
        return out


class LiveMesh(Transformer[PointCloud2, TriangleMesh]):
    """Voxel map clouds in, one TriangleMesh per changed chunk out.

    Each cloud is a whole map; chunks it no longer covers come out empty. A reused
    instance keeps diffing against the last map.
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.08,
        # iso on the [1,2,1]/4 blurred occupancy: a 1-voxel wall peaks at 0.5, a lone voxel at 0.125
        iso: float = 0.2,
        # run in order over the changed chunks, see simplify.py
        simplify: Sequence[Simplifier] = (),
        device: str = "cuda",
    ) -> None:
        self.voxel_size = voxel_size
        self.iso = iso
        self.simplify = simplify
        self.device = device
        self._mesher: OccupancyMesher | None = None

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[TriangleMesh]]:
        if self._mesher is None:
            self._mesher = OccupancyMesher(self.voxel_size, self.iso, self.device, self.simplify)
        for obs in upstream:
            for key, v, n, f in self._mesher.mesh(obs.data.points_f32()):
                yield obs.derive(data=TriangleMesh(v, f, n, key, obs.ts))
