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

"""GPU mesh of a voxel map: blurred occupancy, marching cubes, one mesh per chunk."""

from __future__ import annotations

from collections.abc import Sequence
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.shape_msgs.TriangleMesh import TriangleMesh
from dimos.visualization.rerun.bridge import RerunEntry

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


def _decode(code: np.ndarray) -> np.ndarray:
    m = (1 << 21) - 1
    return np.stack([(code >> 42) - _BIAS, ((code >> 21) & m) - _BIAS, (code & m) - _BIAS], 1)


def _place(ijk: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Every (block code, block-local voxel) a voxel lands in: its own block and its neighbours' pads."""
    blocks, locals_ = [], []
    for off in _OFFSETS:
        bk = ijk // C + off
        local = ijk - bk * C + PAD
        ok = ((local >= 0) & (local < P)).all(1)
        blocks.append(_code(bk[ok]))
        locals_.append(local[ok])
    return np.concatenate(blocks), np.concatenate(locals_)


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
        code, local = _place(np.floor(points / self.vox).astype(np.int64))
        h = ((local[:, 0] * P + local[:, 1]) * P + local[:, 2]) * 2654435761 % (1 << 31)
        codes, inv = np.unique(code, return_inverse=True)
        sums = np.bincount(inv, weights=h.astype(np.float64), minlength=len(codes))
        hashes = dict(zip(codes.tolist(), sums.tolist(), strict=True))
        changed = sorted(
            c for c in hashes.keys() | self.hashes.keys() if hashes.get(c) != self.hashes.get(c)
        )
        self.hashes = hashes
        return self._mesh_codes(code, local, np.array(changed, np.int64))

    def mesh_chunks(self, points: np.ndarray, chunks: np.ndarray) -> list[Chunk]:
        """Mesh just these chunk codes from the voxel map ``points``."""
        ijk = np.floor(points / self.vox).astype(np.int64)
        # only voxels in or next to the chunks can reach them
        near = np.unique(np.concatenate([_code(_decode(chunks) + off) for off in _OFFSETS]))
        code, local = _place(ijk[np.isin(_code(ijk // C), near)])
        return self._mesh_codes(code, local, np.sort(chunks))

    def _mesh_codes(self, code: np.ndarray, local: np.ndarray, chunks: np.ndarray) -> list[Chunk]:
        sel = np.isin(code, chunks)
        code, local = code[sel], local[sel]
        # marching cubes in bounded batches, then the simplifiers once over every batch
        parts = []
        for i in range(0, len(chunks), MAX_BATCH):
            batch = chunks[i : i + MAX_BATCH]
            m = (code >= batch[0]) & (code <= batch[-1])
            b = np.searchsorted(batch, code[m])
            parts.append(self._surface(len(batch), b, local[m], i))
        return self._finish(chunks, parts)

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


class Mesh(Transformer[PointCloud2, TriangleMesh]):
    """Voxel map clouds in, one TriangleMesh per changed chunk out.

    Each cloud is a whole map; chunks it no longer covers come out empty. A reused
    instance keeps diffing against the last map.
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.05,
        # iso on the [1,2,1]/4 blurred occupancy, where a 1-voxel wall peaks at 0.5
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


class MeshModuleConfig(ModuleConfig):
    voxel_size: float = 0.05
    iso: float = 0.2
    # simplifiers in order, see simplify.parse_chain; empty keeps the raw mesh
    # chain: str = "planes,collapse"
    chain: str = ""
    # only points in this height band are meshed, e.g. one storey
    z_band: tuple[float, float] | None = None
    # a chunk is remeshed once its net voxel change since its last mesh reaches
    # min_fraction of its voxels, and at least min_change
    min_change: int = 20
    min_fraction: float = 0.1
    device: str = "cuda"


class ChunkQueue:
    """The voxel map as regions, and which chunks changed enough to remesh.

    Each chunk keeps the set of voxels changed since it was last taken, so a voxel that
    flips back cancels out. A chunk is due once that set reaches ``min_fraction`` of its
    voxels and at least ``min_change``.
    """

    def __init__(self, voxel_size: float, min_change: int, min_fraction: float) -> None:
        self.vox = voxel_size
        self.min_change = min_change
        self.min_fraction = min_fraction
        self._regions: dict[int, np.ndarray] = {}
        self._codes: dict[int, np.ndarray] = {}
        # chunk code -> (block-local voxels changed since taken, last change time)
        self._pending: dict[int, tuple[set[int], float]] = {}
        # chunk code -> voxels it holds
        self._size: dict[int, int] = {}

    def update(self, seq: int, points: np.ndarray, now: float) -> bool:
        """Replace region ``seq``; whether any chunk it touched is now due."""
        codes = np.unique(_code(np.floor(points / self.vox)))
        changed = np.setxor1d(self._codes.get(seq, codes[:0]), codes, assume_unique=True)
        if len(points):
            self._regions[seq], self._codes[seq] = points, codes
        else:
            self._regions.pop(seq, None)
            self._codes.pop(seq, None)
        ijk = _decode(changed)
        own, inv = np.unique(_code(ijk // C), return_inverse=True)
        grow = np.bincount(inv, weights=np.where(np.isin(changed, codes), 1, -1))
        for c, d in zip(own.tolist(), grow.tolist(), strict=True):
            self._size[c] = self._size.get(c, 0) + int(d)
        blocks, local = _place(ijk)
        packed = (local[:, 0] * P + local[:, 1]) * P + local[:, 2]
        touched = np.unique(blocks).tolist()
        for c in touched:
            vox = self._pending.get(c, (set(), 0.0))[0]
            vox ^= set(packed[blocks == c].tolist())
            self._pending[c] = (vox, now)
        return any(self._due(c) for c in touched)

    def take(self, n: int = MAX_BATCH) -> tuple[list[int], bool]:
        """Up to n due chunks, most recently changed first, and whether more are due."""
        due = [c for c in self._pending if self._due(c)]
        due.sort(key=lambda c: self._pending[c][1], reverse=True)
        for c in due[:n]:
            del self._pending[c]
        return due[:n], len(due) > n

    def points(self) -> np.ndarray:
        parts = list(self._regions.values())
        return np.concatenate(parts) if parts else np.zeros((0, 3), np.float32)

    def _due(self, c: int) -> bool:
        n = len(self._pending[c][0])
        return n >= self.min_change and n >= self.min_fraction * self._size.get(c, 0)


class MeshModule(Module):
    """Meshes the voxel map that ``map_regions`` streams, chunks that changed most
    recently first, a batch at a time, always from the current map."""

    config: MeshModuleConfig

    map_regions: In[PointCloud2]
    mesh: Out[TriangleMesh]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        cfg = self.config
        self._queue = ChunkQueue(cfg.voxel_size, cfg.min_change, cfg.min_fraction)
        self._lock = threading.Lock()
        self._dirty = threading.Event()
        self._stopping = False
        self._worker = threading.Thread(target=self._run, name="mesh", daemon=True)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.map_regions.subscribe(self._on_region)))
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stopping = True
        self._dirty.set()
        super().stop()

    def _on_region(self, msg: PointCloud2) -> None:
        with self._lock:
            if self._queue.update(msg.seq, msg.points_f32(), time.time()):
                self._dirty.set()

    def _run(self) -> None:
        from dimos.mapping.experimental.simplify import parse_chain

        cfg = self.config
        mesher = OccupancyMesher(cfg.voxel_size, cfg.iso, cfg.device, parse_chain(cfg.chain))
        while True:
            self._dirty.wait()
            if self._stopping:
                return
            self._dirty.clear()
            with self._lock:
                chunks, more = self._queue.take()
                points = self._queue.points()
            if more:
                self._dirty.set()
            if not chunks:
                continue
            if cfg.z_band is not None:
                lo, hi = cfg.z_band
                points = points[(points[:, 2] >= lo) & (points[:, 2] < hi)]
            ts = time.time()
            for key, v, n, f in mesher.mesh_chunks(points, np.array(chunks, np.int64)):
                self.mesh.publish(TriangleMesh(v, f, n, key, ts))


MESH_ENTITY = "world/mesh"


class MeshColours:
    """Rerun entries for each mesh chunk, turbo by height over the mesh's own range.

    Once the range moves ``recolour_m``, every chunk is recoloured (colours only).
    ``z_range`` pins it. Works as a keyed rerun bridge renderer and in replays.
    """

    keyed_by_seq = True

    def __init__(
        self,
        alpha: float = 0.4,
        z_range: tuple[float, float] | None = None,
        recolour_m: float = 0.5,
        static: bool = True,
    ) -> None:
        self.alpha = alpha
        self.z_range = z_range
        self.recolour_m = recolour_m
        self.static = static
        self._heights: dict[Key, np.ndarray] = {}
        self._range: tuple[float, float] | None = None

    def __call__(self, msg: TriangleMesh) -> list[RerunEntry]:
        out = [RerunEntry(_chunk_path(msg.key), msg.to_rerun(), self.static)]
        if len(msg.faces) == 0:
            self._heights.pop(msg.key, None)
            return out
        self._heights[msg.key] = msg.vertices[:, 2].copy()
        lo, hi = self.z_range or np.percentile(
            np.concatenate(list(self._heights.values())), [2, 98]
        )
        keys = [msg.key]
        if (
            self._range is None
            or max(abs(lo - self._range[0]), abs(hi - self._range[1])) > self.recolour_m
        ):
            self._range = (float(lo), float(hi))
            keys = list(self._heights)
        return out + [
            RerunEntry(_chunk_path(k), self._colours(self._heights[k]), self.static) for k in keys
        ]

    def _colours(self, z: np.ndarray) -> Any:
        import matplotlib
        import rerun as rr

        assert self._range is not None
        lo, hi = self._range
        t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
        colors = (matplotlib.colormaps["turbo"](t)[:, :3] * 255).astype(np.uint8)
        alpha = int(self.alpha * 255)
        return rr.Mesh3D.from_fields(vertex_colors=colors, albedo_factor=[255, 255, 255, alpha])


def _chunk_path(key: Key) -> str:
    return f"{MESH_ENTITY}/{key[0]}_{key[1]}_{key[2]}"
