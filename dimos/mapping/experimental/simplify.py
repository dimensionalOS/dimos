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

"""GPU mesh simplifiers: (vertices, faces) -> (vertices, faces), torch tensors.

LiveMesh hands them block-local metres, so every ``tol`` is in metres.

Open boundary vertices never move, so chunks simplified apart still meet. Faces only
ever drop; vertices are never renumbered, so callers can keep per-vertex tags.
"""

from __future__ import annotations

from collections.abc import Callable
import math

import torch
from torch import Tensor

Simplifier = Callable[[Tensor, Tensor], tuple[Tensor, Tensor]]
_INF = torch.iinfo(torch.int64).max


def _sides(f: Tensor, nv: int) -> Tensor:
    """Undirected edge code lo * nv + hi of every face side, (3F,)."""
    e = torch.cat([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]).sort(1).values
    return e[:, 0] * nv + e[:, 1]


def boundary(f: Tensor, nv: int) -> Tensor:
    """Vertices on an open or non-manifold edge."""
    code, cnt = torch.unique(_sides(f, nv), return_counts=True)
    locked = torch.zeros(nv, dtype=torch.bool, device=f.device)
    edge = code[cnt != 2]
    locked[edge // nv] = True
    locked[edge % nv] = True
    return locked


def _normals(v: Tensor, f: Tensor) -> Tensor:
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return torch.cross(b - a, c - a, dim=1)


def _drop_degenerate(f: Tensor) -> Tensor:
    return f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 2] != f[:, 0])]


class EdgeCollapse:
    """Error-bounded quadric edge collapse, in parallel rounds.

    Each round greedily picks cheap edges whose 1-rings are disjoint, so no two
    collapses touch the same face. An edge collapses only while every original
    plane it absorbed stays within ``tol``, no face flips past ``max_angle``,
    and the link condition keeps the mesh manifold.
    """

    def __init__(self, tol: float = 0.02, max_rounds: int = 200, max_angle: float = 60.0) -> None:
        self.tol = tol
        self.max_rounds = int(max_rounds)
        self.min_cos = math.cos(math.radians(max_angle))

    def __call__(self, v: Tensor, f: Tensor) -> tuple[Tensor, Tensor]:
        v, nv, dev = v.clone(), len(v), v.device
        locked = boundary(f, nv)
        n = torch.nn.functional.normalize(_normals(v, f), dim=1)
        plane = torch.cat([n, -(n * v[f[:, 0]]).sum(1, keepdim=True)], 1)
        q = torch.zeros(nv, 4, 4, device=dev).index_add_(
            0, f.reshape(-1), (plane[:, :, None] * plane[:, None, :]).repeat_interleave(3, 0)
        )
        blocked = torch.zeros(0, dtype=torch.int64, device=dev)
        for _ in range(self.max_rounds):
            code, cnt = torch.unique(_sides(f, nv), return_counts=True)
            a, b = code // nv, code % nv
            p, keep_a, cost = self._best(v, q, a, b, locked[a], locked[b])
            ok = (cost <= self.tol**2) & (cnt <= 2) & ~_member(code, blocked)
            # coarse cost buckets, random within: cheap first but many winners per round
            key = (cost / self.tol**2 * 8).floor().clamp_max(8) + torch.rand_like(cost)
            sel = self._independent(f, nv, a, b, key, ok)
            # masks over all edges, no compaction: each sync stalls the queue
            idx = torch.where(sel, torch.arange(len(a), device=dev), -1)
            at_a = torch.full((nv,), -1, device=dev).scatter_reduce_(0, a, idx, "amax")
            at = at_a.clone().scatter_reduce_(0, b, idx, "amax")
            good = sel & self._no_flip(v, f, a, b, p, at) & self._link(code, nv, b, cnt, at_a)
            bad = code[sel & ~good]
            g = good.nonzero().squeeze(1)
            if len(g) == 0 and len(bad) == 0:
                break
            blocked = torch.cat([blocked, bad]).sort().values
            a, b, p, keep_a = a[g], b[g], p[g], keep_a[g]
            keep, gone = torch.where(keep_a, a, b), torch.where(keep_a, b, a)
            v[keep] = p
            q[keep] += q[gone]
            remap = torch.arange(nv, device=dev)
            remap[gone] = keep
            f = _drop_degenerate(remap[f])
        return v, f

    @staticmethod
    def _best(
        v: Tensor, q: Tensor, a: Tensor, b: Tensor, la: Tensor, lb: Tensor
    ) -> tuple[Tensor, Tensor, Tensor]:
        """Cheapest of a, b, midpoint, honouring locks; (position, keeps a, cost)."""
        cands = torch.stack([v[a], v[b], (v[a] + v[b]) / 2])
        h = torch.cat([cands, torch.ones_like(cands[..., :1])], -1)
        qe = q[a] + q[b]
        cost = (h[..., :, None] * qe * h[..., None, :]).sum((-1, -2)).clamp_min(0)
        allowed = torch.stack([~lb, ~la, ~la & ~lb])
        cost = torch.where(allowed, cost, torch.inf)
        cost, k = cost.min(0)
        p = cands.gather(0, k.view(1, -1, 1).expand(1, -1, 3))[0]
        return p, k != 1, cost

    @staticmethod
    def _independent(
        f: Tensor, nv: int, a: Tensor, b: Tensor, key: Tensor, ok: Tensor, passes: int = 3
    ) -> Tensor:
        """Greedy by key: edges cheapest in their 1-ring, then again away from the winners."""
        rank = torch.empty_like(a)
        rank[torch.argsort(key)] = torch.arange(len(a), device=a.device)
        fv = f.reshape(-1)
        sel, cand = torch.zeros_like(ok), ok
        for i in range(passes):
            r = torch.where(cand, rank, _INF)
            rv = torch.full((nv,), _INF, device=f.device)
            rv.scatter_reduce_(0, a, r, "amin").scatter_reduce_(0, b, r, "amin")
            m = torch.full((nv,), _INF, device=f.device)
            m.scatter_reduce_(0, fv, rv[f].amin(1).repeat_interleave(3), "amin")
            win = cand & (m[a] == r) & (m[b] == r)
            sel = sel | win
            if i == passes - 1:
                break
            # drop every edge whose 1-ring touches a winner
            w = torch.zeros(nv, dtype=torch.int64, device=f.device)
            w.scatter_reduce_(0, a, win.long(), "amax").scatter_reduce_(0, b, win.long(), "amax")
            near = torch.zeros(nv, dtype=torch.int64, device=f.device)
            near.scatter_reduce_(0, fv, w[f].amax(1).repeat_interleave(3), "amax")
            cand = cand & (near[a] == 0) & (near[b] == 0)
        return sel

    def _no_flip(self, v: Tensor, f: Tensor, a: Tensor, b: Tensor, p: Tensor, at: Tensor) -> Tensor:
        """Moving a and b to p turns no surviving face past min_cos; ``at`` maps vertex to edge."""
        fs = at[f].amax(1)
        e = fs.clamp_min(0)
        moved = (f == a[e, None]) | (f == b[e, None])
        live = (fs >= 0) & (moved.sum(1) == 1)  # faces on the edge itself vanish
        old = v[f]
        new = torch.where(moved[..., None], p[e, None, :], old)
        n0 = torch.cross(old[:, 1] - old[:, 0], old[:, 2] - old[:, 0], dim=1)
        n1 = torch.cross(new[:, 1] - new[:, 0], new[:, 2] - new[:, 0], dim=1)
        cos = (n0 * n1).sum(1) / (n0.norm(dim=1) * n1.norm(dim=1)).clamp_min(1e-12)
        flip = (live & (cos < self.min_cos)).long()
        return torch.zeros_like(a).scatter_reduce_(0, e, flip, "amax") == 0

    @staticmethod
    def _link(code: Tensor, nv: int, b: Tensor, cnt: Tensor, at_a: Tensor) -> Tensor:
        """Link condition: a and b share exactly the neighbours of their faces."""
        # every neighbour c of a selected edge's a, looked up as edge (b, c)
        x, c = torch.cat([code // nv, code % nv]), torch.cat([code % nv, code // nv])
        e = at_a[x]
        be = b[e.clamp_min(0)]
        look = torch.minimum(be, c) * nv + torch.maximum(be, c)
        pos = torch.searchsorted(code, look).clamp_max(len(code) - 1)
        hit = ((e >= 0) & (code[pos] == look)).long()
        common = torch.zeros_like(cnt).index_add_(0, e.clamp_min(0), hit)
        return common == cnt


class PlaneSnap:
    """Snap vertices inside planar regions exactly onto the region's fitted plane.

    Regions grow across edges whose smoothed normals differ by under ``max_angle``; the
    smoothing averages out marching cubes ripple. Each region's plane is refit on its
    inliers with a shrinking threshold down to ``tol``, so a region that leaked over an
    edge keeps its dominant plane; the faces left over regrow and fit again, ``peels``
    times. Neighbouring planes within ``merge_angle`` and ``tol`` merge. A vertex
    moves when at least ``rim`` of its faces lie on one plane and none on another, so
    edges between planes stay put; the snap fades in over ``feather`` rings from the
    locked border. Run it before EdgeCollapse.
    """

    def __init__(
        self,
        tol: float = 0.05,
        max_angle: float = 45.0,
        smooth: int = 8,
        min_faces: int = 8,
        peels: int = 8,
        merge_angle: float = 5.0,
        rim: float = 0.5,
        feather: int = 6,
    ) -> None:
        self.feather = int(feather)
        self.peels = int(peels)
        self.merge_cos = math.cos(math.radians(merge_angle))
        self.rim = rim
        self.tol = tol
        self.min_cos = math.cos(math.radians(max_angle))
        self.smooth = int(smooth)
        self.min_faces = int(min_faces)

    def __call__(self, v: Tensor, f: Tensor) -> tuple[Tensor, Tensor]:
        if len(f) == 0:
            return v, f
        nv, dev = len(v), v.device
        tag, pm, pn = self.assign(v, f)
        # a vertex moves if its plane faces are all one plane and at least ``rim`` of its faces
        on = tag >= 0
        fv = f.reshape(-1)
        t3 = tag.repeat_interleave(3)
        on3 = on.repeat_interleave(3)
        lo = torch.full((nv,), _INF, device=dev).scatter_reduce_(
            0, fv, torch.where(on3, t3, _INF), "amin"
        )
        hi = torch.full((nv,), -1, device=dev).scatter_reduce_(
            0, fv, torch.where(on3, t3, -1), "amax"
        )
        deg = torch.bincount(fv, minlength=nv)
        hits = torch.bincount(fv[on3], minlength=nv)
        locked = boundary(f, nv)
        move = (lo == hi) & (hi >= 0) & (hits >= self.rim * deg) & ~locked
        # the plane comes from any of the vertex's plane faces
        owner = torch.full((nv,), -1, device=dev)
        owner[fv[on3]] = torch.arange(len(f), device=dev).repeat_interleave(3)[on3]
        o = owner[move]
        off = ((v[move] - pm[o]) * pn[o]).sum(1, keepdim=True)
        # fade in over ``feather`` rings from the locked border, so no crease at chunk seams
        fade = torch.ones(nv, device=dev)
        if self.feather:
            fade = _rings_from(locked, f, self.feather).float() / self.feather
        v = v.clone()
        v[move] -= off * pn[o] * fade[move, None]
        return v, f

    def assign(self, v: Tensor, f: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        """Per face: plane tag (negative if none), a point on its plane, the plane normal."""
        nv, nf, dev = len(v), len(f), v.device
        raw = _normals(v, f)
        n = torch.nn.functional.normalize(raw, dim=1)
        for _ in range(self.smooth):
            vn = torch.zeros(nv, 3, device=dev).index_add_(
                0, f.reshape(-1), n.repeat_interleave(3, 0)
            )
            n = torch.nn.functional.normalize(vn[f].sum(1), dim=1)
        f1, f2 = _face_pairs(f, nv)
        join = (n[f1] * n[f2]).sum(1) > self.min_cos
        w = raw.norm(dim=1) / 2
        c = v[f].mean(1)
        # per face: its plane (point, normal) and region tag; -1 - face id while unassigned
        pm, pn = torch.zeros_like(c), torch.zeros_like(c)
        tag = -1 - torch.arange(nf, device=dev)
        active = torch.ones(nf, dtype=torch.bool, device=dev)
        for peel in range(self.peels):
            # regrow regions over the faces no plane has claimed, fit, keep the inliers
            j = join & active[f1] & active[f2]
            root = _components(nf, f1[j], f2[j])
            size = torch.bincount(root, weights=active.float(), minlength=nf)
            # only regions that can reach min_faces get a plane; the rest share label nc
            label, nc = _compact(root, size >= self.min_faces)
            inl = active & (label < nc)
            for thr in (4 * self.tol, 2 * self.tol, self.tol):
                mean, normal = _fit_planes(label, nc + 1, w * inl, c, n)
                mean, normal = mean[label], normal[label]
                d = ((v[f] - mean[:, None]) * normal[:, None]).sum(-1).abs().amax(1)
                inl = active & (label < nc) & (d <= thr) & ((n * normal).sum(1) > self.min_cos)
            inl &= (torch.bincount(label, weights=inl.float(), minlength=nc + 1) >= self.min_faces)[
                label
            ]
            pm = torch.where(inl[:, None], mean, pm)
            pn = torch.where(inl[:, None], normal, pn)
            tag = torch.where(inl, peel * nf + label, tag)
            active &= ~inl
        # neighbouring planes that agree become one plane, refit over all their faces
        on = tag >= 0
        same = on[f1] & on[f2]
        a, b = f1[same], f2[same]
        coplanar = (tag[a] == tag[b]) | (
            ((pn[a] * pn[b]).sum(1) > self.merge_cos)
            & (((pm[b] - pm[a]) * pn[a]).sum(1).abs() <= self.tol)
        )
        root = _components(nf, a[coplanar], b[coplanar])
        label, nc = _compact(root, torch.bincount(root, weights=on.float(), minlength=nf) > 0)
        mean, normal = _fit_planes(label, nc + 1, w * on, c, n)
        pm, pn = mean[label], normal[label]
        # the merged plane must still hold its faces, facing its way
        d = ((v[f] - pm[:, None]) * pn[:, None]).sum(-1).abs().amax(1)
        on &= (d <= self.tol) & ((n * pn).sum(1) > self.min_cos)
        tag = torch.where(on, label, -1 - torch.arange(nf, device=dev))
        return tag, pm, pn


def _member(x: Tensor, sorted_set: Tensor) -> Tensor:
    if len(sorted_set) == 0:
        return torch.zeros_like(x, dtype=torch.bool)
    i = torch.searchsorted(sorted_set, x).clamp_max(len(sorted_set) - 1)
    return sorted_set[i] == x


def _rings_from(seed: Tensor, f: Tensor, k: int) -> Tensor:
    """Edge hops from the nearest seed vertex, capped at k."""
    hops = torch.where(seed, 0, k).to(f.device)
    e = torch.cat([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    for _ in range(k):
        hops = hops.scatter_reduce(0, e[:, 0], hops[e[:, 1]] + 1, "amin")
        hops = hops.scatter_reduce(0, e[:, 1], hops[e[:, 0]] + 1, "amin")
    return hops


def _face_pairs(f: Tensor, nv: int) -> tuple[Tensor, Tensor]:
    """The two faces of every manifold edge."""
    _, inv, cnt = torch.unique(_sides(f, nv), return_inverse=True, return_counts=True)
    face = torch.arange(len(f), device=f.device).repeat(3)
    order = torch.argsort(inv, stable=True)
    si, sf = inv[order], face[order]
    pair = (si[1:] == si[:-1]) & (cnt[si[1:]] == 2)
    return sf[:-1][pair], sf[1:][pair]


def _components(n: int, a: Tensor, b: Tensor) -> Tensor:
    """Connected components over n nodes joined by edges a-b: each node's smallest node."""
    label = torch.arange(n, device=a.device)
    while True:
        # hook both nodes and their roots onto the smaller label, then jump pointers
        la, lb = label[a], label[b]
        lo = torch.minimum(la, lb)
        new = label.clone()
        for i in (a, b, la, lb):
            new.scatter_reduce_(0, i, lo, "amin")
        new = new[new][new]
        if torch.equal(new, label):
            return label
        label = new


def _compact(label: Tensor, keep: Tensor) -> tuple[Tensor, int]:
    """Renumber the labels flagged in ``keep`` (indexed by label) 0..k-1, the rest k."""
    new = torch.cumsum(keep, 0) - 1
    k = int(new[-1]) + 1
    return torch.where(keep[label], new[label], k), k


def _fit_planes(label: Tensor, nc: int, w: Tensor, c: Tensor, n: Tensor) -> tuple[Tensor, Tensor]:
    """Weighted least-squares plane per label: (point on plane, unit normal facing like n)."""
    dev = c.device
    sw = torch.zeros(nc, device=dev).index_add_(0, label, w)
    mean = torch.zeros(nc, 3, device=dev).index_add_(0, label, w[:, None] * c)
    mean = mean / sw.clamp_min(1e-12)[:, None]
    d = c - mean[label]
    cov = torch.zeros(nc, 3, 3, device=dev).index_add_(
        0, label, w[:, None, None] * d[:, :, None] * d[:, None, :]
    )
    normal = torch.linalg.eigh(cov).eigenvectors[:, :, 0]
    facing = torch.zeros(nc, 3, device=dev).index_add_(0, label, w[:, None] * n)
    return mean, normal * torch.where((normal * facing).sum(1) < 0, -1.0, 1.0)[:, None]


SIMPLIFIERS: dict[str, Callable[..., Simplifier]] = {
    "planes": PlaneSnap,
    "collapse": EdgeCollapse,
}


def parse_chain(spec: str, tol: float | None = None) -> list[Simplifier]:
    """``"planes:0.04:max_angle=45,collapse"`` -> simplifiers in order.

    The first field after the name is ``tol`` in metres, else the ``tol`` given, else
    the simplifier's own default; the rest are keyword arguments.
    """
    out = []
    for item in filter(None, spec.split(",")):
        name, *fields = item.split(":")
        kw: dict[str, float] = {k: float(x) for k, x in (f.split("=") for f in fields if "=" in f)}
        pos = [f for f in fields if "=" not in f]
        if pos:
            kw["tol"] = float(pos[0])
        elif tol is not None:
            kw["tol"] = tol
        out.append(SIMPLIFIERS[name](**kw))
    return out
