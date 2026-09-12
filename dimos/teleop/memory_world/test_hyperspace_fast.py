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

"""The vectorized query must agree with Hyperspace's reference functions voxel for voxel."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.mapping.hyperspace import patches as hs
from dimos.teleop.memory_world.hyperspace_fast import (
    Frames,
    Patches,
    Rasterized,
    combine,
    normalize_scores,
    pack_keys,
    pool,
    rasterize,
    unpack_keys,
)


def _pose(yaw: float, position: tuple[float, float, float]) -> np.ndarray:
    """A camera at *position* looking along the world XY plane at *yaw* (optical: z forward, y down)."""
    c, s = np.cos(yaw), np.sin(yaw)
    forward = np.array([c, s, 0.0])
    down = np.array([0.0, 0.0, -1.0])
    right = np.cross(down, forward)
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = right, down, forward
    pose[:3, 3] = position
    return pose


def _scene(rng: np.random.Generator, n_frames: int = 4, hot_per_frame: int = 6):
    """Random keyframes with random hot patches, as both the reference and the batched inputs."""
    intrinsics = hs.Intrinsics(width=320, height=240, fx=200.0, fy=200.0, cx=160.0, cy=120.0)
    rows, cols = 6, 8
    keyframes, poses = [], []
    for i in range(n_frames):
        depth = rng.uniform(0.8, 6.0, rows * cols).astype(np.float32)
        depth[rng.integers(0, rows * cols)] = 0.0  # one hole, as real depth has
        keyframes.append(
            hs.Keyframe(
                id=100 + i,
                camera_frame="cam",
                ts=1000.0 + i,
                rows=rows,
                cols=cols,
                intrinsics=intrinsics,
                patch_depth=depth,
            )
        )
        poses.append(_pose(rng.uniform(-np.pi, np.pi), tuple(rng.uniform(-3, 3, 3))))
    hot: list[hs.HotPatch] = []
    frame_idx, cell, depth_of, score = [], [], [], []
    for i, keyframe in enumerate(keyframes):
        for patch in rng.choice(rows * cols, hot_per_frame, replace=False):
            s = float(rng.uniform(0.03, 0.4))
            hot.append(hs.HotPatch(keyframe=keyframe, patch=int(patch), score=s))
            frame_idx.append(i)
            cell.append(int(patch))
            depth_of.append(float(keyframe.patch_depth[patch]))
            score.append(s)
    frames = Frames(
        ids=np.asarray([k.id for k in keyframes]),
        poses=np.stack(poses),
        width=np.full(n_frames, 320.0),
        height=np.full(n_frames, 240.0),
        fx=np.full(n_frames, 200.0),
        fy=np.full(n_frames, 200.0),
        cx=np.full(n_frames, 160.0),
        cy=np.full(n_frames, 120.0),
        rows=np.full(n_frames, rows),
        cols=np.full(n_frames, cols),
        camera_frame=["cam"] * n_frames,
        ts=np.asarray([k.ts for k in keyframes]),
    )
    patches = Patches(
        frame=np.asarray(frame_idx),
        cell=np.asarray(cell),
        depth=np.asarray(depth_of),
        score=np.asarray(score),
    )
    return keyframes, poses, hot, frames, patches


def _reference(hot, poses, keyframes, voxel_size, config):
    """Hyperspace's own rasterize + pool, voxel by voxel."""
    pose_of = {k.id: p for k, p in zip(keyframes, poses, strict=True)}
    evidence: dict[tuple[int, int, int], list[tuple[int, float, int]]] = {}
    rows = []
    for h in hot:
        for index, yaw_bin in hs.rasterize_pyramid(h, pose_of[h.keyframe.id], voxel_size, config):
            evidence.setdefault(index, []).append((h.keyframe.id, h.score, yaw_bin))
            rows.append((index, h.keyframe.id, h.score, yaw_bin))
    pooled = {index: hs.pool(hits, config) for index, hits in evidence.items()}
    return rows, pooled


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_rasterize_matches_reference(seed: int) -> None:
    rng = np.random.default_rng(seed)
    keyframes, poses, hot, frames, patches = _scene(rng)
    config = hs.QueryConfig()
    voxel_size = 0.1

    rows, _ = _reference(hot, poses, keyframes, voxel_size, config)
    ours = rasterize(frames, patches, voxel_size, config)

    expected = sorted((tuple(i), k, round(s, 9), b) for i, k, s, b in rows)
    got = sorted(
        (tuple(int(v) for v in i), int(k), round(float(s), 9), int(b))
        for i, k, s, b in zip(ours.index, ours.frame_id, ours.score, ours.yaw_bin, strict=True)
    )
    assert len(got) > 100, "the random scene should cover plenty of voxels"
    assert got == expected


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_pool_matches_reference(seed: int) -> None:
    rng = np.random.default_rng(seed)
    keyframes, poses, hot, frames, patches = _scene(rng)
    config = hs.QueryConfig()
    voxel_size = 0.1

    _, pooled_ref = _reference(hot, poses, keyframes, voxel_size, config)
    ours = pool(rasterize(frames, patches, voxel_size, config), config)

    got = {tuple(int(v) for v in i): float(s) for i, s in zip(ours.index, ours.score, strict=True)}
    assert got.keys() == pooled_ref.keys()
    for index, score in pooled_ref.items():
        assert got[index] == pytest.approx(score, rel=1e-9, abs=1e-12)

    # And the normalization: same rank-based scale as hs.normalize.
    ref_norm = dict(hs.normalize(list(pooled_ref.items()), config))
    norm = normalize_scores(ours.score, config.normalize_percentile)
    for index, value in zip(ours.index, norm, strict=True):
        assert value == pytest.approx(ref_norm[tuple(int(v) for v in index)], rel=1e-9)


def test_pool_max_per_frame_then_lse_then_yaw_bins() -> None:
    config = hs.QueryConfig(lse_temperature=0.02, yaw_hot_threshold=0.04, yaw_bins=8)
    voxel = np.array([[3, 4, 5]] * 4)
    # Frame 1 saw this voxel twice (0.3 and 0.1: the max counts), frame 2 once, hot; frame 3 once, faint.
    evidence = Rasterized(
        index=voxel,
        frame_id=np.array([1, 1, 2, 3]),
        score=np.array([0.1, 0.3, 0.2, 0.01]),
        yaw_bin=np.array([0, 5, 6, 7]),
    )
    got = pool(evidence, config)
    expected = hs.pool([(1, 0.1, 0), (1, 0.3, 5), (2, 0.2, 6), (3, 0.01, 7)], config)
    assert got.index.tolist() == [[3, 4, 5]]
    assert got.score[0] == pytest.approx(expected)
    # bins 5 and 6 are hot (0.3, 0.2 > 0.04), bin 7 is not: sqrt(2)
    assert got.score[0] == pytest.approx(
        (0.3 + 0.02 * np.log(1 + np.exp((0.2 - 0.3) / 0.02) + np.exp((0.01 - 0.3) / 0.02)))
        * np.sqrt(2)
    )
    # Support like Hyperspace's: three frames, four distinct yaw bins over every hit
    # (frame 1's second look from bin 0 counts, as does the faint one).
    assert got.frames.tolist() == [3]
    assert got.bins.tolist() == [len({0, 5, 6, 7})]


def test_pack_keys_roundtrip_with_negative_indices() -> None:
    index = np.array([[0, 0, 0], [-5, 7, -900000], [123456, -1, 3]])
    assert unpack_keys(pack_keys(index)).tolist() == index.tolist()
    assert len(np.unique(pack_keys(index))) == 3


def test_combine_sums_channels_per_voxel() -> None:
    from dimos.teleop.memory_world.hyperspace_fast import Pooled

    a = Pooled(np.array([[0, 0, 0], [1, 0, 0]]), np.array([1.0, 0.5]))
    b = Pooled(np.array([[1, 0, 0], [2, 0, 0]]), np.array([1.0, 0.25]))
    got = combine(a, b, weight=2.0)
    table = {tuple(i): s for i, s in zip(got.index.tolist(), got.score, strict=True)}
    assert table == {(0, 0, 0): 1.0, (1, 0, 0): 0.5 + 2.0, (2, 0, 0): 0.5}


def test_rasterize_skips_holes_and_empty_input() -> None:
    rng = np.random.default_rng(0)
    _, _, _, frames, patches = _scene(rng)
    config = hs.QueryConfig()
    holes = Patches(
        frame=patches.frame,
        cell=patches.cell,
        depth=np.zeros_like(patches.depth),
        score=patches.score,
    )
    assert len(rasterize(frames, holes, 0.1, config).score) == 0
    empty = Patches(np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0))
    assert len(rasterize(frames, empty, 0.1, config).score) == 0
    assert len(pool(rasterize(frames, empty, 0.1, config), config).score) == 0


def test_near_scene_keeps_heat_touching_the_map() -> None:
    from dimos.teleop.memory_world.hyperspace_fast import near_scene

    scene = np.array([[10, 10, 10], [10, 11, 10], [40, 40, 40]])
    keys = np.sort(pack_keys(scene))
    heat = np.array([[10, 10, 10], [11, 12, 11], [13, 10, 10], [39, 41, 39], [0, 0, 0]])
    assert near_scene(heat, keys, radius=1).tolist() == [True, True, False, True, False]
    assert near_scene(heat, keys, radius=3).tolist() == [True, True, True, True, False]
    assert near_scene(np.zeros((0, 3), np.int64), keys).tolist() == []


def test_combine_keeps_support_when_one_channel_is_empty() -> None:
    from dimos.teleop.memory_world.hyperspace_fast import Pooled

    a = Pooled(np.array([[0, 0, 0]]), np.array([1.0]), np.array([3]), np.array([2]))
    empty = pool(
        Rasterized(
            np.zeros((0, 3), np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0, np.int64)
        ),
        hs.QueryConfig(),
    )
    got = combine(a, empty, weight=1.0)
    assert got.frames.tolist() == [3] and got.bins.tolist() == [2]


def test_support_counts_viewpoints_not_records() -> None:
    """Three segments of one photograph are one viewpoint, not three.

    The segment channel mints a pseudo-keyframe per segment RECORD, so pooling by record id
    made a single camera at a single moment support a voxel three times over — and
    `min_frames` is a filter that exists to ask whether several viewpoints agree. This is
    the fourth place in this package where a record id stood in for a thing in the world.
    """
    import numpy as np

    from dimos.teleop.memory_world.hyperspace_fast import Rasterized, pool

    config = SimpleNamespace(lse_temperature=0.02, yaw_hot_threshold=0.0, yaw_bins=8)
    one_voxel = np.zeros((4, 3), dtype=np.int64)
    evidence = Rasterized(
        index=one_voxel,
        # One patch record and three segment records, all of one photograph.
        frame_id=np.array([7, -1, -2, -3], dtype=np.int64),
        score=np.array([0.5, 0.5, 0.5, 0.5]),
        yaw_bin=np.zeros(4, dtype=np.int64),
        viewpoint=np.array([3, 3, 3, 3], dtype=np.int64),
    )
    assert pool(evidence, config).frames.tolist() == [1]

    # And two genuinely different moments still count as two.
    evidence.viewpoint = np.array([3, 3, 3, 4], dtype=np.int64)
    assert pool(evidence, config).frames.tolist() == [2]


def test_two_channels_number_one_photograph_the_same_without_sharing_a_table() -> None:
    """The patch channel and the segment channel build their frames separately.

    They still have to agree about which photograph is which, or pooling cannot tell one
    camera at one moment from two. That agreement used to come from a process-wide dict
    handing out consecutive numbers, which grew for the life of the server, carried one
    recording's numbering into the next, and was reached for under two different locks
    when two indexes loaded at once. The number comes from the (camera, moment) pair now,
    so agreement holds with no shared state at all.
    """
    import subprocess
    import sys

    import numpy as np

    from dimos.teleop.memory_world.hyperspace_fast import viewpoint_ids

    # What a clean interpreter makes of one photograph, with nothing numbered before it.
    code = (
        "import numpy as np;"
        "from dimos.teleop.memory_world.hyperspace_fast import viewpoint_ids;"
        "print(viewpoint_ids(['cam'], np.array([1234.5]))[0])"
    )
    alone = int(
        subprocess.run(
            [sys.executable, "-c", code], check=True, text=True, capture_output=True
        ).stdout
    )

    # This process numbers twenty other photographs first. A counter against a shared
    # table would hand the same photograph a different number here than there; the pair
    # decides, so it does not.
    viewpoint_ids([f"cam{i}" for i in range(20)], np.arange(20.0))
    assert viewpoint_ids(["cam"], np.array([1234.5]))[0] == alone

    # A second camera at the same moment, and the same camera at a second moment, are
    # both other viewpoints.
    three = viewpoint_ids(["cam", "other", "cam"], np.array([1234.5, 1234.5, 1234.6]))
    assert len(set(three.tolist())) == 3
    # Positive in int64, so nothing downstream that sorts or bincounts them trips.
    assert (three > 0).all()
