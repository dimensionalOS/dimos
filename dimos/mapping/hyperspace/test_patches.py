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

"""Synthetic-scene tests for the hyperspace math: an object planted in frames
taken from known poses must light up the voxel it sits in, and only there."""

from __future__ import annotations

import math

import numpy as np

from dimos.mapping.hyperspace.patches import (
    BufferedFrame,
    HotPatch,
    Intrinsics,
    Keyframe,
    KeyframeGateConfig,
    QueryConfig,
    RollingBuffer,
    grid_distance,
    heatmap,
    per_patch_depth,
    pool,
    quality_gate,
    reproject_depth,
    transform_matrix,
)

ROWS, COLS = 12, 16
CAMERA = Intrinsics(width=160, height=120, fx=120.0, fy=120.0, cx=80.0, cy=60.0)


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """4x4 pose of an optical camera (z forward, x right, y down) at position looking at target."""
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([right, down, forward])
    pose[:3, 3] = position
    return pose


def project(pose: np.ndarray, point: np.ndarray) -> tuple[float, float, float]:
    local = np.linalg.inv(pose) @ np.append(point, 1.0)
    return (
        local[0] / local[2] * CAMERA.fx + CAMERA.cx,
        local[1] / local[2] * CAMERA.fy + CAMERA.cy,
        local[2],
    )


def ring(obj: np.ndarray, count: int, radius: float) -> list[np.ndarray]:
    return [
        look_at(obj + np.array([radius * math.cos(a), radius * math.sin(a), 0.3]), obj)
        for a in (i / count * math.tau for i in range(count))
    ]


def scene(
    obj: np.ndarray, poses: list[np.ndarray], *, with_depth: bool = True
) -> tuple[list[HotPatch], dict[int, np.ndarray]]:
    """Every frame sees the object in exactly one patch; that patch is hot with score 0.5."""
    hot, placed = [], {}
    for index, pose in enumerate(poses):
        u, v, depth = project(pose, obj)
        col, row = int(u * COLS / CAMERA.width), int(v * ROWS / CAMERA.height)
        patch_depth = np.full(ROWS * COLS, depth if with_depth else np.nan, dtype=np.float32)
        keyframe = Keyframe(
            id=index,
            camera_frame="cam",
            ts=10.0 + index,
            rows=ROWS,
            cols=COLS,
            intrinsics=CAMERA,
            patch_depth=patch_depth,
        )
        hot.append(HotPatch(keyframe=keyframe, patch=row * COLS + col, score=0.5))
        placed[index] = pose
    return hot, placed


def voxel_of(point: np.ndarray, size: float) -> tuple[int, int, int]:
    return tuple(int(v) for v in np.floor(point / size))


def voxel_distance(a: tuple[int, int, int], b: tuple[int, int, int]) -> int:
    return max(abs(x - y) for x, y in zip(a, b, strict=True))


def test_object_seen_from_three_poses_lands_on_its_voxel() -> None:
    obj = np.array([3.0, 2.0, 0.5])
    hot, poses = scene(obj, ring(obj, 3, 2.5))
    result = heatmap(hot, lambda kf: poses[kf.id], "odom", 0.1, QueryConfig())
    assert result.voxels, result.stats
    top, score = result.voxels[0]
    assert score == 1.0
    # A 10 px patch at 2.5 m is ~0.2 m wide, and every voxel inside the three
    # pyramids' overlap ties, so the peak can sit two voxels off the point.
    assert voxel_distance(top, voxel_of(obj, 0.1)) <= 2, (top, voxel_of(obj, 0.1))
    far = [s for index, s in result.voxels if voxel_distance(index, voxel_of(obj, 0.1)) > 4]
    assert max(far, default=0.0) < 0.7 * score


def test_rewriting_poses_moves_the_answer() -> None:
    obj = np.array([3.0, 2.0, 0.5])
    hot, poses = scene(obj, ring(obj, 3, 2.5))
    before = heatmap(hot, lambda kf: poses[kf.id], "odom", 0.1, QueryConfig()).voxels[0][0]
    shift = np.eye(4)
    shift[0, 3] = 1.0
    after = heatmap(hot, lambda kf: shift @ poses[kf.id], "odom", 0.1, QueryConfig()).voxels[0][0]
    assert after[0] - before[0] == 10
    assert after[1] == before[1]


def test_depth_caps_pyramids_and_missing_depth_is_counted() -> None:
    obj = np.array([3.0, 2.0, 0.5])
    hot, poses = scene(obj, ring(obj, 3, 2.5))
    result = heatmap(hot, lambda kf: poses[kf.id], "odom", 0.1, QueryConfig())
    for index, _ in result.voxels:
        assert voxel_distance(index, voxel_of(obj, 0.1)) <= 6
    no_depth, _ = scene(obj, ring(obj, 3, 2.5), with_depth=False)
    empty = heatmap(no_depth, lambda kf: poses[kf.id], "odom", 0.1, QueryConfig())
    assert empty.voxels == []
    assert empty.stats["hot_patches_without_depth"] == 3


def test_unplaceable_keyframes_are_skipped() -> None:
    obj = np.array([3.0, 2.0, 0.5])
    hot, poses = scene(obj, ring(obj, 3, 2.5))
    result = heatmap(hot, lambda kf: None, "odom", 0.1, QueryConfig())
    assert result.voxels == []
    assert result.stats["keyframes_placed"] == 0


def test_pooling_rewards_more_frames_and_more_directions() -> None:
    config = QueryConfig()
    one = pool([(0, 0.1, 0)], config)
    two_same = pool([(0, 0.1, 0), (1, 0.1, 0)], config)
    two_dirs = pool([(0, 0.1, 0), (1, 0.1, 4)], config)
    cold_extra = pool([(0, 0.1, 0), (1, 0.001, 4)], config)
    assert two_same > one
    assert two_dirs > two_same
    assert cold_extra < two_same
    assert abs(cold_extra - one) < 0.01


def test_rolling_buffer_keeps_one_frame_per_distinct_view() -> None:
    config = KeyframeGateConfig(
        buffer_len=11, min_interval=None, max_angular_velocity=None, max_dark_fraction=None
    )
    buffer = RollingBuffer(config)
    rng = np.random.default_rng(0)

    def grid(seed: int) -> np.ndarray:
        g = np.random.default_rng(seed).standard_normal((ROWS * COLS, 8)).astype(np.float32)
        return (g / np.linalg.norm(g, axis=1, keepdims=True)).astype(np.float16)

    kept = 0
    for i in range(60):
        view = grid(1 if i < 30 else 2)
        quality = 1.0 + float(rng.random()) * 1e-3
        if (
            buffer.push(BufferedFrame(ts=10.0 + i * 0.2, grid=view, quality=quality, payload=i))
            is not None
        ):
            kept += 1
    kept += len(buffer.flush())
    assert kept == 2


def test_rolling_buffer_prefers_the_sharpest_novel_frame() -> None:
    config = KeyframeGateConfig(buffer_len=3, min_interval=None, patch_novelty_threshold=None)
    buffer = RollingBuffer(config)

    def grid(angle: float) -> np.ndarray:
        return np.tile(np.array([[math.cos(angle), math.sin(angle)]], dtype=np.float16), (2, 1))

    assert buffer.push(BufferedFrame(0.0, grid(0.0), 1.0, None)) is None
    assert buffer.push(BufferedFrame(1.0, grid(1.0), 0.5, None)) is None
    assert (
        buffer.push(BufferedFrame(2.0, grid(2.0), 0.9, None)) is None
    )  # middle loses to the sharper next frame
    winner = buffer.push(BufferedFrame(3.0, grid(2.0), 0.1, None))
    assert winner is not None and winner.ts == 2.0


def test_grid_distance_and_gates() -> None:
    a = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=np.float16)
    b = np.array([[0.0, 1.0], [1.0, 0.0]], dtype=np.float16)
    assert grid_distance(a, a)[0] < 1e-3
    assert abs(grid_distance(a, b)[0] - 1.0) < 1e-3
    dark = np.zeros((8, 8, 3), dtype=np.uint8)
    assert quality_gate(KeyframeGateConfig(), dark, None) == "too_dark"
    assert quality_gate(KeyframeGateConfig(max_dark_fraction=None), dark, None) is None
    assert quality_gate(KeyframeGateConfig(), dark, (9.0, 0.0)) == "too_fast"


def test_per_patch_depth_median_ignores_holes() -> None:
    depth = np.array([[0.0, 2.0, 0.0, 0.0], [4.0, 0.0, 0.0, 0.0]], dtype=np.float32)
    patches = per_patch_depth(depth, 1, 2)
    assert patches[0] == 3.0  # median of [2, 4]
    assert np.isnan(patches[1])


def test_reproject_depth_shifts_with_baseline() -> None:
    small = Intrinsics(width=8, height=4, fx=4.0, fy=4.0, cx=4.0, cy=2.0)
    depth = np.zeros((4, 8), dtype=np.float32)
    depth[2, 4] = 1.0
    identity = reproject_depth(depth, small, small, np.eye(4))
    assert identity[2, 4] == 1.0
    baseline = np.eye(4)
    baseline[0, 3] = 0.25  # 0.25 m sideways at 1 m with fx=4 moves one column
    shifted = reproject_depth(depth, small, small, baseline)
    assert shifted[2, 5] == 1.0
    assert (shifted > 0).sum() == 1


def test_transform_matrix_is_a_proper_pose() -> None:
    matrix = transform_matrix(
        np.array([1.0, 2.0, 3.0]),
        np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]),
    )
    moved = matrix @ np.array([1.0, 0.0, 0.0, 1.0])
    assert np.allclose(moved[:3], [1.0, 3.0, 3.0], atol=1e-9)
