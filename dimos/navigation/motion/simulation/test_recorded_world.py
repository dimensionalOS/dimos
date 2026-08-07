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

"""The pure parts of the recorded world: stability filter, merge, crop."""

import numpy as np

from dimos.navigation.motion.simulation.recorded_world import (
    RecordedWorld,
    band_rects,
    clearance,
    local_map,
    merge,
    merged_boxes,
    stable_mask,
    swept,
    voxel_history,
)


def _world(voxels: np.ndarray, voxel: float = 0.1, floor_z: float = 0.0) -> RecordedWorld:
    return RecordedWorld(
        name="t",
        voxel=voxel,
        voxels=voxels.astype(np.int32),
        floor_z=floor_z,
        start=np.zeros(3),
        goal=np.zeros(2),
        path=np.zeros((0, 2)),
        track=np.zeros((0, 3)),
        rects=np.zeros((0, 4)),
        band_height=0.6,
        seen_total=len(voxels),
        carved=0,
        frames=1,
    )


def test_stability_keeps_the_persistent_and_drops_the_flicker():
    # seen 8 of the 10 frames it spanned vs 2 of 10: the flicker goes.
    seen = np.array([8, 2, 10])
    span = np.array([10, 10, 10])
    assert list(stable_mask(seen, span, frac=0.5)) == [True, False, True]


def test_stability_is_relative_to_the_visibility_span():
    # A voxel that appeared late is judged on its own window, not the run.
    seen, span = np.array([3, 3]), np.array([3, 40])
    assert list(stable_mask(seen, span, frac=0.5)) == [True, False]


def test_min_frames_overrides_a_short_span():
    # Seen once, in one frame: ratio 1.0, but a single sighting is not a wall.
    assert not stable_mask(np.array([1]), np.array([1]), frac=0.5, min_frames=2)[0]


def test_voxel_history_counts_sightings_and_spans():
    a = np.array([[0.05, 0.05, 0.05]])
    b = np.array([[0.35, 0.05, 0.05]])
    keys, seen, span = voxel_history([a, b, a], voxel=0.1)
    got = {tuple(k): (s, p) for k, s, p in zip(keys, seen, span, strict=True)}
    assert got[(0, 0, 0)] == (2, 3)  # frames 0 and 2
    assert got[(3, 0, 0)] == (1, 1)


def test_merge_covers_every_voxel_exactly_once():
    rng = np.random.default_rng(0)
    idx = np.unique(rng.integers(0, 6, size=(120, 3)), axis=0)
    boxes = merge(idx)
    covered = {
        (i, j, k)
        for lo_i, lo_j, lo_k, di, dj, dk in boxes
        for i in range(lo_i, lo_i + di)
        for j in range(lo_j, lo_j + dj)
        for k in range(lo_k, lo_k + dk)
    }
    assert covered == {tuple(v) for v in idx}
    assert sum(b[3] * b[4] * b[5] for b in boxes) == len(idx)  # no overlap


def test_merge_collapses_a_slab_into_one_box():
    idx = np.array([(i, j, k) for i in range(4) for j in range(3) for k in range(2)])
    assert merge(idx).tolist() == [[0, 0, 0, 4, 3, 2]]


def test_merge_of_nothing_is_nothing():
    assert merge(np.zeros((0, 3))).shape == (0, 6)


def test_merged_boxes_are_metric_and_centred():
    centres, half = merged_boxes(np.array([[0.05, 0.05, 0.05], [0.15, 0.05, 0.05]]), voxel=0.1)
    assert len(centres) == 1
    assert np.allclose(centres[0], [0.1, 0.05, 0.05])
    assert np.allclose(half[0], [0.1, 0.05, 0.05])


def test_band_rects_take_only_the_body_slice():
    # one column in the band, one below the floor band, one overhead
    pts = np.array([[0.05, 0.05, 0.2], [0.25, 0.05, 0.01], [0.45, 0.05, 1.2]])
    rects = band_rects(pts, voxel=0.1, floor_z=0.0)
    assert len(rects) == 1
    assert np.allclose(rects[0], [0.05, 0.05, 0.1, 0.1])


def test_local_map_is_range_limited_around_the_pose():
    idx = np.array([[0, 0, 0], [20, 0, 0], [60, 0, 0]])  # 0.05, 2.05, 6.05 m
    cloud = local_map(_world(idx), pose=(0.0, 0.0, 0.0), range_m=5.0)
    assert cloud.frame_id == "world"
    assert len(cloud) == 2


def test_local_map_follows_the_pose():
    idx = np.array([[0, 0, 0], [60, 0, 0]])
    near = local_map(_world(idx), pose=(6.0, 0.0, 0.0), range_m=1.0)
    assert np.allclose(near.points_f32(), [[6.05, 0.05, 0.05]])


def test_clearance_is_signed_against_the_band_rects():
    rects = np.array([[0.0, 0.0, 2.0, 1.0]])  # 2 x 1 slab at the origin
    got = clearance(rects, np.array([[0.0, 0.0], [2.0, 0.0], [1.0, 0.5]]))
    assert np.allclose(got, [-0.5, 1.0, 0.0])


def test_swept_carves_only_what_the_body_covered():
    track = np.array([[0.0, 0.0, 0.0]])
    xy = np.array([[0.4, 0.0], [0.5, 0.0], [0.0, 0.3]])  # in, past the nose, past the flank
    assert list(swept(xy, track, length=0.85, width=0.5)) == [True, False, False]


def test_swept_follows_the_yaw():
    track = np.array([[0.0, 0.0, np.pi / 2]])  # nose along +y
    assert list(swept(np.array([[0.0, 0.4], [0.4, 0.0]]), track, 0.85, 0.5)) == [True, False]
