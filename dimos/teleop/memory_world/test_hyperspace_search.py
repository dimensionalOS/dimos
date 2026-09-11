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

from __future__ import annotations

import sqlite3

import numpy as np

from dimos.teleop.memory_world.hyperspace_search import (
    Cluster,
    assign_points,
    cluster_voxels,
    memory_db_for,
    memory_db_ready,
)


def _blob(origin: tuple[int, int, int], size: int) -> list[tuple[int, int, int]]:
    ox, oy, oz = origin
    return [(ox + i, oy + j, oz + k) for i in range(size) for j in range(size) for k in range(size)]


def test_two_blobs_become_two_clusters_best_first() -> None:
    weak = _blob((0, 0, 0), 2)  # 8 voxels
    strong = _blob((40, 40, 0), 3)  # 27 voxels, far away
    indices = np.asarray(weak + strong)
    scores = np.asarray([0.4] * len(weak) + [0.9] * len(strong))

    clusters, labels = cluster_voxels(indices, scores, voxel_size=0.1)

    assert [c.n_voxels for c in clusters] == [27, 8]
    assert clusters[0].index == 0 and clusters[1].index == 1
    assert np.allclose(clusters[0].centre, ((40 + 1.5) * 0.1, (40 + 1.5) * 0.1, 1.5 * 0.1))
    assert list(labels[: len(weak)]) == [1] * len(weak)
    assert list(labels[len(weak) :]) == [0] * len(strong)
    assert clusters[0].peak == 0.9 and clusters[0].score > clusters[1].score


def test_a_one_voxel_gap_does_not_split_a_cluster() -> None:
    left = _blob((0, 0, 0), 2)
    right = _blob((3, 0, 0), 2)  # x = 3,4; the gap is x = 2
    indices = np.asarray(left + right)
    clusters, labels = cluster_voxels(indices, np.ones(len(indices)), voxel_size=0.1)
    assert len(clusters) == 1
    assert set(labels.tolist()) == {0}


def test_tiny_and_faint_blobs_are_dropped() -> None:
    big = _blob((0, 0, 0), 3)
    speck = [(20, 20, 20), (20, 20, 21)]  # below MIN_CLUSTER_VOXELS
    faint = _blob((60, 0, 0), 2)  # enough voxels, negligible score
    indices = np.asarray(big + speck + faint)
    scores = np.asarray([1.0] * len(big) + [1.0] * len(speck) + [0.01] * len(faint))
    clusters, labels = cluster_voxels(indices, scores, voxel_size=0.1)
    assert len(clusters) == 1
    assert list(labels[len(big) :]) == [-1] * (len(speck) + len(faint))


def test_empty_input() -> None:
    clusters, labels = cluster_voxels(np.zeros((0, 3), int), np.zeros(0), 0.1)
    assert clusters == [] and labels.shape == (0,)


def test_assign_points_uses_radius_plus_slack() -> None:
    clusters = [
        Cluster(index=0, centre=(0.0, 0.0, 0.0), radius=0.5, score=1, peak=1, n_voxels=9),
        Cluster(index=1, centre=(10.0, 0.0, 0.0), radius=0.5, score=1, peak=1, n_voxels=9),
    ]
    points = np.asarray([[0.3, 0, 0], [9.0, 0, 0], [5.0, 0, 0], [0, 1.2, 0]])
    assert assign_points(points, clusters, slack_m=0.75).tolist() == [0, 1, -1, 0]


def test_memory_db_ready_needs_both_streams(tmp_path) -> None:
    recording = tmp_path / "walk.mcap"
    recording.write_bytes(b"")
    assert memory_db_for(recording) == tmp_path / "walk.hyperspace.db"
    assert not memory_db_ready(recording)

    db = sqlite3.connect(memory_db_for(recording))
    db.execute("CREATE TABLE _streams (name TEXT)")
    db.execute("INSERT INTO _streams VALUES ('hyperspace_keyframes')")
    db.execute("CREATE TABLE hyperspace_keyframes (id INTEGER)")
    db.commit()
    assert not memory_db_ready(recording), "patches missing"

    db.execute("INSERT INTO _streams VALUES ('hyperspace_patches')")
    db.commit()
    assert not memory_db_ready(recording), "no keyframes yet"

    db.execute("INSERT INTO hyperspace_keyframes VALUES (1)")
    db.commit()
    db.close()
    assert memory_db_ready(recording)
