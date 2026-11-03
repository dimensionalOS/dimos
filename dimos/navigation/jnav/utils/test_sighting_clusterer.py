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

"""SightingClusterer: glances never publish, the cooldown groups a visit, the Huber test drops outliers, and a moved
medoid republishes under the same sighting id."""

from __future__ import annotations

import pytest

from dimos.navigation.jnav.utils.apriltags import (
    STABLE_POSE_KEY,
    SightingClusterer,
    SightingClustering,
)


def detection(ts, x, marker_id=13, y=0.0):
    return {"ts": ts, "marker_id": marker_id, STABLE_POSE_KEY: [x, y, 0.0, 0.0, 0.0, 0.0, 1.0]}


def feed(clusterer, detections):
    return [update for update in map(clusterer.add, detections) if update is not None]


def test_a_glance_is_never_published():
    clusterer = SightingClusterer(SightingClustering(min_inliers=3))
    assert feed(clusterer, [detection(0.0, 1.0), detection(0.1, 1.0)]) == []
    assert len(feed(clusterer, [detection(0.2, 1.0)])) == 1


def test_detections_within_the_cooldown_are_one_sighting():
    clusterer = SightingClusterer(SightingClustering(cooldown_s=5.0, min_inliers=2))
    updates = feed(clusterer, [detection(ts, 1.0) for ts in (0.0, 4.0, 8.5)])
    assert [update.sighting_id for update in updates] == [0]
    # past the cooldown the tag starts a new sighting
    updates = feed(clusterer, [detection(20.0, 1.0), detection(20.5, 1.0)])
    assert [(update.sighting_id, update.supersedes) for update in updates] == [(1, False)]


def test_other_tags_are_their_own_sightings():
    clusterer = SightingClusterer(SightingClustering(min_inliers=2))
    updates = feed(
        clusterer,
        [
            detection(0.0, 1.0, 13),
            detection(0.1, 5.0, 21),
            detection(0.2, 1.0, 13),
            detection(0.3, 5.0, 21),
        ],
    )
    assert [(update.sighting_id, update.medoid["marker_id"]) for update in updates] == [
        (0, 13),
        (1, 21),
    ]


def test_huber_outliers_do_not_count_or_move_the_medoid():
    clustering = SightingClustering(min_inliers=3, huber_delta_m=0.05, min_huber_weight=0.25)
    clusterer = SightingClusterer(clustering)
    # two agree, two are 0.5 m off in different directions: not enough inliers yet
    assert (
        feed(
            clusterer,
            [detection(0.0, 1.0), detection(0.1, 1.5), detection(0.2, 1.01), detection(0.3, 0.5)],
        )
        == []
    )
    (update,) = feed(clusterer, [detection(0.4, 0.99)])
    assert update.inliers == 3
    assert update.detections == 5
    assert update.medoid[STABLE_POSE_KEY][0] == pytest.approx(1.0)


def test_a_moved_medoid_supersedes_the_published_sighting():
    clusterer = SightingClusterer(
        SightingClustering(min_inliers=2, republish_shift_m=0.02, huber_delta_m=0.5)
    )
    (first,) = feed(clusterer, [detection(0.0, 1.0), detection(0.1, 1.0)])
    # the same medoid again: nothing new to send
    assert feed(clusterer, [detection(0.2, 1.0)]) == []
    # the tag's agreed position moves to 1.1 m: the medoid moves, republished under the same id
    updates = feed(clusterer, [detection(0.3 + 0.1 * i, 1.1) for i in range(4)])
    assert updates
    assert all(update.sighting_id == first.sighting_id and update.supersedes for update in updates)
    assert updates[-1].medoid[STABLE_POSE_KEY][0] == pytest.approx(1.1)
    assert updates[-1].publish_ts >= updates[-1].medoid["ts"]


def test_noisy_glimpses_are_gated():
    clusterer = SightingClusterer(SightingClustering(min_inliers=1, min_sharpness=30.0))
    assert feed(clusterer, [{**detection(0.0, 1.0), "sharpness": 5.0}]) == []
    assert clusterer.rejected == {"blur": 1}
