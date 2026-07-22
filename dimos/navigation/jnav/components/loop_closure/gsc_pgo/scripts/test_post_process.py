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

from gtsam import Point3, Pose3, Rot3
import numpy as np

from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import (
    pose3_from_xyzquat,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.post_process import (
    best_factor_per_keyframe_marker,
    count_visits,
    interpolate_correction,
    select_keyframes,
)
from dimos.navigation.jnav.utils.apriltags import filter_glimpses, glimpse_passes

HEAD_ON_HALF_METER = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]


def _detection(**overrides):
    detection = dict(
        ts=0.0,
        marker_id=1,
        t_cam_marker=HEAD_ON_HALF_METER,
        reproj_px=1.0,
        sharpness=100.0,
        tag_px=40.0,
        speed=(0.1, 5.0),
    )
    detection.update(overrides)
    return detection


def _identity_row(ts, x, y, z):
    return [ts, x, y, z, 0.0, 0.0, 0.0, 1.0]


def test_count_visits_splits_on_time_gap():
    assert count_visits([0.0, 1.0, 2.0]) == 1
    assert count_visits([0.0, 1.0, 100.0, 101.0]) == 2
    assert count_visits([5.0, 4.0, 3.0]) == 1  # unsorted input is handled


def test_glimpse_passes_clean_detection_and_names_rejection_reasons():
    assert glimpse_passes(_detection()) is None
    assert glimpse_passes(_detection(sharpness=10.0)) == "blur"
    assert glimpse_passes(_detection(reproj_px=5.0)) == "reproj"
    assert glimpse_passes(_detection(t_cam_marker=[0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0])) == "far"


def test_glimpse_passes_treats_missing_speed_as_unknown():
    assert glimpse_passes(_detection(speed=None)) is None
    assert glimpse_passes(_detection(speed=(1.0, 5.0))) == "motion"  # known and too fast


def test_filter_glimpses_drops_excluded_and_gated_and_annotates_quality():
    detections = [
        _detection(marker_id=1),
        _detection(marker_id=7),  # excluded id
        _detection(marker_id=2, sharpness=1.0),  # fails gate
    ]
    kept = filter_glimpses(detections, exclude_tags={7})
    assert [detection["marker_id"] for detection in kept] == [1]
    assert np.isclose(kept[0]["distance_m"], 0.5)
    assert kept[0]["view_angle_deg"] < 0.1  # head-on, modulo acos rounding


def test_pose3_from_xyzquat_roundtrips_translation():
    pose = pose3_from_xyzquat([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(pose.translation(), [1.0, 2.0, 3.0])


def test_select_keyframes_triggers_on_translation_threshold():
    rows = np.array([_identity_row(float(i), i * 0.1, 0.0, 0.0) for i in range(11)], float)
    indices, poses, times = select_keyframes(rows)
    assert indices[0] == 0
    assert len(indices) > 1  # 1.0 m of travel exceeds the 0.5 m keyframe threshold
    assert len(poses) == len(indices) == len(times)


def test_select_keyframes_stationary_yields_single_keyframe():
    rows = np.array([_identity_row(float(i), 0.0, 0.0, 0.0) for i in range(10)], float)
    indices, _poses, _times = select_keyframes(rows)
    assert indices == [0]


def test_best_factor_per_keyframe_marker_keeps_lowest_reproj():
    keyframe_times = np.array([0.0, 10.0])
    detections = [
        _detection(ts=0.1, marker_id=1, reproj_px=1.5),
        _detection(ts=0.2, marker_id=1, reproj_px=0.5),  # same keyframe+marker, better
        _detection(ts=9.9, marker_id=1, reproj_px=1.0),  # different keyframe
    ]
    best = best_factor_per_keyframe_marker(detections, keyframe_times)
    assert set(best) == {(0, 1), (1, 1)}
    assert best[(0, 1)]["reproj_px"] == 0.5


def test_interpolate_correction_endpoints_and_midpoint():
    times = np.array([0.0, 1.0])
    corrections = [Pose3(), Pose3(Rot3(), Point3(2.0, 0.0, 0.0))]
    assert np.allclose(interpolate_correction(times, corrections, -5.0).translation(), [0, 0, 0])
    assert np.allclose(interpolate_correction(times, corrections, 5.0).translation(), [2, 0, 0])
    assert np.allclose(interpolate_correction(times, corrections, 0.5).translation(), [1, 0, 0])
