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

"""The diagnosis tool's measurement primitives."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from dimos.navigation.motion.adapter.diagnose import (
    Crop,
    Instant,
    Recording,
    Window,
    arclen,
    classify,
    host_setup,
    is_hold,
    parse_instant,
    payload_ts,
    stamp_dialect,
    voxel_centers,
    voxel_keys,
)
from dimos.navigation.motion.geometry import divergence, resample


def test_resample_walks_even_arc_length():
    line = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    out = resample(line, step=0.5)
    steps = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert np.allclose(steps, 0.5)
    assert abs(arclen(line) - 2.0) < 1e-9


def test_divergence_is_the_offset_between_parallel_plans():
    a = np.column_stack([np.linspace(0, 2, 21), np.zeros(21)])
    assert divergence(a, a) == 0.0
    assert abs(divergence(a, a + np.array([0.0, 0.3])) - 0.3) < 1e-6


def test_divergence_compares_only_the_shared_arc():
    a = np.column_stack([np.linspace(0, 4, 41), np.zeros(41)])
    assert divergence(a, a[:11]) < 1e-6  # a prefix of the same route has not changed its mind


def test_single_pose_path_is_a_hold():
    assert is_hold(np.zeros((1, 2)))
    assert not is_hold(np.zeros((2, 2)))


def test_voxel_keys_round_trip_through_centres():
    pts = np.array([[0.01, 0.01, 0.30], [0.05, 0.05, 0.31], [-1.0, 2.0, 0.0]])
    keys = voxel_keys(pts, 0.08)
    assert len(keys) == 2  # the first two land in the same 0.08 m voxel
    centres = voxel_centers(keys, 0.08)
    for p in pts:
        assert np.abs(centres - p).max(axis=1).min() <= 0.0401


def test_crop_margin_excludes_the_window_edge():
    crop = Crop(centre=np.array([0.0, 0.0]), radius=2.0, z_lo=-1.0, z_hi=1.0)
    pts = np.array([[0.0, 0.0, 0.0], [1.95, 0.0, 0.0], [0.0, 0.0, 0.99]])
    assert list(crop.inside(pts, margin=0.16)) == [True, False, False]


# --- the follower pass's classifier


def test_a_tick_under_the_threshold_matches_on_every_component():
    verdict, gap = classify((0.30, 0.0, 0.5), (0.35, 0.02, 0.44), boundary=False, threshold=0.15)
    assert verdict == "match"
    assert abs(gap - 0.06) < 1e-9  # the WORST component, not the first one over


def test_one_component_over_the_threshold_is_the_whole_tick():
    # a twist that agrees on speed and disagrees on turn rate is a disagreement
    verdict, _ = classify((0.30, 0.0, 0.5), (0.30, 0.0, 0.9), boundary=False, threshold=0.15)
    assert verdict == "MISMATCH"


def test_a_plan_landing_inside_a_control_period_is_unpairable():
    # which plan the module held is genuinely ambiguous there, so it is neither
    # a match nor a finding
    verdict, _ = classify((0.30, 0.0, 0.5), (0.90, 0.0, 0.5), boundary=True, threshold=0.15)
    assert verdict == "boundary"


def test_a_hold_is_held_against_zero_rather_than_the_law():
    # the module never reached its law -- deadman, latch, or a refusal stub
    assert classify((0.0, 0.0, 0.0), None, boundary=False, threshold=0.15)[0] == "hold"
    assert classify((0.0, 0.0, 0.0), None, boundary=True, threshold=0.15)[0] == "hold"


def test_driving_through_a_hold_is_a_finding_not_a_hold():
    verdict, gap = classify((0.40, 0.0, 0.0), None, boundary=False, threshold=0.15)
    assert verdict == "MISMATCH"
    assert gap == 0.40


# --- the time filters


def test_a_bare_number_is_seconds_into_the_recording():
    assert parse_instant("6.9") == Instant(6.9, absolute=False)
    assert parse_instant("6.9").resolve(1000.0) == 1006.9


def test_a_clock_time_is_utc_time_of_day():
    assert parse_instant("06:34:35.4") == Instant(6 * 3600 + 34 * 60 + 35.4, absolute=True)
    midnight = 1_800_000_000.0 - 1_800_000_000.0 % 86400
    assert parse_instant("01:00:00").resolve(midnight + 3000) == midnight + 3600


def test_a_clock_time_takes_the_occurrence_nearest_the_start():
    # a recording that crosses midnight must not be dated by its own start
    midnight = 1_800_000_000.0 - 1_800_000_000.0 % 86400
    late = midnight - 60.0  # 23:59:00 the previous day
    assert parse_instant("23:59:30").resolve(late) == late + 30.0


def test_a_time_that_is_neither_form_is_refused():
    with pytest.raises(ValueError):
        parse_instant("06:34")


def test_an_unset_window_holds_nothing_out():
    w = Window()
    assert not w.bounded
    assert 1e12 in w and -1e12 in w
    assert Window.between(None, None, 100.0) == w


def test_a_window_is_inclusive_at_both_ends():
    w = Window.between(parse_instant("1.0"), parse_instant("3.0"), 100.0)
    assert (w.lo, w.hi) == (101.0, 103.0)
    assert w.bounded and 101.0 in w and 103.0 in w and 100.9 not in w
    assert list(w.mask(np.array([100.5, 101.0, 102.0, 103.5]))) == [False, True, True, False]


# --- what a payload stamp turns out to mean


def _receipts(n=200, rate=30.0, t0=1_785_969_268.0):
    return t0 + np.arange(n) / rate


def test_a_stamp_older_than_its_receipt_is_sensor_time():
    # the synced rig: the stamp is when the lidar saw it, the receipt when the
    # recorder got it, and the difference is the pipeline
    ts = _receipts()
    d = stamp_dialect(ts, ts - 0.093)
    assert d.verdict == "sensor-time"
    assert d.sensor_time
    assert abs(d.delta - 0.093) < 1e-6  # unix-scale floats: microseconds is the resolution


def test_a_stamp_written_at_publish_time_carries_no_age():
    ts = _receipts()
    rng = np.random.default_rng(0)
    d = stamp_dialect(ts, ts - rng.normal(0.0, 3e-4, len(ts)))
    assert d.verdict == "receipt-echo"
    assert not d.sensor_time


def test_a_stamp_from_after_its_own_receipt_is_not_an_age():
    # a host clock skew puts the stamp in the recorder's future; that is a
    # receipt echo with a skew on it, never a negative pipeline age
    ts = _receipts()
    assert stamp_dialect(ts, ts + 0.008).verdict == "receipt-echo"


def test_a_boot_relative_clock_is_foreign():
    # every recording before go2web 7316c06: the lidar's seconds-since-boot
    ts = _receipts()
    d = stamp_dialect(ts, ts - 1_785_861_881.0)
    assert d.verdict == "foreign-clock"
    assert not d.sensor_time


def test_a_stream_with_no_payload_stamp_says_so():
    ts = _receipts(n=10)
    d = stamp_dialect(ts, np.full(10, np.nan))
    assert d.verdict == "no-stamp"
    assert not d.sensor_time
    assert math.isnan(d.delta)


def test_an_untimestamped_message_type_reads_as_no_stamp():
    class Twist:
        pass

    class Odom:
        ts = 12.5

    assert math.isnan(payload_ts(Twist()))
    assert payload_ts(Odom()) == 12.5


# --- both clocks, carried side by side


def _recording(stamps):
    ts = _receipts(n=5)
    return Recording(
        path="x",
        maps=[(ts[0], np.zeros((1, 3)))],
        odom_ts=ts,
        odom_stamp_ts=stamps(ts),
        odom_xy=np.zeros((5, 2)),
        poses=[None] * 5,
        globals=[],
        plans=[],
        dialects={"odometry": stamp_dialect(ts, stamps(ts))},
    )


def test_physics_reads_the_sensor_clock_while_pairing_keeps_the_receipts():
    rec = _recording(lambda ts: ts - 0.09)
    assert rec.dialects["odometry"].sensor_time
    # the module paired on arrival, so replay must too
    assert np.array_equal(rec.odom_ts, _receipts(n=5))
    # but the pose was true 90 ms before it landed
    assert np.allclose(rec.odom_physics_ts, rec.odom_ts - 0.09)


def test_an_old_recording_has_one_clock_and_it_is_the_receipt():
    rec = _recording(lambda ts: ts - 1_785_861_881.0)
    assert rec.dialects["odometry"].verdict == "foreign-clock"
    assert np.array_equal(rec.odom_physics_ts, rec.odom_ts)


# --- the deployed config, as one JSON


def test_the_host_blob_is_read_off_the_follower_section(tmp_path):
    blob = {
        "modules": {
            "trajectory_follower": {
                "topics": {},
                "config": {
                    "track": "blind",
                    "controller_config": {"max_speed": 0.95, "min_speed": 0.45},
                    "max_path_age_s": 2.5,
                    "obstacle_model": "raw_band",
                },
            }
        }
    }
    path = tmp_path / "motion-host.json"
    path.write_text(json.dumps(blob))
    setup = host_setup(str(path))
    assert setup.track == "blind"
    assert setup.controller.max_speed == 0.95
    assert setup.obstacle_model == "raw_band"
    assert setup.max_path_age_s == 2.5
    # the keys the blob leaves out keep the module's own defaults
    assert setup.control_frequency == 10.0
    assert setup.period == 0.1
    assert setup.controller.max_yaw_rate == 1.4


def test_a_blob_with_no_follower_in_it_says_so(tmp_path):
    path = tmp_path / "motion-host.json"
    path.write_text(json.dumps({"modules": {"motion_planner": {"config": {}}}}))
    with pytest.raises(SystemExit, match="trajectory_follower"):
        host_setup(str(path))
