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

import pytest

from dimos.robot.unitree.g1.sonic_dds_timing import summarize_dds_timing


def sample(source_ns=990_000_000, *, group="body", publisher="a", tick=10, **changes):
    return {
        "group": group,
        "publication_handle": publisher,
        "valid_data": True,
        "source_timestamp_ns": source_ns,
        "take_before_monotonic_ns": 100_000_000,
        "take_after_monotonic_ns": 101_000_000,
        "take_before_wall_ns": 1_000_000_000,
        "take_after_wall_ns": 1_001_000_000,
        "tick": tick,
        **changes,
    }


def test_timing_diagnostics_never_claim_acquisition_readiness():
    report = summarize_dds_timing([sample()], duration_ns=2_000_000_000)
    body = report["groups"]["body"]
    assert report["acquisition_alignment_verified"] is False
    assert body["valid_samples"] == 1
    assert body["observed_take_rate_hz"] == 0.5
    assert body["publishers"]["a"]["source_to_take_end_ms"] == {
        "min": 11.0,
        "median": 11.0,
        "max": 11.0,
    }
    assert body["publishers"]["a"]["take_call_ms"]["max"] == 1.0
    assert report["groups"]["left_hand"]["valid_samples"] == 0


def test_publisher_changes_do_not_mix_unrelated_timestamp_histories():
    report = summarize_dds_timing(
        [sample(900_000_000), sample(700_000_000, publisher="b"), sample(902_000_000)],
        duration_ns=1_000_000_000,
    )["groups"]["body"]
    assert report["publisher_transitions"] == 2
    assert report["publishers"]["a"]["source_regressions"] == 0
    assert report["publishers"]["a"]["source_period_ms"]["median"] == 2.0
    assert report["publishers"]["b"]["source_period_ms"] is None


def test_repeated_and_backwards_source_times_and_tick_are_visible():
    report = summarize_dds_timing(
        [
            sample(900_000_000, tick=2**32 - 1),
            sample(900_000_000, tick=0),
            sample(899_000_000, tick=0),
        ],
        duration_ns=1_000_000_000,
    )["groups"]["body"]["publishers"]["a"]
    assert report["source_repeats"] == 1
    assert report["source_regressions"] == 1
    assert report["tick_repeats"] == 1
    assert report["tick_decreases_wrap_or_reset"] == 1
    assert report["tick_negative_step_modulo_2_32"] == [1]


def test_invalid_data_and_missing_source_timestamp_remain_explicit():
    report = summarize_dds_timing(
        [sample(valid_data=False), sample(0)],
        duration_ns=1_000_000_000,
    )["groups"]["body"]
    assert report["invalid_samples"] == 1
    assert report["valid_samples"] == 1
    assert report["publishers"]["a"]["missing_source_timestamps"] == 1
    assert report["publishers"]["a"]["source_to_take_end_ms"] is None


def test_future_source_clock_and_local_wall_step_are_not_hidden():
    rows = [
        sample(2_000_000_000),
        sample(
            2_010_000_000,
            take_before_monotonic_ns=110_000_000,
            take_after_monotonic_ns=111_000_000,
            take_before_wall_ns=980_000_000,
            take_after_wall_ns=981_000_000,
        ),
    ]
    report = summarize_dds_timing(rows, duration_ns=1_000_000_000)
    publisher = report["groups"]["body"]["publishers"]["a"]
    assert publisher["source_after_local_take"] == 2
    assert publisher["wall_minus_monotonic_step_ms"]["min"] == -30.0
    assert report["acquisition_alignment_verified"] is False


@pytest.mark.parametrize(
    "changes",
    [
        {"source_timestamp_ns": True},
        {"take_after_monotonic_ns": 99_000_000},
        {"group": "unknown"},
        {"tick": -1},
    ],
)
def test_malformed_trace_fails_explicitly(changes):
    with pytest.raises(ValueError):
        summarize_dds_timing([sample(**changes)], duration_ns=1_000_000_000)


@pytest.mark.parametrize("duration", [0, -1, True])
def test_nonpositive_or_noninteger_duration_is_rejected(duration):
    with pytest.raises(ValueError):
        summarize_dds_timing([], duration_ns=duration)
