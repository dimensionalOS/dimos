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

from dataclasses import replace

import numpy as np
import pytest

from dimos.robot.unitree.g1.sonic_observation import POLICY_JOINT_NAMES
from dimos.robot.unitree.g1.sonic_observation_pairing import (
    SonicMeasuredState,
    SonicObservationPairer,
)


@pytest.fixture
def state():
    return SonicMeasuredState(
        sequence=7,
        positions={name: i / 10 for i, name in reversed(list(enumerate(POLICY_JOINT_NAMES)))},
        body_observed_at=10.01,
        left_hand_observed_at=10.02,
        right_hand_observed_at=10.0,
    )


@pytest.fixture
def rgb():
    return np.full((480, 640, 3), [10, 20, 30], np.uint8)


def test_pairs_named_joints_and_owned_rgb_using_oldest_acquisition_time(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    assert pairer.push_image(rgb, observed_at=10.015, now=10.03) is None
    rgb[:] = 0
    obs = pairer.push_state(state, now=10.04)
    assert obs is not None and obs.observed_at == 10.0 and obs.sequence == 0
    np.testing.assert_allclose(obs.state, np.arange(28) / 10)
    np.testing.assert_array_equal(obs.rgb[0, 0], [10, 20, 30])
    assert (
        pairer.push_state(
            replace(
                state,
                sequence=8,
                body_observed_at=10.03,
                left_hand_observed_at=10.04,
                right_hand_observed_at=10.02,
            ),
            now=10.05,
        )
        is None
    )


def test_buffer_matches_delayed_image_to_earlier_state(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    pairer.push_state(
        replace(
            state,
            sequence=8,
            body_observed_at=10.08,
            left_hand_observed_at=10.08,
            right_hand_observed_at=10.08,
        ),
        now=10.09,
    )
    obs = pairer.push_image(rgb, observed_at=10.015, now=10.1)
    assert obs is not None and obs.observed_at == 10.0


def test_excess_skew_or_expired_buffer_never_produces_observation(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.01)
    pairer.push_state(state, now=10.03)
    assert pairer.push_image(rgb, observed_at=10.03, now=10.04) is None
    assert pairer.push_image(rgb, observed_at=10.25, now=10.26) is None
    assert pairer.buffered_states == 0


@pytest.mark.parametrize(
    "change",
    [
        {"body_observed_at": 9.0},
        {"left_hand_observed_at": 10.2},
        {"right_hand_observed_at": float("nan")},
        {"positions": {}},
        {"sequence": True},
    ],
)
def test_invalid_or_stale_component_cannot_enter_pairer(state, change):
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError):
        pairer.push_state(replace(state, **change), now=10.05)
    assert pairer.buffered_states == 0


def test_replayed_state_or_frozen_hand_timestamp_rejected(state):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    with pytest.raises(ValueError, match="newer"):
        pairer.push_state(state, now=10.04)
    with pytest.raises(ValueError, match="newer"):
        pairer.push_state(
            replace(state, sequence=8, body_observed_at=10.04, left_hand_observed_at=10.04),
            now=10.05,
        )
    assert pairer.buffered_states == 1


def test_replayed_image_rejected_and_consumed_state_not_reused(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    assert pairer.push_image(rgb, observed_at=10.02, now=10.04) is not None
    with pytest.raises(ValueError, match="newer"):
        pairer.push_image(rgb, observed_at=10.02, now=10.05)
    assert pairer.push_image(rgb, observed_at=10.025, now=10.05) is None


def test_buffer_count_is_bounded(state):
    pairer = SonicObservationPairer(max_skew=0.03, capacity=2)
    for index in range(5):
        stamp = 10 + index / 100
        pairer.push_state(
            replace(
                state,
                sequence=index,
                body_observed_at=stamp,
                left_hand_observed_at=stamp,
                right_hand_observed_at=stamp,
            ),
            now=10.1,
        )
    assert pairer.buffered_states == 2


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_skew": 0},
        {"max_skew": float("inf")},
        {"max_skew": 0.03, "max_age": 0.3},
        {"max_skew": 0.03, "capacity": 0},
    ],
)
def test_invalid_pairing_budget_rejected(kwargs):
    with pytest.raises(ValueError):
        SonicObservationPairer(**kwargs)


def test_camera_uncertainty_is_included_in_age_skew_and_expiry(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    obs = pairer.push_image(rgb, observed_at=10.005, uncertainty=0.01, now=10.03)
    assert obs is not None and obs.observed_at == pytest.approx(9.995)
    with pytest.raises(ValueError, match="acquisition"):
        pairer.push_image(rgb, observed_at=10.2, uncertainty=0.01, now=10.395)


def test_component_uncertainty_preserves_oldest_possible_time(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.04)
    pairer.push_image(rgb, observed_at=10.015, uncertainty=0.002, now=10.04)
    obs = pairer.push_state(
        replace(state, acquisition_uncertainty_seconds=(0.003, 0.004, 0.006)), now=10.04
    )
    assert obs is not None and obs.observed_at == pytest.approx(9.994)
    np.testing.assert_allclose(obs.state, np.arange(28) / 10)


@pytest.mark.parametrize("component", range(3))
@pytest.mark.parametrize("stamp,now", [(9.81, 10.0), (9.99, 10.0)])
def test_any_component_interval_crossing_age_or_future_boundary_is_rejected(
    state, component, stamp, now
):
    errors = [0.0, 0.0, 0.0]
    errors[component] = 0.02
    bounded = replace(
        state,
        body_observed_at=stamp,
        left_hand_observed_at=stamp,
        right_hand_observed_at=stamp,
        acquisition_uncertainty_seconds=tuple(errors),
    )
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError, match="acquisition"):
        pairer.push_state(bounded, now=now)
    assert pairer.buffered_states == 0


@pytest.mark.parametrize(
    "errors",
    [
        (-0.001, 0, 0),
        (0, float("nan"), 0),
        (0, 0, float("inf")),
        (0, 0),
        (0, 0, 0, 0),
        (True, 0, 0),
        ("unknown", 0, 0),
    ],
)
def test_unknown_or_malformed_uncertainty_cannot_enter_pairer(state, errors):
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError, match="uncertainty"):
        pairer.push_state(replace(state, acquisition_uncertainty_seconds=errors), now=10.05)
    assert pairer.buffered_states == 0


def test_intervals_must_fit_skew_even_when_all_nominal_times_match(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(
        replace(
            state,
            body_observed_at=10,
            left_hand_observed_at=10,
            right_hand_observed_at=10,
            acquisition_uncertainty_seconds=(0, 0.016, 0),
        ),
        now=10.03,
    )
    assert pairer.push_image(rgb, observed_at=10, now=10.03) is None


def test_expiry_removes_later_state_with_older_uncertain_lower_bound(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.19)
    pairer.push_state(
        replace(
            state,
            body_observed_at=10.10,
            left_hand_observed_at=10.10,
            right_hand_observed_at=10.10,
        ),
        now=10.15,
    )
    pairer.push_state(
        replace(
            state,
            sequence=8,
            body_observed_at=10.11,
            left_hand_observed_at=10.11,
            right_hand_observed_at=10.11,
            acquisition_uncertainty_seconds=(0.08, 0, 0),
        ),
        now=10.20,
    )
    obs = pairer.push_image(rgb, observed_at=10.18, now=10.24)
    assert pairer.buffered_states == 1
    assert obs is not None and obs.observed_at == pytest.approx(10.10)


def test_changing_error_bound_does_not_make_a_repeated_component_new(state):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    with pytest.raises(ValueError, match="newer"):
        pairer.push_state(
            replace(
                state,
                sequence=8,
                body_observed_at=10.02,
                left_hand_observed_at=10.03,
                acquisition_uncertainty_seconds=(0.001, 0.001, 0.001),
            ),
            now=10.04,
        )


def test_uncertain_states_do_not_emit_observation_time_older_than_previous(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.1)
    pairer.push_state(state, now=10.03)
    assert pairer.push_image(rgb, observed_at=10.01, now=10.04) is not None
    pairer.push_state(
        replace(
            state,
            sequence=8,
            body_observed_at=10.02,
            left_hand_observed_at=10.03,
            right_hand_observed_at=10.01,
            acquisition_uncertainty_seconds=(0, 0, 0.02),
        ),
        now=10.05,
    )
    assert pairer.push_image(rgb, observed_at=10.02, now=10.05) is None


def test_rejected_interval_does_not_consume_state_sequence(state):
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError):
        pairer.push_state(replace(state, acquisition_uncertainty_seconds=(0, -1, 0)), now=10.03)
    assert (
        pairer.push_state(
            replace(state, acquisition_uncertainty_seconds=(0.001, 0.001, 0.001)), now=10.04
        )
        is None
    )
    assert pairer.buffered_states == 1


def test_overflowing_interval_endpoint_is_rejected(state):
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError, match="acquisition"):
        pairer.push_state(
            replace(
                state,
                body_observed_at=1.7e308,
                acquisition_uncertainty_seconds=(1e308, 0, 0),
            ),
            now=1.7e308,
        )
    assert pairer.buffered_states == 0
