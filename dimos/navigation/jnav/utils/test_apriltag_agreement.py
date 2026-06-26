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

"""Tests for the April-tag agreement metric."""

from __future__ import annotations

import numpy as np

from dimos.navigation.jnav.utils.apriltag_agreement import (
    agreement_improvement,
    agreement_report,
    tag_spread,
)


def test_tag_spread_zero_for_identical() -> None:
    assert tag_spread(np.array([[1.0, 2.0, 3.0]] * 4)) == 0.0


def test_tag_spread_grows_with_scatter() -> None:
    tight = tag_spread(np.array([[0.0, 0, 0], [0.1, 0, 0]]))
    loose = tag_spread(np.array([[0.0, 0, 0], [2.0, 0, 0]]))
    assert loose > tight


def test_single_observation_excluded() -> None:
    report = agreement_report({7: np.array([[0.0, 0, 0]])})
    assert report.tag_count == 0  # one sighting carries no agreement signal
    assert report.total_observations == 1
    assert report.mean_spread == 0.0


def test_report_mean_spread() -> None:
    report = agreement_report(
        {
            1: np.array([[0.0, 0, 0], [0.0, 0, 0]]),  # spread 0
            2: np.array([[0.0, 0, 0], [2.0, 0, 0]]),  # spread 1.0
        }
    )
    assert report.tag_count == 2
    assert report.total_observations == 4
    assert abs(report.mean_spread - 0.5) < 1e-9


def test_improvement_positive_when_corrected_tighter() -> None:
    # Drifted: same tag scattered 4 m apart across laps. Corrected: pulled together.
    raw = agreement_report({1: np.array([[0.0, 0, 0], [4.0, 0, 0]])})
    corrected = agreement_report({1: np.array([[0.0, 0, 0], [0.4, 0, 0]])})
    improvement = agreement_improvement(raw, corrected)
    assert 0.85 < improvement <= 1.0


def test_improvement_negative_when_corrected_worse() -> None:
    raw = agreement_report({1: np.array([[0.0, 0, 0], [0.4, 0, 0]])})
    corrected = agreement_report({1: np.array([[0.0, 0, 0], [4.0, 0, 0]])})
    assert agreement_improvement(raw, corrected) < 0.0


def test_improvement_zero_when_no_raw_spread() -> None:
    raw = agreement_report({1: np.array([[0.0, 0, 0], [0.0, 0, 0]])})
    corrected = agreement_report({1: np.array([[0.0, 0, 0], [1.0, 0, 0]])})
    assert agreement_improvement(raw, corrected) == 0.0
