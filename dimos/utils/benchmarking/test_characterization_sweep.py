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

"""The dynamics sweep is anchored at the discovered floor, so every step moves
the robot (the fix for vy, whose floor sat above its first static sweep amp)."""

from __future__ import annotations

import pytest

from dimos.utils.benchmarking.characterization import _floor_anchored_sweep
from dimos.utils.benchmarking.plant import GO2_PLANT_PROFILE


def test_sweep_starts_at_the_floor_and_keeps_point_count() -> None:
    # With a measured floor below the predefined top, the ladder starts at the
    # floor, ends at the predefined top, and keeps the same number of points.
    static = GO2_PLANT_PROFILE.si_amplitudes["vx"]
    amps = _floor_anchored_sweep(GO2_PLANT_PROFILE, "vx", floor=0.2)
    assert len(amps) == len(static)
    assert amps[0] == pytest.approx(0.2)
    assert amps[-1] == pytest.approx(max(static))
    assert amps == sorted(amps)


def test_vy_floor_above_first_static_amp_no_longer_undershoots() -> None:
    # The bug: vy static sweep [0.2, 0.4] starts at 0.2, but vy's floor is ~0.25,
    # so the first step produced no motion. Anchoring fixes it.
    amps = _floor_anchored_sweep(GO2_PLANT_PROFILE, "vy", floor=0.25)
    assert min(amps) >= 0.25
    assert amps[0] == pytest.approx(0.25)


def test_floor_at_or_above_top_still_spans_a_range() -> None:
    # Degenerate case: floor lands at/above the predefined top -> ladder must
    # still climb (not collapse to a single repeated amplitude).
    amps = _floor_anchored_sweep(GO2_PLANT_PROFILE, "wz", floor=2.0)
    assert len(amps) == len(GO2_PLANT_PROFILE.si_amplitudes["wz"])
    assert amps[0] == pytest.approx(2.0)
    assert amps[-1] > amps[0]


def test_single_amplitude_axis_returns_floor() -> None:
    class _Stub:
        si_amplitudes = {"vx": [0.5]}

    assert _floor_anchored_sweep(_Stub(), "vx", floor=0.3) == [0.3]  # type: ignore[arg-type]
