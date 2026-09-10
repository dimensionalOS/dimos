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

"""Physics scheduling must remain independent of camera/viewer frame time."""

import pytest

from dimos.simulation.engines.mujoco_engine import _physics_steps_due

pytestmark = pytest.mark.mujoco


def test_render_delay_is_recovered_with_fixed_physics_steps():
    first_count, deadline = _physics_steps_due(0.0, 0.0, 0.002)
    catchup_count, deadline = _physics_steps_due(0.05, deadline, 0.002)

    assert first_count + catchup_count == 26
    assert deadline == pytest.approx(0.052)


def test_early_wakeup_does_not_advance_physics():
    assert _physics_steps_due(0.001, 0.002, 0.002) == (0, 0.002)


def test_long_pause_caps_recovery_work_and_restarts_the_clock():
    steps, deadline = _physics_steps_due(10.0, 0.0, 0.002)

    assert steps == 64
    assert deadline == pytest.approx(10.002)
