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

import numpy as np

from dimos.simulation.go2_sim.world import STILL, CommandHold


def test_a_fresh_command_is_returned_and_a_stale_one_becomes_still() -> None:
    hold = CommandHold(timeout=0.2)
    assert np.array_equal(hold.current(0.0), STILL)
    command = np.array([0.5, 0.0, 0.1])
    assert hold.update(command, 1.0)
    assert np.array_equal(hold.current(1.1), command)
    assert np.array_equal(hold.current(1.3), STILL)


def test_non_finite_commands_are_refused_and_the_previous_one_kept() -> None:
    hold = CommandHold(timeout=0.2)
    command = np.array([0.5, 0.0, 0.1])
    hold.update(command, 1.0)
    assert not hold.update(np.array([np.nan, 0.0, 0.0]), 1.1)
    assert not hold.update(np.array([0.0, np.inf, 0.0]), 1.1)
    assert np.array_equal(hold.current(1.1), command)
