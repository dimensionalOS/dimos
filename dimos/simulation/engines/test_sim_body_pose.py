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

import time
from unittest.mock import MagicMock

from dimos.simulation.engines.sim_body_pose import SimBodyPose, SimBodyPoseConfig


def _missing_body_source(grace_seconds: float) -> SimBodyPose:
    source = object.__new__(SimBodyPose)
    source.config = SimBodyPoseConfig(
        body_name="plant_pot_1",
        missing_body_grace_seconds=grace_seconds,
    )
    source._sim = MagicMock()
    source._sim.get_body_poses.return_value = {}
    source._missing_logged = False
    source._started_at = time.monotonic()
    return source


def test_missing_body_is_silent_during_simulator_startup_grace() -> None:
    source = _missing_body_source(grace_seconds=5.0)

    assert source._read_pose() is None
    assert source._missing_logged is False


def test_missing_body_is_reported_after_startup_grace() -> None:
    source = _missing_body_source(grace_seconds=0.0)

    assert source._read_pose() is None
    assert source._missing_logged is True
