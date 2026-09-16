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

from dimos.core.global_config import global_config
from dimos.robot.manipulators.common.sim import mujoco_if_sim, sim_modules


@pytest.fixture
def simulation(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")


def test_sim_modules_come_from_the_manifest(simulation: None) -> None:
    assert sim_modules.available() == ["mujoco"]
    assert mujoco_if_sim("scene.xml", 7) == ()
    global_config.simulation = "genesis"
    with pytest.raises(KeyError, match="Unknown simulation module: genesis"):
        mujoco_if_sim("scene.xml", 7)
