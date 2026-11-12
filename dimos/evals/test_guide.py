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

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.environments.hardware import HardwareEnvironment
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.guide import (
    MANIPULATION_GUIDE,
    NAVIGATION_GUIDE,
    PYTHON_API_GUIDE,
    python_guide,
)


@pytest.mark.parametrize(
    ("environment", "section", "other"),
    [
        (MujocoEnvironment(blueprint=["xarm-sim"]), MANIPULATION_GUIDE, NAVIGATION_GUIDE),
        (HabitatEnvironment(blueprint=["habitat-nav"]), NAVIGATION_GUIDE, MANIPULATION_GUIDE),
    ],
    ids=["arm", "mobile"],
)
def test_each_robot_gets_the_general_guide_and_only_its_own_section(
    environment, section, other
) -> None:
    guide = environment.python_guide

    assert guide is not None
    assert guide.startswith(PYTHON_API_GUIDE)
    assert section in guide
    assert other not in guide


def test_hardware_names_its_section() -> None:
    bare = HardwareEnvironment(blueprint=["unitree-go2"], task="walk")
    go2 = HardwareEnvironment(blueprint=["unitree-go2"], task="walk", robot_guide="navigation")

    assert bare.python_guide == PYTHON_API_GUIDE
    assert go2.python_guide == python_guide("navigation")
