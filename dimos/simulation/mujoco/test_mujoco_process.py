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
from pytest_mock import MockerFixture

from dimos.simulation.mujoco import mujoco_process
from dimos.simulation.mujoco.config import MujocoConfig


@pytest.mark.parametrize("robot", ["unitree_go2", "unitree_go1", "unitree_g1", None])
def test_simulation_preserves_requested_robot_for_labels(
    robot: str | None, mocker: MockerFixture
) -> None:
    mocker.patch.object(mujoco_process, "load_scene_xml", return_value="<mujoco/>")
    load_model = mocker.patch.object(
        mujoco_process, "load_model", side_effect=RuntimeError("stop before opening viewer")
    )

    with pytest.raises(RuntimeError, match="stop before opening viewer"):
        mujoco_process._run_simulation(MujocoConfig(), mocker.Mock(), robot_model=robot)

    load_model.assert_called_once_with(
        mocker.ANY, robot=robot or "unitree_go1", scene_xml="<mujoco/>"
    )
