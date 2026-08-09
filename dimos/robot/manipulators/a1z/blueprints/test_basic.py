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

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.a1z.blueprints import basic
from dimos.robot.manipulators.a1z.config import (
    A1Z_DOF,
    A1Z_G1Z_SIM_MODEL_PATH,
    A1Z_GRASP_FRAME_TO_TCP,
    A1Z_PRE_GRASP_DIRECTION,
    A1Z_SIM_HOME,
    a1z_hardware,
    make_a1z_sim_robot_config,
)
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
)


def test_a1z_sim_planning_uses_provider_world_pose() -> None:
    placement = PoseStamped(position=Vector3(1.0, 2.0, 0.3))

    config = make_a1z_sim_robot_config(placement)

    assert config.base_pose == placement
    assert config.get_coordinator_joint_names() == [
        f"arm/joint{index}" for index in range(1, A1Z_DOF + 1)
    ]
    assert config.end_effector_link == "gripper_eef_link"
    assert config.home_joints == A1Z_SIM_HOME
    assert config.grasp_frame_to_tcp == A1Z_GRASP_FRAME_TO_TCP
    assert config.pre_grasp_direction == A1Z_PRE_GRASP_DIRECTION


def test_existing_a1z_blueprint_requests_selected_simulation_provider(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    hardware = a1z_hardware("arm")
    binding = SimulationBinding(
        backend=Blueprint(blueprints=()),
        hardware=(hardware,),
    )
    resolve_robot = mocker.patch.object(
        basic,
        "resolve_robot",
        return_value=binding,
    )
    monkeypatch.setattr(global_config, "simulation_provider", "pimsim")
    monkeypatch.setattr(global_config, "scene_package", "tabletop-test")

    assert basic._resolve_a1z_robot() is binding
    resolve_robot.assert_called_once_with(
        real_hardware=(hardware,),
        simulation=SimulationRequest(
            robot_model="galaxea_a1z",
            model_path=A1Z_G1Z_SIM_MODEL_PATH,
            scene_package="tabletop-test",
            features=frozenset(
                (
                    SimulationFeature.EPISODE_CONTROL,
                    SimulationFeature.MANIPULATION_SCENE,
                )
            ),
        ),
    )
