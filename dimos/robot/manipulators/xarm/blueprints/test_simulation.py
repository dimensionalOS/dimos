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

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.manipulators.xarm.blueprints import simulation
from dimos.robot.manipulators.xarm.config import XARM7_MODEL_PATH, XARM7_TABLETOP_SCENE
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
)


def test_xarm_blueprint_requests_provider_capabilities(monkeypatch, mocker) -> None:
    binding = SimulationBinding(
        backend=Blueprint(blueprints=()),
        hardware=(simulation._xarm7_sim_hw,),
    )
    resolve_robot = mocker.patch.object(
        simulation,
        "resolve_robot",
        return_value=binding,
    )
    monkeypatch.setattr(global_config, "scene_package", "tabletop-test")

    assert simulation._resolve_xarm7_simulation() is binding
    resolve_robot.assert_called_once_with(
        real_hardware=mocker.ANY,
        simulation=SimulationRequest(
            robot_model="xarm7",
            model_path=XARM7_MODEL_PATH,
            scene_package="tabletop-test",
            features=frozenset(
                (
                    SimulationFeature.EPISODE_CONTROL,
                    SimulationFeature.MANIPULATION_SCENE,
                    SimulationFeature.SENSORS,
                )
            ),
        ),
    )


def test_xarm_blueprint_uses_default_scene_package(monkeypatch, mocker) -> None:
    binding = SimulationBinding(
        backend=Blueprint(blueprints=()),
        hardware=(simulation._xarm7_sim_hw,),
    )
    resolve_robot = mocker.patch.object(
        simulation,
        "resolve_robot",
        return_value=binding,
    )
    monkeypatch.setattr(global_config, "scene_package", None)

    assert simulation._resolve_xarm7_simulation() is binding
    assert resolve_robot.call_args.kwargs["simulation"].scene_package == XARM7_TABLETOP_SCENE
