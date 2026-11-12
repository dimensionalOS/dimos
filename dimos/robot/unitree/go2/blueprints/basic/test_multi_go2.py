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

from pathlib import Path
import runpy

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.connection import GO2Connection


@pytest.mark.parametrize("name", ["unitree_go2_multi", "unitree_go2_multi_teleop"])
@pytest.mark.parametrize("robot_ips", [None, "", "   ", " , , "])
def test_empty_robot_ips_are_rejected_before_starting_workers(
    name: str,
    robot_ips: str | None,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(global_config, "robot_ips", robot_ips)
    # Evaluate the composition afresh without changing the imported blueprint cache.
    blueprint = runpy.run_path(str(Path(__file__).with_name(f"{name}.py")))[name]
    mocker.patch("dimos.core.coordination.module_coordinator._run_configurators")
    start = mocker.patch.object(ModuleCoordinator, "start")

    with pytest.raises(SystemExit) as error:
        ModuleCoordinator.build(blueprint)

    assert error.value.code == 1
    assert "Set ROBOT_IPS or --robot-ips to at least one IP." in capsys.readouterr().err
    start.assert_not_called()


@pytest.mark.parametrize(
    ("name", "module_names"),
    [
        ("unitree_go2_multi", ["robot0/go2connection", "robot1/go2connection"]),
        (
            "unitree_go2_multi_teleop",
            [
                "robot0/go2connection",
                "robot0/keyboardteleop",
                "robot1/go2connection",
                "robot1/keyboardteleop",
            ],
        ),
    ],
)
def test_configured_ips_create_namespaced_robots(
    name: str, module_names: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(global_config, "robot_ips", " 192.0.2.10, ,192.0.2.11, ")

    blueprint = runpy.run_path(str(Path(__file__).with_name(f"{name}.py")))[name]

    assert [atom.name for atom in blueprint.active_blueprints] == module_names
    assert {
        atom.name: atom.kwargs["ip"]
        for atom in blueprint.active_blueprints
        if atom.module is GO2Connection
    } == {"robot0/go2connection": "192.0.2.10", "robot1/go2connection": "192.0.2.11"}
    assert blueprint.global_config_overrides["n_workers"] == 4
    assert [check() for check in blueprint.requirement_checks] == [None]
