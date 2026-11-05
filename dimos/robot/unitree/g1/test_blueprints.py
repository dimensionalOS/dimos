# Copyright 2025-2026 Dimensional Inc.
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

"""The two G1 entry points preserve control wiring and hardware safety settings."""

import pytest

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import _check_requirements
from dimos.core.global_config import global_config
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.unitree.g1.blueprints.unitree_g1 import unitree_g1
from dimos.robot.unitree.g1.blueprints.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection


def test_g1_exposes_only_stock_and_groot_blueprints():
    assert {name for name in all_blueprints if name.startswith("unitree-g1")} == {
        "unitree-g1",
        "unitree-g1-groot-wbc",
    }
    assert [atom.module for atom in unitree_g1.active_blueprints] == [G1Connection]
    assert [atom.module for atom in unitree_g1_groot_wbc.active_blueprints] == [
        G1WholeBodyConnection,
        ControlCoordinator,
    ]


@pytest.mark.parametrize("blueprint", [unitree_g1, unitree_g1_groot_wbc])
@pytest.mark.parametrize(
    "flag,value,message",
    [
        ("simulation", "mujoco", "sim2"),
        ("simulation", "sim2", "sim2"),
        ("replay", True, "replay"),
    ],
)
def test_nonhardware_modes_fail_before_deployment(
    blueprint, flag, value, message, monkeypatch, capsys
):
    monkeypatch.setattr(global_config, flag, value)

    with pytest.raises(SystemExit, match="1"):
        _check_requirements(blueprint)

    assert message in capsys.readouterr().err


@pytest.mark.parametrize(
    "blueprint,connection",
    [
        (unitree_g1, G1Connection),
        (unitree_g1_groot_wbc, G1WholeBodyConnection),
    ],
)
def test_network_interface_override_reaches_the_connection(blueprint, connection):
    parsed = BlueprintConfigParser(blueprint).parse(["--network-interface", "test-nic"], environ={})

    assert parsed.module_kwargs(connection.name)["network_interface"] == "test-nic"


def test_groot_keeps_policy_dry_run_and_arm_velocity_limits():
    parsed = BlueprintConfigParser(unitree_g1_groot_wbc).parse(environ={})
    config = ControlCoordinatorConfig(**parsed.module_kwargs(ControlCoordinator.name))
    policy, arms = config.tasks

    assert policy.name == "groot_wbc"
    assert policy.params["auto_arm"] is False
    assert policy.params["auto_dry_run"] is True
    assert policy.params["default_ramp_seconds"] == 10.0
    assert config.tick_rate / policy.params["decimation"] == 50.0
    assert arms.name == JOINT_TRAJECTORY_TASK_NAME
    assert arms.priority < policy.priority
    assert set(arms.joint_names).isdisjoint(policy.joint_names)
    assert set(arms.joint_names) | set(policy.joint_names) == set(config.hardware[0].joints)
    assert len(arms.joint_names) == 14
    assert arms.params["velocity_limits"] == {name: 1.0 for name in arms.joint_names}
    assert (
        unitree_g1_groot_wbc.remapping_map[(ControlCoordinator.name, "twist_command")] == "cmd_vel"
    )
