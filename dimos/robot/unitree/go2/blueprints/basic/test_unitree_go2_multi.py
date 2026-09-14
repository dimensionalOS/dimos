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

from dimos.core.global_config import global_config


@pytest.mark.parametrize("blueprint_name", ["unitree_go2_multi", "unitree_go2_multi_teleop"])
@pytest.mark.parametrize("robot_ips", [None, "", "   ", " , , "])
def test_multi_robot_blueprint_rejects_empty_ips(
    monkeypatch: pytest.MonkeyPatch, blueprint_name: str, robot_ips: str | None
) -> None:
    monkeypatch.setattr(global_config, "robot_ips", robot_ips)

    # Execute afresh without reusing or changing a cached blueprint module.
    with pytest.raises(ValueError, match="ROBOT_IPS.*--robot-ips"):
        runpy.run_path(str(Path(__file__).with_name(f"{blueprint_name}.py")))


@pytest.mark.parametrize(
    ("blueprint_name", "module_names"),
    [
        ("unitree_go2_multi", ["go2connection"]),
        ("unitree_go2_multi_teleop", ["go2connection", "keyboardteleop"]),
    ],
)
@pytest.mark.parametrize("ips", [("192.0.2.10",), ("192.0.2.10", "192.0.2.11")])
def test_multi_robot_blueprint_constructs_modules_per_ip(
    monkeypatch: pytest.MonkeyPatch,
    blueprint_name: str,
    module_names: list[str],
    ips: tuple[str, ...],
) -> None:
    monkeypatch.setattr(global_config, "robot_ips", ", ".join(ips))

    namespace = runpy.run_path(str(Path(__file__).with_name(f"{blueprint_name}.py")))
    blueprint = namespace[blueprint_name]

    assert [atom.name for atom in blueprint.blueprints] == [
        f"robot{i}/{name}" for i in range(len(ips)) for name in module_names
    ]
    assert [
        atom.kwargs["ip"] for atom in blueprint.blueprints if atom.module.name == "go2connection"
    ] == list(ips)
