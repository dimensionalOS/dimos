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

import subprocess
import sys

import pytest


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    ("simulation", "backend", "auto_arm", "auto_dry_run", "ramp_seconds", "decimation"),
    [
        ("", "G1WholeBodyConnection", False, True, 3.0, 2),
        ("mujoco", "MujocoSimModule", True, False, 0.0, 1),
    ],
)
def test_webxr_blueprint_resolves_safe_lifecycle_defaults(
    simulation: str,
    backend: str,
    auto_arm: bool,
    auto_dry_run: bool,
    ramp_seconds: float,
    decimation: int,
) -> None:
    code = f"""
from dimos.core.global_config import global_config
global_config.update(simulation={simulation!r}, viewer="none")
from dimos.robot.get_all_blueprints import get_blueprint_by_name

blueprint = get_blueprint_by_name("unitree-g1-sonic-webxr-teleop")
atoms = blueprint.blueprints
assert any(atom.module.__name__ == {backend!r} for atom in atoms)
coordinator = next(atom for atom in atoms if atom.module.__name__ == "_G1SonicCoordinator")
task = coordinator.kwargs["tasks"][0]
assert task.name == "sonic_teleop"
assert task.type == "g1_sonic_teleop"
assert task.params["auto_arm"] is {auto_arm!r}
assert task.params["auto_dry_run"] is {auto_dry_run!r}
assert task.params["default_ramp_seconds"] == {ramp_seconds!r}
assert task.params["decimation"] == {decimation!r}
assert task.params["zmq_enabled"] is False
"""
    subprocess.run([sys.executable, "-c", code], check=True)
