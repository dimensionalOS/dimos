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
        ("", "G1WholeBodyConnection", False, True, 3.0, 1),
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
coordinator = next(atom for atom in atoms if atom.module.__name__ == "_G1SonicTeleopCoordinator")
task = coordinator.kwargs["tasks"][0]
assert task.name == "sonic_teleop"
assert task.type == "g1_sonic_teleop"
assert task.params["auto_arm"] is {auto_arm!r}
assert task.params["auto_dry_run"] is {auto_dry_run!r}
assert task.params["default_ramp_seconds"] == {ramp_seconds!r}
assert task.params["decimation"] == {decimation!r}
assert task.params["zmq_enabled"] is False
assert coordinator.kwargs["pose_transition_seconds"] == 0.5
"""
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.self_hosted
def test_webxr_blueprint_cli_selects_low_latency_pipeline() -> None:
    code = """
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
global_config.update(simulation="mujoco", viewer="none")
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_wbc import (
    _G1SonicTeleopCoordinatorConfig,
    _configure_sonic_teleop_tasks,
)

blueprint = get_blueprint_by_name("unitree-g1-sonic-webxr-teleop")
parser = BlueprintConfigParser(blueprint)
assert parser.parse().module_kwargs("ControlCoordinator")["sonic_pipeline"] == "sonic-v1.1"
parsed = parser.parse(
    cli_tokens=[
        "--sonic-pipeline", "sonic-low-latency",
        "--pose-transition-seconds", "0.8",
    ]
)
assert parsed.module_kwargs("ControlCoordinator")["sonic_pipeline"] == "sonic-low-latency"
assert parsed.module_kwargs("ControlCoordinator")["pose_transition_seconds"] == 0.8
coordinator = parsed.module_kwargs("ControlCoordinator")
coordinator_config = _G1SonicTeleopCoordinatorConfig(**coordinator)
configured_tasks = _configure_sonic_teleop_tasks(
    coordinator_config.tasks,
    coordinator_config.sonic_pipeline,
    coordinator_config.pose_transition_seconds,
)
assert configured_tasks[0].params["sonic_pipeline"] == "sonic-low-latency"
assert configured_tasks[0].params["encoder_onnx"].endswith("low_latency/model_encoder.onnx")
assert configured_tasks[0].params["decoder_onnx"].endswith("low_latency/model_decoder.onnx")
assert configured_tasks[0].params["pose_transition_seconds"] == 0.8
try:
    parser.parse(cli_tokens=["--sonic-pipeline", "unknown"])
except BlueprintConfigError:
    pass
else:
    raise AssertionError("invalid SONIC pipeline was accepted")
try:
    parser.parse(cli_tokens=["--pose-transition-seconds", "0"])
except BlueprintConfigError:
    pass
else:
    raise AssertionError("non-positive pose transition was accepted")
"""
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.self_hosted
def test_webxr_blueprint_uses_live_skeleton_only_rerun() -> None:
    code = """
from dimos.core.global_config import global_config
global_config.update(simulation="mujoco", viewer="rerun")
from dimos.robot.get_all_blueprints import get_blueprint_by_name

blueprint = get_blueprint_by_name("unitree-g1-sonic-webxr-teleop")
rerun = next(atom for atom in blueprint.blueprints if atom.module.__name__ == "RerunBridgeModule")
assert rerun.kwargs["topics"] == {
    "sonic_pose_reference": "visualization_msgs.SonicPoseReference",
}
assert rerun.kwargs["latest_only"] is True
assert rerun.kwargs["newest_first"] is True
assert rerun.kwargs["memory_limit"] == "32MB"
assert rerun.kwargs["max_hz"] == {"world/sonic_pose_reference": 30.0}
assert rerun.kwargs.get("static", {}) == {}
assert rerun.kwargs.get("visual_override", {}) == {}
"""
    subprocess.run([sys.executable, "-c", code], check=True)
