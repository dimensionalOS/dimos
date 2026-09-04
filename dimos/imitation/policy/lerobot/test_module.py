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

from pydantic import ValidationError
import pytest

from dimos.experimental.isolated_python.module import contract_rpc_names
from dimos.imitation.policy.lerobot.module import LeRobotPolicyConfig, OpenYamLeRobotPolicy


def test_generated_contract_has_profile_ports_and_rpc_surface() -> None:
    blueprint = OpenYamLeRobotPolicy.blueprint(artifact="unused", task="test")
    streams = {stream.name for stream in blueprint.blueprints[0].streams}

    assert OpenYamLeRobotPolicy.implementation == ("dimos_lerobot.runtime:LeRobotPolicyRuntime")
    assert streams == {"button_pressed", "wrist_image", "coordinator_joint_state"}
    assert contract_rpc_names(OpenYamLeRobotPolicy) == {
        "preflight_rollout",
        "rollout_status",
        "start_rollout",
        "stop_rollout",
    }


def test_contract_resolves_sibling_runtime_project() -> None:
    module = OpenYamLeRobotPolicy(artifact="unused", task="test task")
    try:
        assert module.runtime_project == Path(__file__).parent / "python"
    finally:
        module.stop()


def test_config_rejects_blank_artifact_and_unknown_button() -> None:
    with pytest.raises(ValidationError, match="artifact must not be blank"):
        LeRobotPolicyConfig(artifact=" ", task="test")
    with pytest.raises(ValidationError, match="unknown Quest button"):
        LeRobotPolicyConfig(artifact="checkpoint", task="test", rollout_button="NOPE")


def test_existing_relative_artifact_is_resolved(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    monkeypatch.chdir(tmp_path)

    config = LeRobotPolicyConfig(artifact="checkpoint", task="test task")

    assert config.artifact == str(checkpoint)
