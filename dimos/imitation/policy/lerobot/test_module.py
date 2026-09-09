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
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyModule,
    LeRobotPolicyModuleConfig,
)


def test_contract_imports_without_runtime_dependencies() -> None:
    assert LeRobotPolicyModule.implementation == "dimos_lerobot.runtime:LeRobotPolicyRuntime"
    assert contract_rpc_names(LeRobotPolicyModule) == {
        "preflight_rollout",
        "rollout_status",
        "start_rollout",
        "stop_rollout",
    }


def test_contract_resolves_sibling_runtime_project() -> None:
    module = LeRobotPolicyModule(
        policy_path="unused",
        task="test task",
        joint_names=["joint"],
    )
    try:
        assert module.runtime_project == Path(__file__).parent / "python"
    finally:
        module.stop()


@pytest.mark.parametrize(
    ("config", "message"),
    [
        (
            {
                "policy_path": "checkpoint",
                "task": "test task",
                "joint_names": ["joint1", "joint1"],
            },
            "joint_names must not contain duplicates",
        ),
        (
            {
                "policy_path": " ",
                "task": "test task",
                "joint_names": ["joint1"],
            },
            "policy_path must not be blank",
        ),
        (
            {
                "policy_path": "checkpoint",
                "task": "test task",
                "joint_names": ["joint1"],
                "rollout_button": "NOPE",
            },
            "unknown Quest button",
        ),
    ],
)
def test_config_rejects_ambiguous_names(config: dict[str, object], message: str) -> None:
    with pytest.raises(ValidationError, match=message):
        LeRobotPolicyModuleConfig.model_validate(config)


def test_existing_relative_checkpoint_is_resolved_before_isolation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    monkeypatch.chdir(tmp_path)

    config = LeRobotPolicyModuleConfig(
        policy_path="checkpoint",
        task="test task",
        joint_names=["joint1"],
    )

    assert config.policy_path == str(checkpoint)
