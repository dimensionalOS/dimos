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

from dimos.core.python_native_module import contract_rpc_names
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyModule,
    LeRobotPolicyModuleConfig,
)


def test_contract_imports_without_runtime_dependencies() -> None:
    assert LeRobotPolicyModule.implementation == "dimos_lerobot.runtime:LeRobotPolicyRuntime"
    assert contract_rpc_names(LeRobotPolicyModule) == {
        "execute_learned_policy",
        "policy_status",
        "stop_learned_policy",
    }


def test_contract_resolves_sibling_runtime_project() -> None:
    module = LeRobotPolicyModule(
        policies={"smoke": {"policy_path": "unused"}},
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
                "policies": {"default": {"policy_path": "checkpoint"}},
                "joint_names": ["joint1", "joint1"],
            },
            "joint_names must not contain duplicates",
        ),
        (
            {
                "policies": {" ": {"policy_path": "checkpoint"}},
                "joint_names": ["joint1"],
            },
            "policy names must not be empty",
        ),
    ],
)
def test_config_rejects_ambiguous_names(config: dict[str, object], message: str) -> None:
    with pytest.raises(ValidationError, match=message):
        LeRobotPolicyModuleConfig(**config)
