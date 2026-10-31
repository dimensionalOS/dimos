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

from importlib import import_module
from pathlib import Path
import pickle

from pydantic import ValidationError
import pytest

from dimos.experimental.isolated_python.module import contract_rpc_names
from dimos.imitation.policy.module import (
    PolicyModule,
    PolicyModuleConfig,
    policy_class,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.data import get_project_root


def test_contract_imports_without_runtime_dependencies() -> None:
    assert PolicyModule.implementation == "dimos.imitation.policy.runtime:_PolicyRuntime"
    assert contract_rpc_names(PolicyModule) == {
        "get_last_action_response",
        "get_last_preview",
        "set_task",
        "preflight_rollout",
        "rollout_status",
        "start_rollout",
        "stop_rollout",
    }


def test_contract_resolves_checkout_runtime_project(mocker) -> None:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "serve_module_rpc", "start", "stop"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    module = PolicyModule(
        rpc_transport=LCMRPC,
        policy_path="unused",
        task="test task",
        joint_names=["joint"],
    )
    try:
        assert module.runtime_project == get_project_root() / "native/python/lerobot"
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
        PolicyModuleConfig.model_validate(config)


def test_existing_relative_checkpoint_is_resolved_before_isolation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    monkeypatch.chdir(tmp_path)

    config = PolicyModuleConfig(
        policy_path="checkpoint",
        task="test task",
        joint_names=["joint1"],
    )

    assert config.policy_path == str(checkpoint)


def test_dynamic_camera_contract_and_runtime_are_importable_in_fresh_workers():
    declaration = policy_class(("left_image", "right_image"))
    assert pickle.loads(pickle.dumps(declaration)) is declaration
    module_name, name = declaration.implementation.split(":")
    implementation = getattr(import_module(module_name), name)
    assert issubclass(implementation, declaration)
    assert set(declaration.__annotations__) == {"left_image", "right_image"}
