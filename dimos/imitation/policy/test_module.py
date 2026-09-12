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
import pickle
import subprocess
import sys

from pydantic import ValidationError
import pytest

from dimos.experimental.isolated_python.bootstrap import load_class, validate_runtime
from dimos.experimental.isolated_python.module import contract_rpc_names
from dimos.imitation.policy.module import (
    PolicyModule,
    PolicyModuleConfig,
    policy_module,
)


def test_contract_imports_without_runtime_dependencies() -> None:
    assert PolicyModule.implementation == "dimos.imitation.policy.runtime:_PolicyRuntime"
    assert contract_rpc_names(PolicyModule) == {
        "preflight_rollout",
        "rollout_status",
        "start_rollout",
        "stop_rollout",
    }


def test_contract_resolves_sibling_runtime_project() -> None:
    module = PolicyModule(
        policy_path="unused",
        task="test task",
        joint_names=["joint"],
    )
    try:
        assert module.runtime_project == Path(__file__).parent / "lerobot" / "python"
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


def test_camera_ports_and_runtime_survive_a_fresh_process(tmp_path):
    blueprint = PolicyModule.blueprint(
        image_mapping={"left_image": "left", "right_image": "right", "top_image": "top"},
        backend="abc",
        policy_path="checkpoint",
        task="bottles",
        joint_names=["joint"],
    )
    declaration = blueprint.blueprints[0].module
    runtime = load_class(declaration.implementation)
    validate_runtime(declaration, runtime)
    assert {stream.name for stream in blueprint.blueprints[0].streams} == {
        "left_image",
        "right_image",
        "top_image",
        "coordinator_joint_state",
        "button_pressed",
    }
    payload = tmp_path / "blueprint.pkl"
    payload.write_bytes(pickle.dumps(blueprint))
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import pickle, sys; b=pickle.load(open(sys.argv[1], 'rb')); "
            "from dimos.experimental.isolated_python.bootstrap import load_class, validate_runtime; "
            "d=b.blueprints[0].module; validate_runtime(d, load_class(d.implementation)); "
            "print(','.join(sorted(s.name for s in b.blueprints[0].streams)))",
            str(payload),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    assert (
        result.stdout.strip()
        == "button_pressed,coordinator_joint_state,left_image,right_image,top_image"
    )


@pytest.mark.parametrize(
    "mapping", [{"stop": "left"}, {"_private": "left"}, {"left": "same", "right": "same"}, {}]
)
def test_camera_bindings_reject_reserved_ports_and_duplicate_features(mapping):
    with pytest.raises(ValueError):
        policy_module(image_mapping=mapping)
