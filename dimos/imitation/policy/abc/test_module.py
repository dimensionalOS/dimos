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

from dimos.experimental.isolated_python.module import contract_rpc_names
from dimos.imitation.policy.abc.module import DualOpenYamAbcPolicy


def test_abc_contract_is_importable_without_torch() -> None:
    blueprint = DualOpenYamAbcPolicy.blueprint(artifact="checkpoint.pt", task="bottles")
    streams = {stream.name for stream in blueprint.blueprints[0].streams}

    assert streams == {
        "button_pressed",
        "top_image",
        "left_wrist_image",
        "right_wrist_image",
        "coordinator_joint_state",
    }
    assert contract_rpc_names(DualOpenYamAbcPolicy) == {
        "preflight_rollout",
        "rollout_status",
        "start_rollout",
        "stop_rollout",
    }


def test_abc_contract_resolves_its_own_isolated_project() -> None:
    module = DualOpenYamAbcPolicy(artifact="checkpoint.pt", task="bottles")
    try:
        assert module.runtime_project == Path(__file__).parent / "python"
    finally:
        module.stop()
