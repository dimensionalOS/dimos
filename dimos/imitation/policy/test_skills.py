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

import json

import pytest

from dimos.imitation.policy.skills import PolicyRolloutSpec, PolicySkills


@pytest.mark.parametrize("ready", [False, True])
def test_policy_skills_gate_start_on_preflight_and_expose_stop(mocker, ready):
    module = PolicySkills()
    policy = mocker.Mock(spec=PolicyRolloutSpec)
    mocker.patch.object(module, "_policy", policy, create=True)
    status = {
        "active": False,
        "artifact": "checkpoint",
        "backend": "lerobot",
        "task": "bottle",
        "device": "cpu",
        "policy_ready": ready,
        "observations_ready": True,
        "chunks_accepted": 0,
        "last_error": None if ready else "invalid checkpoint",
    }
    policy.rollout_status.return_value = status
    policy.preflight_rollout.return_value = status
    policy.start_rollout.return_value = {**status, "active": True}
    policy.stop_rollout.return_value = status
    try:
        result = json.loads(module.run_policy())
        assert result["active"] is ready
        assert policy.start_rollout.call_count == int(ready)
        assert json.loads(module.stop_policy()) == status
        policy.stop_rollout.assert_called_once_with()
        assert json.loads(module.policy_status()) == status
    finally:
        module.stop()
