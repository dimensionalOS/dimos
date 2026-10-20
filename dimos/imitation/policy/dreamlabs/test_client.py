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

import dream
from dream.transport import Transport
import numpy as np
from pytest_mock import MockerFixture

from dimos.imitation.policy.dreamlabs.conversion import CAMERAS, action_trajectory, policy_config
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics


def test_real_client_validates_pose_observations_and_preserves_joint_chunks(
    mocker: MockerFixture,
) -> None:
    q_left = [0.0, 0.5, 0.5, -0.35, 0.0, 0.0]
    q_right = [0.2, 0.6, 0.7, -0.1, 0.2, -0.3]
    vector = np.array([*q_left, *q_right, 0.2, 0.8])
    response = {
        "status": {"code": "DA-0000"},
        "actions": {
            "q_l_chunk": [[*q_left, 0.2]] * 48,
            "q_r_chunk": [[*q_right, 0.8]] * 48,
            "per_key": {"action.left_arm": [q_left] * 48, "action.right_arm": [q_right] * 48},
        },
    }
    request = mocker.patch.object(Transport, "request", return_value=response)
    lookup = mocker.patch(
        "dream.models.list_models", side_effect=AssertionError("no model lookup expected")
    )
    with dream.DreamClient(api_key="offline-fixture") as client:
        config = policy_config("dreamlabs/dream-1-beta", 1500)
        assert "ik" not in config
        client.policy(config)
        state = DreamlabsKinematics().observation(vector, "fixture")
        client.set_observation(
            {f"video.{role}": "synthetic-jpeg" for role in CAMERAS}, state, timestamp=10.0
        )
        reply = client.get_action(
            commanded={"q_l_chunk": [*q_left, 0.2], "q_r_chunk": [*q_right, 0.8]}
        )
    request.assert_called_once()
    lookup.assert_not_called()
    sent = request.call_args.args[0]["observation"]
    assert sent["embodiment"] == "yam"
    np.testing.assert_allclose(sent["state.left_eef_xyz"], [0.262523, 0, 0.206904], atol=2e-5)
    np.testing.assert_allclose(reply["q_l_chunk"], [[*q_left, 0.2]] * 48, atol=1e-8)
    trajectory = action_trajectory(reply, vector, np.full(14, -3.0), np.full(14, 3.0), 1.0, 1.0)
    np.testing.assert_allclose(
        trajectory.points[1].positions, [*q_left, *q_right, 0.2, 0.8], atol=1e-8
    )


def test_import_and_help_work_without_optional_sdk() -> None:
    code = """
import sys
sys.modules['dream'] = None
sys.modules['dream_ik'] = None
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import dreamlabs_dual_openyam
from dimos.imitation.policy.dreamlabs.probe import main
sys.argv = ['probe', '--help']
main()
"""
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=30
    )
    assert result.returncode == 0, result.stderr
    assert "--live" in result.stdout
