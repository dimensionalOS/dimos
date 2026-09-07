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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.imitation.profile import ImageSource
from dimos.robot.manipulators.dual_openyam.blueprints.agentic import dual_openyam_sim_policy_agent
from dimos.robot.manipulators.dual_openyam.blueprints.sim_learning import dual_openyam_sim_collect
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.learning import (
    DUAL_OPENYAM_LEROBOT_IO,
    DualOpenYamSimRecorder,
)
from dimos.robot.manipulators.dual_openyam.sim import DUAL_OPENYAM_SIM_CAMERAS


def test_simulation_cameras_and_sqlite_ports_match_the_policy_contract():
    profile = DUAL_OPENYAM_LEROBOT_IO
    cameras = {camera.stream: camera for camera in DUAL_OPENYAM_SIM_CAMERAS}
    image_streams = {
        source.stream for source in profile.observations.values() if isinstance(source, ImageSource)
    }
    assert image_streams == set(cameras)
    assert all(camera.fps >= 2 * profile.sync.rate_hz for camera in cameras.values())
    for source in profile.observations.values():
        if isinstance(source, ImageSource):
            camera = cameras[source.stream]
            assert source.shape == (camera.height, camera.width, 3)
    assert profile.action.demonstration.joints == tuple(DUAL_OPENYAM_JOINTS)
    atom = next(
        atom
        for atom in dual_openyam_sim_collect.blueprints
        if atom.module is DualOpenYamSimRecorder
    )
    assert {stream.name for stream in atom.streams if stream.direction == "in"} - {"tf"} == {
        *image_streams,
        "status",
        "coordinator_joint_state",
        "applied_joint_position_command",
    }


def test_policy_artifact_cli_override_and_coordinator_ownership():
    parsed = BlueprintConfigParser(dual_openyam_sim_policy_agent).parse(
        ["--policyrolloutmodule.artifact=/tmp/test-checkpoint", "--policyrolloutmodule.device=cpu"],
        environ={},
    )
    assert parsed.module_kwargs(POLICY_ROLLOUT_INSTANCE_NAME)["artifact"] == "/tmp/test-checkpoint"
    tasks = parsed.module_kwargs("ControlCoordinator")["tasks"]
    policy = next(task for task in tasks if task["name"] == "policy_rollout")
    assert policy["joint_names"] == DUAL_OPENYAM_JOINTS
    assert policy["priority"] > max(task["priority"] for task in tasks if task is not policy)
    assert parsed.module_kwargs("ControlCoordinator")["sim_scene_path"] is not None
