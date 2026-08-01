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

"""Current DimSim spatial stack for an externally driven one-tool Pi session."""

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.benchmark.agent_eval.observation_recorder import (
    DEFAULT_AGENT_EVAL_RECORDING_PATH,
    AgentEvalObservationRecorder,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

_external_pi_navigation_skills = autoconnect(
    NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=GO2Connection.camera_info_static),
    UnitreeSkillContainer.blueprint(),
)

unitree_go2_dimsim_external_pi_eval = autoconnect(
    unitree_go2,
    SpatialMemory.blueprint(),
    AgentEvalObservationRecorder.blueprint(
        db_path=DEFAULT_AGENT_EVAL_RECORDING_PATH,
        on_existing="overwrite",
    ),
    CodePolicyModule.blueprint(recording_path=DEFAULT_AGENT_EVAL_RECORDING_PATH),
    McpServer.blueprint(),
    _external_pi_navigation_skills,
).global_config(
    simulation="dimsim",
    n_workers=14,
    robot_model="unitree_go2",
)
