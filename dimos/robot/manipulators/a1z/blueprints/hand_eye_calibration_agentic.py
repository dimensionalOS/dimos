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

"""A1Z hand-eye calibration via MCP or chat. Supply the camera publisher at launch."""

from dimos.agents.agent import Agent
from dimos.agents.mcp.mcp_server import McpServer
from dimos.calibration.hand_eye_skills import HandEyeCalibrationSkills
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.a1z.blueprints.basic import a1z_planner_coordinator

a1z_hand_eye_calibration_agent = autoconnect(
    a1z_planner_coordinator,
    HandEyeCalibrationSkills.blueprint(),
    McpServer.blueprint(),
    Agent.blueprint(
        system_prompt=(
            "You operate hand-eye calibration. The operator places the fixed target before "
            "requesting calibration. Ask for any missing home joint angles (radians) or camera "
            "mounting frame. Once the operator has supplied both, call start_hand_eye_calibration "
            "once with those values. "
            "Wait for the completion update and report the outcome, metrics and saved path "
            "when available. After a failure, report the view and reason and wait for a new "
            "operator request. Use hand_eye_calibration_status for status requests and "
            "stop_hand_eye_calibration for stop requests."
        )
    ),
)
