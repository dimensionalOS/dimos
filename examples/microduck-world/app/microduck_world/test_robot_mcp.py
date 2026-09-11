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

from unittest.mock import Mock

from dimos.agents.mcp.mcp_server import McpServer, app
from microduck_world.knowledge import DuckKnowledge
from microduck_world.robot_mcp import RobotMcpServer


def test_agent_tools_exclude_foreign_robots_and_operator_endpoints(module_factory, monkeypatch):
    server = module_factory(RobotMcpServer, robot="duck2", port=9991)
    own = Mock(remote_name="duck2/duckknowledge", actor_class=DuckKnowledge)
    foreign = Mock(remote_name="duck3/duckknowledge", actor_class=DuckKnowledge)
    operator = Mock(remote_name="duck2/robotmcpserver", actor_class=RobotMcpServer)
    register = Mock()
    monkeypatch.setattr(McpServer, "on_system_modules", register)
    server.on_system_modules([own, foreign, operator])
    register.assert_called_once_with([own])


def test_mcp_calls_target_the_deployed_instance_not_the_shared_class(module_factory, monkeypatch):
    for field, value in (("skills", []), ("skills_by_name", {}), ("rpc_calls", {})):
        monkeypatch.setattr(app.state, field, value)
    server = module_factory(RobotMcpServer, robot="duck2", port=9991)
    info = Mock(func_name="observe", class_name="DuckKnowledge")
    own = Mock(remote_name="duck2/duckknowledge", actor_class=DuckKnowledge)
    own.get_skills.return_value = [info]
    server.on_system_modules([own])
    assert app.state.rpc_calls["observe"].remote_name == "duck2/duckknowledge"
