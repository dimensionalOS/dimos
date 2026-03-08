# Copyright 2025-2026 Dimensional Inc.
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

"""Stress test blueprint: StressTestModule + McpServer.

Lightweight, no hardware. Used for e2e daemon/MCP lifecycle testing.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.blueprints import autoconnect
from dimos.core.tests.stress_test_module import StressTestModule

demo_mcp_stress_test = autoconnect(
    StressTestModule.blueprint(),
    McpServer.blueprint(),
)

__all__ = ["demo_mcp_stress_test"]
