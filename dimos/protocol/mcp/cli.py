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

"""CLI entry point for Dimensional MCP Server.

Usage:
    python -m dimos.protocol.mcp.cli           # Standalone stdio server
    python -m dimos.protocol.mcp.cli --bridge  # Bridge to running DimOS
"""

from __future__ import annotations

import asyncio
import sys


def main() -> None:
    """Main entry point for the MCP CLI."""
    if "--bridge" in sys.argv or "-b" in sys.argv:
        from dimos.protocol.mcp.bridge import main as bridge_main

        asyncio.run(bridge_main())
    else:
        asyncio.run(run_server())


async def run_server() -> None:
    """Run standalone MCP server with default skill containers."""
    from dimos.protocol.mcp.server import DimensionalMCPServer

    server = DimensionalMCPServer()

    try:
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server.register_skills(UnitreeSkillContainer())
    except ImportError:
        pass

    await server.run()


if __name__ == "__main__":
    main()
