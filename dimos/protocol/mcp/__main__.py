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

"""CLI entry point for Dimensional MCP Bridge.

Connects Claude Code (or other MCP clients) to a running DimOS agent.

Usage:
    python -m dimos.protocol.mcp  # Bridge to running DimOS on default port
"""

from __future__ import annotations

import asyncio

from dimos.protocol.mcp.bridge import main as bridge_main


def main() -> None:
    """Main entry point - connects to running DimOS via bridge."""
    asyncio.run(bridge_main())


if __name__ == "__main__":
    main()
