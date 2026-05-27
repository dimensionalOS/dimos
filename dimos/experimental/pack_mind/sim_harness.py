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

"""Minimal CUDA-free MCP harness for validating the PACK MIND conductor locally.

The full ``unitree-go2-agentic`` stack needs a CUDA GPU (EdgeTAM perception), so
it cannot run on a CPU-only / Apple Silicon dev machine. PACK MIND's milestones
G0 (MCP reachability) and the conductor wire format do NOT need perception — they
need a live MCP server exposing ``speak``. This harness deploys exactly that:
``SpeakSkill`` behind an ``McpServer``, on CPU.

Run::

    uv run python dimos/experimental/pack_mind/sim_harness.py

Then from another shell::

    curl -s -X POST localhost:9990/mcp -H 'content-type: application/json' \\
      -d '{"jsonrpc":"2.0","id":"t","method":"tools/list"}'

or point the conductor at it::

    uv run python dimos/experimental/pack_mind/conductor.py --dog alpha=localhost:9990
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator


def main() -> None:
    blueprint = autoconnect(SpeakSkill.blueprint(), McpServer.blueprint())
    coordinator = ModuleCoordinator.build(blueprint)
    coordinator.loop()


if __name__ == "__main__":
    main()
