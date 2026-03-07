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

"""Generate agent context file for a running DimOS instance (DIM-687).

Writes a markdown file with module diagram, available skills, CLI commands,
and MCP endpoint info — everything an AI agent needs to interact with the
running system.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.core.module_coordinator import ModuleCoordinator

logger = setup_logger()


def generate_context(
    coordinator: ModuleCoordinator,
    blueprint_name: str,
    run_id: str,
    log_dir: Path,
    mcp_port: int = 9990,
) -> Path:
    """Generate a context.md file for agents in the log directory.

    Returns the path to the generated file.
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    context_path = log_dir / "context.md"

    lines = [
        f"# DimOS Agent Context — {blueprint_name}",
        "",
        f"**Run ID:** {run_id}",
        f"**PID:** {os.getpid()}",
        f"**Blueprint:** {blueprint_name}",
        "",
        "## MCP Endpoint",
        "",
        f"- URL: `http://localhost:{mcp_port}/mcp`",
        "- Protocol: JSON-RPC 2.0",
        "- Methods: `initialize`, `tools/list`, `tools/call`, `dimos/status`, `dimos/list_modules`",
        "",
        "## CLI Commands",
        "",
        "```",
        "dimos status              # show running instance",
        "dimos stop                # stop the instance",
        "dimos stop --force        # force kill (SIGKILL)",
        "dimos mcp list-tools      # list available MCP tools",
        "dimos mcp call <tool>     # call a tool",
        "dimos mcp status          # module/skill info via MCP",
        "dimos mcp modules         # module-skill mapping",
        'dimos agent-send "msg"   # send message to running agent',
        "```",
        "",
    ]

    # Module info
    lines.append("## Deployed Modules")
    lines.append("")
    for w in coordinator.workers:
        for name in w.module_names:
            lines.append(f"- {name} (worker {w.worker_id}, pid {w.pid})")
    lines.append("")

    # Skills info (via MCP)
    lines.append("## Available Skills (MCP Tools)")
    lines.append("")
    lines.append("Use `dimos mcp list-tools` for full schema, or call via:")
    lines.append("```")
    curl_example = (
        f'curl -s localhost:{mcp_port}/mcp -d \'{{"jsonrpc":"2.0","id":1,"method":"tools/list"}}\''
    )
    lines.append(curl_example)
    lines.append("```")
    lines.append("")

    # Log info
    lines.append("## Logs")
    lines.append("")
    lines.append(f"- Directory: `{log_dir}`")
    lines.append(f"- Structured log: `{log_dir}/main.jsonl`")
    lines.append("- Format: structlog JSON, one line per event")
    lines.append('- Filter by module: `grep \'"logger":"module_name"\' main.jsonl`')
    lines.append("")

    context_path.write_text("\n".join(lines))
    logger.info("Agent context written", path=str(context_path))
    return context_path
