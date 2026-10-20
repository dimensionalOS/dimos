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

"""Finding and validating the dimos a live eval case drives."""

from collections.abc import Sequence
import subprocess
import time

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.core.run_registry import list_runs

_REGISTRY_POLL_S = 0.5


def default_mcp_url() -> str:
    """The MCP endpoint of the newest running dimos, for attaching to one."""
    return McpAdapter.from_run_entry().url


def launched_mcp_url(process: subprocess.Popen[bytes], deadline: float) -> str:
    """The MCP endpoint a launched ``dimos run`` registered for itself.

    The run entry is written once the blueprint is up, so this also waits out
    startup. Fails when the process exits first or the deadline passes.
    """
    while True:
        entry = next((run for run in list_runs(alive_only=True) if run.pid == process.pid), None)
        if entry is not None:
            if entry.mcp_url is None:
                raise RuntimeError(f"dimos run {entry.run_id} has no McpServer in its blueprint")
            return entry.mcp_url
        if process.poll() is not None:
            raise RuntimeError(
                f"dimos exited with code {process.returncode} before registering its run"
            )
        if time.monotonic() >= deadline:
            raise TimeoutError("dimos did not register its run before the launch deadline")
        time.sleep(_REGISTRY_POLL_S)


def validate_blueprints(names: Sequence[str]) -> None:
    """Validate the registry names passed to ``dimos run``."""
    # Resolving blueprints loads their optional robot dependencies.
    from dimos.robot.get_all_blueprints import get_by_name

    for name in names:
        get_by_name(name)
