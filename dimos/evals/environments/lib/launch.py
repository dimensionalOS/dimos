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

from collections.abc import Mapping, Sequence
import os
import subprocess
import sys
import time

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.global_config import global_config
from dimos.core.run_registry import RunEntry, list_runs

_REGISTRY_POLL_S = 0.5


def attached_mcp_url() -> str:
    """The MCP endpoint of the newest running dimos, the one an attach drives; "" without one."""
    runs = list_runs(alive_only=True)  # oldest first: run IDs start with a timestamp
    return (runs[-1].mcp_url or "") if runs else ""


def launched_run(process: subprocess.Popen[bytes], deadline: float) -> RunEntry:
    """The run entry a launched ``dimos run`` registered for itself.

    The run entry is written once the blueprint is up, so this also waits out
    startup. Fails when the process exits first or the deadline passes.
    """
    while True:
        entry = next((run for run in list_runs(alive_only=True) if run.pid == process.pid), None)
        if entry is not None:
            return entry
        if process.poll() is not None:
            raise RuntimeError(
                f"dimos exited with code {process.returncode} before registering its run"
            )
        if time.monotonic() >= deadline:
            raise TimeoutError("dimos did not register its run before the launch deadline")
        time.sleep(_REGISTRY_POLL_S)


def launched_mcp_url(process: subprocess.Popen[bytes], deadline: float) -> str:
    """The MCP endpoint a launched ``dimos run`` registered for itself."""
    entry = launched_run(process, deadline)
    if entry.mcp_url is None:
        raise RuntimeError(f"dimos run {entry.run_id} has no McpServer in its blueprint")
    return entry.mcp_url


def wait_connectable(
    deadline: float,
    process: subprocess.Popen[bytes] | None = None,
    client_env: Mapping[str, str] | None = None,
) -> None:
    """Wait until the coordinator answers the ping a ``Dimos.connect()`` client starts with.

    The coordinator answers only after every module has started. ``client_env`` holds the
    variables a client needs to reach this dimos, such as a transport its launch forced.
    """
    while True:
        try:
            _ping(max(1.0, min(5.0, deadline - time.monotonic())), client_env or {})
            return
        except TimeoutError:
            if process is not None and process.poll() is not None:
                raise RuntimeError(
                    f"dimos exited with code {process.returncode} before it could be reached"
                ) from None
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "no dimos coordinator answered before the launch deadline"
                ) from None


def _ping(timeout: float, client_env: Mapping[str, str]) -> None:
    """Ping the coordinator, from a client with ``client_env`` when it changes the transport."""
    transport = client_env.get("DIMOS_TRANSPORT", global_config.transport)
    if transport == global_config.transport:
        CoordinatorRPC.connect(timeout=timeout).stop()
        return
    # The transport is read at import, so ping from a process that starts with it.
    code = (
        "from dimos.core.coordination.coordinator_rpc import CoordinatorRPC; "
        f"CoordinatorRPC.connect(timeout={timeout}).stop()"
    )
    try:
        result = subprocess.run(
            [sys.executable, "-c", code],
            env={**os.environ, **client_env},
            capture_output=True,
            timeout=timeout + 60.0,
        )
    except subprocess.TimeoutExpired as e:
        raise TimeoutError("the coordinator ping did not finish") from e
    if result.returncode:
        lines = result.stderr.decode(errors="replace").strip().splitlines()
        if lines and lines[-1].startswith("TimeoutError"):
            raise TimeoutError(lines[-1])
        raise RuntimeError("the coordinator ping could not run: " + " / ".join(lines[-3:]))


def validate_blueprints(names: Sequence[str]) -> None:
    """Validate the registry names passed to ``dimos run``."""
    # Resolving blueprints loads their optional robot dependencies.
    from dimos.robot.get_all_blueprints import get_by_name

    for name in names:
        get_by_name(name)
