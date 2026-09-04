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

"""E2e regression test for `dimos run --daemon` with deterministic local Zenoh.

Everything runs in subprocesses: the daemon must survive a real double fork
(zenoh's tokio runtime does not survive fork, so the daemon process must not
inherit any zenoh state), and the client must be a genuinely separate process.
This also keeps zenoh threads out of the pytest process.

Multicast is disabled so the coordinator, workers, and external client can
communicate only through the coordinator-owned loopback peer seed and gossip.
This turns the macOS discovery race into a deterministic contract. The
per-worker state directory keeps this run's advertised seed isolated, with
`--dist=loadfile` keeping this file on one xdist worker.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

_CLIENT = """
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

client = CoordinatorRPC.connect(timeout=30)
print(client.call("ping"))
client.stop()
"""


def _sweep_leftover_daemons(state_dir: Path) -> None:
    """Kill any daemon process group a failed run left behind."""
    for entry_path in state_dir.glob("dimos/runs/*.json"):
        try:
            pid = json.loads(entry_path.read_text())["pid"]
        except (json.JSONDecodeError, KeyError, OSError):
            continue
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(pid), sig)
            except (ProcessLookupError, PermissionError):
                break
            time.sleep(1.0)


def test_daemon_serves_coordinator_ping_over_zenoh(tmp_path: Path) -> None:
    env = os.environ | {
        "DIMOS_TRANSPORT": "zenoh",
        # Prove bootstrap and RPC do not depend on multicast discovery.
        "ZENOH_MULTICAST": "0",
        # Never touch the developer's real registry/config, even without xdist.
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "XDG_CONFIG_HOME": str(tmp_path / "config"),
    }
    # File, not pipe: the daemon's forkserver inherits the CLI's stdout/stderr
    # and holds them open forever, so a pipe would block subprocess.run.
    run_log = tmp_path / "run.log"
    try:
        with run_log.open("w") as log:
            run = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "dimos.cli.dimos",
                    "run",
                    "demo-mcp-stress-test",
                    "--daemon",
                    "--disable",
                    "mcp-server",
                    "--viewer",
                    "none",
                    "--n-workers",
                    "1",
                ],
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                timeout=120,
            )
        assert run.returncode == 0, f"dimos run --daemon failed:\n{run_log.read_text()}"

        client = subprocess.run(
            [sys.executable, "-c", _CLIENT],
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert "pong" in client.stdout, (
            f"Coordinator/ping unreachable from an out-of-process zenoh client:\n"
            f"{client.stdout}\n{client.stderr}"
        )
    finally:
        subprocess.run(
            [sys.executable, "-m", "dimos.cli.dimos", "stop"],
            env=env,
            capture_output=True,
            timeout=60,
        )
        _sweep_leftover_daemons(tmp_path / "state")
