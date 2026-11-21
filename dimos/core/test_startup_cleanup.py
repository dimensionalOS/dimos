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

from collections.abc import Iterator
from pathlib import Path
import subprocess
import sys
import uuid

import pytest

from dimos.core.coordination.process_lifecycle import is_pid_alive, kill_run_processes
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def startup_run_id() -> Iterator[str]:
    run_id = f"startup-cleanup-{uuid.uuid4().hex}"
    yield run_id
    kill_run_processes(run_id)


@pytest.mark.parametrize("failure_stage", ["build", "health", "rpc"])
def test_failed_start_cleans_up_detached_viewer(
    tmp_path: Path, startup_run_id: str, failure_stage: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Select the native launch path for the fake viewer even on headless runners.
    monkeypatch.delenv("WAYLAND_SOCKET", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    monkeypatch.setenv("DISPLAY", ":0")
    viewer_pid_file = tmp_path / "viewer.pid"
    viewer = tmp_path / "dimos-viewer"
    viewer.write_text(
        f"#!{sys.executable}\n"
        "import os, signal\n"
        "from pathlib import Path\n"
        f"Path({str(viewer_pid_file)!r}).write_text(str(os.getpid()))\n"
        "signal.pause()\n"
    )
    viewer.chmod(0o755)
    code = """
from pathlib import Path
import sys
from unittest.mock import Mock, patch

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.daemon import run_coordinator
from dimos.utils.testing.waiting import wait_until
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT
from dimos.visualization.rerun.init import spawn_viewer

viewer, pid_file, run_id, failure_stage = sys.argv[1:]
coordinator = Mock()
coordinator.health_check.return_value = failure_stage != "health"
coordinator.start_rpc_service.side_effect = RuntimeError("RPC startup failed")

def build(*args, **kwargs):
    with patch("dimos.visualization.rerun.init.find_executable", return_value=viewer):
        assert spawn_viewer(f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy", "10%")
    # The viewer creates the file before it writes its PID to it.
    wait_until(lambda: Path(pid_file).exists() and Path(pid_file).read_text(), timeout=5)
    if failure_stage == "build":
        raise TimeoutError("Module startup failed")
    return coordinator

with patch("dimos.core.coordination.module_coordinator.ModuleCoordinator.build", side_effect=build):
    run_coordinator(autoconnect().solidify(), None, run_id=run_id, log_dir=Path(pid_file).parent)
"""
    output = tmp_path / "startup.log"
    with output.open("w") as log:
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                code,
                str(viewer),
                str(viewer_pid_file),
                startup_run_id,
                failure_stage,
            ],
            stdout=log,
            stderr=log,
            timeout=20,
        )

    assert result.returncode == 1, output.read_text()
    expected_error = {
        "build": "Module startup failed",
        "health": "health check failed",
        "rpc": "RPC startup failed",
    }[failure_stage]
    assert expected_error in output.read_text()
    assert viewer_pid_file.exists(), output.read_text()
    viewer_pid = int(viewer_pid_file.read_text())
    wait_until(
        lambda: not is_pid_alive(viewer_pid),
        timeout=10,
        message="Failed startup left its detached viewer running",
    )
    assert not is_pid_alive(viewer_pid)
