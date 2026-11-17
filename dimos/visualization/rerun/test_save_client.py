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

"""Save-client shutdown from workers that ignore terminal interrupts."""

import os
import signal
import sys
import time

import pytest
import rerun as rr

from dimos.constants import WORKER_SHUTDOWN_TIMEOUT
from dimos.utils.testing.waiting import wait_until
from dimos.visualization.rerun import bridge


@pytest.fixture(params=[False, True], ids=["direct", "wrapper"])
def save_client(monkeypatch, mocker, tmp_path, request):
    cli = tmp_path / "rerun"
    cli.write_text(
        f"#!{sys.executable}\n"
        "import pathlib, signal, sys\n"
        "path = pathlib.Path(sys.argv[sys.argv.index('--save') + 1])\n"
        "try:\n"
        "    path.write_text('ready')\n"
        "    signal.pause()\n"
        "except KeyboardInterrupt:\n"
        "    path.write_text('flushed')\n"
    )
    cli.chmod(0o755)
    if request.param:
        wrapper = tmp_path / "rerun-wrapper"
        wrapper.write_text(
            f"#!{sys.executable}\n"
            "import signal, subprocess, sys\n"
            "signal.signal(signal.SIGINT, lambda *_: None)\n"
            f"sys.exit(subprocess.call([{str(cli)!r}, *sys.argv[1:]]))\n"
        )
        wrapper.chmod(0o755)
        cli = wrapper
    monkeypatch.setattr(bridge, "find_executable", lambda name: str(cli))
    monkeypatch.setattr(bridge, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setenv("DIMOS_RUN_ID", "test-run")
    recording = mocker.Mock()
    mocker.patch.object(rr, "get_global_data_recording", return_value=recording)
    module = bridge.RerunBridgeModule()
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    client = None
    try:
        client = bridge._start_save_client("unused-test-uri")
        assert client is not None
        module._save_client = client
        path = tmp_path / "test-run" / "rerun.rrd"
        # The client creates the file before it writes to it.
        wait_until(lambda: path.exists() and path.read_text() == "ready", timeout=5, interval=0.01)
        yield module, client, path, recording
    finally:
        signal.signal(signal.SIGINT, previous)
        if client is not None and client.poll() is None:
            os.killpg(client.pid, signal.SIGKILL)
            client.wait(timeout=5)
        module.stop()


def test_save_client_flushes_and_exits_before_worker_timeout(save_client):
    module, client, path, recording = save_client

    started = time.monotonic()
    module.stop()

    assert path.read_text() == "flushed"
    assert client.returncode == 0
    assert time.monotonic() - started < WORKER_SHUTDOWN_TIMEOUT
    assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
    recording.flush.assert_called_once_with()
    # A repeated stop must not signal a PID that could have been reused.
    module.stop()
    recording.flush.assert_called_once_with()
