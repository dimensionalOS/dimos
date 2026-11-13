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

"""Custom viewer launch, stock Rerun fallback, and bridge integration."""

import os
import shutil
import subprocess
import threading
from unittest.mock import ANY

import pytest
import rerun as rr

from dimos.core.global_config import GlobalConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun import bridge
from dimos.visualization.rerun.bridge import Config, _resolve_pubsubs, _start_save_client
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT
from dimos.visualization.rerun.init import _VIEWER_RUST_LOG, spawn_viewer


class TestViewerBinaryInstallation:
    """Verify dimos-viewer binary is installed and functional."""

    def test_binary_on_path(self):
        """dimos-viewer binary must be discoverable on PATH."""
        path = shutil.which("dimos-viewer")
        assert path is not None, (
            "dimos-viewer binary not found on PATH. "
            "Ensure 'dimos-viewer' is in pyproject.toml dependencies."
        )

    def test_binary_executable(self):
        """dimos-viewer binary must be executable."""
        path = shutil.which("dimos-viewer")
        assert path is not None
        assert os.access(path, os.X_OK), f"dimos-viewer at {path} is not executable"


class TestViewerLaunch:
    @pytest.mark.parametrize("beside_interpreter", [True, False])
    def test_custom_viewer_connects_without_sdk_version_check(
        self, mocker, monkeypatch, tmp_path, beside_interpreter
    ):
        monkeypatch.delenv("RUST_LOG", raising=False)
        monkeypatch.setenv("PATH", "")
        # The viewer beside the interpreter is launched, whatever PATH holds.
        # With none there, the bare name is left to PATH.
        viewer = tmp_path / "dimos-viewer"
        if beside_interpreter:
            viewer.touch()
            viewer.chmod(0o755)
        monkeypatch.setattr("sys.executable", str(tmp_path / "python"))
        reaped = threading.Event()
        process = mocker.Mock(spec=subprocess.Popen)
        process.wait.side_effect = reaped.set
        launch = mocker.patch.object(subprocess, "Popen", return_value=process)
        sdk_spawn = mocker.patch.object(rr, "spawn")
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert spawn_viewer(server_uri, "25%", "ws://localhost:4321/ws")
        assert reaped.wait(timeout=2)

        launch.assert_called_once_with(
            [
                str(viewer) if beside_interpreter else "dimos-viewer",
                "--connect",
                server_uri,
                "--ws-url",
                "ws://localhost:4321/ws",
                "--memory-limit",
                "25%",
                "--expect-data-soon",
            ],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env=ANY,
        )
        assert launch.call_args.kwargs["env"]["RUST_LOG"] == _VIEWER_RUST_LOG
        sdk_spawn.assert_not_called()
        assert "RUST_LOG" not in os.environ

    @pytest.mark.parametrize("error", [FileNotFoundError, PermissionError])
    def test_custom_viewer_launch_failure_uses_stock_rerun(
        self, mocker, monkeypatch, tmp_path, error
    ):
        rerun = tmp_path / "rerun"
        rerun.touch()
        rerun.chmod(0o755)
        monkeypatch.setattr("sys.executable", str(tmp_path / "python"))
        monkeypatch.setenv("PATH", "")
        mocker.patch.object(subprocess, "Popen", side_effect=error)
        sdk_spawn = mocker.patch.object(rr, "spawn")
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert spawn_viewer(server_uri, "25%")

        sdk_spawn.assert_called_once_with(
            connect=True, memory_limit="25%", executable_path=str(rerun)
        )

    def test_unavailable_viewers_allow_headless_operation(self, mocker):
        mocker.patch.object(subprocess, "Popen", side_effect=FileNotFoundError)
        mocker.patch.object(rr, "spawn", side_effect=RuntimeError("No display"))
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert not spawn_viewer(server_uri, "25%")


def test_save_client_launches_without_venv_on_path(mocker, monkeypatch, tmp_path):
    rerun = tmp_path / "rerun"
    rerun.touch()
    rerun.chmod(0o755)
    monkeypatch.setattr("sys.executable", str(tmp_path / "python"))
    monkeypatch.setenv("PATH", "")
    monkeypatch.setenv("DIMOS_RUN_ID", "test-run")
    monkeypatch.setattr(bridge, "RECORDINGS_DIR", tmp_path / "recordings")
    launch = mocker.patch.object(subprocess, "Popen")
    server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

    assert _start_save_client(server_uri) is launch.return_value

    output = tmp_path / "recordings" / "test-run" / "rerun.rrd"
    launch.assert_called_once_with(
        [str(rerun), "--save", str(output), "--bind", "127.0.0.1", "--port", "auto", server_uri],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    assert output.parent.is_dir()


def test_save_client_without_rerun_skips_recording(mocker, monkeypatch, tmp_path):
    monkeypatch.setattr("sys.executable", str(tmp_path / "python"))
    monkeypatch.setenv("PATH", str(tmp_path))
    launch = mocker.patch.object(subprocess, "Popen")

    assert _start_save_client(f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy") is None

    launch.assert_not_called()


class ExplicitPubSubOverride:
    def subscribe_all(self, callback):
        return lambda: None


class TestBridgePubsubResolution:
    def test_legacy_lcm_pubsubs_defers_to_transport_default(self):
        config = Config(pubsubs=[LCM()], g=GlobalConfig(transport="lcm"))
        pubsubs = _resolve_pubsubs(config)

        assert len(pubsubs) == 1
        assert isinstance(pubsubs[0], LCM)

    def test_explicit_custom_pubsubs_override_is_honored(self):
        custom = ExplicitPubSubOverride()
        config = Config(pubsubs=[custom], g=GlobalConfig(transport="lcm"))
        pubsubs = _resolve_pubsubs(config)

        assert pubsubs == [custom]
