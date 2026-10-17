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

import pytest
import rerun as rr

from dimos.core.global_config import GlobalConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import Config, _resolve_pubsubs
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
    def test_custom_viewer_connects_without_sdk_version_check(self, mocker, monkeypatch):
        monkeypatch.delenv("RUST_LOG", raising=False)
        reaped = threading.Event()
        process = mocker.Mock(spec=subprocess.Popen)
        process.wait.side_effect = reaped.set
        launch = mocker.patch.object(subprocess, "Popen", return_value=process)
        sdk_spawn = mocker.patch.object(rr, "spawn")
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert spawn_viewer(server_uri, "25%")
        assert reaped.wait(timeout=2)

        launch.assert_called_once_with(
            [
                "dimos-viewer",
                "--connect",
                server_uri,
                "--memory-limit",
                "25%",
                "--expect-data-soon",
            ],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "RUST_LOG": _VIEWER_RUST_LOG},
        )
        sdk_spawn.assert_not_called()
        assert "RUST_LOG" not in os.environ

    @pytest.mark.parametrize("error", [FileNotFoundError, PermissionError])
    def test_custom_viewer_launch_failure_uses_stock_rerun(self, mocker, error):
        mocker.patch.object(subprocess, "Popen", side_effect=error)
        sdk_spawn = mocker.patch.object(rr, "spawn")
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert spawn_viewer(server_uri, "25%")

        sdk_spawn.assert_called_once_with(connect=True, memory_limit="25%")

    def test_unavailable_viewers_allow_headless_operation(self, mocker):
        mocker.patch.object(subprocess, "Popen", side_effect=FileNotFoundError)
        mocker.patch.object(rr, "spawn", side_effect=RuntimeError("No display"))
        server_uri = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

        assert not spawn_viewer(server_uri, "25%")


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
