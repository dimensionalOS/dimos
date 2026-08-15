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

"""Tests for dimos-viewer integration with RerunBridgeModule.

These tests verify that:
1. The dimos-viewer binary is installed and discoverable
2. The bridge owns the spawned viewer process
3. The stock Rerun viewer remains a fallback

These run in CI where dimos-viewer is a core dependency, so the binary
is always available. Process ownership lets topology replacement release
the old viewer's decoded static meshes.
"""

import os
import shutil
import subprocess

from dimos.core.global_config import GlobalConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule, _resolve_pubsubs


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


class TestBridgeViewerLifecycle:
    def test_stop_owned_viewer_terminates_and_reaps_process(self, mocker):
        bridge = RerunBridgeModule()
        process = mocker.Mock()
        process.poll.return_value = None
        bridge._viewer_process = process

        try:
            bridge._stop_native_viewer()
        finally:
            bridge.stop()

        assert bridge._viewer_process is None
        process.terminate.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=5.0)
        process.kill.assert_not_called()

    def test_spawn_keeps_owned_dimos_viewer_process(self, mocker):
        bridge = RerunBridgeModule(memory_limit="2GB")
        process = mocker.Mock()
        popen = mocker.patch(
            "dimos.visualization.rerun.bridge.subprocess.Popen",
            return_value=process,
        )
        mocker.patch(
            "dimos.visualization.rerun.bridge.shutil.which",
            side_effect=lambda name: "/venv/bin/dimos-viewer" if name == "dimos-viewer" else None,
        )

        try:
            spawned = bridge._spawn_native_viewer("rerun+http://127.0.0.1:9877/proxy")
            assert bridge._viewer_process is process
        finally:
            bridge._viewer_process = None
            bridge.stop()

        assert spawned is True
        popen.assert_called_once_with(
            [
                "/venv/bin/dimos-viewer",
                "--connect",
                "rerun+http://127.0.0.1:9877/proxy",
                "--memory-limit",
                "2GB",
                "--expect-data-soon",
                "--ws-url",
                "ws://127.0.0.1:3030/ws",
            ],
            stdin=subprocess.DEVNULL,
        )

    def test_spawn_falls_back_to_stock_rerun(self, mocker):
        bridge = RerunBridgeModule()
        process = mocker.Mock()
        popen = mocker.patch(
            "dimos.visualization.rerun.bridge.subprocess.Popen",
            return_value=process,
        )
        mocker.patch(
            "dimos.visualization.rerun.bridge.shutil.which",
            side_effect=lambda name: "/venv/bin/rerun" if name == "rerun" else None,
        )

        try:
            spawned = bridge._spawn_native_viewer("rerun+http://127.0.0.1:9877/proxy")
        finally:
            bridge._viewer_process = None
            bridge.stop()

        assert spawned is True
        args = popen.call_args.args[0]
        assert args[0] == "/venv/bin/rerun"
        assert "--ws-url" not in args


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
