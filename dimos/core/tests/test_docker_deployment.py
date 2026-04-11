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

"""
Smoke tests for Docker module deployment routing.

These tests verify that the ModuleCoordinator correctly detects and routes
docker modules to DockerModuleProxy WITHOUT actually running Docker.
"""

from __future__ import annotations

from pathlib import Path
import threading
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.docker_module import DockerModuleConfig, DockerModuleProxy
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import Out

# -- Fixtures -----------------------------------------------------------------


class FakeDockerConfig(DockerModuleConfig):
    docker_image: str = "fake:latest"
    docker_file: Path | None = None
    docker_gpus: str | None = None
    docker_rm: bool = True
    docker_restart_policy: str = "no"


class FakeDockerModule(Module):
    config: "FakeDockerConfig"
    deployment = "docker"
    output: Out[str]


class FakeRegularModule(Module):
    output: Out[str]


# -- Tests -------------------------------------------------------------------


class TestDeploymentClassVar:
    def test_docker_module_has_docker_deployment(self):
        assert FakeDockerModule.deployment == "docker"

    def test_regular_module_has_python_deployment(self):
        assert FakeRegularModule.deployment == "python"

    def test_bare_module_has_python_deployment(self):
        class Bare(Module):
            pass

        assert Bare.deployment == "python"


class TestModuleCoordinatorDockerRouting:
    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_deploy_routes_docker_module(self, mock_proxy_cls, dimos_cluster):
        mock_dm = MagicMock()
        mock_proxy_cls.return_value = mock_dm

        result = dimos_cluster.deploy(FakeDockerModule)

        mock_proxy_cls.assert_called_once()
        assert result is mock_dm
        assert dimos_cluster.get_instance(FakeDockerModule) is mock_dm

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_deploy_docker_propagates_failure(self, mock_proxy_cls, dimos_cluster):
        mock_proxy_cls.side_effect = RuntimeError("launch failed")

        with pytest.raises(RuntimeError, match="launch failed"):
            dimos_cluster.deploy(FakeDockerModule)

    def test_deploy_routes_regular_module_not_to_docker(self, dimos_cluster):
        # Regular modules should not go through DockerModuleProxy
        assert FakeRegularModule.deployment == "python"

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_deploy_parallel_deploys_docker_module(self, mock_proxy_cls, dimos_cluster):
        mock_dm = MagicMock()
        mock_proxy_cls.return_value = mock_dm

        specs = [
            (FakeDockerModule, (), {}),
        ]
        results = dimos_cluster.deploy_parallel(specs, {})

        mock_proxy_cls.assert_called_once()
        assert results[0] is mock_dm

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_stop_cleans_up_docker_modules(self, mock_proxy_cls, dimos_cluster):
        mock_dm = MagicMock()
        mock_proxy_cls.return_value = mock_dm

        dimos_cluster.deploy(FakeDockerModule)
        dimos_cluster.stop()

        # stop() is called at least once (fixture teardown may call it again)
        mock_dm.stop.assert_called()


class TestDockerModuleProxyGetattr:
    """Tests for DockerModuleProxy.__getattr__ avoiding infinite recursion."""

    def test_getattr_no_recursion_when_rpcs_not_set(self):
        """If __init__ fails before self.rpcs is assigned, __getattr__ must not recurse."""

        dm = DockerModuleProxy.__new__(DockerModuleProxy)
        # Don't set rpcs, _module_class, or any instance attrs — simulates early __init__ failure
        with pytest.raises(AttributeError):
            _ = dm.some_method

    def test_getattr_no_recursion_on_cleanup_attrs(self):
        """Accessing cleanup-related attrs before they exist must raise, not recurse."""

        dm = DockerModuleProxy.__new__(DockerModuleProxy)
        # These are accessed during _cleanup() — if rpcs isn't set, they must not recurse
        for attr in ("rpc", "config", "_container_name", "_unsub_fns"):
            with pytest.raises(AttributeError):
                getattr(dm, attr)

    def test_getattr_delegates_to_rpc_when_rpcs_set(self):
        dm = DockerModuleProxy.__new__(DockerModuleProxy)
        dm.rpcs = {"do_thing"}

        # _module_class needs a real method with __name__ for RpcCall
        class FakeMod:
            def do_thing(self) -> None: ...

        dm._module_class = FakeMod
        dm.rpc = MagicMock()
        dm.remote_name = "FakeMod"
        dm._unsub_fns = []

        result = dm.do_thing
        assert isinstance(result, RpcCall)

    def test_getattr_raises_for_unknown_method(self):
        dm = DockerModuleProxy.__new__(DockerModuleProxy)
        dm.rpcs = {"do_thing"}

        with pytest.raises(AttributeError, match="not found"):
            _ = dm.nonexistent


class TestDockerModuleProxyCleanupReconnect:
    """Tests for DockerModuleProxy._cleanup with docker_reconnect_container."""

    def test_cleanup_skips_stop_when_reconnected(self):
        with patch.object(DockerModuleProxy, "__init__", lambda self: None):
            dm = DockerModuleProxy.__new__(DockerModuleProxy)
            dm._running = threading.Event()
            dm._running.set()
            dm._container_name = "test_container"
            dm._unsub_fns = []
            dm.rpc = MagicMock()
            dm.remote_name = "TestModule"
            dm._reconnected = True

            dm.config = FakeDockerConfig(docker_reconnect_container=True)
            with (
                patch("dimos.core.docker_module._run") as mock_run,
                patch("dimos.core.docker_module._remove_container") as mock_rm,
            ):
                dm._cleanup()
                mock_run.assert_not_called()
                mock_rm.assert_not_called()

    def test_cleanup_stops_container_when_not_reconnected(self):
        with patch.object(DockerModuleProxy, "__init__", lambda self: None):
            dm = DockerModuleProxy.__new__(DockerModuleProxy)
            dm._running = threading.Event()
            dm._running.set()
            dm._container_name = "test_container"
            dm._unsub_fns = []
            dm.rpc = MagicMock()
            dm.remote_name = "TestModule"
            dm._reconnected = False

            dm.config = FakeDockerConfig(docker_reconnect_container=False)
            with (
                patch("dimos.core.docker_module._run") as mock_run,
                patch("dimos.core.docker_module._remove_container") as mock_rm,
            ):
                dm._cleanup()
                mock_run.assert_called_once()  # docker stop
                mock_rm.assert_called_once()  # docker rm -f

    def test_stop_skips_remote_rpc_when_reconnected(self):
        """stop() should not send the remote stop RPC for a reconnected container."""
        with patch.object(DockerModuleProxy, "__init__", lambda self: None):
            dm = DockerModuleProxy.__new__(DockerModuleProxy)
            dm._running = threading.Event()
            dm._running.set()
            dm._container_name = "test_container"
            dm._unsub_fns = []
            dm.rpc = MagicMock()
            dm.remote_name = "TestModule"
            dm._reconnected = True
            dm.config = FakeDockerConfig(docker_reconnect_container=True)

            with (
                patch("dimos.core.docker_module._run"),
                patch("dimos.core.docker_module._remove_container"),
            ):
                dm.stop()
                dm.rpc.call_nowait.assert_not_called()
