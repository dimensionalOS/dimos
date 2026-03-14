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
docker modules to DockerModule WITHOUT actually running Docker.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.docker_runner import DockerModuleConfig, is_docker_module
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import Out

# -- Fixtures: fake module classes -------------------------------------------


class FakeDockerConfig(DockerModuleConfig):
    docker_image: str = "fake:latest"
    docker_file: Path | None = None
    docker_gpus: str | None = None
    docker_rm: bool = True
    docker_restart_policy: str = "no"


class FakeDockerModule(Module["FakeDockerConfig"]):
    default_config = FakeDockerConfig
    output: Out[str]


class FakeRegularModule(Module):
    output: Out[str]


# -- Tests -------------------------------------------------------------------


class TestIsDockerModule:
    def test_docker_module_detected(self):
        assert is_docker_module(FakeDockerModule) is True

    def test_regular_module_not_detected(self):
        assert is_docker_module(FakeRegularModule) is False

    def test_plain_class_not_detected(self):
        assert is_docker_module(str) is False

    def test_no_default_config(self):
        class Bare(Module):
            pass

        # Module has default_config = ModuleConfig, which is not DockerModuleConfig
        assert is_docker_module(Bare) is False


class TestModuleCoordinatorDockerRouting:
    @patch("dimos.core.docker_runner.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_routes_docker_module(self, mock_worker_manager_cls, mock_docker_module_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        mock_dm = MagicMock()
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()

        result = coordinator.deploy(FakeDockerModule)

        # Should NOT go through worker manager
        mock_worker_mgr.deploy.assert_not_called()
        # Should construct a DockerModule (container launch happens inside __init__)
        mock_docker_module_cls.assert_called_once_with(
            FakeDockerModule, global_config=global_config
        )
        # start() is NOT called during deploy — it's called in start_all_modules
        mock_dm.start.assert_not_called()
        assert result is mock_dm
        assert coordinator.get_instance(FakeDockerModule) is mock_dm

        coordinator.stop()

    @patch("dimos.core.docker_runner.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_docker_propagates_constructor_failure(
        self, mock_worker_manager_cls, mock_docker_module_cls
    ):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        # Container launch fails inside __init__; DockerModule handles its own cleanup
        mock_docker_module_cls.side_effect = RuntimeError("launch failed")

        coordinator = ModuleCoordinator()
        coordinator.start()

        with pytest.raises(RuntimeError, match="launch failed"):
            coordinator.deploy(FakeDockerModule)

        coordinator.stop()

    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_routes_regular_module_to_worker_manager(self, mock_worker_manager_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr
        mock_proxy = MagicMock()
        mock_worker_mgr.deploy.return_value = mock_proxy

        coordinator = ModuleCoordinator()
        coordinator.start()

        result = coordinator.deploy(FakeRegularModule)

        mock_worker_mgr.deploy.assert_called_once_with(FakeRegularModule, global_config, {})
        assert result is mock_proxy

        coordinator.stop()

    @patch("dimos.core.docker_worker_manager.DockerWorkerManager.deploy_parallel")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_parallel_separates_docker_and_regular(
        self, mock_worker_manager_cls, mock_docker_deploy
    ):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        regular_proxy = MagicMock()
        mock_worker_mgr.deploy_parallel.return_value = [regular_proxy]

        mock_dm = MagicMock()
        mock_docker_deploy.return_value = [mock_dm]

        coordinator = ModuleCoordinator()
        coordinator.start()

        specs = [
            (FakeRegularModule, (), {}),
            (FakeDockerModule, (), {}),
        ]
        results = coordinator.deploy_parallel(specs)

        # Regular module goes through worker manager
        mock_worker_mgr.deploy_parallel.assert_called_once_with([(FakeRegularModule, (), {})])
        # Docker specs go through DockerWorkerManager
        mock_docker_deploy.assert_called_once_with([(FakeDockerModule, (), {})])
        # start() is NOT called during deploy — it's called in start_all_modules
        mock_dm.start.assert_not_called()

        # Results preserve input order
        assert results[0] is regular_proxy
        assert results[1] is mock_dm

        coordinator.stop()

    @patch("dimos.core.docker_runner.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_stop_cleans_up_docker_modules(self, mock_worker_manager_cls, mock_docker_module_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        mock_dm = MagicMock()
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()
        coordinator.deploy(FakeDockerModule)
        coordinator.stop()

        # stop() called exactly once (no double cleanup)
        assert mock_dm.stop.call_count == 1
        # Worker manager also closed
        mock_worker_mgr.close_all.assert_called_once()


class TestDockerModuleGetattr:
    """Tests for DockerModule.__getattr__ avoiding infinite recursion."""

    def test_getattr_no_recursion_when_rpcs_not_set(self):
        """If __init__ fails before self.rpcs is assigned, __getattr__ must not recurse."""
        from dimos.core.docker_runner import DockerModule

        dm = DockerModule.__new__(DockerModule)
        # Don't set rpcs, _module_class, or any instance attrs — simulates early __init__ failure
        with pytest.raises(AttributeError):
            _ = dm.some_method

    def test_getattr_no_recursion_on_cleanup_attrs(self):
        """Accessing cleanup-related attrs before they exist must raise, not recurse."""
        from dimos.core.docker_runner import DockerModule

        dm = DockerModule.__new__(DockerModule)
        # These are accessed during _cleanup() — if rpcs isn't set, they must not recurse
        for attr in ("rpc", "config", "_container_name", "_unsub_fns"):
            with pytest.raises(AttributeError):
                getattr(dm, attr)

    def test_getattr_delegates_to_rpc_when_rpcs_set(self):
        from dimos.core.docker_runner import DockerModule
        from dimos.core.rpc_client import RpcCall

        dm = DockerModule.__new__(DockerModule)
        dm.rpcs = {"do_thing"}

        # _module_class needs a real method with __name__ for RpcCall
        class FakeMod:
            def do_thing(self) -> None: ...

        dm._module_class = FakeMod
        dm.rpc = MagicMock()
        dm.remote_name = "FakeMod"
        dm._unsub_fns = []
        dm._rpc_timeouts = {}

        result = dm.do_thing
        assert isinstance(result, RpcCall)

    def test_getattr_raises_for_unknown_method(self):
        from dimos.core.docker_runner import DockerModule

        dm = DockerModule.__new__(DockerModule)
        dm.rpcs = {"do_thing"}

        with pytest.raises(AttributeError, match="not found"):
            _ = dm.nonexistent


class TestDockerModuleCleanupReconnect:
    """Tests for DockerModule._cleanup with docker_reconnect_container."""

    def test_cleanup_skips_stop_when_reconnect(self):
        from dimos.core.docker_runner import DockerModule

        with patch.object(DockerModule, "__init__", lambda self: None):
            dm = DockerModule.__new__(DockerModule)
            dm._running = True
            dm._container_name = "test_container"
            dm._unsub_fns = []
            dm.rpc = MagicMock()
            dm.remote_name = "TestModule"

            # reconnect mode: should NOT stop/rm the container
            dm.config = FakeDockerConfig(docker_reconnect_container=True)
            with (
                patch("dimos.core.docker_runner._run") as mock_run,
                patch("dimos.core.docker_runner._remove_container") as mock_rm,
            ):
                dm._cleanup()
                mock_run.assert_not_called()
                mock_rm.assert_not_called()

    def test_cleanup_stops_container_when_not_reconnect(self):
        from dimos.core.docker_runner import DockerModule

        with patch.object(DockerModule, "__init__", lambda self: None):
            dm = DockerModule.__new__(DockerModule)
            dm._running = True
            dm._container_name = "test_container"
            dm._unsub_fns = []
            dm.rpc = MagicMock()
            dm.remote_name = "TestModule"

            # normal mode: should stop and rm the container
            dm.config = FakeDockerConfig(docker_reconnect_container=False)
            with (
                patch("dimos.core.docker_runner._run") as mock_run,
                patch("dimos.core.docker_runner._remove_container") as mock_rm,
            ):
                dm._cleanup()
                mock_run.assert_called_once()  # docker stop
                mock_rm.assert_called_once()  # docker rm -f
