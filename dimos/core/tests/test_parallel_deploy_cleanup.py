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
Tests that deploy_parallel cleans up successfully-started modules when a
sibling deployment fails ("middle module throws" scenario).
"""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.core.worker_manager import WorkerManager
from dimos.core.worker_manager_docker import WorkerManagerDocker
from dimos.utils.safe_thread_map import ExceptionGroup


class TestWorkerManagerDockerPartialFailure:
    """WorkerManagerDocker.deploy_parallel must stop successful containers when one fails."""

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_middle_module_fails_stops_siblings(self, mock_docker_module_cls):
        """Deploy 3 modules where the middle one fails. The other two must be stopped."""
        mod_a = MagicMock(name="ModuleA")
        mod_c = MagicMock(name="ModuleC")

        barrier = threading.Barrier(3, timeout=5)

        def fake_constructor(cls, *args, **kwargs):
            label = cls.__name__
            barrier.wait()
            if label == "B":
                raise RuntimeError("B failed to start")
            return mod_a if label == "A" else mod_c

        mock_docker_module_cls.side_effect = fake_constructor

        FakeA = type("A", (), {})
        FakeB = type("B", (), {})
        FakeC = type("C", (), {})

        with pytest.raises(ExceptionGroup, match="safe_thread_map failed") as exc_info:
            WorkerManagerDocker(g=GlobalConfig()).deploy_parallel(
                [
                    (FakeA, (), {}),
                    (FakeB, (), {}),
                    (FakeC, (), {}),
                ]
            )

        assert len(exc_info.value.exceptions) == 1
        assert "B failed to start" in str(exc_info.value.exceptions[0])

        # Both successful modules must have been stopped exactly once
        mod_a.stop.assert_called_once()
        mod_c.stop.assert_called_once()

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_multiple_failures_raises_exception_group(self, mock_docker_module_cls):
        """Deploy 3 modules where two fail. Should raise ExceptionGroup with both errors."""
        mod_a = MagicMock(name="ModuleA")

        barrier = threading.Barrier(3, timeout=5)

        def fake_constructor(cls, *args, **kwargs):
            label = cls.__name__
            barrier.wait()
            if label == "B":
                raise RuntimeError("B failed")
            if label == "C":
                raise ValueError("C failed")
            return mod_a

        mock_docker_module_cls.side_effect = fake_constructor

        FakeA = type("A", (), {})
        FakeB = type("B", (), {})
        FakeC = type("C", (), {})

        with pytest.raises(ExceptionGroup, match="safe_thread_map failed") as exc_info:
            WorkerManagerDocker(g=GlobalConfig()).deploy_parallel(
                [
                    (FakeA, (), {}),
                    (FakeB, (), {}),
                    (FakeC, (), {}),
                ]
            )

        assert len(exc_info.value.exceptions) == 2
        messages = {str(e) for e in exc_info.value.exceptions}
        assert "B failed" in messages
        assert "C failed" in messages

        # The one successful module must have been stopped
        mod_a.stop.assert_called_once()

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_all_succeed_no_stops(self, mock_docker_module_cls):
        """When all deployments succeed, no modules should be stopped."""
        mocks = [MagicMock(name=f"Mod{i}") for i in range(3)]

        def fake_constructor(cls, *args, **kwargs):
            return mocks[["A", "B", "C"].index(cls.__name__)]

        mock_docker_module_cls.side_effect = fake_constructor

        FakeA = type("A", (), {})
        FakeB = type("B", (), {})
        FakeC = type("C", (), {})

        results = WorkerManagerDocker(g=GlobalConfig()).deploy_parallel(
            [
                (FakeA, (), {}),
                (FakeB, (), {}),
                (FakeC, (), {}),
            ]
        )

        assert len(results) == 3
        for m in mocks:
            m.stop.assert_not_called()

    @patch("dimos.core.docker_module.DockerModuleProxy")
    def test_stop_failure_does_not_mask_deploy_error(self, mock_docker_module_cls):
        """If stop() itself raises during cleanup, the original deploy error still propagates."""
        mod_a = MagicMock(name="ModuleA")
        mod_a.stop.side_effect = OSError("stop failed")

        barrier = threading.Barrier(2, timeout=5)

        def fake_constructor(cls, *args, **kwargs):
            barrier.wait()
            if cls.__name__ == "B":
                raise RuntimeError("B exploded")
            return mod_a

        mock_docker_module_cls.side_effect = fake_constructor

        FakeA = type("A", (), {})
        FakeB = type("B", (), {})

        with pytest.raises(ExceptionGroup, match="safe_thread_map failed"):
            WorkerManagerDocker(g=GlobalConfig()).deploy_parallel(
                [(FakeA, (), {}), (FakeB, (), {})]
            )

        # stop was attempted despite it raising
        mod_a.stop.assert_called_once()


class TestWorkerManagerPartialFailure:
    """WorkerManager.deploy_parallel must shut down workers when a deploy fails."""

    def test_middle_module_fails_cleans_up_siblings(self):
        manager = WorkerManager(g=GlobalConfig(n_workers=2))

        mock_workers = [MagicMock(name=f"Worker{i}") for i in range(2)]
        for w in mock_workers:
            w.module_count = 0
            w.reserve_slot = MagicMock(
                side_effect=lambda w=w: setattr(w, "module_count", w.module_count + 1)
            )

        manager._workers = mock_workers
        manager._started = True

        def fake_deploy_module(module_class, args=(), kwargs=None):
            if module_class.__name__ == "B":
                raise RuntimeError("B failed to deploy")
            return MagicMock(name=f"actor_{module_class.__name__}")

        for w in mock_workers:
            w.deploy_module = fake_deploy_module

        FakeA = type("A", (), {})
        FakeB = type("B", (), {})
        FakeC = type("C", (), {})

        with patch("dimos.core.worker_manager.RPCClient"):
            with pytest.raises(ExceptionGroup, match="safe_thread_map failed"):
                manager.deploy_parallel(
                    [
                        (FakeA, (), {}),
                        (FakeB, (), {}),
                        (FakeC, (), {}),
                    ]
                )

        # Workers must have been shut down
        for w in mock_workers:
            w.stop.assert_called_once()
