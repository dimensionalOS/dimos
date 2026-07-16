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

from __future__ import annotations

from unittest.mock import Mock

from dimos.core.coordination import worker_manager_external_python as external_manager
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.external_python_module import ExternalPythonModule
from dimos.core.global_config import global_config


class _ExternalDeclaration(ExternalPythonModule):
    implementation = "test_runtime.module.Runtime"


def test_coordinator_registers_mixed_managers() -> None:
    coordinator = ModuleCoordinator(global_config)

    assert set(coordinator._managers) == {"python", "external-python"}
    assert coordinator._managers["external-python"].health_check()
    assert coordinator._managers["python"].health_check()


def test_coordinator_keeps_blueprint_override_kwargs_for_restart() -> None:
    coordinator = ModuleCoordinator(global_config)
    manager = Mock()
    manager.deploy_parallel.return_value = [Mock()]
    coordinator._managers = {"external-python": manager}

    coordinator.deploy_parallel(
        [(_ExternalDeclaration, global_config, {"declared": "value"})],
        {_ExternalDeclaration.name: {"override": "value"}},
    )

    assert coordinator._deployed_kwargs[_ExternalDeclaration] == {
        "declared": "value",
        "override": "value",
    }


def test_external_manager_dispatches_and_undeploys(monkeypatch) -> None:
    worker = Mock()
    worker.pid = 123
    proxy = Mock()
    monkeypatch.setattr(external_manager, "ExternalPythonWorker", Mock(return_value=worker))
    monkeypatch.setattr(external_manager.RPCClient, "remote", Mock(return_value=proxy))

    manager = external_manager.WorkerManagerExternalPython(global_config)
    deployed = manager.deploy(_ExternalDeclaration, global_config, {})

    assert deployed is proxy
    worker.start.assert_called_once()
    assert manager.health_check()
    manager.undeploy(proxy)
    worker.stop.assert_called_once()
    assert manager.health_check()


def test_external_manager_passes_configuration_to_child_constructor(monkeypatch) -> None:
    worker = Mock(pid=123)
    proxy = Mock()
    worker_factory = Mock(return_value=worker)
    monkeypatch.setattr(external_manager, "ExternalPythonWorker", worker_factory)
    monkeypatch.setattr(external_manager.RPCClient, "remote", Mock(return_value=proxy))

    manager = external_manager.WorkerManagerExternalPython(global_config)
    manager.deploy(_ExternalDeclaration, global_config, {"setting": object()})

    constructor_kwargs = worker_factory.call_args.args[1]
    assert constructor_kwargs["setting"] is not None
    assert constructor_kwargs["g"] is global_config


def test_external_manager_cleans_up_worker_when_start_fails(monkeypatch) -> None:
    worker = Mock()
    worker.start.side_effect = RuntimeError("startup failed")
    monkeypatch.setattr(external_manager, "ExternalPythonWorker", Mock(return_value=worker))

    manager = external_manager.WorkerManagerExternalPython(global_config)
    try:
        manager.deploy(_ExternalDeclaration, global_config, {})
    except RuntimeError as error:
        assert str(error) == "startup failed"
    else:
        raise AssertionError("deploy should propagate startup failure")

    worker.stop.assert_called_once()


def test_external_manager_deploy_fresh_uses_a_new_worker(monkeypatch) -> None:
    workers = [Mock(pid=1), Mock(pid=2)]
    proxies = [Mock(), Mock()]
    monkeypatch.setattr(
        external_manager,
        "ExternalPythonWorker",
        Mock(side_effect=workers),
    )
    monkeypatch.setattr(external_manager.RPCClient, "remote", Mock(side_effect=proxies))

    manager = external_manager.WorkerManagerExternalPython(global_config)
    manager.deploy(_ExternalDeclaration, global_config, {})
    manager.deploy_fresh(_ExternalDeclaration, global_config, {})

    assert workers[0].start.call_count == 1
    assert workers[1].start.call_count == 1
    manager.stop()
    workers[0].stop.assert_called_once()
    workers[1].stop.assert_called_once()


def test_external_manager_reports_failed_worker_diagnostics(monkeypatch) -> None:
    worker = Mock(pid=None, declaration=_ExternalDeclaration)
    worker.diagnostics.return_value = "x" * 10000
    proxy = Mock()
    monkeypatch.setattr(external_manager, "ExternalPythonWorker", Mock(return_value=worker))
    monkeypatch.setattr(external_manager.RPCClient, "remote", Mock(return_value=proxy))

    manager = external_manager.WorkerManagerExternalPython(global_config)
    manager.deploy(_ExternalDeclaration, global_config, {})

    assert not manager.health_check()
    worker.diagnostics.assert_called_once()
