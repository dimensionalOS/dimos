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

from contextlib import contextmanager
import importlib

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain.dimos import Dimos
from dimos.porcelain.remote_module_source import RemoteModuleSource, _RemoteProxy


class NamedRemoteModule(Module):
    @rpc
    def ping_name(self) -> str:
        return self.config.instance_name or "default"


@contextmanager
def _remote_source_with_instances(*instance_names: str):
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    coordinator.start()
    try:
        for instance_name in instance_names:
            coordinator.deploy(NamedRemoteModule, instance_name=instance_name)
        coordinator.start_rpc_service()
        source = RemoteModuleSource()
        try:
            yield source
        finally:
            source.close()
    finally:
        coordinator.stop()


def test_connect_no_running_system(tmp_path, monkeypatch):
    import dimos.core.run_registry as run_registry

    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "runs")
    with pytest.raises(RuntimeError, match="No running DimOS instance"):
        Dimos.connect(timeout=0.5)


def test_connect_skill_call(running_app, client):
    assert client.skills.ping() == "pong"
    assert client.skills.echo(message="hello") == "hello"
    client.stop()
    assert running_app.is_running
    assert running_app.skills.ping() == "pong"


def test_connect_rpc_method_call(client):
    module = client.StressTestModule
    assert module.ping() == "pong"


def test_connect_restart_invalidates_cache(client):
    source = client._source
    m_before = source.get_module("StressTestModule")
    client.restart(StressTestModule, reload_source=False)
    m_after = source.get_module("StressTestModule")
    assert m_before is not m_after
    assert client.skills.ping() == "pong"


def test_connect_run_by_name_adds_module(running_app, client):
    client.run("mcp-server")
    assert "McpServer" in client._source.list_module_names()
    assert "McpServer" in running_app._source.list_module_names()


def test_connect_repr_marks_remote(client):
    rep = repr(client)
    assert "remote" in rep
    assert "StressTestModule" in rep


def test_connect_stop_does_not_kill_remote(running_app, client):
    client.stop()
    assert not client.is_running
    assert running_app.is_running
    assert running_app.skills.ping() == "pong"


def test_connect_list_module_names(client):
    names = client._source.list_module_names()
    assert "StressTestModule" in names


def test_connect_get_module_caches(client):
    source = client._source
    m1 = source.get_module("StressTestModule")
    m2 = source.get_module("StressTestModule")
    assert m1 is m2


def test_get_module_class_name_resolves_single_namespaced_instance():
    with _remote_source_with_instances("robot0/namedremotemodule") as source:
        module = source.get_module("NamedRemoteModule")
        assert module.ping_name() == "robot0/namedremotemodule"


def test_get_module_class_name_raises_when_ambiguous():
    with _remote_source_with_instances(
        "robot0/namedremotemodule", "robot1/namedremotemodule"
    ) as source:
        with pytest.raises(ValueError, match="Multiple instances"):
            source.get_module("NamedRemoteModule")

        module = source.get_module("robot1/namedremotemodule")
        assert module.ping_name() == "robot1/namedremotemodule"


def test_remote_proxy_fallback_when_class_unimportable(client, monkeypatch):
    """If `importlib.import_module` raises, get_module returns a names-only proxy."""
    real_import = importlib.import_module

    def fake_import(name: str, *args, **kwargs):  # type: ignore[no-untyped-def]
        if "stress_test_module" in name:
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("dimos.porcelain.remote_module_source.importlib.import_module", fake_import)
    client._source.invalidate("StressTestModule")
    proxy = client._source.get_module("StressTestModule")
    assert isinstance(proxy, _RemoteProxy)
    assert proxy.ping() == "pong"
