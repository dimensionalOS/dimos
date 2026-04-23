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

import pytest

from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain.dimos import Dimos


def test_connect_no_running_system(tmp_path, monkeypatch):
    import dimos.core.run_registry as run_registry

    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "runs")
    with pytest.raises(RuntimeError, match="No running DimOS instance"):
        Dimos.connect()


def test_connect_via_host_port_skill_call(running_app, client):
    assert client.skills.ping() == "pong"
    assert client.skills.echo(message="hello") == "hello"
    client.stop()
    assert running_app.is_running
    assert running_app.skills.ping() == "pong"


def test_connect_attribute_access(client):
    module = client.StressTestModule
    assert module._module_closed is False


@pytest.mark.slow
def test_connect_restart_invalidates_cache(client):
    source = client._source
    m_before = source.get_rpyc_module("StressTestModule")
    client.restart(StressTestModule, reload_source=False)
    m_after = source.get_rpyc_module("StressTestModule")
    assert m_before is not m_after
    assert client.skills.ping() == "pong"


@pytest.mark.slow
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


def test_connect_get_rpyc_module_caches(client):
    source = client._source
    m1 = source.get_rpyc_module("StressTestModule")
    m2 = source.get_rpyc_module("StressTestModule")
    assert m1 is m2
