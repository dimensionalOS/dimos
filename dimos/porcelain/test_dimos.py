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

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain.dimos import Dimos, _resolve_target


def test_resolve_module_class():
    bp = _resolve_target(StressTestModule)
    assert bp is not None


def test_resolve_blueprint_object():
    bp = StressTestModule.blueprint()
    assert _resolve_target(bp) is bp


def test_resolve_string_name():
    bp = _resolve_target("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_unknown_string():
    with pytest.raises(ValueError, match="Unknown"):
        _resolve_target("nonexistent-blueprint-xyz")


def test_resolve_invalid_type():
    with pytest.raises(TypeError, match="run\\(\\) expects"):
        _resolve_target(42)  # type: ignore[arg-type]


def test_default_construction(app):
    assert not app.is_running


def test_construction_with_overrides():
    instance = Dimos(n_workers=4)
    try:
        assert instance._config_overrides == {"n_workers": 4}
    finally:
        instance.stop()


def test_repr_when_stopped(app):
    assert "stopped" in repr(app)


def test_skills_before_run(app):
    with pytest.raises(RuntimeError, match="No modules are running"):
        _ = app.skills


def test_restart_before_run(app):
    with pytest.raises(RuntimeError, match="No modules are running"):
        app.restart(StressTestModule)


def test_stop_is_idempotent(app):
    app.run(StressTestModule)
    app.stop()
    app.stop()  # second stop should not raise


def test_run_after_stop(app):
    app.stop()
    with pytest.raises(RuntimeError, match="stopped"):
        app.run(StressTestModule)


def test_getattr_private_raises(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError):
        _ = app._nonexistent


def test_getattr_unknown_module(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="No module named"):
        _ = app.Nonexistent


def test_getattr_exists_but_not_running(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="exists but is not running"):
        _ = app.CameraModule


def test_run_module_class(app):
    app.run(StressTestModule)
    assert app.is_running
    assert "StressTestModule" in repr(app)
    app.stop()
    assert not app.is_running


def test_run_blueprint_object(app):
    bp = StressTestModule.blueprint()
    app.run(bp)
    assert app.is_running


def test_rpyc_module_access(running_app):
    module = running_app.StressTestModule
    # Access an attribute from ModuleBase
    assert module._module_closed is False


def test_dir_lists_modules(running_app):
    d = dir(running_app)
    assert "StressTestModule" in d
    assert "run" in d
    assert "stop" in d


def test_restart_no_reload(running_app):
    running_app.restart(StressTestModule, reload_source=False)
    result = running_app.skills.ping()
    assert result == "pong"


def test_skills_accessible(running_app):
    skills = running_app.skills
    assert "ping" in dir(skills)


def test_connected_run_by_name_adds_module(running_app, client):
    client.run("mcp-server")
    assert "McpServer" in client._source.list_module_names()
    assert "McpServer" in running_app._source.list_module_names()


def test_connected_run_by_class_adds_module(client):
    client.run(McpServer)
    assert "McpServer" in client._source.list_module_names()


def test_connected_run_by_blueprint_object(client):
    client.run(McpServer.blueprint())
    assert "McpServer" in client._source.list_module_names()


def test_connected_restart_no_reload(client):
    client.restart(StressTestModule, reload_source=False)
    assert client.skills.ping() == "pong"


def test_connected_repr(client):
    r = repr(client)
    assert "remote=True" in r


def test_connected_dir(client):
    d = dir(client)
    assert "StressTestModule" in d


def test_connected_skills(client):
    result = client.skills.ping()
    assert result == "pong"
