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

import asyncio
from contextlib import ExitStack
from pathlib import Path

import pytest

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.demo_calculator_skill import DemoCalculatorSkill
from dimos.core.core import rpc
from dimos.core.demos.stress_test_module import StressTestModule
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.porcelain.dimos import Dimos, _resolve_target
from dimos.utils.testing.waiting import wait_until


class TickerModule(Module):
    tick: Out[int]

    async def main(self):
        task = asyncio.create_task(self._publisher())
        try:
            yield
        finally:
            task.cancel()

    async def _publisher(self) -> None:
        i = 1001
        try:
            while True:
                await asyncio.sleep(0.05)
                self.tick.publish(i)
                i += 1
        except asyncio.CancelledError:
            pass


class RecordingSource(Module):
    odom: Out[PoseStamped]

    @rpc
    def emit(self, ts: float) -> None:
        self.odom.publish(PoseStamped(ts=ts))


class RuntimeRecordingSource(Module):
    odom: Out[PoseStamped]
    runtime_pose: Out[PoseStamped]
    ignored: Out[PoseStamped]
    raw: Out[dict]

    @rpc
    def start(self) -> None:
        super().start()
        self.odom.publish(PoseStamped(ts=2.0))
        self.runtime_pose.publish(PoseStamped(ts=3.0))
        self.ignored.publish(PoseStamped(ts=4.0))
        self.raw.publish({"value": 5})


@pytest.fixture(params=["zenoh", "lcm"])
def recording_app(app, tmp_path: Path, monkeypatch, request):
    global_config.update(
        n_workers=1,
        viewer="none",
        transport=request.param,
        record="sqlite",
        record_engine="python",
        replay=False,
    )
    monkeypatch.setattr(global_config, "record_topics", "odom,runtime_*,raw")
    monkeypatch.setattr(tap, "recording_dir", lambda: tmp_path)
    app.run(RecordingSource)
    try:
        yield app
    finally:
        # These parent-side transports outlive the manual recording context.
        for transport in app._coordinator.transports.values():
            transport.stop()


@pytest.mark.parametrize("remote", [False, True])
def test_run_adds_streams_to_active_recording(recording_app, tmp_path: Path, remote) -> None:
    app = recording_app
    coordinator = app._coordinator
    with ExitStack() as stack:
        target = app
        if remote:
            coordinator.start_rpc_service()
            target = Dimos.connect()
            stack.callback(target.stop)
        with tap.recording(coordinator):
            with SqliteStore(path=str(tmp_path / "memory.db"), must_exist=True) as store:
                app.RecordingSource.emit(1.0)
                wait_until(lambda: "odom" in store.list_streams(), timeout=5)
                target.run(RuntimeRecordingSource)
                wait_until(
                    lambda: "runtime_pose" in store.list_streams()
                    and store.stream("odom", PoseStamped).count() >= 2,
                    timeout=5,
                )

    with SqliteStore(path=str(tmp_path / "memory.db"), must_exist=True) as store:
        assert sorted(store.list_streams()) == ["odom", "runtime_pose"]
        assert [o.ts for o in store.stream("odom", PoseStamped)] == [1.0, 2.0]
        assert [o.ts for o in store.stream("runtime_pose", PoseStamped)] == [3.0]


def test_resolve_module_class():
    bp = _resolve_target(StressTestModule)
    assert bp is not None


def test_resolve_blueprint_object():
    bp = StressTestModule.blueprint()
    assert _resolve_target(bp) is bp


def test_resolve_string_name():
    bp = _resolve_target("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_external_string_name_uses_shared_resolver(monkeypatch):
    expected = StressTestModule.blueprint()

    def fake_get_by_name(name: str):
        assert name == "my-test-stack.demo"
        return expected

    monkeypatch.setattr("dimos.porcelain.dimos.get_by_name", fake_get_by_name)

    assert _resolve_target("my-test-stack.demo") is expected


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
        app.skills  # noqa: B018


def test_peek_stream_before_run(app):
    with pytest.raises(RuntimeError, match="No modules are running"):
        app.peek_stream("whatever")


def test_peek_stream_unknown_raises(running_app):
    with pytest.raises(LookupError, match="No running module exposes"):
        running_app.peek_stream("definitely_not_a_stream")


def test_peek_stream_returns_published_value(app):
    app.run(TickerModule)
    value = app.peek_stream("tick", timeout=2.0)
    assert value is not None
    assert isinstance(value, int)
    assert value > 1000


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
        app._nonexistent  # noqa: B018


def test_getattr_unknown_module(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="No module named"):
        app.Nonexistent  # noqa: B018


def test_getattr_exists_but_not_running(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="exists but is not running"):
        app.CameraModule  # noqa: B018


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


def test_module_rpc_call(running_app):
    module = running_app.StressTestModule
    assert module.ping() == "pong"


def test_list_modules_returns_structured_instance_information(running_app):
    modules = running_app.list_modules()

    assert len(modules) == 1
    assert modules[0].instance_name == "StressTestModule"
    assert modules[0].class_name == "StressTestModule"
    assert modules[0].qualified_path.endswith(".StressTestModule")
    assert repr(modules[0]).startswith("<Module StressTestModule")


def test_get_module_supports_public_lookup(running_app):
    assert running_app.get_module("StressTestModule").ping() == "pong"


def test_list_rpcs_includes_skills_and_preserves_metadata(running_app):
    rpcs = running_app.list_rpcs("StressTestModule")
    ping = next(rpc_info for rpc_info in rpcs if rpc_info.name == "ping")
    slow = next(rpc_info for rpc_info in rpcs if rpc_info.name == "slow")

    assert ping.documentation == "Simple health check. Returns 'pong'."
    assert slow.module_name == "StressTestModule"
    assert slow.signature == "(self, seconds: 'float' = 1.0) -> 'str'"
    assert repr(slow) == "<RPC StressTestModule.slow(seconds: float = 1.0) -> str>"


def test_list_rpcs_accepts_module_proxy(running_app):
    proxy = running_app.get_module("StressTestModule")
    assert {rpc_info.name for rpc_info in running_app.list_rpcs(proxy)} >= {"ping", "echo"}


def test_describe_module_and_rpc(running_app):
    module = running_app.get_module("StressTestModule")

    assert running_app.describe(module).instance_name == "StressTestModule"
    assert running_app.describe(module.ping).name == "ping"
    assert running_app.describe("StressTestModule.ping").name == "ping"


def test_describe_rejects_unqualified_rpc_name(running_app):
    with pytest.raises(ValueError, match="not qualified"):
        running_app.describe("ping")


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
    client.run("demo-calculator-skill")
    assert "DemoCalculatorSkill" in client._source.list_module_names()
    assert "DemoCalculatorSkill" in running_app._source.list_module_names()
    assert {module.instance_name for module in client.list_modules()} >= {
        "StressTestModule",
        "DemoCalculatorSkill",
    }


def test_connected_run_by_class_adds_module(client):
    client.run(DemoCalculatorSkill)
    assert "DemoCalculatorSkill" in client._source.list_module_names()


def test_connected_run_by_blueprint_object(client, mcp_port, mcp_url):
    client.run(McpServer.blueprint(host="localhost", port=mcp_port))
    assert "McpServer" in client._source.list_module_names()
    assert client.McpServer.mcp_url() == mcp_url


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
