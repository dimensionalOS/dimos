# Copyright 2025 Dimensional Inc.
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

import asyncio
import importlib
import json
import logging
import sys
import threading

from aiohttp.test_utils import make_mocked_request
import psutil
import pytest


@pytest.fixture
def dashboard_server(monkeypatch):
    # Avoid psutil permission issues when importing the dashboard module in test environments.
    monkeypatch.setattr(psutil.Process, "parents", lambda self: [])

    # Force a clean import so patched psutil behavior is used.
    for name in ("dimos.dashboard.server", "dimos.dashboard.module", "dimos.dashboard"):
        sys.modules.pop(name, None)

    return importlib.import_module("dimos.dashboard.server")


def test_thread_passes_config_and_daemon_flag(monkeypatch, dashboard_server):
    captured: dict = {}

    def fake_start_dashboard_server(config, log):
        captured["config"] = config
        captured["thread"] = threading.current_thread()

    monkeypatch.setattr(dashboard_server, "start_dashboard_server", fake_start_dashboard_server)

    logger = logging.getLogger("test-thread-config")
    thread = dashboard_server.start_dashboard_server_thread(
        auto_open=True,
        port=5555,
        dashboard_host="0.0.0.0",
        terminal_commands={"shell": "echo hi"},
        https_enabled=True,
        zellij_host="10.0.0.2",
        zellij_port=8765,
        zellij_url="http://example.com:1111",
        zellij_session_name="custom-session",
        https_key_path="/tmp/key.pem",
        https_cert_path="/tmp/cert.pem",
        logger=logger,
        rrd_url="rrd+tcp://localhost:2222",
        keep_alive=True,
        zellij_token="token-123",
    )
    thread.join(timeout=1)
    assert not thread.is_alive()
    assert captured["thread"] == thread

    config = captured["config"]
    assert config["port"] == 5555
    assert config["dashboard_host"] == "0.0.0.0"
    assert config["zellij_port"] == 8765
    assert config["zellij_url"] == "http://example.com:1111"
    assert config["zellij_session_name"] == "custom-session"
    assert config["https_enabled"] is True
    assert config["https_key_path"] == "/tmp/key.pem"
    assert config["https_cert_path"] == "/tmp/cert.pem"
    assert config["protocol"] == "https"
    assert config["rrd_url"] == "rrd+tcp://localhost:2222"
    assert config["zellij_token"] == "token-123"
    assert config["terminals"] == {"shell": "echo hi"}
    assert thread.daemon is False
    assert thread.name == "proxy-server"


def test_default_zellij_url_and_daemon(monkeypatch, dashboard_server):
    captured: dict = {}

    def fake_start_dashboard_server(config, log):
        captured.update(config=config, thread=threading.current_thread())

    monkeypatch.setattr(dashboard_server, "start_dashboard_server", fake_start_dashboard_server)

    thread = dashboard_server.start_dashboard_server_thread(
        port=1234,
        zellij_port=9999,
        zellij_host="192.168.1.5",
        https_enabled=False,
    )
    thread.join(timeout=1)
    assert not thread.is_alive()

    config = captured["config"]
    assert config["zellij_url"] == "http://192.168.1.5:9999"
    assert config["protocol"] == "http"
    assert config["port"] == 1234
    assert config["https_enabled"] is False
    assert captured["thread"].daemon is True


@pytest.mark.parametrize("zellij_enabled", [True, False])
def test_serves_html_and_tracks_zellij_availability(monkeypatch, zellij_enabled, dashboard_server):
    responses: dict = {}

    class FakeZellijManager:
        def __init__(self, *, log, session_name, port, terminal_commands, token):
            self.log = log
            self.session_name = session_name
            self.port = port
            self.token = token
            self.enabled = zellij_enabled

        async def start_zellij_server(self, token_holder):
            responses["start_called"] = True
            return None

        async def run_zellij_list_sessions(self):
            return {"success": True, "sessions": [], "count": 0}

    def fake_html_code_gen(rrd_url, zellij_enabled, zellij_token, session_name):
        responses["html_args"] = {
            "rrd_url": rrd_url,
            "zellij_enabled": zellij_enabled,
            "zellij_token": zellij_token,
            "session_name": session_name,
        }
        return f"<html><body>{zellij_enabled}-{zellij_token}-{session_name}</body></html>"

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        responses["host"] = host
        responses["port"] = port
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        route = next(iter(app.router.routes()))
        try:
            app._set_loop(loop)  # type: ignore[attr-defined]
            app.freeze()
            loop.run_until_complete(app.startup())
            req = make_mocked_request("GET", "/", app=app)
            resp = loop.run_until_complete(route.handler(req))
            responses["status"] = resp.status
            responses["body"] = resp.text
        finally:
            loop.run_until_complete(app.shutdown())
            loop.run_until_complete(app.cleanup())
            loop.close()
            asyncio.set_event_loop(None)

    monkeypatch.setattr(dashboard_server, "ZellijManager", FakeZellijManager)
    monkeypatch.setattr(dashboard_server, "html_code_gen", fake_html_code_gen)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    thread = dashboard_server.start_dashboard_server_thread(
        port=9876,
        dashboard_host="localhost",
        terminal_commands={"shell": "echo hi"} if zellij_enabled else {},
        zellij_session_name="sess-name",
        zellij_token="token-xyz",
        rrd_url="rrd://abc",
    )
    thread.join(timeout=5)
    assert not thread.is_alive()

    assert responses["status"] == 200
    assert responses.get("body")
    assert responses["html_args"]["zellij_enabled"] == zellij_enabled
    assert responses["html_args"]["zellij_token"] == "token-xyz"
    assert responses["html_args"]["session_name"].startswith("sess-name")
    assert responses["port"] == 9876


def test_health_endpoint_reports_services(monkeypatch, dashboard_server):
    responses: dict = {}

    class FakeZellijManager:
        def __init__(self, *, log, session_name, port, terminal_commands, token):
            self.enabled = True

        async def start_zellij_server(self, token_holder):
            responses["zellij_started"] = True
            return None

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        responses["host"] = host
        responses["port"] = port
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        route = next(iter(app.router.routes()))
        try:
            app._set_loop(loop)  # type: ignore[attr-defined]
            app.freeze()
            loop.run_until_complete(app.startup())
            req = make_mocked_request("GET", "/health", app=app)
            resp = loop.run_until_complete(route.handler(req))
            responses["status"] = resp.status
            responses["payload"] = json.loads(resp.text)
        finally:
            loop.run_until_complete(app.shutdown())
            loop.run_until_complete(app.cleanup())
            loop.close()
            asyncio.set_event_loop(None)

    monkeypatch.setattr(dashboard_server, "ZellijManager", FakeZellijManager)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    thread = dashboard_server.start_dashboard_server_thread(
        port=5050,
        dashboard_host="dash.test",
        zellij_url="http://zellij:9999",
        rrd_url="rrd+tcp://rrd:123",
    )
    thread.join(timeout=5)
    assert not thread.is_alive()

    payload = responses["payload"]
    assert payload["status"] == "ok"
    assert payload["services"]["frontend"] == "http://dash.test:5050/zviewer"
    assert payload["services"]["api"] == "http://dash.test:5050/zviewer/api"
    assert payload["services"]["rerun"] == "rrd+tcp://rrd:123"
    assert payload["services"]["zellij"] == "http://zellij:9999"
    assert responses["status"] == 200
    assert responses["host"] == "dash.test"
    assert responses["port"] == 5050
    assert responses["zellij_started"] is True


def test_api_routes_have_cors_and_sessions(monkeypatch, dashboard_server):
    responses: dict = {}

    class FakeZellijManager:
        def __init__(self, *, log, session_name, port, terminal_commands, token):
            self.enabled = True

        async def start_zellij_server(self, token_holder):
            responses["zellij_started"] = True
            return None

        async def run_zellij_list_sessions(self):
            return {"success": True, "sessions": ["alpha"], "count": 1}

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        route = next(iter(app.router.routes()))
        try:
            app._set_loop(loop)  # type: ignore[attr-defined]
            app.freeze()
            loop.run_until_complete(app.startup())

            req_health = make_mocked_request("GET", "/zviewer/api/health", app=app)
            resp_health = loop.run_until_complete(route.handler(req_health))
            responses["health_status"] = resp_health.status
            responses["health_payload"] = json.loads(resp_health.text)
            responses["health_cors"] = resp_health.headers.get("Access-Control-Allow-Origin")

            req_sessions = make_mocked_request("GET", "/zviewer/api/sessions", app=app)
            resp_sessions = loop.run_until_complete(route.handler(req_sessions))
            responses["sessions_status"] = resp_sessions.status
            responses["sessions_payload"] = json.loads(resp_sessions.text)
            responses["sessions_cors"] = resp_sessions.headers.get("Access-Control-Allow-Headers")

            req_options = make_mocked_request("OPTIONS", "/zviewer/api/health", app=app)
            resp_options = loop.run_until_complete(route.handler(req_options))
            responses["options_status"] = resp_options.status
            responses["options_cors"] = resp_options.headers.get("Access-Control-Allow-Origin")
            responses["options_methods"] = resp_options.headers.get("Access-Control-Allow-Methods")
        finally:
            loop.run_until_complete(app.shutdown())
            loop.run_until_complete(app.cleanup())
            loop.close()
            asyncio.set_event_loop(None)

    monkeypatch.setattr(dashboard_server, "ZellijManager", FakeZellijManager)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    thread = dashboard_server.start_dashboard_server_thread(
        dashboard_host="127.0.0.1",
        zellij_session_name="cors-sess",
        zellij_token="tok-cors",
        rrd_url="rrd://noop",
    )
    thread.join(timeout=5)
    assert not thread.is_alive()

    assert responses["health_status"] == 200
    assert responses["health_payload"]["status"] == "ok"
    assert responses["health_cors"] == "*"
    assert responses["sessions_status"] == 200
    assert responses["sessions_payload"]["count"] == 1
    assert responses["sessions_payload"]["sessions"] == ["alpha"]
    assert responses["sessions_cors"] == "*"
    assert responses["options_status"] == 204
    assert responses["options_cors"] == "*"
    assert "GET" in responses["options_methods"]
    assert responses["zellij_started"] is True


def test_auto_open_launches_browser(monkeypatch, dashboard_server):
    browser_calls: list = []

    class FakeZellijManager:
        def __init__(self, *, log, session_name, port, terminal_commands, token):
            self.enabled = False

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            app._set_loop(loop)  # type: ignore[attr-defined]
            app.freeze()
            loop.run_until_complete(app.startup())
            loop.run_until_complete(asyncio.sleep(0.35))
        finally:
            loop.run_until_complete(app.shutdown())
            loop.run_until_complete(app.cleanup())
            loop.close()
            asyncio.set_event_loop(None)

    monkeypatch.setattr(dashboard_server, "ZellijManager", FakeZellijManager)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)
    monkeypatch.setattr(dashboard_server.webbrowser, "open", lambda url: browser_calls.append(url))

    thread = dashboard_server.start_dashboard_server_thread(
        auto_open=True,
        port=4444,
        dashboard_host="dash.local",
        terminal_commands={},
        rrd_url="rrd://noop",
    )
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert browser_calls == ["http://dash.local:4444/zviewer"]


def test_https_context_is_built_and_passed(monkeypatch, dashboard_server):
    responses: dict = {}

    class FakeSSLContext:
        def __init__(self, protocol):
            self.protocol = protocol
            self.loaded = None

        def load_cert_chain(self, certfile, keyfile):
            self.loaded = (certfile, keyfile)

    class FakeZellijManager:
        def __init__(self, *, log, session_name, port, terminal_commands, token):
            self.enabled = False

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        responses["host"] = host
        responses["port"] = port
        responses["ssl_context"] = ssl_context
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            app._set_loop(loop)  # type: ignore[attr-defined]
            app.freeze()
            loop.run_until_complete(app.startup())
        finally:
            loop.run_until_complete(app.shutdown())
            loop.run_until_complete(app.cleanup())
            loop.close()
            asyncio.set_event_loop(None)

    monkeypatch.setattr(dashboard_server.ssl, "SSLContext", FakeSSLContext)
    monkeypatch.setattr(dashboard_server, "ZellijManager", FakeZellijManager)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    thread = dashboard_server.start_dashboard_server_thread(
        https_enabled=True,
        https_key_path="/certs/key.pem",
        https_cert_path="/certs/cert.pem",
        port=4433,
        rrd_url="rrd://noop",
    )
    thread.join(timeout=5)
    assert not thread.is_alive()

    ctx = responses["ssl_context"]
    assert isinstance(ctx, FakeSSLContext)
    assert ctx.loaded == ("/certs/cert.pem", "/certs/key.pem")
    assert responses["host"] == "localhost"
    assert responses["port"] == 4433
