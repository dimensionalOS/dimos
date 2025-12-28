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


def test_build_and_start_uses_config(monkeypatch, dashboard_server):
    captured: dict = {}

    def fake_run_app(
        app, host=None, port=None, ssl_context=None, access_log=None, handle_signals=None
    ):
        captured.update(host=host, port=port, ssl_context=ssl_context)
        return None

    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    cfg = dashboard_server.build_dashboard_config(
        launcher="browser",
        port=5555,
        dashboard_host="0.0.0.0",
        https_enabled=False,
        rrd_url="rrd+tcp://localhost:2222",
    )
    logger = logging.getLogger("test-config")
    dashboard_server.start_dashboard_server(cfg, logger)

    assert captured["host"] == "0.0.0.0"
    assert captured["port"] == 5555
    assert cfg.protocol == "http"
    assert cfg.rrd_url == "rrd+tcp://localhost:2222"


def test_build_dashboard_config_defaults(monkeypatch, dashboard_server):
    cfg = dashboard_server.build_dashboard_config(
        port=1234, https_enabled=False, rrd_url="rrd://example"
    )
    assert cfg.protocol == "http"
    assert cfg.port == 1234
    assert cfg.https_enabled is False


def test_serves_html_without_zellij(monkeypatch, dashboard_server):
    responses: dict = {}

    def fake_html_code_gen(rrd_url, zellij_enabled, zellij_token, session_name="unused"):
        responses["html_args"] = {
            "rrd_url": rrd_url,
            "zellij_enabled": zellij_enabled,
            "zellij_token": zellij_token,
        }
        return "<html><body>no-zellij</body></html>"

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

    monkeypatch.setattr(dashboard_server, "html_code_gen", fake_html_code_gen)
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    cfg = dashboard_server.build_dashboard_config(
        port=9876,
        dashboard_host="localhost",
        rrd_url="rrd://abc",
    )
    dashboard_server.start_dashboard_server(cfg, logging.getLogger("serve-html"))

    assert responses["status"] == 200
    assert responses.get("body") == "<html><body>no-zellij</body></html>"
    assert responses["html_args"]["zellij_enabled"] is False
    assert responses["html_args"]["zellij_token"] is None
    assert responses["port"] == 9876


def test_health_endpoint_reports_services(monkeypatch, dashboard_server):
    responses: dict = {}

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

    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    cfg = dashboard_server.build_dashboard_config(
        port=5050,
        dashboard_host="dash.test",
        rrd_url="rrd+tcp://rrd:123",
    )
    dashboard_server.start_dashboard_server(cfg, logging.getLogger("health"))

    payload = responses["payload"]
    assert payload["status"] == "ok"
    assert payload["services"]["frontend"] == "http://dash.test:5050/zviewer"
    assert payload["services"]["api"] == "http://dash.test:5050/zviewer/api"
    assert payload["services"]["rerun"] == "rrd+tcp://rrd:123"
    assert responses["status"] == 200
    assert responses["host"] == "dash.test"
    assert responses["port"] == 5050


def test_api_routes_have_cors(monkeypatch, dashboard_server):
    responses: dict = {}

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

    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    cfg = dashboard_server.build_dashboard_config(
        dashboard_host="127.0.0.1",
        rrd_url="rrd://noop",
    )
    dashboard_server.start_dashboard_server(cfg, logging.getLogger("cors"))

    assert responses["health_status"] == 200
    assert responses["health_payload"]["status"] == "ok"
    assert responses["health_cors"] == "*"
    assert responses["options_status"] == 204
    assert responses["options_cors"] == "*"
    assert "GET" in responses["options_methods"]


def test_auto_open_launches_browser(monkeypatch, dashboard_server):
    browser_calls: list = []

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

    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)
    monkeypatch.setattr(dashboard_server.webbrowser, "open", lambda url: browser_calls.append(url))

    cfg = dashboard_server.build_dashboard_config(
        launcher="browser",
        port=4444,
        dashboard_host="dash.local",
        rrd_url="rrd://noop",
    )
    dashboard_server.start_dashboard_server(cfg, logging.getLogger("autoopen"))
    assert browser_calls == ["http://dash.local:4444/zviewer"]


def test_https_context_is_built_and_passed(monkeypatch, dashboard_server):
    responses: dict = {}

    class FakeSSLContext:
        def __init__(self, protocol):
            self.protocol = protocol
            self.loaded = None

        def load_cert_chain(self, certfile, keyfile):
            self.loaded = (certfile, keyfile)

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
    monkeypatch.setattr(dashboard_server.web, "run_app", fake_run_app)

    cfg = dashboard_server.build_dashboard_config(
        https_enabled=True,
        https_key_path="/certs/key.pem",
        https_cert_path="/certs/cert.pem",
        port=4433,
        rrd_url="rrd://noop",
    )
    dashboard_server.start_dashboard_server(cfg, logging.getLogger("https"))

    ctx = responses["ssl_context"]
    assert isinstance(ctx, FakeSSLContext)
    assert ctx.loaded == ("/certs/cert.pem", "/certs/key.pem")
    assert responses["host"] == "localhost"
    assert responses["port"] == 4433
