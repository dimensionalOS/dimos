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

import builtins
import socket
from typing import Any

import pytest

from dimos.core.run_registry import RunEntry
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT
from dimos.visualization.rerun.topic_monitor import (
    PortAllocationError,
    TopicMonitorDependencyError,
    TopicMonitorSidecar,
    _require_visualization_dependencies,
    allocate_monitor_ports,
    open_selector_url,
    resolve_run_context,
)


def test_allocate_monitor_ports_avoids_default_rerun_ports() -> None:
    ports = allocate_monitor_ports(start=RERUN_GRPC_PORT, end=RERUN_GRPC_PORT + 20)

    assert RERUN_GRPC_PORT not in {
        ports.rerun_grpc,
        ports.rerun_web,
        ports.selector_frontend,
        ports.selector_api,
        ports.reflex_backend,
    }


def test_allocate_monitor_ports_fails_for_explicit_conflict() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.listen(1)
    try:
        with pytest.raises(PortAllocationError):
            allocate_monitor_ports(port_base=port)
    finally:
        sock.close()


def test_topic_monitor_urls_use_allocated_connected_viewer_url() -> None:
    sidecar = TopicMonitorSidecar(
        host="127.0.0.1",
        ports=allocate_monitor_ports(start=11150, end=11180),
    )

    urls = sidecar.urls

    assert str(sidecar.ports.selector_frontend) in urls.selector
    assert str(sidecar.ports.selector_api) in urls.selector_api
    assert str(sidecar.ports.rerun_grpc) in urls.rerun_connect
    assert "url=rerun%2Bhttp" in urls.rerun_viewer


def test_resolve_run_context_uses_latest_or_bus_only(monkeypatch: pytest.MonkeyPatch) -> None:
    entry = RunEntry(
        run_id="20260610-test",
        pid=123,
        blueprint="demo",
        started_at="now",
        log_dir="/tmp/logs",
    )
    monkeypatch.setattr(
        "dimos.visualization.rerun.topic_monitor.get_most_recent",
        lambda alive_only=True: entry,
    )

    context = resolve_run_context()

    assert context.entry is entry
    assert "20260610-test" in context.label

    monkeypatch.setattr(
        "dimos.visualization.rerun.topic_monitor.get_most_recent",
        lambda alive_only=True: None,
    )

    assert resolve_run_context().entry is None


def test_resolve_run_context_fails_for_missing_explicit_run(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "dimos.visualization.rerun.topic_monitor.list_runs", lambda alive_only=True: []
    )

    with pytest.raises(ValueError, match="No active DimOS run"):
        resolve_run_context("missing")


def test_open_selector_url_is_non_fatal() -> None:
    assert open_selector_url("http://localhost", opener=lambda _url: False) is False

    def boom(_url: str) -> bool:
        raise RuntimeError("no browser")

    assert open_selector_url("http://localhost", opener=boom) is False


def test_missing_dependency_error_mentions_visualization_extra(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def fake_import(name: str, *args: Any, **kwargs: Any) -> object:
        if name == "reflex":
            raise ImportError("missing reflex")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)

    with pytest.raises(TopicMonitorDependencyError, match="uv sync --extra visualization"):
        _require_visualization_dependencies()
