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

from collections.abc import Callable, Iterator
from typing import Any

import pytest
from starlette.testclient import TestClient

from dimos.core.module import Module
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.constants import RerunOpenOption
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


@pytest.fixture
def make_dashboard(
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[Callable[..., WebsocketVisModule]]:
    modules: list[Module] = []

    def make(rerun_open: RerunOpenOption | None, **overrides: Any) -> WebsocketVisModule:
        bridge = None
        if rerun_open is not None:
            bridge = RerunBridgeModule(pubsubs=[], rerun_open=rerun_open)
            modules.append(bridge)
        dashboard = WebsocketVisModule(**overrides)
        modules.append(dashboard)
        monkeypatch.setattr(dashboard, "_rerun_bridge", bridge)
        return dashboard

    yield make

    for module in reversed(modules):
        module.stop()


@pytest.mark.parametrize(
    ("rerun_open", "overrides", "open_browser", "status"),
    [
        ("web", {}, True, 200),
        ("both", {}, True, 200),
        ("native", {}, False, 307),
        ("none", {}, False, 307),
        (None, {}, False, 307),
        ("web", {"open_browser": False}, False, 200),
        ("both", {"show_rerun_dashboard": False}, True, 307),
        (None, {"open_browser": True, "show_rerun_dashboard": True}, True, 200),
    ],
)
def test_dashboard_follows_bridge_with_independent_overrides(
    make_dashboard: Callable[..., WebsocketVisModule],
    rerun_open: RerunOpenOption | None,
    overrides: dict[str, bool],
    open_browser: bool,
    status: int,
) -> None:
    dashboard = make_dashboard(rerun_open, **overrides)

    dashboard._resolve_viewer_settings()
    dashboard._create_server()
    with TestClient(dashboard.app) as client:
        response = client.get("/", follow_redirects=False)

    assert dashboard.config.open_browser is open_browser
    assert response.status_code == status
