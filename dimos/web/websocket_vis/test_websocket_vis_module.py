# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

from dimos.core.global_config import GlobalConfig
from dimos.web.websocket_vis import websocket_vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketConfig, WebsocketVisModule


def test_websocket_vis_uses_module_global_config_listen_host(monkeypatch) -> None:
    captured: dict[str, Any] = {}

    class FakeServer:
        def __init__(self, config: Any) -> None:
            captured["config"] = config

        def run(self) -> None:
            captured["ran"] = True

    def fake_config(app: Any, **kwargs: Any) -> dict[str, Any]:
        captured["app"] = app
        captured["kwargs"] = kwargs
        return kwargs

    monkeypatch.setattr(websocket_vis_module.uvicorn, "Config", fake_config)
    monkeypatch.setattr(websocket_vis_module.uvicorn, "Server", FakeServer)

    module = WebsocketVisModule.__new__(WebsocketVisModule)
    module.app = object()
    module.config = WebsocketConfig(
        port=7779,
        g=GlobalConfig(listen_host="0.0.0.0"),
    )

    module._run_uvicorn_server()

    assert captured["kwargs"]["host"] == "0.0.0.0"
    assert captured["kwargs"]["port"] == 7779
    assert captured["ran"] is True
