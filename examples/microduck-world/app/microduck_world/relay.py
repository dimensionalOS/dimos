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

"""Project relay policy and discovery, using the existing bridge and transport."""

import asyncio
from dataclasses import replace
from pathlib import Path
from typing import Any, Literal

import requests
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.relay_bridge.protocol import Tx
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule, _Session
from dimos.web.relay_bridge.relay_process import RelayProcess
from langchain_core.messages.base import BaseMessage
from microduck_world.gateway import GatewayConfig
from microduck_world.scene import PROJECT_ROOT
from pydantic import BaseModel, ConfigDict, Field


def relay_settings() -> GatewayConfig:
    return GatewayConfig.model_validate_json((PROJECT_ROOT / "config/tailnet.json").read_text())


def private_relay_get(path: str) -> dict[str, Any]:
    with requests.Session() as client:
        client.trust_env = False
        response = client.get(relay_settings().upstream_url + path, timeout=1)
        response.raise_for_status()
        value = response.json()
        if not isinstance(value, dict):
            raise ValueError("Invalid private relay response")
        return value


class WorldCommand(BaseModel):
    model_config = ConfigDict(strict=True, allow_inf_nan=False)
    name: Literal["set_mode", "policy", "cancel_nav", "respawn", "drop_ball"]
    args: dict[str, Any] = Field(default_factory=dict)


class WorldBridge(RelayBridgeModule):
    world_state: In[str]
    duck1_ball_camera: In[str]
    duck2_ball_camera: In[str]
    duck3_ball_camera: In[str]
    duck4_ball_camera: In[str]
    duck5_ball_camera: In[str]
    duck6_ball_camera: In[str]
    world_compare_image: In[Image]
    agent: In[BaseMessage]
    agent_idle: In[bool]
    mode: In[str]
    policy_state: In[str]
    path: In[NavPath]
    nav_state: In[str]
    places: In[str]

    def _on_wire_tx(self, msg: Tx) -> None:
        # Extend this project's command vocabulary while retaining the stock
        # manifest, sequence, rate-limit, validation and publishing path.
        handler = self._tx_defs.get(msg.ch)
        if msg.ch == "ui_command" and handler is not None and handler.model is not WorldCommand:
            self._tx_defs[msg.ch] = replace(handler, model=WorldCommand)
        super()._on_wire_tx(msg)

    def _spawn_relay(self, open_browser: bool, serve_dir: Path | None) -> str:
        self._relay = RelayProcess(
            port=self.config.local_port,
            web_dir=PROJECT_ROOT / "relay",
            cockpit_dir=PROJECT_ROOT / "vendor/dimos/web/cockpit/dist",
            sdk_dir=PROJECT_ROOT / "vendor/dimos/web/sdk/dist",
            entrypoint=PROJECT_ROOT / "relay/main.ts",
        )
        return self._relay.start().wt_url

    async def _connect_and_hello(self) -> _Session:
        # Refresh discovery on every reconnect: the supervised relay's QUIC
        # port and private registration credential rotate after a restart.
        for attempt in range(60):
            try:
                info = await asyncio.to_thread(private_relay_get, "/internal/robot-info")
                break
            except requests.RequestException:
                if attempt == 59:
                    raise
                await asyncio.sleep(0.5)
        previous = self._url
        try:
            self._url = info["wtUrl"]
            return await super()._connect_and_hello()
        finally:
            self._url = previous
