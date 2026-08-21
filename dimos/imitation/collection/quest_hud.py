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

import json
import time
from typing import Any

from fastapi import WebSocket

from dimos.core.stream import In
from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


class QuestCollectionTeleopModule(ArmTeleopModule):
    """Arm teleop that sends collection status to the Quest HUD."""

    status: In[EpisodeStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_episode_status = EpisodeStatus(
            ts=time.time(),
            state="idle",
            episodes_saved=0,
            episodes_discarded=0,
        )

    async def handle_status(self, status: EpisodeStatus) -> None:
        with self._lock:
            self._latest_episode_status = status
        self._broadcast_text(self._encode_episode_status(status))

    def _client_connected(self, ws: WebSocket) -> bool:
        if not super()._client_connected(ws):
            return False
        with self._lock:
            status = self._latest_episode_status
        self._broadcast_text(self._encode_episode_status(status))
        return True

    @staticmethod
    def _encode_episode_status(status: EpisodeStatus) -> str:
        payload = status.model_dump(mode="json")
        payload["type"] = "episode_status"
        payload["elapsed_s"] = (
            max(0.0, time.time() - status.ts) if status.state == "recording" else 0.0
        )
        return json.dumps(payload, separators=(",", ":"))
