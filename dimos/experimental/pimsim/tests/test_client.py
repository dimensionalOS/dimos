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

from __future__ import annotations

import json

from dimos.experimental.pimsim.client import PimSimClient


class _CapturingConnection:
    def __init__(self) -> None:
        self.sent: list[dict[str, object]] = []

    def send(self, payload: str) -> None:
        self.sent.append(json.loads(payload))

    def close(self) -> None:
        pass


def _client_with_fake_ws() -> tuple[PimSimClient, _CapturingConnection]:
    client = PimSimClient()
    fake = _CapturingConnection()
    client._ws = fake  # type: ignore[assignment]
    return client, fake


def test_set_agent_position_emits_respawn_at() -> None:
    client, fake = _client_with_fake_ws()
    client.set_agent_position(1.0, 2.0)
    assert fake.sent == [{"type": "respawn_at", "point": [1.0, 2.0, 0.0]}]


def test_add_wall_emits_entity_add_wall_with_endpoints() -> None:
    client, fake = _client_with_fake_ws()
    client.add_wall(0.0, 0.0, 3.0, 4.0, height=2.0, thickness=0.2)
    assert fake.sent == [
        {
            "type": "entity_add_wall",
            "x1": 0.0,
            "y1": 0.0,
            "x2": 3.0,
            "y2": 4.0,
            "height": 2.0,
            "thickness": 0.2,
        }
    ]


def test_clear_entities_emits_entity_clear() -> None:
    client, fake = _client_with_fake_ws()
    client.clear_entities()
    assert fake.sent == [{"type": "entity_clear"}]
