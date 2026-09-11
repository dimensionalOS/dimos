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

from unittest.mock import Mock

from dimos.web.relay_bridge.protocol import Tx
from dimos.web.relay_bridge.relay_bridge_module import TX_CHANNELS
from microduck_world.relay import WorldBridge


def test_respawn_uses_existing_command_guards_without_changing_global_bridge(
    module_factory, monkeypatch
):
    bridge = module_factory(WorldBridge)
    handler = next(item for item in TX_CHANNELS if item.ch == "ui_command")
    bridge._tx_defs["ui_command"] = handler
    publish = Mock()
    monkeypatch.setattr(bridge.ui_command, "publish", publish)
    clock = Mock(return_value=10.0)
    monkeypatch.setattr("dimos.web.relay_bridge.relay_bridge_module.time.monotonic", clock)
    message = Tx(ch="ui_command", seq=1, data={"name": "respawn", "args": {}})

    bridge._on_wire_tx(message)
    bridge._on_wire_tx(message)
    publish.assert_called_once_with('{"name":"respawn","args":{}}')
    assert next(item for item in TX_CHANNELS if item.ch == "ui_command") is handler

    clock.return_value = 10.1
    bridge._on_wire_tx(Tx(ch="ui_command", seq=2, data={"name": "arbitrary-rpc"}))
    publish.assert_called_once()
