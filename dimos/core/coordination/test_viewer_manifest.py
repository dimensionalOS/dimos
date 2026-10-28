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

from types import SimpleNamespace
from unittest.mock import Mock

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import _deploy_all_modules, _viewer_message_types
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.core.transport import ZenohTransport
from dimos.messages.std_msgs.msg import String
from dimos.visualization.rerun.bridge import RerunBridgeModule


class Source(Module):
    text: Out[String]


def test_manifest_uses_materialized_channel_and_is_pickled_into_viewer_config():
    blueprint = autoconnect(Source.blueprint(), RerunBridgeModule.blueprint())
    transport = ZenohTransport("custom/channel", String)
    transports = {("text", String): transport}
    coordinator = SimpleNamespace(
        _transport_registry={}, _deployed_atoms={}, deploy_parallel=Mock()
    )
    _deploy_all_modules(blueprint, coordinator, GlobalConfig(), {}, transports)
    specs = coordinator.deploy_parallel.call_args.args[0]
    viewer = next(kwargs for module, _, kwargs in specs if module is RerunBridgeModule)
    assert viewer["message_types"]["custom/channel"] is String


def test_existing_transport_wins_when_loading_more_modules():
    blueprint = autoconnect(Source.blueprint())
    existing = {("text", String): ZenohTransport("existing/channel", String)}
    proposed = {("text", String): ZenohTransport("new/channel", String)}
    assert _viewer_message_types(blueprint, proposed, existing) == {"existing/channel": String}
