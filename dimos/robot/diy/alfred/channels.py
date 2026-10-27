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

"""The raw channels Alfred's coordinator adapters meet their modules on."""

from __future__ import annotations

from dimos.core.coordination.blueprints import TransportSpec
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.core.transport_factory import transport_topic


def channel_spec(name: str, msg_type: type) -> TransportSpec:
    """Pin a module stream to the channel ``make_transport(name)`` publishes on.

    The coordinator's transport adapters (``transport_lcm`` for the pillar's whole-body
    streams and for the FlowBase's twist) talk straight through ``make_transport``, outside
    the blueprint's stream graph. A module stream that has to meet them is pinned to the
    same topic on whichever backend ``global_config.transport`` resolves to, so the same
    blueprint runs over zenoh (the default, and what Desktop reads) or LCM.
    """
    topic = transport_topic(name)
    cls = ZenohTransport if global_config.transport == "zenoh" else LCMTransport
    return cls.spec(topic, msg_type)
