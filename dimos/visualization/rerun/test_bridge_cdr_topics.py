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

"""The explicit generated-type path of `_subscribe` on zenoh.

Two things the support feature must get right, because CDR keys carry no type
suffix and hosted runs publish under `dimos-domains/<domain>/`:

* the firehose hands its declared channel->type map to the pubsub decoder, so a
  bare CDR key can be decoded without guessing from the payload;
* an allowlisted topic is subscribed under the active domain prefix, or the
  bridge would listen on a key nobody publishes to.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
from typing import Any

from dimos_messages.sensor_msgs.msg import PointCloud2
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.visualization.rerun.bridge import RerunBridgeModule


class FakeZenoh(Zenoh):
    """Records subscriptions instead of opening a session."""

    keys: list[str]
    subscribed_all: bool
    keyed: list[str]
    message_types: Mapping[str, type] | None

    def __init__(self) -> None:
        self.keys = []
        self.subscribed_all = False
        self.keyed = []
        self.message_types = None

    def subscribe(self, topic: Any, callback: Any) -> Any:
        self.keys.append(topic.key_expr)
        return lambda: None

    def subscribe_all(
        self,
        callback: Any,
        keyed: Mapping[str, Any] | None = None,
        message_types: Mapping[str, type] | None = None,
        **kwargs: Any,
    ) -> Any:
        self.subscribed_all = True
        self.keyed = list(keyed or {})
        self.message_types = message_types
        return lambda: None


@pytest.fixture
def make_bridge() -> Iterator[Callable[..., RerunBridgeModule]]:
    bridges: list[RerunBridgeModule] = []

    def make(**config: Any) -> RerunBridgeModule:
        bridge = RerunBridgeModule(pubsubs=[], **config)
        bridges.append(bridge)
        return bridge

    yield make
    for bridge in bridges:
        bridge.stop()


def test_firehose_hands_declared_types_to_the_pubsub(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    pubsub = FakeZenoh()
    bridge = make_bridge(message_types={"dimos/cloud": PointCloud2})
    bridge._subscribe(pubsub)
    declared = pubsub.message_types
    assert declared is not None
    assert declared is bridge.config.message_types
    assert declared["dimos/cloud"] is PointCloud2


def test_live_type_update_reaches_the_declared_map(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    bridge = make_bridge(message_types={"dimos/cloud": PointCloud2})
    pubsub = FakeZenoh()
    bridge._subscribe(pubsub)
    bridge.update_message_types({"dimos/new_cloud": PointCloud2})
    declared = pubsub.message_types
    assert declared is not None
    assert declared is bridge.config.message_types
    assert declared["dimos/new_cloud"] is PointCloud2
    with pytest.raises(ValueError, match="Conflicting message types"):
        bridge.update_message_types({"dimos/other": PointCloud2, "dimos/cloud": str})
    assert "dimos/other" not in bridge.config.message_types


def test_named_topics_use_the_active_domain_key(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    pubsub = FakeZenoh()
    bridge = make_bridge(topics=["tf"], g=GlobalConfig(zenoh_domain="r1"))
    bridge.update_message_types({"dimos/tf": PointCloud2})
    bridge._subscribe(pubsub)
    # Generated CDR keys are bare: the wildcard was for legacy LCM type suffixes.
    assert pubsub.keys == ["dimos-domains/r1/tf"]
    assert not pubsub.subscribed_all


def test_named_topics_without_a_domain_stay_plain(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    pubsub = FakeZenoh()
    bridge = make_bridge(topics=["tf"])
    bridge.update_message_types({"dimos/tf": PointCloud2})
    bridge._subscribe(pubsub)
    assert pubsub.keys == ["dimos/tf"]


def test_a_scoped_declared_channel_matches_the_requested_name(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    pubsub = FakeZenoh()
    bridge = make_bridge(topics=["tf"], g=GlobalConfig(zenoh_domain="r1"))
    # The runtime graph names channels with the transport prefix.
    bridge.update_message_types({"dimos-domains/r1/tf": PointCloud2})
    bridge._subscribe(pubsub)
    assert pubsub.keys == ["dimos-domains/r1/tf"]
