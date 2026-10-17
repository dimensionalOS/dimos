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

"""`topics` subscribes per zenoh key; without it the bridge takes the firehose.

The point of the allowlist is that an unlisted topic never crosses the network,
so what matters is which subscriptions are declared, not what is filtered after
the bytes arrive.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Iterator
from functools import partial
from typing import Any
from unittest.mock import patch

import pytest
from pytest_mock import MockerFixture
import rerun as rr

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, Zenoh
from dimos.visualization.rerun.bridge import (
    RerunBridgeModule,
    RerunEntry,
    is_rerun_multi,
    keyed_by_seq,
    region_entity,
)


class FakeZenoh(Zenoh):
    """Records subscriptions instead of opening a session."""

    def __init__(self) -> None:
        self.keys: list[str] = []
        self.subscribed_all = False
        self.unconflated: list[str] = []

    def subscribe(self, topic: Any, callback: Any) -> Any:
        self.keys.append(topic.key_expr)
        return lambda: None

    def subscribe_all(self, callback: Any, unconflated: Collection[str] = ()) -> Any:
        self.subscribed_all = True
        self.unconflated = list(unconflated)
        return lambda: None


@pytest.fixture
def make_bridge() -> Iterator[Callable[..., RerunBridgeModule]]:
    """Build real bridge modules. Constructing one opens a transport, so each is stopped again."""
    bridges: list[RerunBridgeModule] = []

    def make(**config: Any) -> RerunBridgeModule:
        bridge = RerunBridgeModule(pubsubs=[], **config)
        bridges.append(bridge)
        return bridge

    yield make
    for bridge in bridges:
        bridge.stop()


def test_named_topics_become_one_key_each(make_bridge: Callable[..., RerunBridgeModule]) -> None:
    pubsub = FakeZenoh()
    make_bridge(topics=["tf", "/local_map"])._subscribe(pubsub)
    # The trailing wildcard is the key's type segment, resolved per sample.
    assert pubsub.keys == ["dimos/tf/*", "dimos/local_map/*"]
    assert not pubsub.subscribed_all


def test_no_topics_keeps_the_firehose(make_bridge: Callable[..., RerunBridgeModule]) -> None:
    pubsub = FakeZenoh()
    make_bridge()._subscribe(pubsub)
    assert pubsub.subscribed_all
    assert pubsub.keys == []
    assert pubsub.unconflated == []


def test_the_firehose_takes_keyed_renderers_unconflated(
    make_bridge: Callable[..., RerunBridgeModule],
) -> None:
    @keyed_by_seq
    def by_cell(msg: PointCloud2) -> None:
        return None

    pubsub = FakeZenoh()
    make_bridge(
        visual_override={
            "world/surface_map": partial(by_cell),
            "world/map_regions": by_cell,
            "world/local_map": lambda msg: None,
        },
    )._subscribe(pubsub)
    assert sorted(pubsub.unconflated) == ["map_regions", "surface_map"]

    pubsub = FakeZenoh()
    make_bridge(entity_prefix="hall", visual_override={"hall/surface_map": by_cell})._subscribe(
        pubsub
    )
    assert pubsub.unconflated == ["surface_map"]


def test_a_leaf_stack_does_not_claim_the_coordinator_name() -> None:
    """Two stacks share one zenoh bus only if the second declines the bus-wide name."""
    coordinator = ModuleCoordinator(
        g=GlobalConfig(serve_coordinator_rpc=False, n_workers=0, viewer="none")
    )

    with patch("dimos.core.coordination.module_coordinator.CoordinatorRPC.serve") as serve:
        coordinator.start_rpc_service()

    serve.assert_not_called()
    assert coordinator._coordinator_rpc is None


def test_multi_entries_log_their_static_flag_and_pin_the_frame_once(
    mocker: MockerFixture, make_bridge: Callable[..., RerunBridgeModule]
) -> None:
    cells = [
        RerunEntry("world/surface_map/0_0", rr.Points3D([[0.0, 0.0, 0.0]]), static=True),
        ("world/surface_map/1_0", rr.Points3D([[1.0, 0.0, 0.0]])),
    ]
    assert is_rerun_multi(cells)
    log = mocker.patch("rerun.log")
    topic = ZenohTopic(topic="dimos/surface_map", lcm_type=PointCloud2)
    bridge = make_bridge(visual_override={"world/surface_map": lambda m: cells})
    entity = bridge._get_entity_path(topic)
    msg = PointCloud2(frame_id="odom")
    bridge._on_message(msg, topic)
    bridge._on_message(msg, topic)
    calls = [(c.args[0], c.kwargs.get("static", False)) for c in log.call_args_list]
    assert calls.count(("world/surface_map/0_0", True)) == 2
    assert calls.count(("world/surface_map/1_0", False)) == 2
    pins = [c for c in log.call_args_list if c.args[0] == entity]
    assert len(pins) == 1 and isinstance(pins[0].args[1], rr.Transform3D)


def test_region_cells_unpack_from_the_seq_the_planner_packs() -> None:
    assert region_entity("world/surface_map", (-3 << 16) | (5 & 0xFFFF)) == "world/surface_map/-3_5"
    assert region_entity("world/node_edges", (7 << 16) | (-2 & 0xFFFF)) == "world/node_edges/7_-2"
