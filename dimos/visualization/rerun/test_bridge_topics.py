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

from typing import Any

from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, Zenoh
from dimos.visualization.rerun.bridge import RerunBridgeModule


class FakeZenoh(Zenoh):
    """Records subscriptions instead of opening a session."""

    def __init__(self) -> None:
        self.keys: list[str] = []
        self.subscribed_all = False
        self.unconflated: list[str] = []

    def subscribe(self, topic: Any, callback: Any) -> Any:
        self.keys.append(topic.key_expr)
        return lambda: None

    def subscribe_all(self, callback: Any, unconflated: Any = ()) -> Any:
        self.subscribed_all = True
        self.unconflated = list(unconflated)
        return lambda: None


def _subscribe(pubsub: FakeZenoh, **config: Any) -> None:
    """Run the real `_subscribe` on a real module. Constructing one opens a
    transport, so it is stopped again."""
    bridge = RerunBridgeModule(pubsubs=[], **config)
    try:
        bridge._subscribe(pubsub)
    finally:
        bridge.stop()


def test_named_topics_become_one_key_each() -> None:
    pubsub = FakeZenoh()
    _subscribe(pubsub, topics=["tf", "/local_map"])
    # The trailing wildcard is the key's type segment, resolved per sample.
    assert pubsub.keys == ["dimos/tf/*", "dimos/local_map/*"]
    assert not pubsub.subscribed_all


def test_no_topics_keeps_the_firehose() -> None:
    pubsub = FakeZenoh()
    _subscribe(pubsub)
    assert pubsub.subscribed_all
    assert pubsub.keys == []
    assert pubsub.unconflated == []


def test_the_firehose_takes_keyed_renderers_unconflated() -> None:
    from functools import partial

    from dimos.visualization.rerun.bridge import keyed_by_seq

    @keyed_by_seq
    def by_cell(msg: Any) -> Any:
        return None

    pubsub = FakeZenoh()
    _subscribe(
        pubsub,
        visual_override={
            "world/surface_map": partial(by_cell),
            "world/map_regions": by_cell,
            "world/local_map": lambda msg: None,
        },
    )
    assert sorted(pubsub.unconflated) == ["map_regions", "surface_map"]

    pubsub = FakeZenoh()
    _subscribe(pubsub, entity_prefix="hall", visual_override={"hall/surface_map": by_cell})
    assert pubsub.unconflated == ["surface_map"]


def test_a_leaf_stack_does_not_claim_the_coordinator_name() -> None:
    """Two stacks share one zenoh bus only if the second declines the bus-wide name."""
    from unittest.mock import patch

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.global_config import GlobalConfig

    coordinator = ModuleCoordinator(
        g=GlobalConfig(serve_coordinator_rpc=False, n_workers=0, viewer="none")
    )

    with patch("dimos.core.coordination.module_coordinator.CoordinatorRPC.serve") as serve:
        coordinator.start_rpc_service()

    serve.assert_not_called()
    assert coordinator._coordinator_rpc is None


def test_multi_entries_log_their_static_flag_and_pin_the_frame_once(mocker: Any) -> None:
    import rerun as rr

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.visualization.rerun.bridge import RerunEntry, is_rerun_multi

    cells = [
        RerunEntry("world/surface_map/0_0", rr.Points3D([[0.0, 0.0, 0.0]]), static=True),
        ("world/surface_map/1_0", rr.Points3D([[1.0, 0.0, 0.0]])),
    ]
    assert is_rerun_multi(cells)
    log = mocker.patch("rerun.log")
    topic = ZenohTopic(topic="dimos/surface_map", lcm_type=PointCloud2)
    bridge = RerunBridgeModule(pubsubs=[], visual_override={"world/surface_map": lambda m: cells})
    try:
        entity = bridge._get_entity_path(topic)
        msg = PointCloud2(frame_id="odom")
        bridge._on_message(msg, topic)
        bridge._on_message(msg, topic)
    finally:
        bridge.stop()
    calls = [(c.args[0], c.kwargs.get("static", False)) for c in log.call_args_list]
    assert calls.count(("world/surface_map/0_0", True)) == 2
    assert calls.count(("world/surface_map/1_0", False)) == 2
    pins = [c for c in log.call_args_list if c.args[0] == entity]
    assert len(pins) == 1 and isinstance(pins[0].args[1], rr.Transform3D)
