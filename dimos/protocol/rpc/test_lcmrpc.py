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

from collections.abc import Callable, Generator
import threading
from typing import Any

import pytest

from dimos.constants import LCM_MAX_CHANNEL_NAME_LENGTH
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT


@pytest.fixture
def lcmrpc() -> Generator[LCMRPC, None, None]:
    ret = LCMRPC(rpc_timeouts={}, default_rpc_timeout=DEFAULT_RPC_TIMEOUT)
    ret.start()
    yield ret
    ret.stop()


def test_short_name(lcmrpc) -> None:
    actual = lcmrpc.topicgen("Hello/say", req_or_res=True)
    assert actual.topic == "/rpc/Hello/say/res"


def test_long_name(lcmrpc) -> None:
    long = "GreatyLongComplexExampleClassNameForTestingStuff/create"
    long_topic = lcmrpc.topicgen(long, req_or_res=True).topic
    assert long_topic == "/rpc/2cudPuFGMJdWxM5KZb/res"

    less_long = long[:-1]
    less_long_topic = lcmrpc.topicgen(less_long, req_or_res=True).topic
    assert less_long_topic == "/rpc/GreatyLongComplexExampleClassNameForTestingStuff/creat/res"

    assert len(less_long_topic) == LCM_MAX_CHANNEL_NAME_LENGTH


class _OverlapRPC(LCMRPC):
    """Interleaves the first subscribe for `x` replies with a dispatching `y`
    reply: each side waits for the other to arrive before going on, so the
    subscribe runs while the loop thread holds the dispatch lock and the
    reply handler is about to take the response-subscriptions lock."""

    def __init__(self) -> None:
        super().__init__(url="memq://")
        self.x_subscribing = threading.Event()
        self.y_dispatching = threading.Event()

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        if str(topic) == str(self.topicgen("x", True)):
            self.x_subscribing.set()
            assert self.y_dispatching.wait(timeout=2.0)
        return super().subscribe(topic, callback)

    def _wire_version_ok(self, msg: Any, topic: Topic) -> bool:
        if str(topic) == str(self.topicgen("y", True)):
            assert self.x_subscribing.wait(timeout=2.0)
            self.y_dispatching.set()
        return super()._wire_version_ok(msg, topic)


def test_first_call_overlapping_another_methods_reply() -> None:
    rpc = _OverlapRPC()
    rpc.start()
    try:
        rpc.serve_rpc(lambda: "x", "x")
        rpc.serve_rpc(lambda: "y", "y")
        replies: dict[str, Any] = {}
        y_replied = threading.Event()
        x_replied = threading.Event()

        def on_y(res: Any) -> None:
            replies["y"] = res
            y_replied.set()

        def on_x(res: Any) -> None:
            replies["x"] = res
            x_replied.set()

        rpc.call_cb("y", ([], {}), on_y)
        caller = threading.Thread(target=rpc.call_cb, args=("x", ([], {}), on_x), daemon=True)
        caller.start()
        caller.join(timeout=5.0)
        assert not caller.is_alive(), "first call to x deadlocked against the y reply"
        assert y_replied.wait(timeout=5.0)
        assert x_replied.wait(timeout=5.0)
        assert replies == {"x": "x", "y": "y"}
    finally:
        rpc.stop()
