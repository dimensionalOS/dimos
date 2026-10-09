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
from typing import Any

from experimental.gateway.utils.topic_rates import TopicWatch, parse_key


class FakeSession:
    def __init__(self, declared: list[str] | None = None) -> None:
        self.callback: Any = None
        self.declared = declared or []

    def declare_subscriber(self, key: str, callback: Any) -> Any:
        assert key == "dimos/**"
        self.callback = callback
        return SimpleNamespace(undeclare=lambda: None)

    def put(self, key: str, size: int) -> None:
        self.callback(SimpleNamespace(key_expr=key, payload=b"x" * size))

    def liveliness(self) -> Any:
        replies = [SimpleNamespace(ok=SimpleNamespace(key_expr=key)) for key in self.declared]
        return SimpleNamespace(get=lambda selector, timeout: replies)


def test_parse_key() -> None:
    assert parse_key("dimos/lidar/sensor_msgs.PointCloud2") == ("/lidar", "sensor_msgs.PointCloud2")
    assert parse_key("dimos/a/b/std_msgs.Bool") == ("/a/b", "std_msgs.Bool")
    for other in [
        "dimos/rpc/Module/call",
        "dimos/x",
        "other/x/a.B",
        "dimos/x/notatype",
        "dimos/x/@adv/pub/a.B",
    ]:
        assert parse_key(other) is None


def test_a_topic_heard_once_stays_listed_after_it_goes_quiet() -> None:
    now = [100.0]
    session = FakeSession(declared=["dimos/goal/geometry_msgs.PoseStamped/@adv/pub/abc"])
    watch = TopicWatch(lambda: session, clock=lambda: now[0])
    watch.start()
    for _ in range(4):
        session.put("dimos/odom/geometry_msgs.PoseStamped", 100)
    session.put("dimos/clicked_point/geometry_msgs.PointStamped", 40)
    rows = {row["topic"]: row for row in watch.snapshot()["topics"]}
    assert (
        rows["/odom"]["hz"] == 2.0
        and rows["/odom"]["bps"] == 200.0
        and rows["/odom"]["messages"] == 4
    )
    assert rows["/clicked_point"]["messages"] == 1
    # declared but never published: listed, never heard
    assert rows["/goal"]["declared"] and rows["/goal"]["lastSeen"] is None
    now[0] += 30
    rows = {row["topic"]: row for row in watch.snapshot()["topics"]}
    assert rows["/clicked_point"]["hz"] == 0 and rows["/clicked_point"]["lastSeen"] == 30.0
    assert rows["/odom"]["messages"] == 4


def test_a_bus_that_cant_open_says_so() -> None:
    def fail() -> Any:
        raise RuntimeError("no zenoh")

    answer = TopicWatch(fail).snapshot()
    assert answer["up"] is False and "no zenoh" in answer["error"] and answer["topics"] == []
