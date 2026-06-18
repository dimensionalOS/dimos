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

import asyncio
from typing import Any

import pytest

from dimos.core.module import Module, WebHostSpec, web_init, web_module
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.web.ts_bridge.ts_bridge_module import (
    ClientSubscription,
    DimosWebsocket,
    _Connection,
    binary_frame,
    media_type_for,
    merge_tf,
    passes_filter,
    rpc_exposed,
    topic_name,
)


def _sub(whitelist=None, blacklist=None, qos=None, rate_limit=None) -> ClientSubscription:
    return ClientSubscription(whitelist or [], blacklist or [], qos or {}, rate_limit or {})


class FakeWebSocket:
    """Captures what a connection's sender writes to the wire."""

    def __init__(self) -> None:
        self.frames: list[bytes] = []
        self.json: list[dict[str, Any]] = []

    async def send_bytes(self, data: bytes) -> None:
        self.frames.append(data)

    async def send_json(self, data: dict[str, Any]) -> None:
        self.json.append(data)


class FakePubSub:
    """A minimal encoding pubsub: prefixes the payload so we can assert on it."""

    def subscribe_all(self, callback: Any, /) -> Any:
        return lambda: None

    def encode(self, message: Any, topic: Any, /) -> bytes:
        return b"ENC:" + message


class FakeTopic:
    def __init__(self, name: str) -> None:
        self.name = name


class FakeWebHost:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, bytes]]] = []

    def web_register(self, name: str, files: dict[str, bytes]) -> None:
        self.calls.append((name, files))


async def _drain(connection: _Connection) -> FakeWebSocket:
    """Run the connection's sender long enough to flush its outbox, then stop."""
    task = asyncio.create_task(connection.sender())
    await asyncio.sleep(0.02)
    task.cancel()
    return connection.websocket  # type: ignore[return-value]


# --- filtering -------------------------------------------------------------


def test_whitelist_and_blacklist_together_is_rejected() -> None:
    with pytest.raises(ValueError):
        _sub(whitelist=["/a"], blacklist=["/b"])


def test_whitelist_only_passes_listed_streams() -> None:
    sub = _sub(whitelist=["/color_image"])
    assert sub.allows("/color_image")
    assert not sub.allows("/lidar")


def test_blacklist_blocks_listed_streams() -> None:
    sub = _sub(blacklist=["/lidar"])
    assert not sub.allows("/lidar")
    assert sub.allows("/color_image")


def test_no_lists_allows_everything() -> None:
    assert _sub().allows("/anything")


# --- QoS -------------------------------------------------------------------


def test_rate_limit_throttles_within_interval() -> None:
    sub = _sub(rate_limit={"/color_image": 10})
    assert sub.rate_ok("/color_image", now=0.0)
    assert not sub.rate_ok("/color_image", now=0.05)  # within 100ms window
    assert sub.rate_ok("/color_image", now=0.11)  # past the window


def test_rate_unlisted_stream_is_unthrottled() -> None:
    sub = _sub(rate_limit={"/color_image": 10})
    assert sub.rate_ok("/lidar", now=0.0)
    assert sub.rate_ok("/lidar", now=0.0)


def test_qos_rate_takes_priority_and_throttles() -> None:
    sub = _sub(qos={"/tf": {"rate": 5}})
    assert sub.qos_for("/tf").rate == 5
    assert sub.rate_ok("/tf", now=0.0)
    assert not sub.rate_ok("/tf", now=0.1)  # 5 Hz -> 200ms window


def test_qos_reliable_defaults_to_depth_ten() -> None:
    sub = _sub(qos={"/tf": {"reliability": "reliable"}})
    profile = sub.qos_for("/tf")
    assert profile.reliability == "reliable"
    assert profile.depth == 10


def test_qos_best_effort_defaults_to_depth_one() -> None:
    assert _sub(qos={"/tf": {}}).qos_for("/tf").depth == 1


def test_qos_durability_transient_local_parsed() -> None:
    sub = _sub(qos={"/tf": {"durability": "transient_local"}})
    assert sub.qos_for("/tf").durability == "transient_local"


def test_qos_default_for_unmatched_stream_is_volatile_best_effort() -> None:
    profile = _sub(qos={"/tf": {"reliability": "reliable"}}).qos_for("/other")
    assert profile.reliability == "best_effort"
    assert profile.durability == "volatile"


def test_update_qos_changes_resolved_profile_at_runtime() -> None:
    sub = _sub(qos={"/tf": {"rate": 30}})
    assert sub.qos_for("/tf").rate == 30
    sub.update_qos({"/tf": {"rate": 5, "durability": "transient_local"}})
    assert sub.qos_for("/tf").rate == 5  # re-resolved, not the stale cached value
    assert sub.qos_for("/tf").durability == "transient_local"


# --- backend exposure control ----------------------------------------------


def test_passes_filter_whitelist_blacklist() -> None:
    assert passes_filter("/tf", ["/tf"], [])
    assert not passes_filter("/lidar", ["/tf"], [])
    assert not passes_filter("/tf", [], ["/tf"])
    assert passes_filter("/anything", [], [])


def test_passes_filter_supports_globs() -> None:
    assert passes_filter("/sensors/lidar", ["/sensors/*"], [])
    assert not passes_filter("/sensors/lidar/raw", ["/sensors/*"], [])  # * stops at /
    assert passes_filter("/sensors/lidar/raw", ["/sensors/**"], [])  # ** spans /
    assert not passes_filter("/cmd_vel", ["/sensors/*"], [])


def test_rpc_exposed_hides_internal_and_private_methods() -> None:
    assert not rpc_exposed("GO2Connection", "start", [], [])
    assert not rpc_exposed("GO2Connection", "set_transport", [], [])
    assert not rpc_exposed("GO2Connection", "_secret", [], [])
    assert rpc_exposed("GO2Connection", "standup", [], [])  # default allow


def test_rpc_exposed_whitelist_by_module_method_and_glob() -> None:
    assert rpc_exposed("GO2Connection", "standup", ["GO2Connection"], [])  # whole module
    assert rpc_exposed("GO2Connection", "standup", ["GO2Connection.standup"], [])  # exact
    assert rpc_exposed("GO2Connection", "standup", ["GO2Connection.*"], [])  # glob
    assert not rpc_exposed("GO2Connection", "standup", ["OtherModule.*"], [])


def test_rpc_exposed_blacklist() -> None:
    assert not rpc_exposed("GO2Connection", "shutdown", [], ["GO2Connection.shutdown"])
    assert rpc_exposed("GO2Connection", "standup", [], ["GO2Connection.shutdown"])


# --- wire framing ----------------------------------------------------------


def test_binary_frame_prefixes_stream_name() -> None:
    frame = binary_frame("/tf", b"\xaa\xbb")
    name_len = int.from_bytes(frame[:2], "big")
    assert name_len == len("/tf")
    assert frame[2 : 2 + name_len] == b"/tf"
    assert frame[2 + name_len :] == b"\xaa\xbb"


# --- TF aggregation (every published frame reaches the web) -----------------


def test_merge_tf_accumulates_partial_messages_without_dropping_frames() -> None:
    frames: dict[tuple[str, str], object] = {}
    merge_tf(frames, TFMessage(Transform(frame_id="world", child_frame_id="a")))
    merge_tf(frames, TFMessage(Transform(frame_id="world", child_frame_id="b")))
    merged = merge_tf(frames, TFMessage(Transform(frame_id="a", child_frame_id="a2")))
    pairs = {(t.frame_id, t.child_frame_id) for t in merged.transforms}
    assert pairs == {("world", "a"), ("world", "b"), ("a", "a2")}


def test_merge_tf_keeps_latest_transform_per_frame() -> None:
    frames: dict[tuple[str, str], object] = {}
    merge_tf(frames, TFMessage(Transform(frame_id="world", child_frame_id="a", ts=1.0)))
    merged = merge_tf(frames, TFMessage(Transform(frame_id="world", child_frame_id="a", ts=2.0)))
    assert len(merged.transforms) == 1
    assert merged.transforms[0].ts == 2.0


# --- topic naming ----------------------------------------------------------


def test_topic_name_strips_lcm_type_suffix() -> None:
    assert topic_name(FakeTopic("/tf#tf2_msgs.TFMessage")) == "/tf"


def test_topic_name_passes_plain_name_through() -> None:
    assert topic_name(FakeTopic("/cmd_vel")) == "/cmd_vel"


# --- bundle media types (favicon can be any image, sniffed) ----------------


def test_media_type_by_extension() -> None:
    assert media_type_for("index.html", b"<h1>") == "text/html"


def test_media_type_sniffs_png_without_extension() -> None:
    assert media_type_for("icon", b"\x89PNG\r\n\x1a\n\x00\x00") == "image/png"


def test_media_type_sniffs_svg_without_extension() -> None:
    assert media_type_for("icon", b'<svg xmlns="http://www.w3.org/2000/svg"/>') == "image/svg+xml"


def test_media_type_falls_back_to_octet_stream() -> None:
    assert media_type_for("blob", b"\x07\x08\x09") == "application/octet-stream"


# --- @web_module / @web_init -----------------------------------------------


def test_web_module_rejects_non_module() -> None:
    with pytest.raises(TypeError, match="Module subclass"):

        @web_module  # type: ignore[arg-type]
        class NotAModule:
            pass


def test_web_module_requires_a_web_init_method() -> None:
    with pytest.raises(TypeError, match="web_init"):

        @web_module
        class Page(Module):
            pass


def test_web_module_injects_web_host_reference() -> None:
    @web_module
    class Page(Module):
        @web_init
        def _web_init(self) -> dict[str, Any]:
            return {}

    assert Page.__annotations__["_web_host"] is WebHostSpec


def test_web_module_start_reads_files_and_registers_bundle(tmp_path: Any) -> None:
    (tmp_path / "index.html").write_text("<h1>hi</h1>")
    (tmp_path / "logo.svg").write_bytes(b"<svg/>")

    @web_module
    class Page(Module):
        @web_init
        def _web_init(self) -> dict[str, Any]:
            return {"index.html": tmp_path / "index.html", "icon": tmp_path / "logo.svg"}

    host = FakeWebHost()
    page = Page()
    page._web_host = host  # type: ignore[attr-defined]  # injected by the blueprint in production
    page.start()
    try:
        name, files = host.calls[0]
        assert name == "Page"
        assert files == {"index.html": b"<h1>hi</h1>", "icon": b"<svg/>"}
    finally:
        page.stop()


# --- per-connection streaming (outbox depth, filter, framing) --------------


async def test_connection_depth_one_coalesces_to_newest() -> None:
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub(qos={"/tf": {"depth": 1}})
    connection._accept("/tf", b"a")
    connection._accept("/tf", b"b")  # depth 1 -> only the newest survives
    websocket = await _drain(connection)
    assert websocket.frames == [binary_frame("/tf", b"b")]


async def test_connection_reliable_preserves_buffered_order() -> None:
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub(qos={"/tf": {"reliability": "reliable"}})  # depth 10
    for payload in (b"a", b"b", b"c"):
        connection._accept("/tf", payload)
    websocket = await _drain(connection)
    assert websocket.frames == [binary_frame("/tf", p) for p in (b"a", b"b", b"c")]


async def test_connection_drops_streams_outside_its_filter() -> None:
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub(whitelist=["/tf"])
    connection._accept("/lidar", b"x")  # not allowed
    connection._accept("/tf", b"y")
    websocket = await _drain(connection)
    assert websocket.frames == [binary_frame("/tf", b"y")]


# --- bridge fan-out, transient_local replay, backend gate ------------------


@pytest.fixture
def bridge() -> Any:
    """A bridge with no real pubsub (tests drive `_on_message` directly)."""
    instance = DimosWebsocket(pubsubs=[])
    try:
        yield instance
    finally:
        instance.stop()


async def test_on_message_encodes_caches_and_fans_out(bridge: DimosWebsocket) -> None:
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub()  # allow all
    bridge._connections.add(connection)

    task = asyncio.create_task(connection.sender())
    bridge._on_message(b"hello", FakeTopic("/tf"), FakePubSub())
    await asyncio.sleep(0.02)
    task.cancel()

    assert bridge._latest["/tf"] == b"ENC:hello"
    assert connection.websocket.frames == [binary_frame("/tf", b"ENC:hello")]  # type: ignore[attr-defined]


async def test_on_message_backend_blacklist_blocks_stream(bridge: DimosWebsocket) -> None:
    bridge.config.blacklist.append("/tf")
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub()
    bridge._connections.add(connection)

    task = asyncio.create_task(connection.sender())
    bridge._on_message(b"hello", FakeTopic("/tf"), FakePubSub())
    await asyncio.sleep(0.02)
    task.cancel()

    assert "/tf" not in bridge._latest
    assert connection.websocket.frames == []  # type: ignore[attr-defined]


async def test_transient_local_replays_only_durable_streams_to_late_joiner(
    bridge: DimosWebsocket,
) -> None:
    bridge._latest["/tf"] = b"snapshot"
    bridge._latest["/volatile"] = b"ignored"
    connection = _Connection(FakeWebSocket(), asyncio.get_running_loop())
    connection.subscription = _sub(qos={"/tf": {"durability": "transient_local"}})

    task = asyncio.create_task(connection.sender())
    bridge._deliver_transient_local(connection)
    await asyncio.sleep(0.02)
    task.cancel()

    assert connection.websocket.frames == [binary_frame("/tf", b"snapshot")]  # type: ignore[attr-defined]
