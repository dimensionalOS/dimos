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

"""RelayBridgeModule unit tests: no network, no Deno, no LCM.

A fake relay client is injected under `connect_with_backoff` and fake
transports under the module's `In` streams, so lazy subscribe/unsubscribe,
the maxHz gate, the encode path, and reconnect are all observable directly.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
from dataclasses import replace
import json
from pathlib import Path
import socket
import subprocess
import sys
import textwrap
import threading
import time
from types import SimpleNamespace
from typing import Any
import zlib

from langchain_core.messages import (
    AIMessage,
    AIMessageChunk,
    BaseMessage,
    ChatMessage,
    FunctionMessage,
    HumanMessage,
    HumanMessageChunk,
    SystemMessage,
    SystemMessageChunk,
    ToolMessage,
    ToolMessageChunk,
)
import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.mujoco.constants import VIDEO_FPS
from dimos.web.relay_bridge import relay_bridge_module
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.manifest import ManifestError, parse_manifest
from dimos.web.relay_bridge.protocol import (
    Msg,
    Stop as WireStop,
    Subs,
    TeleopStart as WireTeleopStart,
    TeleopStop as WireTeleopStop,
    Twist as WireTwist,
    Tx,
)
from dimos.web.relay_bridge.relay_bridge_module import (
    CHANNELS,
    TX_CHANNELS,
    RelayBridgeConfig,
    RelayBridgeModule,
    default_manifest,
    resolve_robot_info,
    with_relay_bridge,
)
from dimos.web.relay_bridge.wt_client import RelayRejectedError


class FakeWriter:
    def __init__(self) -> None:
        self.offers: list[tuple[bytes, dict[str, Any] | None]] = []
        # ts kept apart so offer-list equality asserts ignore it (a replay
        # carries its source arrival time, a live frame None).
        self.tss: list[float | None] = []

    def offer(
        self, payload: bytes, meta: dict[str, Any] | None = None, ts: float | None = None
    ) -> None:
        self.offers.append((payload, meta))
        self.tss.append(ts)


class FakeClient:
    """Duck-typed RelayClient: everything the module touches, nothing else."""

    def __init__(self, hello_error: Exception | None = None) -> None:
        self.hello_args: tuple[Any, Any] | None = None
        self.hello_error = hello_error
        self.control_msgs: asyncio.Queue[Msg] = asyncio.Queue()
        self.closed = asyncio.Event()
        self.writers: dict[str, FakeWriter] = {}
        self.frames: list[tuple[str, bytes, str, dict[str, Any] | None]] = []
        self.close_count = 0

    async def hello(self, timeout: float = 5.0, *, robot: Any = None, manifest: Any = None) -> None:
        self.hello_args = (robot, manifest)
        if self.hello_error is not None:
            raise self.hello_error

    def latest_writer(self, ch: str, *, stale_after: float = 0.5) -> FakeWriter:
        writer = FakeWriter()
        self.writers[ch] = writer
        return writer

    def send_frame(
        self,
        ch: str,
        payload: bytes,
        *,
        delivery: str = "reliable",
        meta: dict[str, Any] | None = None,
        ts: float | None = None,
    ) -> int:
        self.frames.append((ch, bytes(payload), delivery, meta))
        return 1

    async def control_messages(self) -> AsyncIterator[Msg]:
        while True:
            get = asyncio.ensure_future(self.control_msgs.get())
            closed = asyncio.ensure_future(self.closed.wait())
            try:
                done, _ = await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
            finally:
                closed.cancel()
                if not get.done():
                    get.cancel()
            if get in done:
                yield get.result()
                continue
            return

    async def close(self) -> None:
        self.close_count += 1
        self.closed.set()


class FakeTransport:
    """In-stream transport stub: counts subscribers, publishes synchronously
    (the test thread plays the LCM callback thread)."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []
        self.unsubscribed = 0
        self.unsubscribe_attempts = 0
        self.unsubscribe_error: Exception | None = None

    def subscribe(self, cb: Callable[[Any], Any], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)

        def unsubscribe() -> None:
            self.unsubscribe_attempts += 1
            if self.unsubscribe_error is not None:
                raise self.unsubscribe_error
            self.subscribers.remove(cb)
            self.unsubscribed += 1

        return unsubscribe

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def stop(self) -> None:  # called by In.stop() during module close
        self.subscribers.clear()


class FakeRelay:
    """RelayProcess stand-in for the respawn/teardown paths."""

    def __init__(self, running: bool) -> None:
        self.running = running
        self.stops = 0

    def is_running(self) -> bool:
        return self.running

    def poll(self) -> int | None:
        return None  # what a FAILED start reads: no process at all

    def stop(self) -> None:
        self.stops += 1
        self.running = False


def wait_until(cond: Callable[[], bool], timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def flush_loop(module: RelayBridgeModule) -> None:
    """Wait until all callbacks already queued on the module loop have run."""
    loop = module._loop
    assert loop is not None
    flushed = threading.Event()
    loop.call_soon_threadsafe(flushed.set)
    assert flushed.wait(timeout=5.0)


def _make_bridge(
    monkeypatch,
    *,
    wire: tuple[str, ...] = ("color_image", "odom"),
    available_channels: tuple[str, ...] | None = None,
    manifest: dict[str, Any] | None = None,
    hello_errors: tuple[Exception | None, ...] = (),
    relay: FakeRelay | None = None,
) -> tuple[RelayBridgeModule, list[FakeClient]]:
    clients: list[FakeClient] = []

    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        error = hello_errors[len(clients)] if len(clients) < len(hello_errors) else None
        clients.append(FakeClient(hello_error=error))
        return clients[-1]

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)
    module = RelayBridgeModule(
        relay_url="https://127.0.0.1:1",
        open_browser=False,
        robot_id="unit-bot",
        available_channels=available_channels,
        manifest=manifest,
    )
    module._relay = relay
    for ch in wire:
        getattr(module, ch).transport = FakeTransport()
    module.start()
    return module, clients


@pytest.fixture
def bridge(monkeypatch):
    module, clients = _make_bridge(monkeypatch)
    try:
        yield module, clients
    finally:
        stop_module(module)


def push(module: RelayBridgeModule, client: FakeClient, msg: Msg) -> None:
    """Deliver a relay push onto the module's own event loop (queue affinity)."""
    assert module._loop is not None
    module._loop.call_soon_threadsafe(client.control_msgs.put_nowait, msg)


def kill_session(module: RelayBridgeModule, client: FakeClient) -> None:
    assert module._loop is not None
    module._loop.call_soon_threadsafe(client.closed.set)


def image_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.color_image.transport
    assert isinstance(transport, FakeTransport)
    return transport


def odom_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.odom.transport
    assert isinstance(transport, FakeTransport)
    return transport


def costmap_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.global_costmap.transport
    assert isinstance(transport, FakeTransport)
    return transport


def test_manifest_and_robot_info_content() -> None:
    config = RelayBridgeConfig(robot_id="go2-lab", robot_name="Lab", image_max_hz=12.0)
    # A video panel for the camera and a map2d panel binding costmap + pose;
    # rates and quality flow from the config fields.
    assert default_manifest(config, ("color_image", "odom", "global_costmap")) == {
        "version": 1,
        "channels": [
            {
                "ch": "color_image",
                "dir": "rx",
                "encoding": "jpeg.v1",
                "delivery": "latest",
                "maxHz": 12.0,
                "params": {"quality": 75},
            },
            {
                "ch": "odom",
                "dir": "rx",
                "encoding": "pose.json.v1",
                "delivery": "reliable",
                "maxHz": 20.0,
                "params": {},
            },
            {
                "ch": "global_costmap",
                "dir": "rx",
                "encoding": "costmap.zlib.v1",
                "delivery": "latest",
                "maxHz": 5.0,
                "params": {},
            },
        ],
        "panels": [
            {"id": "p0", "kind": "video", "title": "", "channels": ["color_image"], "params": {}},
            {
                "id": "p1",
                "kind": "map2d",
                "title": "",
                "channels": ["global_costmap", "odom"],
                "params": {},
            },
        ],
        "layout": {"row": ["p0", "p1"], "shares": [2, 1]},
        "pages": [],
    }

    info = resolve_robot_info(config)
    assert (info.id, info.name) == ("go2-lab", "Lab")
    # Fallback chain: explicit id -> global robot_id -> hostname.
    fallback = resolve_robot_info(RelayBridgeConfig())
    assert fallback.id == (RelayBridgeConfig().g.robot_id or socket.gethostname())
    assert fallback.name == fallback.id


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("image_max_hz", 0.0),
        ("image_max_hz", -1.0),
        ("odom_max_hz", 0.0),
        ("odom_max_hz", -1.0),
        ("costmap_max_hz", 0.0),
        ("costmap_max_hz", -1.0),
        ("jpeg_quality", -1),
        ("jpeg_quality", 101),
    ],
)
def test_config_rejects_out_of_range_values(field: str, value: float) -> None:
    with pytest.raises(ValidationError):
        RelayBridgeConfig(**{field: value})


def test_start_registers_but_subscribes_nothing(bridge) -> None:
    # Lazy-encode guard: if someone ever adds handle_color_image/handle_odom,
    # _auto_bind_handlers would eagerly subscribe here and this fails.
    module, clients = bridge
    robot, manifest = clients[0].hello_args
    assert robot.id == "unit-bot"
    assert isinstance(manifest, dict) and len(manifest["channels"]) == 2
    assert image_transport(module).subscribers == []
    assert odom_transport(module).subscribers == []


def test_subs_snapshot_toggles_subscriptions(bridge) -> None:
    module, clients = bridge
    push(module, clients[0], Subs(chs=["odom"], n=1))
    assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)
    assert image_transport(module).subscribers == []

    # A stale (already-seen n) snapshot must be ignored.
    push(module, clients[0], Subs(chs=[], n=1))
    time.sleep(0.1)
    assert len(odom_transport(module).subscribers) == 1

    push(module, clients[0], Subs(chs=[], n=2))
    assert wait_until(lambda: odom_transport(module).subscribers == [])


def test_unknown_channels_in_snapshot_are_ignored(bridge) -> None:
    module, clients = bridge
    push(module, clients[0], Subs(chs=["mystery", "odom"], n=1))
    assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)
    assert image_transport(module).subscribers == []


def test_encode_paths_and_max_hz_gate(bridge) -> None:
    module, clients = bridge
    client = clients[0]
    push(module, client, Subs(chs=["color_image", "odom"], n=1))
    assert wait_until(
        lambda: image_transport(module).subscribers and odom_transport(module).subscribers
    )

    pose = PoseStamped(ts=42.5, position=[1.5, -2.5, 0.25], orientation=[0.0, 0.0, 0.0, 1.0])
    odom_transport(module).publish(pose)
    assert module.encoded["odom"] == 1
    assert wait_until(lambda: len(client.frames) == 1)
    ch, payload, delivery, _ = client.frames[0]
    assert (ch, delivery) == ("odom", "reliable")
    decoded = json.loads(payload)
    assert decoded == {"x": 1.5, "y": -2.5, "z": 0.25, "yaw": 0.0, "ts": 42.5}

    image = Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8))
    image_transport(module).publish(image)
    assert module.encoded["color_image"] == 1
    assert wait_until(lambda: client.writers["color_image"].offers)
    jpeg, meta = client.writers["color_image"].offers[0]
    assert jpeg[:2] == b"\xff\xd8"  # JPEG magic: TurboJPEG really encoded
    assert meta == {"w": 12, "h": 8}

    # maxHz gate: the first publish warmed the encode path (lazy imports cost
    # ~250 ms), so this back-to-back pair reliably lands inside the 50 ms
    # interval - exactly one of the two encodes.
    time.sleep(0.06)
    count = module.encoded["odom"]
    odom_transport(module).publish(pose)
    odom_transport(module).publish(pose)
    assert module.encoded["odom"] == count + 1

    # After the interval passes, encoding resumes.
    time.sleep(0.06)
    odom_transport(module).publish(pose)
    assert wait_until(lambda: module.encoded["odom"] == count + 2)


def test_default_image_gate_preserves_mujoco_video_rate() -> None:
    config = RelayBridgeConfig()
    last_input: dict[str, float] = {}
    times = [100.0 + frame / VIDEO_FPS for frame in range(VIDEO_FPS)]
    accepted = [
        now
        for now in times
        if relay_bridge_module._passes_rate_gate(
            last_input,
            "color_image",
            now,
            1.0 / config.image_max_hz,
        )
    ]

    assert accepted == times


def test_no_jpeg_encode_while_unsubscribed(bridge, monkeypatch) -> None:
    # The ticket-mandated spy: encode work must not happen without viewers,
    # independent of the module.encoded bookkeeping.
    module, clients = bridge
    calls = {"n": 0}
    real = Image.to_jpeg_bytes

    def spy(self: Image, quality: int = 75) -> bytes:
        calls["n"] += 1
        return real(self, quality=quality)

    monkeypatch.setattr(Image, "to_jpeg_bytes", spy)
    image = Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8))
    image_transport(module).publish(image)
    image_transport(module).publish(image)
    flush_loop(module)
    assert calls["n"] == 0
    assert module.encoded["color_image"] == 0

    push(module, clients[0], Subs(chs=["color_image"], n=1))
    assert wait_until(lambda: image_transport(module).subscribers)
    image_transport(module).publish(image)
    assert calls["n"] == 1


# Covers every value class of the wire contract: -1 unknown -> 255, 0 free,
# graded cost, 100 lethal.
COSTMAP_GRID = OccupancyGrid(
    grid=np.array([[-1, 0, 50], [100, 0, -1]], dtype=np.int8),
    resolution=0.05,
    origin=Pose(-1.25, 2.5, 0.0),
    ts=42.5,
)
COSTMAP_CELLS = bytes([255, 0, 50, 100, 0, 255])


@pytest.fixture
def costmap_bridge(monkeypatch):
    module, clients = _make_bridge(monkeypatch, wire=("color_image", "odom", "global_costmap"))
    try:
        yield module, clients
    finally:
        stop_module(module)


def test_costmap_encode_roundtrip_and_meta(costmap_bridge) -> None:
    module, clients = costmap_bridge
    client = clients[0]
    push(module, client, Subs(chs=["global_costmap"], n=1))
    # 2 subscribers = the always-on raw cache + the viewer-driven encoder.
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)

    costmap_transport(module).publish(COSTMAP_GRID)
    assert module.encoded["global_costmap"] == 1
    assert wait_until(lambda: client.writers["global_costmap"].offers)
    payload, meta = client.writers["global_costmap"].offers[0]
    assert zlib.decompress(payload) == COSTMAP_CELLS
    assert meta is not None
    assert (meta["w"], meta["h"], meta["res"]) == (3, 2, 0.05)
    assert meta["origin"][:2] == [-1.25, 2.5]
    assert meta["origin"][2] == pytest.approx(0.0, abs=1e-12)


def test_costmap_empty_grid_is_skipped(costmap_bridge) -> None:
    module, clients = costmap_bridge
    push(module, clients[0], Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)

    costmap_transport(module).publish(OccupancyGrid())
    flush_loop(module)
    assert module.encoded["global_costmap"] == 0
    assert clients[0].writers["global_costmap"].offers == []


def test_no_costmap_encode_while_unsubscribed(costmap_bridge, monkeypatch) -> None:
    # The ticket-mandated spy, mirroring the jpeg one: compression must not
    # happen without viewers, independent of the module.encoded bookkeeping.
    module, clients = costmap_bridge
    calls = {"n": 0}
    real = zlib.compress

    def spy(data: Any, level: int = -1) -> bytes:
        calls["n"] += 1
        return real(data, level)

    monkeypatch.setattr(relay_bridge_module.zlib, "compress", spy)
    costmap_transport(module).publish(COSTMAP_GRID)
    costmap_transport(module).publish(COSTMAP_GRID)
    flush_loop(module)
    assert calls["n"] == 0
    assert module.encoded["global_costmap"] == 0

    push(module, clients[0], Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)
    costmap_transport(module).publish(COSTMAP_GRID)
    # Two compresses: the subscribe replayed the cached grid, then the live
    # frame encoded.
    assert calls["n"] == 2
    assert module.encoded["global_costmap"] == 1


def test_costmap_resent_on_resubscribe(costmap_bridge) -> None:
    # A channel going 0 -> 1 viewers replays the cached frame: a fresh
    # subscription must not wait for the next publish (the producer may have
    # gone quiet).
    module, clients = costmap_bridge
    client = clients[0]
    push(module, client, Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)
    costmap_transport(module).publish(COSTMAP_GRID)
    assert wait_until(lambda: len(client.writers["global_costmap"].offers) == 1)

    push(module, client, Subs(chs=[], n=2))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 1)

    push(module, client, Subs(chs=["global_costmap"], n=3))
    assert wait_until(lambda: len(client.writers["global_costmap"].offers) == 2)
    # Re-encoded from the raw cache; the counter tracks live encodes only.
    assert module.encoded["global_costmap"] == 1
    first, second = client.writers["global_costmap"].offers
    assert first == second


def test_costmap_cache_survives_reconnect(costmap_bridge) -> None:
    module, clients = costmap_bridge
    push(module, clients[0], Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)
    costmap_transport(module).publish(COSTMAP_GRID)
    assert wait_until(lambda: clients[0].writers["global_costmap"].offers)

    kill_session(module, clients[0])
    assert wait_until(lambda: len(clients) == 2)
    push(module, clients[1], Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: clients[1].writers["global_costmap"].offers)
    assert (
        clients[1].writers["global_costmap"].offers == clients[0].writers["global_costmap"].offers
    )
    assert module.encoded["global_costmap"] == 1


def test_costmap_cold_start_replay_on_first_subscribe(costmap_bridge) -> None:
    # A grid published before any viewer exists must reach the first
    # subscriber: the raw cache is always on, not tied to viewer state.
    module, clients = costmap_bridge
    client = clients[0]
    t0 = time.time()
    costmap_transport(module).publish(COSTMAP_GRID)
    t1 = time.time()
    assert module.encoded["global_costmap"] == 0

    push(module, client, Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: client.writers["global_costmap"].offers)
    payload, meta = client.writers["global_costmap"].offers[0]
    assert zlib.decompress(payload) == COSTMAP_CELLS
    assert meta is not None and (meta["w"], meta["h"]) == (3, 2)
    assert module.encoded["global_costmap"] == 0  # a replay is not a live encode
    ts = client.writers["global_costmap"].tss[0]
    assert ts is not None and t0 <= ts <= t1  # arrival time, not replay time


def test_costmap_replay_uses_message_published_while_unsubscribed(costmap_bridge) -> None:
    # Cache map A with a viewer, drop to zero viewers, publish map B: the
    # next subscriber must get B, not a stale A.
    module, clients = costmap_bridge
    client = clients[0]
    push(module, client, Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)
    costmap_transport(module).publish(COSTMAP_GRID)
    assert wait_until(lambda: len(client.writers["global_costmap"].offers) == 1)

    push(module, client, Subs(chs=[], n=2))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 1)
    grid_b = OccupancyGrid(
        grid=np.array([[100, 100, 100], [0, 0, 0]], dtype=np.int8),
        resolution=0.05,
        origin=Pose(-1.25, 2.5, 0.0),
        ts=43.0,
    )
    costmap_transport(module).publish(grid_b)

    push(module, client, Subs(chs=["global_costmap"], n=3))
    assert wait_until(lambda: len(client.writers["global_costmap"].offers) == 2)
    payload, _ = client.writers["global_costmap"].offers[1]
    assert zlib.decompress(payload) == bytes([100, 100, 100, 0, 0, 0])
    assert module.encoded["global_costmap"] == 1  # only A's live encode


def test_costmap_empty_cached_grid_is_not_replayed(costmap_bridge) -> None:
    module, clients = costmap_bridge
    client = clients[0]
    costmap_transport(module).publish(OccupancyGrid())
    push(module, client, Subs(chs=["global_costmap"], n=1))
    assert wait_until(lambda: len(costmap_transport(module).subscribers) == 2)
    flush_loop(module)
    assert client.writers["global_costmap"].offers == []


def test_stop_disposes_costmap_cache_subscription(monkeypatch) -> None:
    module, _clients = _make_bridge(monkeypatch, wire=("color_image", "odom", "global_costmap"))
    transport = costmap_transport(module)
    assert len(transport.subscribers) == 1  # the always-on raw cache
    stop_module(module)
    assert transport.unsubscribed == 1


def test_bridge_import_does_not_pull_matplotlib() -> None:
    # OccupancyGrid's visualization imports are lazy so the bridge does not
    # cost every relay worker matplotlib's ~0.5 s / ~28 MiB (review issue 9).
    code = (
        "import sys; import dimos.web.relay_bridge.relay_bridge_module; "
        "assert 'matplotlib' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_bridge_imports_without_langchain() -> None:
    # langchain-core is the `agents` extra, not a bridge dependency: a web-only
    # install (`uv sync --extra web`) must still build every existing cockpit
    # and run `--local-relay`. A None entry in sys.modules makes the import
    # raise exactly like a missing package; the stand-in base keeps the agent
    # stream declared and the chat encoder is duck-typed.
    code = textwrap.dedent(
        """
        import json, sys
        from types import SimpleNamespace
        sys.modules["langchain_core"] = None
        from dimos.web.relay_bridge import relay_bridge_module as m
        from dimos.web.cockpit import cockpit
        assert not any(
            name.split(".")[0].startswith("langchain") and mod is not None
            for name, mod in sys.modules.items()
        )
        cockpit()  # the default preset (go2-style cockpits) still compiles
        atom = m.RelayBridgeModule.blueprint().blueprints[0]
        agent = next(s for s in atom.streams if s.name == "agent")
        assert (agent.direction, agent.type) == ("in", m.BaseMessage)
        msg = SimpleNamespace(
            type="ai", content="hi", name=None, id="r1",
            tool_calls=[{"id": "c1", "name": "look", "args": {"at": "ball"}}],
        )
        payload, meta = m._encode_chat(None, m._LogEntry(msg, 0.0, 7))
        entry = json.loads(payload)
        assert meta is None and entry["n"] == 7, entry
        assert (entry["role"], entry["content"], entry["id"]) == ("ai", "hi", "r1"), entry
        assert entry["tool_calls"] == [{"id": "c1", "name": "look", "args": {"at": "ball"}}]
        """
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_agent_stream_is_typed_by_langchain_base_message() -> None:
    # With the agents extra installed the stream type must be langchain's own
    # class: autoconnect keys transports on (name, type), so anything else
    # would silently never wire McpClient.agent (Out[BaseMessage]).
    assert relay_bridge_module.BaseMessage is BaseMessage
    atom = RelayBridgeModule.blueprint().blueprints[0]
    agent = next(s for s in atom.streams if s.name == "agent")
    assert (agent.direction, agent.type) == ("in", BaseMessage)


def test_manifest_omits_pose_binding_when_odom_unwired(monkeypatch) -> None:
    module, clients = _make_bridge(monkeypatch, wire=("color_image", "global_costmap"))
    try:
        _, manifest = clients[0].hello_args
        assert isinstance(manifest, dict)
        assert [c["ch"] for c in manifest["channels"]] == ["color_image", "global_costmap"]
        map_panel = next(p for p in manifest["panels"] if p["kind"] == "map2d")
        assert map_panel["channels"] == ["global_costmap"]
        # The map2d panel survives losing its pose overlay, so the layout
        # still splits video/map.
        assert manifest["layout"] == {"row": ["p0", "p1"], "shares": [2, 1]}
    finally:
        stop_module(module)


def test_session_loss_stops_encoders_and_reconnects(bridge) -> None:
    module, clients = bridge
    push(module, clients[0], Subs(chs=["odom"], n=5))
    assert wait_until(lambda: odom_transport(module).subscribers)

    kill_session(module, clients[0])
    # Encoders stop the moment the session dies (no consumer = no work) ...
    assert wait_until(lambda: odom_transport(module).subscribers == [])
    # ... and the supervisor dials a fresh session with a reset n horizon,
    # closing the dead client first (else its UDP socket leaks until GC).
    assert wait_until(lambda: len(clients) == 2)
    assert clients[0].close_count >= 1
    push(module, clients[1], Subs(chs=["odom"], n=1))
    assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)


def test_encode_started_in_old_session_is_not_sent_to_replacement(
    monkeypatch,
) -> None:
    encode_started = threading.Event()
    release_encode = threading.Event()

    def blocking_encode(
        module: RelayBridgeModule, msg: PoseStamped
    ) -> tuple[bytes, dict[str, Any] | None]:
        encode_started.set()
        assert release_encode.wait(timeout=5.0)
        return b"old-session-frame", None

    odom = next(channel for channel in CHANNELS if channel.ch == "odom")
    monkeypatch.setattr(
        relay_bridge_module,
        "CHANNELS",
        tuple(
            replace(channel, encode=blocking_encode) if channel.ch == "odom" else channel
            for channel in CHANNELS
        ),
    )
    module, clients = _make_bridge(monkeypatch)
    publisher = threading.Thread(
        target=odom_transport(module).publish,
        args=(
            PoseStamped(
                ts=42.5,
                position=[1.5, -2.5, 0.25],
                orientation=[0.0, 0.0, 0.0, 1.0],
            ),
        ),
    )
    try:
        push(module, clients[0], Subs(chs=[odom.ch], n=1))
        assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)

        publisher.start()
        assert encode_started.wait(timeout=5.0)
        kill_session(module, clients[0])
        assert wait_until(lambda: len(clients) == 2)
        flush_loop(module)

        release_encode.set()
        publisher.join(timeout=5.0)
        assert not publisher.is_alive()
        flush_loop(module)

        assert clients[1].frames == []
    finally:
        release_encode.set()
        publisher.join(timeout=5.0)
        stop_module(module)


def test_stop_unsubscribes_and_closes(bridge) -> None:
    module, clients = bridge
    push(module, clients[0], Subs(chs=["color_image", "odom"], n=1))
    assert wait_until(lambda: image_transport(module).subscribers)

    image_tr, odom_tr = image_transport(module), odom_transport(module)
    module.stop()
    # The module's own teardown unsubscribed (not just the transports closing).
    assert (image_tr.unsubscribed, odom_tr.unsubscribed) == (1, 1)
    assert image_tr.subscribers == [] and odom_tr.subscribers == []
    assert clients[0].close_count >= 1


def test_failed_respawn_retries_until_success(bridge, monkeypatch) -> None:
    # One failed respawn must not latch respawning off: the old gate read
    # poll() - None after a failed start - and skipped the respawn branch
    # forever, dialing the dead relay's old port instead.
    module, clients = bridge
    monkeypatch.setattr(relay_bridge_module, "_RECONNECT_PAUSE_S", 0.01)
    spawns: list[int] = []

    # No default on serve_dir: the fake must keep _spawn_relay's exact arity
    # (a respawn-path call with the wrong arg count once turned into a silent
    # retry loop in production; _blocking_call's *args hides it from mypy).
    def fake_spawn(open_browser: bool, serve_dir: Path | None) -> str:
        spawns.append(1)
        if len(spawns) == 1:
            raise RuntimeError("ready-line timeout")
        module._relay = FakeRelay(running=True)
        return "https://127.0.0.1:2"

    module._relay = FakeRelay(running=False)  # the post-failed-start poison
    monkeypatch.setattr(module, "_spawn_relay", fake_spawn)
    kill_session(module, clients[0])
    assert wait_until(lambda: len(spawns) == 2)
    assert wait_until(lambda: len(clients) == 2)


def test_stop_waits_for_in_flight_respawn_and_stops_spawned_child(monkeypatch) -> None:
    module, clients = _make_bridge(monkeypatch)
    loop = module._loop
    assert loop is not None
    dead_relay = FakeRelay(running=False)
    spawned_relay = FakeRelay(running=True)
    spawn_started = threading.Event()
    release_spawn = threading.Event()
    spawn_completed = threading.Event()
    stop_entered = threading.Event()
    stop_returned = threading.Event()
    stop_errors: list[BaseException] = []
    stop_returned_before_spawn: list[bool] = []

    def blocking_spawn(open_browser: bool, serve_dir: Path | None) -> str:
        spawn_started.set()
        assert release_spawn.wait(timeout=5.0)
        module._relay = spawned_relay
        spawn_completed.set()
        return "https://127.0.0.1:2"

    real_stop_main = module._stop_main

    def observed_stop_main() -> None:
        stop_entered.set()
        real_stop_main()

    def stop() -> None:
        try:
            module.stop()
            stop_returned_before_spawn.append(not spawn_completed.is_set())
        except BaseException as error:
            stop_errors.append(error)
        finally:
            stop_returned.set()

    monkeypatch.setattr(module, "_spawn_relay", blocking_spawn)
    monkeypatch.setattr(module, "_stop_main", observed_stop_main)
    module._relay = dead_relay
    kill_session(module, clients[0])
    assert spawn_started.wait(timeout=5.0)

    stopper = threading.Thread(target=stop)
    stopper.start()
    try:
        assert stop_entered.wait(timeout=5.0)
        assert not stop_returned.wait(timeout=0.25)
    finally:
        release_spawn.set()
        stopper.join(timeout=5.0)
        if module._loop is not None:
            module.stop()
        if not loop.is_closed():
            loop.run_until_complete(loop.shutdown_default_executor())

    assert not stopper.is_alive()
    assert stop_errors == []
    assert stop_returned_before_spawn == [False]
    assert spawned_relay.stops >= 1
    assert not spawned_relay.is_running()


def test_stop_survives_dead_task(monkeypatch) -> None:
    # A task that already died re-raises its exception when teardown awaits
    # it; teardown must still unsubscribe inputs, close the client, and reach
    # relay.stop() instead of aborting and orphaning everything.
    crashed = threading.Event()

    async def crash(module: RelayBridgeModule) -> None:
        crashed.set()
        raise RuntimeError("watchdog crashed earlier")

    monkeypatch.setattr(RelayBridgeModule, "_watch_child", crash)
    relay = FakeRelay(running=True)
    module, clients = _make_bridge(monkeypatch, relay=relay)
    try:
        assert crashed.wait(timeout=5.0)
        push(module, clients[0], Subs(chs=["color_image", "odom"], n=1))
        assert wait_until(lambda: image_transport(module).subscribers)

        image_tr, odom_tr = image_transport(module), odom_transport(module)
        stop_module(module)
        assert (image_tr.unsubscribed, odom_tr.unsubscribed) == (1, 1)
        assert clients[0].close_count >= 1
        assert relay.stops >= 1
    finally:
        stop_module(module)


def test_unsubscribe_failure_does_not_skip_other_cleanup_or_leak_into_new_session(
    bridge,
) -> None:
    module, clients = bridge
    color = image_transport(module)
    odom = odom_transport(module)
    push(module, clients[0], Subs(chs=["color_image", "odom"], n=1))
    assert wait_until(lambda: len(color.subscribers) == 1 and len(odom.subscribers) == 1)
    old_session = module._session
    assert old_session is not None
    color.unsubscribe_error = RuntimeError("unsubscribe failed")

    kill_session(module, clients[0])
    assert wait_until(lambda: color.unsubscribe_attempts == 1)
    assert wait_until(lambda: odom.unsubscribed == 1)
    assert wait_until(lambda: len(clients) == 2)
    assert old_session.unsubs == {}

    color.publish(Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8)))
    flush_loop(module)

    assert clients[1].writers["color_image"].offers == []


def test_unwired_input_is_not_advertised_or_subscribed(monkeypatch) -> None:
    module, clients = _make_bridge(monkeypatch, wire=("odom",))
    try:
        _, manifest = clients[0].hello_args
        assert isinstance(manifest, dict)
        assert [channel["ch"] for channel in manifest["channels"]] == ["odom"]
        assert manifest["panels"] == []  # the video panel drops with its channel
        assert manifest["layout"] is None

        push(module, clients[0], Subs(chs=["color_image", "odom"], n=1))
        assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)
        assert module._session is not None
        assert "color_image" not in module._session.unsubs
        assert len(clients) == 1  # supervisor alive, session not recycled
        push(module, clients[0], Subs(chs=[], n=2))
        assert wait_until(lambda: odom_transport(module).subscribers == [])
    finally:
        stop_module(module)


def test_composition_channel_allowlist_filters_bound_inputs(monkeypatch) -> None:
    module, clients = _make_bridge(monkeypatch, available_channels=("odom",))
    try:
        _, manifest = clients[0].hello_args
        assert isinstance(manifest, dict)
        assert [channel["ch"] for channel in manifest["channels"]] == ["odom"]

        push(module, clients[0], Subs(chs=["color_image"], n=1))
        assert wait_until(lambda: module._session is not None and module._session.last_n == 1)
        assert image_transport(module).subscribers == []
    finally:
        stop_module(module)


def test_start_with_authored_manifest_rates_and_quality(monkeypatch) -> None:
    manifest = {
        "version": 1,
        "channels": [
            {
                "ch": "color_image",
                "encoding": "jpeg.v1",
                "delivery": "latest",
                "maxHz": 4.0,
                "params": {"quality": 33},
            }
        ],
        "panels": [{"id": "p0", "kind": "video", "channels": ["color_image"]}],
        "layout": "p0",
    }
    module, clients = _make_bridge(monkeypatch, wire=("color_image",), manifest=manifest)
    try:
        _, hello_manifest = clients[0].hello_args
        # The hello carries the normalized form (defaults made explicit).
        assert hello_manifest["channels"][0]["dir"] == "rx"
        assert hello_manifest["panels"][0]["title"] == ""
        assert hello_manifest["layout"] == "p0"
        # Rate and quality come from the manifest, not the config fields.
        assert module._min_interval == {"color_image": 1.0 / 4.0}
        assert module._jpeg_quality == 33
        qualities: list[int] = []
        real = Image.to_jpeg_bytes

        def spy(self: Image, quality: int = 75) -> bytes:
            qualities.append(quality)
            return real(self, quality=quality)

        monkeypatch.setattr(Image, "to_jpeg_bytes", spy)
        push(module, clients[0], Subs(chs=["color_image"], n=1))
        assert wait_until(lambda: image_transport(module).subscribers)
        image_transport(module).publish(Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8)))
        assert wait_until(lambda: qualities == [33])
    finally:
        stop_module(module)


def test_start_with_invalid_manifest_fails(monkeypatch) -> None:
    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise AssertionError("must not reach the relay with an invalid manifest")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)

    def start_with(manifest: dict[str, Any]) -> RelayBridgeModule:
        module = RelayBridgeModule(
            relay_url="https://127.0.0.1:1", robot_id="unit-bot", manifest=manifest
        )
        module.odom.transport = FakeTransport()
        try:
            module.start()
        finally:
            stop_module(module)
        return module

    with pytest.raises(ManifestError):
        start_with({"version": 2, "channels": []})
    # A channel this bridge has no encoder for (or the wrong encoding) fails
    # start too: it could never produce a frame.
    with pytest.raises(RuntimeError, match="no matching encoder"):
        start_with(
            {
                "version": 1,
                "channels": [
                    {"ch": "lidar", "encoding": "jpeg.v1", "delivery": "latest", "maxHz": 1.0}
                ],
            }
        )
    with pytest.raises(RuntimeError, match="no matching encoder"):
        start_with(
            {
                "version": 1,
                "channels": [
                    {"ch": "odom", "encoding": "jpeg.v1", "delivery": "latest", "maxHz": 1.0}
                ],
            }
        )
    with pytest.raises(RuntimeError, match="quality"):
        start_with(
            {
                "version": 1,
                "channels": [
                    {
                        "ch": "color_image",
                        "encoding": "jpeg.v1",
                        "delivery": "latest",
                        "maxHz": 1.0,
                        "params": {"quality": 101},
                    }
                ],
            }
        )


def test_advertised_unwired_channel_is_not_probed(monkeypatch) -> None:
    manifest = default_manifest(RelayBridgeConfig(), ("color_image", "odom", "global_costmap"))
    module, clients = _make_bridge(monkeypatch, wire=("odom",), manifest=manifest)
    try:
        _, hello_manifest = clients[0].hello_args
        # No runtime stream probing: everything the manifest declares is
        # advertised, wired or not.
        assert [c["ch"] for c in hello_manifest["channels"]] == [
            "color_image",
            "odom",
            "global_costmap",
        ]
        push(module, clients[0], Subs(chs=["color_image", "odom", "global_costmap"], n=1))
        assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)
        assert module._session is not None
        assert set(module._session.unsubs) == {"odom"}
        assert len(clients) == 1  # supervisor alive, no reconcile crash
        push(module, clients[0], Subs(chs=[], n=2))
        assert wait_until(lambda: odom_transport(module).subscribers == [])
    finally:
        stop_module(module)


def test_default_manifest_matches_cockpit_default_preset() -> None:
    # Drift guard: the auto-mode manifest for a fully-wired go2 must equal
    # what the authoring API's default preset produces.
    from dimos.web.cockpit import cockpit

    (atom,) = cockpit().blueprints
    assert atom.kwargs["manifest"] == default_manifest(
        RelayBridgeConfig(), ("color_image", "odom", "global_costmap", "tele_cmd_vel")
    )
    # Normalization is idempotent: the parser accepts its own output.
    assert parse_manifest(atom.kwargs["manifest"]).model_dump() == atom.kwargs["manifest"]


def test_relay_hello_rejection_stops_reconnect_attempts(monkeypatch) -> None:
    conflict = RelayRejectedError("robot_id_conflict", "already connected")
    module, clients = _make_bridge(monkeypatch, hello_errors=(None, conflict))
    try:
        kill_session(module, clients[0])
        assert wait_until(lambda: len(clients) == 2)
        flush_loop(module)
        assert module._session is None
        assert len(clients) == 2
    finally:
        stop_module(module)


def test_supervisor_survives_reconcile_error(bridge, monkeypatch) -> None:
    # An exception out of _reconcile must recycle the session (closing the old
    # client), not silently end supervision with encoders latched on.
    module, clients = bridge
    monkeypatch.setattr(relay_bridge_module, "_RECONNECT_PAUSE_S", 0.01)
    real = module._reconcile
    calls: list[int] = []

    def flaky(session: Any, want: set[str]) -> None:
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError("boom")
        real(session, want)

    monkeypatch.setattr(module, "_reconcile", flaky)
    push(module, clients[0], Subs(chs=["odom"], n=1))
    assert wait_until(lambda: len(clients) == 2)
    assert clients[0].close_count >= 1
    push(module, clients[1], Subs(chs=["odom"], n=1))
    assert wait_until(lambda: len(odom_transport(module).subscribers) == 1)


def test_port_conflict_fails_before_build(monkeypatch) -> None:
    # The port probe precedes the build: a start that cannot get the port
    # must not touch the dist another running relay is serving.
    builds: list[int] = []
    monkeypatch.setattr(
        relay_bridge_module, "ensure_web_dist", lambda *args, **kwargs: builds.append(1)
    )
    spawns: list[int] = []
    monkeypatch.setattr(
        RelayBridgeModule, "_spawn_relay", lambda self, open_browser, serve_dir: spawns.append(1)
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as blocker:
        blocker.bind(("127.0.0.1", 0))
        blocker.listen(1)
        port = blocker.getsockname()[1]
        module = RelayBridgeModule(local_port=port, open_browser=False, robot_id="unit-bot")
        module.odom.transport = FakeTransport()
        with pytest.raises(RuntimeError, match="unavailable"):
            module.start()
        stop_module(module)
    assert builds == [] and spawns == []


def test_build_cancellation_is_bounded(monkeypatch) -> None:
    # Cancelling a start mid-build must reach the build (which kills its
    # child) promptly instead of waiting out the 600 s build timeout.
    started = threading.Event()
    finished = threading.Event()

    def fake_ensure(web_dir: Path, cancel: threading.Event | None = None) -> None:
        assert cancel is not None
        started.set()
        cancel.wait(timeout=30.0)
        finished.set()

    monkeypatch.setattr(relay_bridge_module, "ensure_web_dist", fake_ensure)
    monkeypatch.setattr(relay_bridge_module, "find_web_dir", lambda: Path("/nonexistent"))
    module = RelayBridgeModule(relay_url="https://127.0.0.1:1", open_browser=False)
    try:
        assert module._loop is not None
        future = asyncio.run_coroutine_threadsafe(module._build_web_dist(), module._loop)
        assert started.wait(timeout=5.0)
        future.cancel()
        assert finished.wait(timeout=5.0)  # the cancel event reached the build
        assert wait_until(lambda: module._build_cancel is None)
    finally:
        stop_module(module)


def test_close_cancels_in_flight_build() -> None:
    # stop() racing a still-starting main() (start blocked in the build) must
    # cancel the build via _close_module rather than wait for its timeout.
    module = RelayBridgeModule(relay_url="https://127.0.0.1:1", open_browser=False)
    cancel = threading.Event()
    module._build_cancel = cancel
    stop_module(module)
    assert cancel.is_set()


def test_serve_dir_rejected_with_relay_url(tmp_path: Path) -> None:
    # serve_dir is a local-relay feature; silently ignoring it against an
    # external relay would leave the user's page unserved.
    module = RelayBridgeModule(
        relay_url="https://127.0.0.1:1", serve_dir=str(tmp_path), open_browser=False
    )
    with pytest.raises(RuntimeError, match="serve_dir requires"):
        module.start()
    stop_module(module)


def test_missing_serve_dir_fails_before_build_and_spawn(monkeypatch, tmp_path: Path) -> None:
    # A typo'd directory must produce the labeled error, not a minute-long
    # build followed by a relay child dying with a stderr dump.
    builds: list[int] = []
    monkeypatch.setattr(
        relay_bridge_module, "ensure_web_dist", lambda *args, **kwargs: builds.append(1)
    )
    spawns: list[int] = []
    monkeypatch.setattr(
        RelayBridgeModule, "_spawn_relay", lambda self, open_browser, serve_dir: spawns.append(1)
    )
    module = RelayBridgeModule(
        local_port=0, open_browser=False, serve_dir=str(tmp_path / "nope"), robot_id="unit-bot"
    )
    with pytest.raises(RuntimeError, match="serve_dir does not exist"):
        module.start()
    stop_module(module)
    assert builds == [] and spawns == []


def test_failed_start_stops_spawned_relay(monkeypatch) -> None:
    # First-connect failure after a successful spawn happens before main yields;
    # its unified finally must still reap the fresh child.
    async def fail_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise OSError("connect refused")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fail_connect)
    # web_build=False: the build now runs in main() before _spawn_relay,
    # so the fake spawn below no longer shields this test from it.
    module = RelayBridgeModule(
        local_port=0, open_browser=False, web_build=False, robot_id="unit-bot"
    )
    relay = FakeRelay(running=True)

    def fake_spawn(open_browser: bool, serve_dir: Path | None) -> str:
        module._relay = relay
        return "https://127.0.0.1:2"

    monkeypatch.setattr(module, "_spawn_relay", fake_spawn)
    with pytest.raises(OSError):
        module.start()
    stop_module(module)
    assert relay.stops == 1


# Teleop (the tele_cmd_vel tx channel).

# Short deadman window so silence tests stay fast; well above the 50 ms
# watchdog poll.
_TELEOP_TEST_WATCHDOG_MS = 120.0


def teleop_manifest(**params: Any) -> dict[str, Any]:
    merged: dict[str, Any] = {
        "maxLinear": 0.8,
        "maxAngular": 1.0,
        "boost": 2.0,
        "watchdogMs": _TELEOP_TEST_WATCHDOG_MS,
    }
    merged.update(params)
    return {
        "version": 1,
        "channels": [
            {
                "ch": "tele_cmd_vel",
                "dir": "tx",
                "encoding": "twist.json.v1",
                "delivery": "latest",
                "maxHz": 15.0,
                "params": merged,
            }
        ],
        "panels": [{"id": "p0", "kind": "teleop", "channels": ["tele_cmd_vel"]}],
        "layout": "p0",
    }


def wire_twist(vx: float, vy: float, wz: float, seq: float, gen: int | None = 1) -> WireTwist:
    # gen defaults to 1: the relay stamps every robot-bound message, so a
    # first lease's twists arrive as gen 1.
    return WireTwist(vx=vx, vy=vy, wz=wz, seq=seq, ts=time.time(), gen=gen)


@pytest.fixture
def teleop_bridge(monkeypatch):
    module, clients = _make_bridge(monkeypatch, manifest=teleop_manifest())
    twists: list[Twist] = []
    module.tele_cmd_vel.subscribe(twists.append)
    try:
        yield module, clients, twists
    finally:
        stop_module(module)


def settle(module: RelayBridgeModule) -> None:
    """Give queued teleop handling time to run before a negative assert."""
    flush_loop(module)
    time.sleep(0.03)
    flush_loop(module)


def test_teleop_twist_publishes_geometry_twist(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.4, 0.2, -0.5, seq=1))
    assert wait_until(lambda: len(twists) == 1)
    assert twists[0] == Twist(linear=Vector3(0.4, 0.2, 0.0), angular=Vector3(0.0, 0.0, -0.5))


def test_teleop_clamps_to_boost_bounds(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(100.0, -100.0, -100.0, seq=1))
    assert wait_until(lambda: len(twists) == 1)
    # maxLinear 0.8 * boost 2.0; maxAngular 1.0 * boost 2.0.
    assert twists[0] == Twist(linear=Vector3(1.6, -1.6, 0.0), angular=Vector3(0.0, 0.0, -2.0))


def test_teleop_seq_guard_drops_stale_within_live_stream(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.1, 0.0, 0.0, seq=5))
    push(module, clients[0], wire_twist(0.9, 0.0, 0.0, seq=4))  # reordered: dropped
    push(module, clients[0], wire_twist(0.2, 0.0, 0.0, seq=6))
    assert wait_until(lambda: len(twists) == 2)
    settle(module)
    assert [t.linear.x for t in twists] == [0.1, 0.2]


def test_teleop_watchdog_deadline_and_high_water_survives_silence(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=100))
    assert wait_until(lambda: len(twists) == 1)
    started = time.monotonic()
    # Silence while driving: the deadman publishes exactly one zero, within
    # the watchdog window plus its poll granularity (margin covers the 10 ms
    # wait_until poll and thread scheduling).
    assert wait_until(lambda: len(twists) == 2)
    elapsed = time.monotonic() - started
    assert twists[1].is_zero()
    deadline = _TELEOP_TEST_WATCHDOG_MS / 1000 + relay_bridge_module._TELEOP_POLL_S + 0.5
    assert elapsed < deadline, f"deadman zero took {elapsed:.3f}s (deadline {deadline:.3f}s)"
    time.sleep(3 * _TELEOP_TEST_WATCHDOG_MS / 1000)
    settle(module)
    assert len(twists) == 2  # no repeated idle zeros
    # The high-water mark survives silence: a delayed lower-seq twist from
    # the same lease stays dead, while the paused holder resumes with a
    # rising seq.
    push(module, clients[0], wire_twist(0.9, 0.0, 0.0, seq=99))
    settle(module)
    assert len(twists) == 2
    push(module, clients[0], wire_twist(0.3, 0.0, 0.0, seq=101))
    assert wait_until(lambda: len(twists) == 3)
    assert twists[2].linear.x == 0.3


def test_teleop_zero_only_on_release_edge(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    # Idle zeros never publish (MovementManager cancels a nav goal on every
    # teleop message, so a parked cockpit must stay silent).
    push(module, clients[0], wire_twist(0.0, 0.0, 0.0, seq=1))
    settle(module)
    assert twists == []
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=2))
    push(module, clients[0], wire_twist(0.0, 0.0, 0.0, seq=3))
    push(module, clients[0], wire_twist(0.0, 0.0, 0.0, seq=4))  # release burst repeat
    assert wait_until(lambda: len(twists) == 2)
    settle(module)
    assert len(twists) == 2
    assert twists[1].is_zero()


def test_teleop_stop_is_unconditional(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    # E-stop from idle still publishes (it must cancel an autonomous nav
    # goal), and every repeat publishes again.
    push(module, clients[0], WireStop(seq=1, ts=time.time(), gen=1))
    push(module, clients[0], WireStop(seq=2, ts=time.time(), gen=1))
    assert wait_until(lambda: len(twists) == 2)
    assert all(t.is_zero() for t in twists)


def test_teleop_stop_blocks_stale_reordered_twist(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=9))
    push(module, clients[0], WireStop(seq=10, ts=time.time(), gen=1))
    assert wait_until(lambda: len(twists) == 2)
    # A pre-e-stop twist arriving late must not restart the robot.
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=8))
    settle(module)
    assert len(twists) == 2
    push(module, clients[0], wire_twist(0.2, 0.0, 0.0, seq=11))
    assert wait_until(lambda: len(twists) == 3)


def test_teleop_lease_end_zeroes_once_and_resets_seq(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=50))
    assert wait_until(lambda: len(twists) == 1)
    push(module, clients[0], WireTeleopStop(gen=1))
    assert wait_until(lambda: len(twists) == 2)
    assert twists[1].is_zero()
    push(module, clients[0], WireTeleopStop(gen=1))  # repeat: dead on the gen gate
    settle(module)
    assert len(twists) == 2
    # The next holder's lease (relay bumps by exactly 1) starts a fresh seq
    # space immediately.
    push(module, clients[0], wire_twist(0.3, 0.0, 0.0, seq=1, gen=2))
    assert wait_until(lambda: len(twists) == 3)


def test_teleop_session_drop_zeroes(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=1))
    assert wait_until(lambda: len(twists) == 1)
    kill_session(module, clients[0])
    assert wait_until(lambda: len(twists) == 2 and twists[1].is_zero())
    assert wait_until(lambda: len(clients) == 2)  # supervisor reconnected
    settle(module)
    assert len(twists) == 2


def test_teleop_lease_end_blocks_stale_gen_twist_permanently(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=50))
    assert wait_until(lambda: len(twists) == 1)
    push(module, clients[0], WireTeleopStop(gen=1))
    assert wait_until(lambda: len(twists) == 2 and twists[1].is_zero())
    # Past the watchdog window, delayed twists from the released lease must
    # stay dead regardless of seq: a reordered pre-stop command must never
    # restart the robot after a stop.
    time.sleep(2 * _TELEOP_TEST_WATCHDOG_MS / 1000)
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=51))
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=999))
    settle(module)
    assert len(twists) == 2
    # The next lease is admitted immediately.
    push(module, clients[0], wire_twist(0.3, 0.0, 0.0, seq=1, gen=2))
    assert wait_until(lambda: len(twists) == 3)
    assert twists[2].linear.x == 0.3


def test_teleop_stale_gen_estop_is_void(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=10, gen=2))
    assert wait_until(lambda: len(twists) == 1)
    # An e-stop from a voided lease must not blip the current holder.
    push(module, clients[0], WireStop(seq=999, ts=time.time(), gen=1))
    settle(module)
    assert len(twists) == 1
    push(module, clients[0], wire_twist(0.2, 0.0, 0.0, seq=11, gen=2))
    assert wait_until(lambda: len(twists) == 2)
    assert twists[1].linear.x == 0.2


def test_teleop_start_adopts_new_gen_and_zeroes_lost_stop(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=50))
    assert wait_until(lambda: len(twists) == 1)
    # The lease changed hands but its teleop_stop datagram was lost: the
    # next grant's announcement stops the robot and voids the old lease.
    push(module, clients[0], WireTeleopStart(gen=2))
    assert wait_until(lambda: len(twists) == 2 and twists[1].is_zero())
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=51))  # old lease
    settle(module)
    assert len(twists) == 2
    push(module, clients[0], wire_twist(0.3, 0.0, 0.0, seq=1, gen=2))
    assert wait_until(lambda: len(twists) == 3)
    # A duplicated start (idempotent re-arm resend) must not reset the
    # high-water: the holder's own reordered twist stays dead.
    push(module, clients[0], WireTeleopStart(gen=2))
    push(module, clients[0], wire_twist(0.9, 0.0, 0.0, seq=1, gen=2))
    settle(module)
    assert len(twists) == 3


def test_teleop_estop_does_not_lower_high_water(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=12))
    assert wait_until(lambda: len(twists) == 1)
    # A stale reordered e-stop still zeroes (safe direction) but must not
    # lower the high-water and let the superseded twist 11 re-apply.
    push(module, clients[0], WireStop(seq=10, ts=time.time(), gen=1))
    assert wait_until(lambda: len(twists) == 2 and twists[1].is_zero())
    push(module, clients[0], wire_twist(0.9, 0.0, 0.0, seq=11))
    settle(module)
    assert len(twists) == 2
    push(module, clients[0], wire_twist(0.2, 0.0, 0.0, seq=13))
    assert wait_until(lambda: len(twists) == 3)
    assert twists[2].linear.x == 0.2


def test_teleop_stale_lease_end_does_not_blip_new_holder(teleop_bridge) -> None:
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=5, gen=2))
    assert wait_until(lambda: len(twists) == 1)
    # The previous lease's stop arrives late: it must neither zero nor
    # reset the current lease's state.
    push(module, clients[0], WireTeleopStop(gen=1))
    settle(module)
    assert len(twists) == 1
    push(module, clients[0], wire_twist(0.2, 0.0, 0.0, seq=6, gen=2))
    assert wait_until(lambda: len(twists) == 2)
    assert twists[1].linear.x == 0.2


def test_teleop_genless_messages_ignored(teleop_bridge) -> None:
    # Unstamped wire messages (a version-skewed relay) must never move the
    # robot or disturb the lease state.
    module, clients, twists = teleop_bridge
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=1, gen=None))
    push(module, clients[0], WireStop(seq=2, ts=time.time()))
    push(module, clients[0], WireTeleopStart())
    push(module, clients[0], WireTeleopStop())
    settle(module)
    assert twists == []
    push(module, clients[0], wire_twist(0.3, 0.0, 0.0, seq=1))
    assert wait_until(lambda: len(twists) == 1)


def test_teleop_ignored_without_a_teleop_channel(bridge) -> None:
    # No tx channel in the manifest: teleop messages are protocol noise.
    module, clients = bridge
    twists: list[Twist] = []
    module.tele_cmd_vel.subscribe(twists.append)
    push(module, clients[0], wire_twist(0.5, 0.0, 0.0, seq=1))
    push(module, clients[0], WireStop(seq=2, ts=time.time(), gen=1))
    settle(module)
    assert twists == []


def test_teleop_manifest_validation_fails_start(monkeypatch) -> None:
    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise AssertionError("must not reach the relay with an invalid manifest")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)

    def start_with(manifest: dict[str, Any]) -> None:
        module = RelayBridgeModule(
            relay_url="https://127.0.0.1:1", robot_id="unit-bot", manifest=manifest
        )
        try:
            module.start()
        finally:
            stop_module(module)

    bad_params = teleop_manifest(boost=-1.0)
    with pytest.raises(RuntimeError, match="boost"):
        start_with(bad_params)
    # A tx channel this bridge cannot handle (wrong encoding vs TX_CHANNELS)
    # fails start too - channel-only, since the teleop panel rule would
    # reject the encoding first.
    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(
            {
                "version": 1,
                "channels": [
                    {
                        "ch": "tele_cmd_vel",
                        "dir": "tx",
                        "encoding": "chat.json.v1",
                        "delivery": "latest",
                        "maxHz": 15.0,
                    }
                ],
            }
        )


def test_default_manifest_teleop_degradations() -> None:
    config = RelayBridgeConfig()
    solo = default_manifest(config, ("tele_cmd_vel",))
    assert [c["ch"] for c in solo["channels"]] == ["tele_cmd_vel"]
    assert [p["kind"] for p in solo["panels"]] == ["teleop"]
    assert solo["layout"] == "p0"
    with_video = default_manifest(config, ("color_image", "tele_cmd_vel"))
    assert [p["kind"] for p in with_video["panels"]] == ["video", "teleop"]
    assert with_video["layout"] == {"row": ["p0", "p1"], "shares": [2, 1]}


# --- Microduck cockpit channels: generic tx commands and the new rx encoders.

MICRODUCK_RX = (
    "chase_image",
    "agent",
    "agent_idle",
    "path",
    "nav_state",
    "mode",
    "places",
    "policy_state",
)
MICRODUCK_TX = ("human_input", "goal_request", "ui_command")


def channel_def(ch: str) -> relay_bridge_module.ChannelDef:
    return next(cd for cd in CHANNELS if cd.ch == ch)


def microduck_manifest(
    rx: tuple[str, ...] = MICRODUCK_RX, tx: tuple[str, ...] = MICRODUCK_TX
) -> dict[str, Any]:
    """Channel-only manifest for the Microduck streams (no panels: the
    bridge validates channels against its tables, not panel bindings)."""
    by_tx = {td.ch: td for td in TX_CHANNELS}
    channels: list[dict[str, Any]] = [
        {
            "ch": ch,
            "dir": "rx",
            "encoding": channel_def(ch).encoding,
            "delivery": channel_def(ch).delivery,
            "maxHz": channel_def(ch).max_hz(RelayBridgeConfig()),
        }
        for ch in rx
    ]
    channels += [
        {
            "ch": ch,
            "dir": "tx",
            "encoding": by_tx[ch].encoding,
            "delivery": by_tx[ch].delivery,
            "maxHz": 5.0,
        }
        for ch in tx
    ]
    return {"version": 1, "channels": channels}


def transport_of(module: RelayBridgeModule, ch: str) -> FakeTransport:
    transport = getattr(module, ch).transport
    assert isinstance(transport, FakeTransport)
    return transport


def frames_on(client: FakeClient, ch: str) -> list[dict[str, Any]]:
    """Decoded reliable frames sent on `ch`, in order."""
    return [json.loads(payload) for c, payload, _delivery, _meta in client.frames if c == ch]


def test_microduck_manifest_starts_with_every_channel(monkeypatch) -> None:
    module, clients = _make_bridge(monkeypatch, wire=MICRODUCK_RX, manifest=microduck_manifest())
    try:
        _, hello_manifest = clients[0].hello_args
        assert [c["ch"] for c in hello_manifest["channels"]] == [*MICRODUCK_RX, *MICRODUCK_TX]
        # Every tx channel but the twist goes through the generic Tx path.
        assert set(module._tx_defs) == set(MICRODUCK_TX)
        assert module._teleop_params is None
        # All the new rx channels replay on subscribe: one always-on raw
        # cache subscription each, nothing encoding yet.
        for ch in MICRODUCK_RX:
            assert len(transport_of(module, ch).subscribers) == 1, ch
        assert all(module.encoded[ch] == 0 for ch in MICRODUCK_RX)
        assert module._min_interval["agent"] == pytest.approx(1.0 / 30.0)
    finally:
        stop_module(module)


def test_cockpit_microduck_layout_starts(monkeypatch) -> None:
    # The authored Microduck cockpit (Control bar, chase camera, Chat, NavMap)
    # compiles to a manifest this bridge accepts end to end: every panel
    # binding has an encoder/handler with the matching encoding+delivery.
    from dimos.web.cockpit import Chat, Col, Control, NavMap, Row, Video, cockpit

    (atom,) = cockpit(
        layout=Col(Control(), Row(Video("chase_image"), Chat(), NavMap()), shares=[1, 12])
    ).blueprints
    module, clients = _make_bridge(monkeypatch, wire=(), manifest=atom.kwargs["manifest"])
    try:
        assert clients[0].hello_args is not None
        assert set(module._tx_defs) == set(MICRODUCK_TX)
        assert {cd.ch for cd in module._channel_defs} >= {"chase_image", "agent", "path", "mode"}
    finally:
        stop_module(module)


def test_microduck_manifest_rejects_mismatches(monkeypatch) -> None:
    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise AssertionError("must not reach the relay with an invalid manifest")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)

    def start_with(manifest: dict[str, Any]) -> None:
        module = RelayBridgeModule(
            relay_url="https://127.0.0.1:1", robot_id="unit-bot", manifest=manifest
        )
        try:
            module.start()
        finally:
            stop_module(module)

    def with_channel(ch: str, **overrides: Any) -> dict[str, Any]:
        base = microduck_manifest(rx=("agent",), tx=("human_input",))
        for channel in base["channels"]:
            if channel["ch"] == ch:
                channel.update(overrides)
        return base

    with pytest.raises(RuntimeError, match="no matching encoder"):
        start_with(with_channel("agent", encoding="flag.json.v1"))
    with pytest.raises(RuntimeError, match="no matching encoder"):
        start_with(with_channel("agent", delivery="latest"))
    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(with_channel("human_input", encoding="pose_goal.json.v1"))
    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(with_channel("human_input", delivery="latest"))
    unknown = microduck_manifest(rx=(), tx=())
    unknown["channels"].append(
        {
            "ch": "mystery",
            "dir": "tx",
            "encoding": "text.json.v1",
            "delivery": "reliable",
            "maxHz": 1.0,
        }
    )
    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(unknown)


def test_default_manifest_advertises_microduck_channels_channel_only() -> None:
    config = RelayBridgeConfig(image_max_hz=25.0)
    manifest = default_manifest(config, ("chase_image", "agent", "policy_state"))
    assert [(c["ch"], c["dir"], c["encoding"], c["maxHz"]) for c in manifest["channels"]] == [
        ("chase_image", "rx", "jpeg.v1", 25.0),
        ("agent", "rx", "chat.json.v1", 30.0),
        ("policy_state", "rx", "policy.json.v1", 10.0),
    ]
    assert manifest["panels"] == []
    # And the parser accepts the auto-mode output for them too.
    parse_manifest(manifest)


def tx(ch: str, seq: int, **data: Any) -> Tx:
    return Tx(ch=ch, seq=seq, data=data)


@pytest.fixture
def tx_bridge(monkeypatch):
    module, clients = _make_bridge(monkeypatch, wire=(), manifest=microduck_manifest(rx=()))
    texts: list[str] = []
    goals: list[PoseStamped] = []
    commands: list[str] = []
    module.human_input.subscribe(texts.append)
    module.goal_request.subscribe(goals.append)
    module.ui_command.subscribe(commands.append)
    try:
        yield module, clients, texts, goals, commands
    finally:
        stop_module(module)


def unthrottle(module: RelayBridgeModule, ch: str) -> None:
    """Lift a channel's rate floor so a test can exercise the other gates
    with back-to-back sends."""
    module._tx_defs[ch] = replace(module._tx_defs[ch], min_interval_s=0.0)


def test_tx_human_input_publishes_stripped_text(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    push(module, clients[0], tx("human_input", 1, text="  go to the kitchen \n"))
    assert wait_until(lambda: texts == ["go to the kitchen"])
    assert goals == [] and commands == []


def test_tx_goal_request_publishes_pose_stamped(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    t0 = time.time()
    push(module, clients[0], tx("goal_request", 1, x=1.5, y=-2, yaw=1.2, frame="map"))
    assert wait_until(lambda: len(goals) == 1)
    goal = goals[0]
    assert isinstance(goal, PoseStamped)
    assert goal.frame_id == "map"
    assert (goal.position.x, goal.position.y, goal.position.z) == (1.5, -2.0, 0.0)
    assert goal.yaw == pytest.approx(1.2)
    assert goal.orientation == Quaternion.from_euler(Vector3(0.0, 0.0, 1.2))
    assert t0 <= goal.ts <= time.time()
    # yaw and frame default; an int coordinate is a number too.
    time.sleep(0.25)  # the goal channel's rate floor
    push(module, clients[0], tx("goal_request", 2, x=0, y=3))
    assert wait_until(lambda: len(goals) == 2)
    assert goals[1].frame_id == "world"
    assert goals[1].yaw == pytest.approx(0.0)
    assert (goals[1].position.x, goals[1].position.y) == (0.0, 3.0)


def test_tx_ui_command_publishes_compact_json(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    push(module, clients[0], tx("ui_command", 1, name="set_mode", args={"mode": "agent"}))
    assert wait_until(lambda: len(commands) == 1)
    assert commands[0] == '{"name":"set_mode","args":{"mode":"agent"}}'
    time.sleep(0.06)  # the command channel's rate floor
    push(module, clients[0], tx("ui_command", 2, name="cancel_nav"))
    assert wait_until(lambda: len(commands) == 2)
    assert json.loads(commands[1]) == {"name": "cancel_nav", "args": {}}


def test_tx_stale_seq_dropped_while_channel_busy(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    unthrottle(module, "human_input")
    push(module, clients[0], tx("human_input", 5, text="five"))
    push(module, clients[0], tx("human_input", 4, text="four"))  # reordered: dropped
    push(module, clients[0], tx("human_input", 5, text="five again"))  # duplicated: dropped
    push(module, clients[0], tx("human_input", 6, text="six"))
    assert wait_until(lambda: len(texts) == 2)
    settle(module)
    assert texts == ["five", "six"]


def test_tx_seq_rebaselines_after_silence(tx_bridge, monkeypatch) -> None:
    # A reloaded page (or a second tab) restarts its counter at 1; without
    # a lease generation the only tell is time, so a quiet channel accepts
    # any seq again.
    module, clients, texts, goals, commands = tx_bridge
    unthrottle(module, "human_input")
    monkeypatch.setattr(relay_bridge_module, "_TX_SEQ_WINDOW_S", 0.05)
    push(module, clients[0], tx("human_input", 50, text="old tab"))
    assert wait_until(lambda: texts == ["old tab"])
    time.sleep(0.1)
    push(module, clients[0], tx("human_input", 1, text="new tab"))
    assert wait_until(lambda: texts == ["old tab", "new tab"])
    # The high-water mark followed the rebaseline: 2 is fresh now.
    push(module, clients[0], tx("human_input", 2, text="next"))
    assert wait_until(lambda: texts == ["old tab", "new tab", "next"])


def test_tx_rate_floor_drops_bursts(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    push(module, clients[0], tx("ui_command", 1, name="policy", args={"policy": "kick_left"}))
    push(module, clients[0], tx("ui_command", 2, name="policy", args={"policy": "kick_right"}))
    assert wait_until(lambda: len(commands) == 1)
    settle(module)
    assert json.loads(commands[0])["args"] == {"policy": "kick_left"}
    time.sleep(0.06)
    push(module, clients[0], tx("ui_command", 3, name="cancel_nav"))
    assert wait_until(lambda: len(commands) == 2)


@pytest.mark.parametrize(
    ("ch", "data"),
    [
        ("human_input", {"text": "   "}),
        ("human_input", {"text": ""}),
        ("human_input", {"text": 5}),
        ("human_input", {}),
        ("goal_request", {"x": 100.0, "y": 0.0}),
        ("goal_request", {"x": 0.0, "y": -50.5}),
        ("goal_request", {"x": "1", "y": 0.0}),
        ("goal_request", {"x": 1.0}),
        ("goal_request", {"x": 1.0, "y": 1.0, "yaw": "north"}),
        ("goal_request", {"x": 1.0, "y": 1.0, "frame": ""}),
        ("ui_command", {"name": "explode"}),
        ("ui_command", {"name": "policy", "args": []}),
        ("ui_command", {"args": {}}),
    ],
)
def test_tx_invalid_record_dropped(tx_bridge, ch: str, data: dict[str, Any]) -> None:
    module, clients, texts, goals, commands = tx_bridge
    unthrottle(module, ch)
    push(module, clients[0], Tx(ch=ch, seq=1, data=data))
    settle(module)
    assert (texts, goals, commands) == ([], [], [])
    # An invalid record consumes neither the seq nor the rate slot: the
    # viewer's corrected resend at the same seq goes through.
    valid = {
        "human_input": {"text": "ok"},
        "goal_request": {"x": 1.0, "y": 1.0},
        "ui_command": {"name": "cancel_nav"},
    }[ch]
    push(module, clients[0], Tx(ch=ch, seq=1, data=valid))
    assert wait_until(lambda: len(texts) + len(goals) + len(commands) == 1)
    assert len(clients) == 1  # supervisor alive


def test_tx_unhandled_channel_dropped(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    twists: list[Twist] = []
    module.tele_cmd_vel.subscribe(twists.append)
    # A Tx on the twist channel is not a twist (no lease, no params) and an
    # unknown channel has no Out: both dropped, the supervisor unharmed.
    push(module, clients[0], tx("tele_cmd_vel", 1, vx=1.0, vy=0.0, wz=0.0))
    push(module, clients[0], tx("mystery", 1, text="hi"))
    push(module, clients[0], tx("human_input", 1, text="still works"))
    assert wait_until(lambda: texts == ["still works"])
    settle(module)
    assert twists == [] and goals == [] and commands == []
    assert len(clients) == 1


def test_tx_ignored_without_tx_channels(bridge) -> None:
    module, clients = bridge
    texts: list[str] = []
    module.human_input.subscribe(texts.append)
    push(module, clients[0], tx("human_input", 1, text="hi"))
    settle(module)
    assert texts == []
    assert len(clients) == 1


def test_tx_seq_state_resets_with_the_session(tx_bridge) -> None:
    module, clients, texts, goals, commands = tx_bridge
    unthrottle(module, "human_input")
    push(module, clients[0], tx("human_input", 9, text="first session"))
    assert wait_until(lambda: texts == ["first session"])
    kill_session(module, clients[0])
    assert wait_until(lambda: len(clients) == 2)
    # New session, new viewers, counters from 1 - immediately, no window wait.
    push(module, clients[1], tx("human_input", 1, text="second session"))
    assert wait_until(lambda: texts == ["first session", "second session"])


def test_tx_twist_path_untouched_alongside_generic_tx(monkeypatch) -> None:
    manifest = teleop_manifest()
    manifest["channels"] += microduck_manifest(rx=())["channels"]
    module, clients = _make_bridge(monkeypatch, manifest=manifest)
    twists: list[Twist] = []
    texts: list[str] = []
    module.tele_cmd_vel.subscribe(twists.append)
    module.human_input.subscribe(texts.append)
    try:
        assert module._teleop_params is not None
        assert set(module._tx_defs) == set(MICRODUCK_TX)
        push(module, clients[0], wire_twist(0.4, 0.0, 0.0, seq=1))
        push(module, clients[0], tx("human_input", 1, text="hello"))
        assert wait_until(lambda: len(twists) == 1 and texts == ["hello"])
        assert twists[0].linear.x == pytest.approx(0.4)
    finally:
        stop_module(module)


def encode_chat(module: RelayBridgeModule, msg: Any) -> dict[str, Any]:
    encoded = relay_bridge_module._encode_chat(module, msg)
    assert encoded is not None
    payload, meta = encoded
    assert meta is None
    assert b": " not in payload and b", " not in payload  # compact
    return json.loads(payload)


def test_encode_chat_shapes(bridge) -> None:
    module, _clients = bridge
    t0 = time.time()
    human = encode_chat(module, HumanMessage(content="hi there"))
    assert human == {
        "n": human["n"],
        "role": "human",
        "content": "hi there",
        "name": None,
        "tool_calls": [],
        "tool_call_id": None,
        "id": None,
        "t": human["t"],
    }
    assert isinstance(human["n"], int) and t0 <= human["t"] <= time.time()

    ai = encode_chat(
        module,
        AIMessage(
            content=[
                {"type": "text", "text": "Looking."},
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}},
                {"type": "text", "text": "Found it."},
            ],
            tool_calls=[{"name": "navigate_to", "args": {"place": "kitchen"}, "id": "c1"}],
            id="run-1",
        ),
    )
    assert ai["role"] == "ai"
    assert ai["content"] == "Looking.\n[image]\nFound it."
    assert ai["tool_calls"] == [{"id": "c1", "name": "navigate_to", "args": {"place": "kitchen"}}]
    assert ai["tool_call_id"] is None and ai["id"] == "run-1"

    tool = encode_chat(module, ToolMessage(content="ok", tool_call_id="c1", name="navigate_to"))
    assert (tool["role"], tool["content"], tool["tool_call_id"], tool["name"]) == (
        "tool",
        "ok",
        "c1",
        "navigate_to",
    )
    assert tool["tool_calls"] == []

    system = encode_chat(module, SystemMessage(content="You are a duck."))
    assert (system["role"], system["content"]) == ("system", "You are a duck.")

    # n: bridge-process monotonic entry numbers.
    assert human["n"] < ai["n"] < tool["n"] < system["n"]


def test_encode_chat_caps_content(bridge) -> None:
    module, _clients = bridge
    cap = relay_bridge_module._CHAT_CONTENT_MAX_CHARS
    entry = encode_chat(module, ToolMessage(content="x" * (cap + 5000), tool_call_id="c1"))
    assert entry["content"] == "x" * cap + "\n[truncated]"
    exact = encode_chat(module, HumanMessage(content="y" * cap))
    assert exact["content"] == "y" * cap


@pytest.mark.parametrize(
    ("msg", "role"),
    [
        (HumanMessageChunk(content="h"), "human"),
        (AIMessageChunk(content="a"), "ai"),
        (ToolMessageChunk(content="t", tool_call_id="c1"), "tool"),
        (SystemMessageChunk(content="s"), "system"),
        (ChatMessage(content="c", role="narrator"), "system"),
        (FunctionMessage(content="f", name="fn"), "system"),
        # The encoder reads the message's own `type` tag, nothing langchain
        # specific: any object shaped like a message encodes.
        (SimpleNamespace(type="human", content="duck"), "human"),
        (SimpleNamespace(type="mystery", content=["a", "b"]), "system"),
    ],
    ids=lambda v: v if isinstance(v, str) else type(v).__name__,
)
def test_encode_chat_role_from_message_type(bridge, msg: Any, role: str) -> None:
    module, _clients = bridge
    entry = encode_chat(module, msg)
    assert entry["role"] == role
    assert entry["tool_calls"] == [] and entry["tool_call_id"] == getattr(msg, "tool_call_id", None)
    assert isinstance(entry["n"], int) and isinstance(entry["t"], float)


def test_encode_chat_uses_the_log_entry_number(bridge) -> None:
    # A message that went through a replay log carries its number with it;
    # a bare message (a hypothetical replay_depth 1 chat channel) is
    # numbered on the way through, after every number handed out so far.
    module, _clients = bridge
    logged = encode_chat(module, relay_bridge_module._LogEntry(HumanMessage(content="x"), 1.0, 41))
    assert logged["n"] == 41 and logged["content"] == "x"
    bare = encode_chat(module, HumanMessage(content="y"))
    assert bare["n"] >= 1
    assert encode_chat(module, HumanMessage(content="z"))["n"] == bare["n"] + 1


def test_encode_flag_and_path(bridge) -> None:
    module, _clients = bridge
    payload, meta = relay_bridge_module._encode_flag(module, True)
    flag = json.loads(payload)
    assert meta is None and flag["value"] is True and isinstance(flag["t"], float)

    def pose(i: int) -> PoseStamped:
        return PoseStamped(ts=1.0, frame_id="map", position=Vector3(float(i), -0.5 * i, 0.0))

    short = NavPath(ts=1.0, frame_id="map", poses=[pose(i) for i in range(3)])
    payload, meta = relay_bridge_module._encode_path(module, short)
    path = json.loads(payload)
    assert meta is None
    assert path["frame"] == "map"
    assert path["points"] == [[0.0, 0.0], [1.0, -0.5], [2.0, -1.0]]
    assert isinstance(path["t"], float)

    cap = relay_bridge_module._PATH_MAX_POINTS
    long = NavPath(ts=1.0, frame_id="map", poses=[pose(i) for i in range(1000)])
    payload, _ = relay_bridge_module._encode_path(module, long)
    points = json.loads(payload)["points"]
    # Uniform decimation keeps both endpoints and the order.
    assert len(points) == cap
    assert points[0] == [0.0, 0.0] and points[-1] == [999.0, -499.5]
    assert points == sorted(points)


@pytest.mark.parametrize("ch", ["nav_state", "mode", "places", "policy_state"])
def test_passthrough_encoders_validate_json(bridge, ch: str) -> None:
    module, _clients = bridge
    encode = channel_def(ch).encode
    assert encode(module, "not json") is None
    assert encode(module, "[1, 2]") is None
    assert encode(module, '"a string"') is None
    encoded = encode(module, '{"state": "idle", "goal": null, "t": 12.5}')
    assert encoded is not None
    payload, meta = encoded
    assert meta is None
    # Re-dumped compactly, the producer's own "t" kept.
    assert payload == b'{"state":"idle","goal":null,"t":12.5}'
    # A producer that forgot "t" gets the bridge clock.
    payload, _ = encode(module, '{"mode": "agent"}')
    record = json.loads(payload)
    assert record["mode"] == "agent" and isinstance(record["t"], float)


def test_passthrough_drop_is_not_a_frame(monkeypatch) -> None:
    module, clients = _make_bridge(
        monkeypatch, wire=("nav_state",), manifest=microduck_manifest(rx=("nav_state",), tx=())
    )
    try:
        client = clients[0]
        push(module, client, Subs(chs=["nav_state"], n=1))
        assert wait_until(lambda: len(transport_of(module, "nav_state").subscribers) == 2)
        transport_of(module, "nav_state").publish("garbage")
        transport_of(module, "nav_state").publish('{"state":"navigating","t":1.0}')
        assert wait_until(lambda: client.writers["nav_state"].offers)
        assert module.encoded["nav_state"] == 1
        assert client.writers["nav_state"].offers == [(b'{"state":"navigating","t":1.0}', None)]
    finally:
        stop_module(module)


@pytest.fixture
def agent_bridge(monkeypatch):
    module, clients = _make_bridge(
        monkeypatch,
        wire=("agent", "agent_idle"),
        manifest=microduck_manifest(rx=("agent", "agent_idle"), tx=()),
    )
    try:
        yield module, clients
    finally:
        stop_module(module)


def test_agent_log_replays_in_order_and_caps(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    depth = channel_def("agent").replay_depth
    assert depth == 200
    # A transcript that grew before any viewer attached (cold start): only
    # the newest `depth` entries are kept.
    for i in range(depth + 5):
        transport_of(module, "agent").publish(HumanMessage(content=f"m{i}"))
    assert module.encoded["agent"] == 0
    assert len(transport_of(module, "agent").subscribers) == 1

    push(module, client, Subs(chs=["agent"], n=1))
    assert wait_until(lambda: len(frames_on(client, "agent")) == depth)
    flush_loop(module)
    entries = frames_on(client, "agent")
    assert [e["content"] for e in entries] == [f"m{i}" for i in range(5, depth + 5)]
    assert all(e["role"] == "human" for e in entries)
    ns = [e["n"] for e in entries]
    assert ns == list(range(ns[0], ns[0] + depth))  # in order, gapless
    assert module.encoded["agent"] == 0  # replays are not live encodes
    assert all(delivery == "reliable" for _, _, delivery, _ in client.frames)
    # Live frames are fed from the log subscription: no second subscription.
    assert len(transport_of(module, "agent").subscribers) == 1


def test_agent_live_and_replayed_entries_share_n(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    push(module, client, Subs(chs=["agent"], n=1))
    assert wait_until(lambda: "agent" in (module._session.unsubs if module._session else {}))
    transport_of(module, "agent").publish(HumanMessage(content="hello"))
    transport_of(module, "agent").publish(AIMessage(content="hi!"))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 2)
    live = frames_on(client, "agent")
    assert module.encoded["agent"] == 2

    push(module, client, Subs(chs=[], n=2))
    assert wait_until(lambda: module._session is not None and "agent" not in module._session.unsubs)
    transport_of(module, "agent").publish(ToolMessage(content="done", tool_call_id="c1"))
    flush_loop(module)
    assert len(frames_on(client, "agent")) == 2  # nobody watching: no encode
    assert module.encoded["agent"] == 2

    push(module, client, Subs(chs=["agent"], n=3))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 5)
    replayed = frames_on(client, "agent")[2:]
    # The two entries seen live come back with the same n (the viewer's
    # dedupe key), followed by the one published while unwatched.
    assert [(e["n"], e["content"]) for e in replayed[:2]] == [(e["n"], e["content"]) for e in live]
    assert replayed[2]["content"] == "done" and replayed[2]["n"] == live[1]["n"] + 1
    assert module.encoded["agent"] == 2


def test_agent_log_entry_keeps_its_number_after_eviction(agent_bridge) -> None:
    # _replay snapshots the log and encodes afterwards; an entry the
    # transcript pushes out in between must still go out under the number
    # the live viewers saw, never as a fresh, newer-looking one.
    module, _clients = agent_bridge
    depth = channel_def("agent").replay_depth
    transport_of(module, "agent").publish(HumanMessage(content="first"))
    (snapshot,) = module._replay_log["agent"]
    first_n = snapshot.n
    for i in range(depth):
        transport_of(module, "agent").publish(HumanMessage(content=f"m{i}"))
    log = module._replay_log["agent"]
    assert len(log) == depth and snapshot not in log
    replayed = encode_chat(module, snapshot)
    assert (replayed["n"], replayed["content"]) == (first_n, "first")
    assert [entry.n for entry in log] == list(range(first_n + 1, first_n + depth + 1))


def test_channel_def_rejects_unreplayable_logs() -> None:
    # A log is fed by the always-on cache subscription, which only
    # resend_on_subscribe channels get: the table refuses a row that would
    # never deliver a live frame.
    with pytest.raises(ValueError, match="requires resend_on_subscribe"):
        replace(channel_def("agent"), resend_on_subscribe=False)
    with pytest.raises(ValueError, match="replay_depth must be >= 1"):
        replace(channel_def("odom"), replay_depth=0)
    replace(channel_def("odom"), replay_depth=1)  # a plain channel is fine


def test_tx_channel_def_unpacks_as_a_triple() -> None:
    # TX_CHANNELS rows used to be (ch, encoding, delivery) tuples; consumers
    # that still unpack them that way (dimos/web/cockpit.py before its
    # _tx_registry) must keep working.
    rows = [(ch, encoding, delivery) for ch, encoding, delivery in TX_CHANNELS]
    assert rows == [(td.ch, td.encoding, td.delivery) for td in TX_CHANNELS]
    assert rows[0] == ("tele_cmd_vel", "twist.json.v1", "latest")


def test_agent_no_rate_gate(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    push(module, client, Subs(chs=["agent", "agent_idle"], n=1))
    assert wait_until(lambda: len(transport_of(module, "agent_idle").subscribers) == 2)
    # Back-to-back publishes, far inside the advertised maxHz intervals
    # (30 Hz / 10 Hz): every one is a frame - these are events, not samples.
    for i in range(5):
        transport_of(module, "agent").publish(HumanMessage(content=f"burst{i}"))
    transport_of(module, "agent_idle").publish(False)
    transport_of(module, "agent_idle").publish(True)
    assert wait_until(lambda: len(frames_on(client, "agent")) == 5)
    assert wait_until(lambda: len(frames_on(client, "agent_idle")) == 2)
    assert [e["content"] for e in frames_on(client, "agent")] == [f"burst{i}" for i in range(5)]
    assert [e["value"] for e in frames_on(client, "agent_idle")] == [False, True]


def test_agent_idle_replays_newest_flag(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    transport_of(module, "agent_idle").publish(False)
    transport_of(module, "agent_idle").publish(True)
    push(module, client, Subs(chs=["agent_idle"], n=1))
    assert wait_until(lambda: frames_on(client, "agent_idle"))
    flush_loop(module)
    assert [e["value"] for e in frames_on(client, "agent_idle")] == [True]
    assert module.encoded["agent_idle"] == 0


def test_agent_log_survives_reconnect(agent_bridge) -> None:
    module, clients = agent_bridge
    push(module, clients[0], Subs(chs=["agent"], n=1))
    assert wait_until(lambda: "agent" in (module._session.unsubs if module._session else {}))
    transport_of(module, "agent").publish(HumanMessage(content="before"))
    assert wait_until(lambda: len(frames_on(clients[0], "agent")) == 1)
    kill_session(module, clients[0])
    assert wait_until(lambda: len(clients) == 2)
    # The retired session is detached from the log; the new one replays it.
    assert module._log_live["agent"] == ()
    push(module, clients[1], Subs(chs=["agent"], n=1))
    assert wait_until(lambda: len(frames_on(clients[1], "agent")) == 1)
    assert frames_on(clients[1], "agent")[0]["n"] == frames_on(clients[0], "agent")[0]["n"]


def test_chase_image_uses_its_own_quality(monkeypatch) -> None:
    manifest = microduck_manifest(rx=("chase_image",), tx=())
    manifest["channels"][0]["params"] = {"quality": 60}
    manifest["channels"].append(
        {
            "ch": "color_image",
            "dir": "rx",
            "encoding": "jpeg.v1",
            "delivery": "latest",
            "maxHz": 4.0,
            "params": {"quality": 33},
        }
    )
    module, clients = _make_bridge(
        monkeypatch, wire=("chase_image", "color_image"), manifest=manifest
    )
    try:
        assert module._jpeg_quality_for("chase_image") == 60
        assert module._jpeg_quality_for("color_image") == 33
        qualities: list[int] = []
        real = Image.to_jpeg_bytes

        def spy(self: Image, quality: int = 75) -> bytes:
            qualities.append(quality)
            return real(self, quality=quality)

        monkeypatch.setattr(Image, "to_jpeg_bytes", spy)
        client = clients[0]
        push(module, client, Subs(chs=["chase_image"], n=1))
        assert wait_until(lambda: len(transport_of(module, "chase_image").subscribers) == 2)
        transport_of(module, "chase_image").publish(
            Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8))
        )
        assert wait_until(lambda: qualities == [60])
        assert wait_until(lambda: client.writers["chase_image"].offers)
        assert client.writers["chase_image"].offers[0][1] == {"w": 12, "h": 8}
    finally:
        stop_module(module)


# Composition helpers live at module level: under PEP 563 (`from __future__
# import annotations`) a Module class defined inside a function loses its
# streams, because its annotations cannot be resolved from module globals.
class _EmptyConfig(ModuleConfig):
    pass


class _ImageProducer(Module):
    config: _EmptyConfig
    color_image: Out[Image]


class _CostmapProducer(Module):
    config: _EmptyConfig
    global_costmap: Out[OccupancyGrid]


class _TwistConsumer(Module):
    config: _EmptyConfig
    tele_cmd_vel: In[Twist]


class _BareModule(Module):
    config: _EmptyConfig


def test_composition_adds_relay_to_non_visual_blueprint() -> None:
    blueprint = with_relay_bridge(_ImageProducer.blueprint())
    relay_atoms = [atom for atom in blueprint.blueprints if atom.module is RelayBridgeModule]

    assert len(relay_atoms) == 1
    assert relay_atoms[0].kwargs["available_channels"] == ("color_image",)


def test_composition_includes_costmap_producer() -> None:
    blueprint = with_relay_bridge(
        autoconnect(_ImageProducer.blueprint(), _CostmapProducer.blueprint())
    )
    relay_atom = next(atom for atom in blueprint.blueprints if atom.module is RelayBridgeModule)

    assert relay_atom.kwargs["available_channels"] == ("color_image", "global_costmap")


def test_composition_derives_tx_channels_from_consumers() -> None:
    # A MovementManager-like consumer of tele_cmd_vel makes the tx channel
    # available (Out transports exist regardless of wiring, so consumers are
    # the availability signal).
    blueprint = with_relay_bridge(
        autoconnect(_ImageProducer.blueprint(), _TwistConsumer.blueprint())
    )
    relay_atom = next(atom for atom in blueprint.blueprints if atom.module is RelayBridgeModule)

    assert relay_atom.kwargs["available_channels"] == ("color_image", "tele_cmd_vel")


def test_composition_ignores_disabled_producers() -> None:
    source = _ImageProducer.blueprint().disabled_modules(_ImageProducer)
    blueprint = with_relay_bridge(source)
    relay_atom = next(atom for atom in blueprint.blueprints if atom.module is RelayBridgeModule)

    assert relay_atom.kwargs["available_channels"] == ()


def test_composition_preserves_existing_relay() -> None:
    existing = RelayBridgeModule.blueprint(local_port=8899, available_channels=("odom",))
    blueprint = with_relay_bridge(autoconnect(_BareModule.blueprint(), existing))
    relay_atoms = [atom for atom in blueprint.blueprints if atom.module is RelayBridgeModule]

    assert len(relay_atoms) == 1
    assert relay_atoms[0].kwargs == {
        "local_port": 8899,
        "available_channels": ("odom",),
    }
