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
transports under the module's `In` streams (module_test_support.py), so lazy
subscribe/unsubscribe, the maxHz gate, the encode path, and reconnect are all
observable directly. cockpit()-authored channels and the publish path are
covered in test_relay_bridge_authoring.py.
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from typing import Any
import zlib

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.mujoco.constants import VIDEO_FPS
from dimos.web.relay_bridge import builtin_codecs, relay_bridge_module
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.manifest import ManifestError, parse_manifest
from dimos.web.relay_bridge.module_test_support import (
    FakeClient,
    FakeRelay,
    FakeTransport,
    costmap_transport,
    flush_loop,
    image_transport,
    kill_session,
    make_bridge,
    odom_transport,
    push,
    wait_until,
)
from dimos.web.relay_bridge.protocol import (
    Stop as WireStop,
    Subs,
    TeleopStart as WireTeleopStart,
    TeleopStop as WireTeleopStop,
    Twist as WireTwist,
)
from dimos.web.relay_bridge.relay_bridge_module import (
    RelayBridgeConfig,
    RelayBridgeModule,
    default_manifest,
    resolve_robot_info,
)
from dimos.web.relay_bridge.wt_client import RelayRejectedError


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
                "publish": "none",
                "requiredScope": None,
            },
            {
                "ch": "odom",
                "dir": "rx",
                "encoding": "pose.json.v1",
                "delivery": "reliable",
                "maxHz": 20.0,
                "params": {},
                "publish": "none",
                "requiredScope": None,
            },
            {
                "ch": "global_costmap",
                "dir": "rx",
                "encoding": "costmap.zlib.v1",
                "delivery": "latest",
                "maxHz": 5.0,
                "params": {},
                "publish": "none",
                "requiredScope": None,
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
    module, clients = make_bridge(monkeypatch, wire=("color_image", "odom", "global_costmap"))
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

    monkeypatch.setattr(builtin_codecs.zlib, "compress", spy)
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
    module, _clients = make_bridge(monkeypatch, wire=("color_image", "odom", "global_costmap"))
    transport = costmap_transport(module)
    assert len(transport.subscribers) == 1  # the always-on raw cache
    stop_module(module)
    assert transport.unsubscribed == 1


def test_bridge_import_does_not_pull_matplotlib_or_langchain() -> None:
    # OccupancyGrid's visualization imports are lazy so the bridge does not
    # cost every relay worker matplotlib's ~0.5 s / ~28 MiB (review issue 9),
    # and langchain belongs to the [agents] extra: a web-only install must
    # still run the bridge (the chat codec loads with the Chat panel).
    code = (
        "import sys; import dimos.web.relay_bridge.relay_bridge_module; "
        "assert 'matplotlib' not in sys.modules; "
        "assert 'langchain_core' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_manifest_omits_pose_binding_when_odom_unwired(monkeypatch) -> None:
    module, clients = make_bridge(monkeypatch, wire=("color_image", "global_costmap"))
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

    def blocking_encode(msg: PoseStamped) -> bytes:
        encode_started.set()
        assert release_encode.wait(timeout=5.0)
        return b"old-session-frame"

    module, clients = make_bridge(monkeypatch)
    # Swap the resolved odom spec's encoder for the blocking one before any
    # viewer subscribes (the runtime specs are the encoder source now).
    module._channel_specs = tuple(
        replace(spec, encoder=blocking_encode, encoder_takes_params=False)
        if spec.ch == "odom"
        else spec
        for spec in module._channel_specs
    )
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
        push(module, clients[0], Subs(chs=["odom"], n=1))
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
    module, clients = make_bridge(monkeypatch)
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
    module, clients = make_bridge(monkeypatch, relay=relay)
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
    module, clients = make_bridge(monkeypatch, wire=("odom",))
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
    module, clients = make_bridge(monkeypatch, available_channels=("odom",))
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
    module, clients = make_bridge(monkeypatch, wire=("color_image",), manifest=manifest)
    try:
        _, hello_manifest = clients[0].hello_args
        # The hello carries the normalized form (defaults made explicit).
        assert hello_manifest["channels"][0]["dir"] == "rx"
        assert hello_manifest["panels"][0]["title"] == ""
        assert hello_manifest["layout"] == "p0"
        # Rate and quality come from the manifest, not the config fields.
        assert module._min_interval == {"color_image": 1.0 / 4.0}
        (image_spec,) = module._channel_specs
        assert image_spec.params["quality"] == 33
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
    module, clients = make_bridge(monkeypatch, wire=("odom",), manifest=manifest)
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
    module, clients = make_bridge(monkeypatch, hello_errors=(None, conflict))
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
    module, clients = make_bridge(monkeypatch, manifest=teleop_manifest())
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
