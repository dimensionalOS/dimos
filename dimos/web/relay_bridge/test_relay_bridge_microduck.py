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

"""Microduck cockpit streams, legacy commands, and bounded chat replay."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import json
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, ToolMessage
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.pollen.microduck import web_codecs
from dimos.robot.pollen.microduck.blueprints.microduck_cockpit_sim import (
    MICRODUCK_COCKPIT_CHANNELS,
    MICRODUCK_COCKPIT_LAYOUT,
)
from dimos.web.cockpit import Channel, Chat, Video, cockpit
from dimos.web.relay_bridge import relay_bridge_module
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.manifest import parse_manifest
from dimos.web.relay_bridge.module_test_support import (
    FakeClient,
    FakeTransport,
    flush_loop,
    kill_session,
    make_bridge,
    push,
    start_authored,
    wait_until,
)
from dimos.web.relay_bridge.protocol import (
    Subs,
    Tx,
)
from dimos.web.relay_bridge.relay_bridge_module import (
    TX_CHANNELS,
    RelayBridgeConfig,
    RelayBridgeModule,
    RuntimeChannelSpec,
    default_manifest,
)
from dimos.web.relay_bridge.test_relay_bridge_module import teleop_manifest, wire_twist


def settle(module: RelayBridgeModule) -> None:
    """Give queued teleop handling time to run before a negative assert."""
    flush_loop(module)
    time.sleep(0.03)
    flush_loop(module)


# The authored cockpit's rx channels in manifest order: the bridge built-ins
# first, then the MICRODUCK_COCKPIT_CHANNELS declarations in panel order.
MICRODUCK_RX = (
    "color_image",
    "odom",
    "global_costmap",
    "mode",
    "policy_state",
    "nav_state",
    # Built-ins first (BUILTIN_CHANNELS order), then the authored ones in
    # first-declaration order - which is panel order, so chase_image leads:
    # it is the main feed of the first video panel.
    "chase_image",
    "path",
    "places",
    "agent",
    "agent_idle",
)
# The tx channels that go through the generic Tx path; the twist keeps its own
# lease-guarded one.
MICRODUCK_TX = ("human_input", "goal_request", "ui_command")


def microduck_channels(*streams: str) -> tuple[Channel, ...]:
    """The named MICRODUCK_COCKPIT_CHANNELS declarations, so a test can build
    a slice of the duck's cockpit without repeating its authoring."""
    declared = {c.stream: c for c in MICRODUCK_COCKPIT_CHANNELS}
    return tuple(declared[stream] for stream in streams)


def microduck_tx_manifest(tx: tuple[str, ...] = MICRODUCK_TX) -> dict[str, Any]:
    """Channel-only manifest for the Microduck tx commands (no panels: the
    bridge validates tx channels against TX_CHANNELS, not panel bindings)."""
    by_tx = {td.ch: td for td in TX_CHANNELS}
    return {
        "version": 1,
        "channels": [
            {
                "ch": ch,
                "dir": "tx",
                "encoding": by_tx[ch].encoding,
                "delivery": by_tx[ch].delivery,
                "maxHz": 5.0,
            }
            for ch in tx
        ],
    }


def transport_of(module: RelayBridgeModule, ch: str) -> FakeTransport:
    transport = getattr(module, ch).transport
    assert isinstance(transport, FakeTransport)
    return transport


def spec_of(module: RelayBridgeModule, ch: str) -> RuntimeChannelSpec:
    return next(spec for spec in module._channel_specs if spec.ch == ch)


def frames_on(client: FakeClient, ch: str) -> list[dict[str, Any]]:
    """Decoded reliable frame payloads sent on `ch`, in order."""
    return [json.loads(payload) for c, payload, _delivery, _meta in client.frames if c == ch]


def ns_on(client: FakeClient, ch: str) -> list[int | None]:
    """The `n` frame meta of those frames: the bridge's log entry numbers,
    which is where the viewer's dedupe key lives (never in the payload)."""
    return [None if meta is None else meta.get("n") for c, _p, _d, meta in client.frames if c == ch]


@pytest.fixture
def microduck_bridge(monkeypatch):
    """The full authored Microduck cockpit, every rx input wired."""
    module, clients = start_authored(
        monkeypatch,
        cockpit(layout=MICRODUCK_COCKPIT_LAYOUT, channels=MICRODUCK_COCKPIT_CHANNELS),
        wire=MICRODUCK_RX,
    )
    try:
        yield module, clients
    finally:
        stop_module(module)


def test_microduck_manifest_starts_with_every_channel(microduck_bridge) -> None:
    module, clients = microduck_bridge
    _, hello_manifest = clients[0].hello_args
    assert [c["ch"] for c in hello_manifest["channels"]] == [
        *MICRODUCK_RX,
        "tele_cmd_vel",
        *MICRODUCK_TX,
    ]
    # The twist keeps the lease-guarded path; every other tx channel goes
    # through the generic Tx handler.
    assert module._teleop_params is not None
    assert set(module._tx_defs) == set(MICRODUCK_TX)
    # Every declared channel replays on subscribe: one always-on raw cache
    # subscription each, nothing encoding yet.
    for channel in MICRODUCK_COCKPIT_CHANNELS:
        assert len(transport_of(module, channel.stream).subscribers) == 1, channel.stream
    assert all(module.encoded[ch] == 0 for ch in MICRODUCK_RX)
    assert module._min_interval["agent"] == pytest.approx(1.0 / 30.0)


def test_cockpit_microduck_layout_starts(microduck_bridge) -> None:
    # The authored cockpit (Control bar, chase camera, Chat, NavMap) compiles
    # to runtime specs this bridge runs: a typed port per declaration, the
    # encoder resolved from web_codecs by id, and the authoring flags carried
    # through - the event streams skip the rate gate, the transcript keeps a
    # replay log.
    module, _clients = microduck_bridge
    for channel in MICRODUCK_COCKPIT_CHANNELS:
        spec = spec_of(module, channel.stream)
        assert module.inputs[channel.stream].type is channel.message_type
        assert (spec.encoding, spec.delivery, spec.max_hz) == (
            channel.encoding,
            channel.delivery,
            channel.max_hz,
        )
        assert (spec.resend_on_subscribe, spec.rate_gate, spec.replay_depth) == (
            channel.resend_on_subscribe,
            channel.rate_gate,
            channel.replay_depth,
        )
    assert spec_of(module, "agent").encoder is web_codecs.encode_chat
    assert spec_of(module, "places").encoder is web_codecs.encode_state_json


def test_microduck_manifest_rejects_mismatches(monkeypatch) -> None:
    # Authoring time: a declaration whose encoding does not match the stream's
    # message type, or that contradicts the panel binding the same stream,
    # fails at blueprint definition.
    with pytest.raises(ValueError, match="encodes BaseMessage, not bool"):
        cockpit(channels=[Channel("agent", bool, encoding="chat.json.v1")])
    with pytest.raises(ValueError, match="conflicting requirements for stream 'agent'"):
        cockpit(
            layout=Chat(),
            channels=[Channel("agent", BaseMessage, encoding="chat.json.v1", delivery="latest")],
        )

    # Start time: the tx side is still the bridge's own table.
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
        base = microduck_tx_manifest(tx=("human_input",))
        for channel in base["channels"]:
            if channel["ch"] == ch:
                channel.update(overrides)
        return base

    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(with_channel("human_input", encoding="pose_goal.json.v1"))
    with pytest.raises(RuntimeError, match="no matching handler"):
        start_with(with_channel("human_input", delivery="latest"))
    unknown = microduck_tx_manifest(tx=())
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


def test_microduck_channels_are_advertised_only_when_authored() -> None:
    # The auto (no-manifest) mode knows BUILTIN_CHANNELS and nothing else, so
    # the duck's streams are simply not available there any more...
    auto = default_manifest(
        RelayBridgeConfig(image_max_hz=25.0), ("chase_image", "agent", "policy_state")
    )
    assert auto["channels"] == [] and auto["panels"] == []
    # ... they reach a cockpit through the blueprint's own declarations, which
    # advertise them channel-only when no panel binds them.
    (atom,) = cockpit(channels=MICRODUCK_COCKPIT_CHANNELS).blueprints
    manifest = atom.kwargs["manifest"]
    assert [(c["ch"], c["dir"], c["encoding"], c["maxHz"]) for c in manifest["channels"]] == [
        (c.stream, "rx", c.encoding, c.max_hz) for c in MICRODUCK_COCKPIT_CHANNELS
    ]
    assert manifest["panels"] == [] and manifest["layout"] is None
    # And the parser accepts the channel-only output too.
    parse_manifest(manifest)


def tx(ch: str, seq: int, **data: Any) -> Tx:
    return Tx(ch=ch, seq=seq, data=data)


@pytest.fixture
def tx_bridge(monkeypatch):
    module, clients = make_bridge(monkeypatch, wire=(), manifest=microduck_tx_manifest())
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
    manifest["channels"] += microduck_tx_manifest()["channels"]
    module, clients = make_bridge(monkeypatch, manifest=manifest)
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


@pytest.fixture
def agent_bridge(monkeypatch):
    """Just the transcript pair, no panels: these exercise the bridge's log
    and rate-gate machinery, not the duck's layout."""
    module, clients = start_authored(
        monkeypatch,
        cockpit(channels=microduck_channels("agent", "agent_idle")),
        wire=("agent", "agent_idle"),
    )
    try:
        yield module, clients
    finally:
        stop_module(module)


def test_agent_log_replays_in_order_and_caps(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    depth = spec_of(module, "agent").replay_depth
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
    ns = ns_on(client, "agent")
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
    live, live_ns = frames_on(client, "agent"), ns_on(client, "agent")
    assert module.encoded["agent"] == 2

    push(module, client, Subs(chs=[], n=2))
    assert wait_until(lambda: module._session is not None and "agent" not in module._session.unsubs)
    transport_of(module, "agent").publish(ToolMessage(content="done", tool_call_id="c1"))
    flush_loop(module)
    assert len(frames_on(client, "agent")) == 2  # nobody watching: no encode
    assert module.encoded["agent"] == 2

    push(module, client, Subs(chs=["agent"], n=3))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 5)
    replayed, replayed_ns = frames_on(client, "agent")[2:], ns_on(client, "agent")[2:]
    # The two entries seen live come back with the same n (the viewer's
    # dedupe key), followed by the one published while unwatched.
    assert replayed_ns[:2] == live_ns
    assert [e["content"] for e in replayed[:2]] == [e["content"] for e in live]
    assert replayed[2]["content"] == "done" and replayed_ns[2] == live_ns[1] + 1
    assert module.encoded["agent"] == 2


def test_chat_entry_number_is_frame_meta_not_payload(agent_bridge) -> None:
    # The number belongs to the bridge's log, not to the message: the encoder
    # never sees it, so it rides the frame meta and the payload stays clean.
    module, clients = agent_bridge
    client = clients[0]
    push(module, client, Subs(chs=["agent"], n=1))
    assert wait_until(lambda: "agent" in (module._session.unsubs if module._session else {}))
    transport_of(module, "agent").publish(HumanMessage(content="x"))
    transport_of(module, "agent").publish(HumanMessage(content="y"))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 2)
    first, second = ns_on(client, "agent")
    assert isinstance(first, int) and second == first + 1
    assert all("n" not in entry for entry in frames_on(client, "agent"))
    # A replay_depth 1 channel numbers nothing: there is no log to dedupe.
    push(module, client, Subs(chs=["agent", "agent_idle"], n=2))
    assert wait_until(lambda: len(transport_of(module, "agent_idle").subscribers) == 2)
    transport_of(module, "agent_idle").publish(True)
    assert wait_until(lambda: frames_on(client, "agent_idle"))
    assert ns_on(client, "agent_idle") == [None]


def test_agent_log_entry_keeps_its_number_after_eviction(agent_bridge) -> None:
    module, _clients = agent_bridge
    spec = spec_of(module, "agent")
    depth = spec.replay_depth
    transport_of(module, "agent").publish(HumanMessage(content="first"))
    (first,) = module._replay_log["agent"]
    for i in range(depth):
        transport_of(module, "agent").publish(HumanMessage(content=f"m{i}"))
    log = module._replay_log["agent"]
    assert len(log) == depth and first not in log  # the transcript rolled past it
    assert [entry.n for entry in log] == list(range(first.n + 1, first.n + depth + 1))
    # _replay snapshots (msg, recv_ts, n) triples under the lock and encodes
    # afterwards, so an entry evicted in between still goes out under the
    # number the live viewers saw, at its own arrival time - never as a
    # fresh, newer-looking one.
    sent: list[tuple[bytes, dict[str, Any] | None, float | None]] = []
    with module._log_lock:
        module._replay_log["agent"] = deque([first], maxlen=depth)
    assert module._session is not None
    module._replay(
        module._session, spec, lambda payload, meta, ts: sent.append((payload, meta, ts))
    )
    ((payload, meta, ts),) = sent
    assert (meta, ts) == ({"n": first.n}, first.recv_ts)
    assert json.loads(payload)["content"] == "first"


def test_runtime_spec_rejects_unreplayable_logs() -> None:
    # A log is fed by the always-on cache subscription, which only
    # resend_on_subscribe channels get: a spec that would never deliver a
    # live frame is refused.
    (atom,) = cockpit(channels=microduck_channels("agent")).blueprints
    (agent,) = atom.kwargs["channels"]
    assert (agent.replay_depth, agent.resend_on_subscribe) == (200, True)
    with pytest.raises(ValueError, match="requires resend_on_subscribe"):
        replace(agent, resend_on_subscribe=False)
    with pytest.raises(ValueError, match="replay_depth must be >= 1"):
        replace(agent, replay_depth=0)
    replace(agent, replay_depth=1, resend_on_subscribe=False)  # a plain channel is fine


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
    assert not spec_of(module, "agent").rate_gate


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
    assert ns_on(clients[1], "agent") == ns_on(clients[0], "agent")


def test_chase_image_uses_its_own_quality(microduck_bridge, monkeypatch) -> None:
    # The chase cam's rate and quality are authored twice - on its Video panel
    # and on the Channel that gives the stream its port - and cockpit()'s
    # merge would have rejected any disagreement, so the compiled spec is
    # both halves at once, not the Video panel's 75 default.
    module, clients = microduck_bridge
    (chase,) = microduck_channels("chase_image")
    spec = spec_of(module, "chase_image")
    assert spec.params == dict(chase.params)
    assert spec.params["quality"] != Video().quality
    # The head cam is a built-in port with no Channel, so it is authored by
    # its panel alone - and the two cameras are free to differ.
    assert spec_of(module, "color_image").params["quality"] != Video().quality
    assert module._min_interval["chase_image"] == pytest.approx(1.0 / chase.max_hz)

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
    assert wait_until(lambda: qualities == [chase.params["quality"]])
    assert wait_until(lambda: client.writers["chase_image"].offers)
    assert client.writers["chase_image"].offers[0][1] == {"w": 12, "h": 8}


def test_late_viewer_replays_live_chat_without_resubscribing(agent_bridge) -> None:
    module, clients = agent_bridge
    client = clients[0]
    transport_of(module, "agent").publish(HumanMessage(content="hello"))
    push(module, client, Subs(chs=["agent"], n=1))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 1)
    first_number = ns_on(client, "agent")[0]
    push(module, client, Subs(chs=["agent"], n=2, replay=["agent"]))
    assert wait_until(lambda: len(frames_on(client, "agent")) == 2)
    assert [entry["content"] for entry in frames_on(client, "agent")] == ["hello", "hello"]
    assert ns_on(client, "agent") == [first_number, first_number]
    assert len(transport_of(module, "agent").subscribers) == 1
    push(module, client, Subs(chs=["agent"], n=2, replay=["agent"]))
    flush_loop(module)
    assert len(frames_on(client, "agent")) == 2


def test_agent_stream_is_typed_by_langchain_base_message() -> None:
    # The transcript port is generated from the blueprint's declaration, and
    # its type must be langchain's own class: autoconnect keys transports on
    # (name, type), so anything else would silently never wire
    # McpClient.agent (Out[BaseMessage]).
    (atom,) = cockpit(channels=microduck_channels("agent")).blueprints
    assert atom.module is not RelayBridgeModule  # a generated subclass
    agent = next(s for s in atom.streams if s.name == "agent")
    assert (agent.direction, agent.type) == ("in", BaseMessage)
