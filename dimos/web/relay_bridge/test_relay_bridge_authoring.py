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

"""cockpit()-authored RelayBridgeModule tests: blueprint composition, runtime
channel specs with custom codecs, and the generic publish path (W7). Same
no-network harness as test_relay_bridge_module.py (see module_test_support).
"""

from __future__ import annotations

from dataclasses import replace
import json
import pickle
import struct
from typing import Any

import numpy as np
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.cockpit import Channel, Video, cockpit
from dimos.web.codecs import EncodedPayload, PublishContext, web_decoder, web_encoder
from dimos.web.relay_bridge import builtin_codecs, relay_bridge_module
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.module_test_support import (
    FakeClient,
    FakeTransport,
    flush_loop,
    push,
    start_authored,
    transport_of,
    wait_until,
)
from dimos.web.relay_bridge.protocol import (
    DataFrame,
    FrameHeader,
    PubAck,
    PubNack,
    Subs,
)
from dimos.web.relay_bridge.relay_bridge_module import (
    RelayBridgeConfig,
    RelayBridgeModule,
    RuntimeChannelSpec,
    with_relay_bridge,
)


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


@web_encoder("path.rbm.v1")
def _encode_path_points(msg: NavPath) -> EncodedPayload:
    payload = b"".join(struct.pack("<ff", p.position.x, p.position.y) for p in msg.poses)
    return EncodedPayload(payload, {"n": len(msg.poses)})


@web_encoder("boom.rbm.v1")
def _encode_boom(msg: NavPath) -> bytes:
    raise RuntimeError("boom")


_NAV_PATH = NavPath(
    ts=1.0,
    frame_id="world",
    poses=[PoseStamped(ts=1.0, position=[1.5, -2.5, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])],
)
_NAV_PATH_PAYLOAD = struct.pack("<ff", 1.5, -2.5)


def test_config_accepts_runtime_specs_by_identity_and_pickles() -> None:
    spec = RuntimeChannelSpec(
        ch="odom",
        message_type=PoseStamped,
        dir="rx",
        encoding="pose.json.v1",
        delivery="reliable",
        max_hz=20.0,
        params={"a": 1},
        encoder=builtin_codecs.encode_pose,
    )
    config = RelayBridgeConfig(channels=(spec,))
    # Pydantic must pass the frozen dataclass through untouched; a rebuilt
    # copy would break the is-checks blueprints rely on.
    assert config.channels is not None and config.channels[0] is spec
    restored = pickle.loads(pickle.dumps(config))
    assert restored.channels[0].encoder is builtin_codecs.encode_pose
    assert restored.channels[0].params == {"a": 1}


def test_authored_specs_drive_generated_port(monkeypatch) -> None:
    blueprint = cockpit(
        channels=[Channel("nav_path", NavPath, encoding="path.rbm.v1", max_hz=1000.0)]
    )
    module, clients = start_authored(monkeypatch, blueprint, wire=("nav_path",))
    try:
        nav = transport_of(module, "nav_path")
        nav.publish(_NAV_PATH)
        flush_loop(module)
        assert clients[0].frames == []  # no viewers: no encode, no send
        assert module.encoded == {"nav_path": 0}

        push(module, clients[0], Subs(chs=["nav_path"], n=1))
        assert wait_until(lambda: nav.subscribers)
        nav.publish(_NAV_PATH)
        assert wait_until(lambda: clients[0].frames)
        ch, payload, delivery, meta = clients[0].frames[0]
        assert (ch, delivery) == ("nav_path", "reliable")
        assert payload == _NAV_PATH_PAYLOAD
        assert meta == {"n": 1}
        assert module.encoded["nav_path"] == 1

        push(module, clients[0], Subs(chs=[], n=2))
        assert wait_until(lambda: not nav.subscribers)
    finally:
        stop_module(module)


def test_two_jpeg_channels_use_independent_quality(monkeypatch) -> None:
    # The second camera the old single-int _jpeg_quality could not express.
    blueprint = cockpit(
        layout=Video("color_image", quality=33, max_hz=1000.0),
        channels=[
            Channel(
                "rear_cam",
                Image,
                encoding="jpeg.v1",
                delivery="latest",
                max_hz=1000.0,
                params={"quality": 90},
            )
        ],
    )
    module, clients = start_authored(monkeypatch, blueprint, wire=("color_image", "rear_cam"))
    try:
        qualities: list[int] = []
        real = Image.to_jpeg_bytes

        def spy(self: Image, quality: int = 75) -> bytes:
            qualities.append(quality)
            return real(self, quality=quality)

        monkeypatch.setattr(Image, "to_jpeg_bytes", spy)
        push(module, clients[0], Subs(chs=["color_image", "rear_cam"], n=1))
        front, rear = transport_of(module, "color_image"), transport_of(module, "rear_cam")
        assert wait_until(lambda: front.subscribers and rear.subscribers)
        image = Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8))
        front.publish(image)
        rear.publish(image)
        assert wait_until(lambda: sorted(qualities) == [33, 90])
    finally:
        stop_module(module)


def test_resend_flag_replays_cache_on_generated_port(monkeypatch) -> None:
    # The replay mechanism is spec-driven now; prove it on a generated input
    # (the flag is internal - cockpit() sets it only for the built-in costmap).
    blueprint = cockpit(
        channels=[Channel("nav_path", NavPath, encoding="path.rbm.v1", max_hz=1000.0)]
    )
    (atom,) = blueprint.blueprints
    atom.kwargs["channels"] = tuple(
        replace(spec, resend_on_subscribe=True) for spec in atom.kwargs["channels"]
    )
    module, clients = start_authored(monkeypatch, blueprint, wire=("nav_path",))
    try:
        nav = transport_of(module, "nav_path")
        assert len(nav.subscribers) == 1  # the always-on raw cache
        nav.publish(_NAV_PATH)  # nobody watching; only the cache sees it
        flush_loop(module)
        assert clients[0].frames == []

        push(module, clients[0], Subs(chs=["nav_path"], n=1))
        # The cached message replays without a new publish.
        assert wait_until(lambda: clients[0].frames)
        assert clients[0].frames[0][1] == _NAV_PATH_PAYLOAD
        assert module.encoded["nav_path"] == 0  # replays do not count
    finally:
        stop_module(module)


def test_encoder_failure_is_isolated_and_rate_limited(monkeypatch) -> None:
    blueprint = cockpit(
        channels=[
            Channel("bad_path", NavPath, encoding="boom.rbm.v1", max_hz=1000.0),
            Channel("target_pose", PoseStamped, encoding="pose.json.v1", max_hz=1000.0),
        ]
    )
    module, clients = start_authored(monkeypatch, blueprint, wire=("bad_path", "target_pose"))
    try:
        exceptions: list[str] = []
        monkeypatch.setattr(
            relay_bridge_module.logger,
            "exception",
            lambda msg, *args, **kwargs: exceptions.append(msg),
        )
        push(module, clients[0], Subs(chs=["bad_path", "target_pose"], n=1))
        bad, pose = transport_of(module, "bad_path"), transport_of(module, "target_pose")
        assert wait_until(lambda: bad.subscribers and pose.subscribers)
        # Drop the input rate gates so every publish reaches the encoder.
        module._min_interval = {"bad_path": 0.0, "target_pose": 0.0}
        for _ in range(3):
            bad.publish(_NAV_PATH)
        pose.publish(PoseStamped(ts=2.0, position=[1.0, 2.0, 0.0], orientation=[0, 0, 0, 1]))
        assert wait_until(lambda: clients[0].frames)
        # The healthy channel flows; the broken one drops every sample.
        assert all(frame[0] == "target_pose" for frame in clients[0].frames)
        assert module.encoded == {"bad_path": 0, "target_pose": 1}
        # Three failures inside the log window produce one exception log.
        assert len(exceptions) == 1
        assert "bad_path" in exceptions[0]
    finally:
        stop_module(module)


def test_channels_without_manifest_fails(monkeypatch) -> None:
    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise AssertionError("must not reach the relay without a manifest")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)
    spec = RuntimeChannelSpec(
        ch="odom",
        message_type=PoseStamped,
        dir="rx",
        encoding="pose.json.v1",
        delivery="reliable",
        max_hz=20.0,
        params={},
        encoder=builtin_codecs.encode_pose,
    )
    module = RelayBridgeModule(
        relay_url="https://127.0.0.1:1", robot_id="unit-bot", channels=(spec,)
    )
    with pytest.raises(RuntimeError, match="require a manifest"):
        try:
            module.start()
        finally:
            stop_module(module)


def test_spec_manifest_mismatch_fails(monkeypatch) -> None:
    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        raise AssertionError("must not reach the relay with mismatched specs")

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)
    manifest = {
        "version": 1,
        "channels": [
            {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 5.0}
        ],
    }

    def start_with(spec: RuntimeChannelSpec, match: str) -> None:
        module = RelayBridgeModule(
            relay_url="https://127.0.0.1:1",
            robot_id="unit-bot",
            manifest=manifest,
            channels=(spec,),
        )
        module.odom.transport = FakeTransport()
        with pytest.raises(RuntimeError, match=match):
            try:
                module.start()
            finally:
                stop_module(module)

    good = RuntimeChannelSpec(
        ch="odom",
        message_type=PoseStamped,
        dir="rx",
        encoding="pose.json.v1",
        delivery="reliable",
        max_hz=5.0,
        params={},
        encoder=builtin_codecs.encode_pose,
    )
    start_with(replace(good, ch="target_pose"), "do not match the compiled runtime specs")
    # Every advertised field that drives behavior is cross-checked: a spec
    # gating at 500 Hz while the manifest promises 5 Hz (or drifted params,
    # encoding, delivery, direction) must fail start.
    start_with(replace(good, encoding="json.v1"), "does not match its compiled runtime spec")
    start_with(replace(good, delivery="latest"), "does not match its compiled runtime spec")
    start_with(replace(good, max_hz=500.0), "does not match its compiled runtime spec")
    start_with(replace(good, params={"q": 1}), "does not match its compiled runtime spec")
    start_with(replace(good, dir="tx"), "does not match its compiled runtime spec")
    start_with(
        replace(good, message_type=Twist),
        "message type Twist does not match the RelayBridgeModule port type PoseStamped",
    )


def test_composition_preserves_generated_bridge() -> None:
    blueprint = cockpit(channels=[Channel("target_pose", PoseStamped, encoding="pose.json.v1")])
    assert with_relay_bridge(blueprint) is blueprint


# Generic publish (W7): forwarded viewer publishes arriving as tx DataFrames
# on the carrier -> decode -> Out.publish -> @control ack.


_PUB_CONTEXTS: list[PublishContext] = []


@web_decoder("ctx.probe.json.v1")
def _decode_ctx_probe(value: dict[str, Any], context: PublishContext) -> dict:
    _PUB_CONTEXTS.append(context)
    return dict(value)


@web_decoder("wrongtype.json.v1")
def _decode_wrong_type(value: Any) -> str:
    return 42  # type: ignore[return-value]  # deliberately violates the annotation


@web_decoder("boom.json.v1")
def _decode_boom(value: Any) -> str:
    raise ValueError("nope β")


def _pub_meta(**over: Any) -> dict[str, Any]:
    return {"id": "p1", "principal": "local", "relayTs": 41.5, **over}


_DEFAULT_META = object()  # sentinel: meta=None must mean a header WITHOUT meta


def _pub_frame(
    payload: bytes, meta: Any = _DEFAULT_META, ch: str = "human_input", seq: int = 1
) -> DataFrame:
    if meta is _DEFAULT_META:
        meta = _pub_meta()
    return DataFrame(
        header=FrameHeader(ch=ch, seq=seq, ts=41.5, delivery="reliable", meta=meta),
        payload=payload,
    )


def _start_pub_bridge(monkeypatch, *channels: Channel):
    blueprint = cockpit(
        channels=[
            Channel("human_input", str, dir="tx", encoding="text.json.v1", publish="shared"),
            Channel("target_pose", PoseStamped, encoding="pose.json.v1", max_hz=1000.0),
            *channels,
        ]
    )
    return start_authored(monkeypatch, blueprint, wire=("target_pose",))


def test_publish_frame_decodes_publishes_then_acks(monkeypatch) -> None:
    module, clients = _start_pub_bridge(monkeypatch)
    try:
        seen: list[tuple[str, int]] = []  # (value, acks already sent when it arrived)
        module.human_input.subscribe(
            lambda value: seen.append((value, len(clients[0].control_frames)))
        )
        push(module, clients[0], _pub_frame(json.dumps("salut β").encode()))
        assert wait_until(lambda: clients[0].control_frames)
        # The consumer received the exact string BEFORE the ack went out.
        assert seen == [("salut β", 0)]
        (ack,) = clients[0].control_frames
        assert isinstance(ack, PubAck)
        assert ack.id == "p1" and ack.ch == "human_input"
        assert ack.relayTs == 41.5 and ack.bridgeTs > 0
        # The reliable carrier cannot duplicate, so the bridge is dedup-free
        # by design: a replayed frame publishes (and acks) again.
        push(module, clients[0], _pub_frame(json.dumps("salut β").encode(), seq=2))
        assert wait_until(lambda: len(clients[0].control_frames) == 2)
        assert [value for value, _ in seen] == ["salut β", "salut β"]
    finally:
        stop_module(module)


def test_publish_decoder_context_and_no_context_paths(monkeypatch) -> None:
    module, clients = _start_pub_bridge(
        monkeypatch,
        Channel("probe", dict, dir="tx", encoding="ctx.probe.json.v1", publish="shared"),
        Channel("counter", int, dir="tx", publish="shared"),  # generic json.v1
    )
    try:
        _PUB_CONTEXTS.clear()
        counts: list[int] = []
        module.counter.subscribe(counts.append)
        push(
            module,
            clients[0],
            _pub_frame(b'{"x":1.5}', _pub_meta(id="p7", clientTs=40.25), ch="probe"),
        )
        assert wait_until(lambda: clients[0].control_frames)
        (context,) = _PUB_CONTEXTS
        assert context == PublishContext(
            robot="unit-bot",
            ch="probe",
            relay_ts=41.5,
            request_id="p7",
            principal="local",
            client_ts=40.25,
        )
        # Generic json.v1 decode: the identity value passes the declared-type
        # check; a JSON bool is not an int (Python bool subclasses int).
        push(module, clients[0], _pub_frame(b"7", _pub_meta(id="p8"), ch="counter"))
        assert wait_until(lambda: counts == [7])
        push(module, clients[0], _pub_frame(b"true", _pub_meta(id="p9"), ch="counter"))
        assert wait_until(lambda: len(clients[0].control_frames) == 3)
        nack = clients[0].control_frames[-1]
        assert isinstance(nack, PubNack) and nack.id == "p9"
        assert nack.code == "decode_failed" and "bool" in nack.message
        assert counts == [7]
    finally:
        stop_module(module)


def test_publish_failures_nack_without_recycling_the_session(monkeypatch) -> None:
    module, clients = _start_pub_bridge(
        monkeypatch,
        Channel("boom", str, dir="tx", encoding="boom.json.v1", publish="shared"),
        Channel("wrong", str, dir="tx", encoding="wrongtype.json.v1", publish="shared"),
    )
    try:
        seen: list[str] = []
        module.human_input.subscribe(seen.append)

        def nacked(request_id: str, code: str, needle: str) -> bool:
            last = clients[0].control_frames[-1] if clients[0].control_frames else None
            return (
                isinstance(last, PubNack)
                and last.id == request_id
                and last.code == code
                and needle in last.message
            )

        push(module, clients[0], _pub_frame(b"{not json", _pub_meta(id="a")))
        assert wait_until(lambda: nacked("a", "decode_failed", "JSONDecodeError"))
        # Deep-but-valid JSON is a request-local decode failure, never a
        # RecursionError escaping into the supervisor.
        deep = b"[" * 10_000 + b"]" * 10_000
        push(module, clients[0], _pub_frame(deep, _pub_meta(id="deep")))
        assert wait_until(lambda: nacked("deep", "decode_failed", "nests deeper than 100"))
        push(module, clients[0], _pub_frame(b'"x"', _pub_meta(id="b"), ch="ghost"))
        assert wait_until(lambda: nacked("b", "unknown_channel", "ghost"))
        push(module, clients[0], _pub_frame(b'"x"', _pub_meta(id="c"), ch="boom"))
        assert wait_until(lambda: nacked("c", "decode_failed", "ValueError: nope"))
        push(module, clients[0], _pub_frame(b'"x"', _pub_meta(id="d"), ch="wrong"))
        assert wait_until(lambda: nacked("d", "decode_failed", "int, not str"))

        def raising_publish(msg: Any) -> None:
            raise RuntimeError("transport down β")

        monkeypatch.setattr(module.human_input, "publish", raising_publish)
        push(module, clients[0], _pub_frame(b'"x"', _pub_meta(id="e")))
        assert wait_until(lambda: nacked("e", "publish_failed", "RuntimeError"))
        monkeypatch.undo()

        # No traceback ever reaches the wire, messages stay bounded, and the
        # session was never recycled by any of the failures.
        assert all(
            len(m.message) <= 200 and "Traceback" not in m.message
            for m in clients[0].control_frames
            if isinstance(m, PubNack)
        )
        assert len(clients) == 1

        # Channel isolation: the healthy publish channel and the rx path
        # still work after every failure above.
        push(module, clients[0], _pub_frame(json.dumps("încă merge").encode(), _pub_meta(id="f")))
        assert wait_until(lambda: seen == ["încă merge"])
        push(module, clients[0], Subs(chs=["target_pose"], n=1))
        assert wait_until(lambda: transport_of(module, "target_pose").subscribers)
    finally:
        stop_module(module)


def test_publish_frame_with_unusable_meta_is_dropped(monkeypatch) -> None:
    module, clients = _start_pub_bridge(monkeypatch)
    try:
        seen: list[str] = []
        module.human_input.subscribe(seen.append)
        for meta in [
            None,
            {"principal": "local", "relayTs": 41.5},  # no id
            _pub_meta(id=""),
            _pub_meta(id="x" * 65),
            _pub_meta(principal=5),
            _pub_meta(relayTs="soon"),
            _pub_meta(relayTs=True),
            _pub_meta(clientTs="now"),
        ]:
            push(module, clients[0], _pub_frame(b'"x"', meta))
        # A later valid publish proves all of the above were processed
        # (ordered queue) and dropped without an ack or a publish.
        push(module, clients[0], _pub_frame(json.dumps("ok").encode(), _pub_meta(id="z")))
        assert wait_until(lambda: seen == ["ok"])
        assert [m.id for m in clients[0].control_frames if isinstance(m, PubAck)] == ["z"]
        assert module._pub_invalid == 8
    finally:
        stop_module(module)
