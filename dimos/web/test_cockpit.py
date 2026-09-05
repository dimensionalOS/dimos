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

"""Authoring-API tests: cockpit()/panels/layout compile to pinned manifests."""

from dataclasses import dataclass
import pickle
import struct
import subprocess
import sys

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.cockpit import (
    Channel,
    ChannelRequest,
    Col,
    Map2D,
    Panel,
    Row,
    Teleop,
    Video,
    cockpit,
)
from dimos.web.codecs import EncodedPayload, decode_json_v1, encode_json_v1, web_encoder
from dimos.web.relay_bridge.builtin_codecs import decode_text
from dimos.web.relay_bridge.manifest import ManifestError, parse_manifest
from dimos.web.relay_bridge.protocol import (
    MAX_CONTROL_PAYLOAD_BYTES,
    PROTOCOL_VERSION,
    Hello,
    RobotInfo,
    encode_datagram,
)
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule

# The frozen-contract example (see the plan/spec): what the go2 cockpit
# blueprint authors. Golden below is exact; edits here are manifest changes
# and need the TS side reviewed too.
GO2_LAYOUT = Row(
    Video("color_image"),
    Col(Map2D(costmap="global_costmap", pose="odom"), Teleop(), shares=[3, 1]),
    shares=[2, 1],
)

GO2_MANIFEST = {
    "version": 1,
    "channels": [
        {
            "ch": "color_image",
            "dir": "rx",
            "encoding": "jpeg.v1",
            "delivery": "latest",
            "maxHz": 30.0,
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
        {
            "ch": "tele_cmd_vel",
            "dir": "tx",
            "encoding": "twist.json.v1",
            "delivery": "latest",
            "maxHz": 15.0,
            "params": {"maxLinear": 0.8, "maxAngular": 1.0, "boost": 2.0, "watchdogMs": 300.0},
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
        {"id": "p2", "kind": "teleop", "title": "", "channels": ["tele_cmd_vel"], "params": {}},
    ],
    "layout": {"row": ["p0", {"col": ["p1", "p2"], "shares": [3, 1]}], "shares": [2, 1]},
    "pages": [],
}


def manifest_of(blueprint) -> dict:
    (atom,) = blueprint.blueprints
    assert atom.module is RelayBridgeModule
    return atom.kwargs["manifest"]


def test_go2_example_manifest_golden() -> None:
    manifest = manifest_of(cockpit(layout=GO2_LAYOUT))
    assert manifest == GO2_MANIFEST
    # Normalization is idempotent: the parser accepts its own output.
    assert parse_manifest(manifest).model_dump() == manifest


def test_default_preset() -> None:
    # cockpit() with no layout: video left (2/3); costmap+pose over teleop
    # right (1/3). Same shape as the go2 example.
    manifest = manifest_of(cockpit())
    assert manifest == GO2_MANIFEST


def test_blueprint_pickles() -> None:
    # Blueprint kwargs cross the forkserver Pipe; the manifest dict must
    # survive a pickle round-trip unchanged.
    blueprint = cockpit(layout=GO2_LAYOUT)
    restored = pickle.loads(pickle.dumps(blueprint))
    assert manifest_of(restored) == GO2_MANIFEST


def test_shared_stream_rates_merge_to_max() -> None:
    manifest = manifest_of(
        cockpit(layout=Row(Video("color_image", max_hz=12.0), Video("color_image", max_hz=24.0)))
    )
    (channel,) = manifest["channels"]
    assert channel["maxHz"] == 24.0
    # Both panels bind the one merged channel.
    assert [p["channels"] for p in manifest["panels"]] == [["color_image"], ["color_image"]]


def test_conflicting_params_raise() -> None:
    with pytest.raises(ValueError, match="conflicting requirements for stream 'color_image'"):
        cockpit(layout=Row(Video("color_image", quality=50), Video("color_image", quality=90)))


def test_unknown_stream_raises_listing_valid_ones() -> None:
    with pytest.raises(ValueError, match="unknown stream 'lidar'.*color_image"):
        cockpit(layout=Video("lidar"))


def test_wrong_encoding_for_stream_raises() -> None:
    # A Map2D pointed at the image stream wants costmap.zlib.v1 from a
    # jpeg.v1 stream.
    with pytest.raises(ValueError, match="'color_image' encodes jpeg.v1, not costmap.zlib.v1"):
        cockpit(layout=Map2D(costmap="color_image", pose=None))


def test_unknown_tx_stream_raises() -> None:
    with pytest.raises(ValueError, match="unknown tx stream 'cmd_vel'"):
        cockpit(layout=Teleop(stream="cmd_vel"))


def test_wrong_tx_encoding_raises() -> None:
    class Sender(Panel):
        kind = "sender"
        title = ""

        def _channel_requests(self) -> tuple[ChannelRequest, ...]:
            return (ChannelRequest("tele_cmd_vel", "tx", "chat.json.v1", 15.0),)

    with pytest.raises(ValueError, match="'tele_cmd_vel' encodes twist.json.v1, not chat.json.v1"):
        cockpit(layout=Sender())


def test_pages_get_ids_after_the_grid() -> None:
    manifest = manifest_of(cockpit(layout=Video("color_image"), pages=[Map2D(pose=None)]))
    assert [p["id"] for p in manifest["panels"]] == ["p0", "p1"]
    assert manifest["layout"] == "p0"
    assert manifest["pages"] == ["p1"]


@pytest.mark.parametrize(
    "build",
    [
        lambda: Row(),
        lambda: Row(Video(), Map2D(), shares=[1]),
        lambda: Row(Video(), shares=[2, 1]),
        lambda: Row(Video(), Map2D(), shares=[0, 1]),
        lambda: Row(Video(), Map2D(), shares=[-1.5, 1]),
        lambda: Row(Video(), Map2D(), shares=[True, 1]),
        lambda: Col("color_image"),  # bare stream names are not panels
        lambda: Video(""),
        lambda: Video("color_image", max_hz=0),
        lambda: Video("color_image", quality=101),
        lambda: Video("color_image", quality=True),
        lambda: Map2D(costmap=""),
        lambda: Map2D(costmap_hz=-5.0),
        lambda: Teleop(stream=""),
        lambda: Teleop(max_linear=0),
        lambda: Teleop(boost=-2.0),
        lambda: Teleop(publish_hz=True),
        lambda: Teleop(watchdog_ms=0),
    ],
    ids=[
        "row_empty",
        "row_one_share_missing",
        "row_extra_share",
        "share_zero",
        "share_negative",
        "share_bool",
        "col_string_child",
        "video_empty_stream",
        "video_zero_rate",
        "video_quality_high",
        "video_quality_bool",
        "map2d_empty_costmap",
        "map2d_negative_rate",
        "teleop_empty_stream",
        "teleop_zero_linear",
        "teleop_negative_boost",
        "teleop_bool_rate",
        "teleop_zero_watchdog",
    ],
)
def test_authoring_validation_errors(build) -> None:
    with pytest.raises(ValueError):
        build()


def test_pages_reject_non_panels() -> None:
    with pytest.raises(ValueError, match="pages entries must be panels"):
        cockpit(layout=Video(), pages=[Row(Map2D())])


def test_go2_hello_fits_the_control_payload_cap() -> None:
    # The whole manifest rides one @control hello frame (wt_client caps its
    # payload at MAX_CONTROL_PAYLOAD_BYTES and raises loudly beyond). The v5
    # cap is generous - the v4 datagram budget was ~1 KB and the go2 hello
    # sat at 999 B - so this is a sanity pin, not a tight budget.
    hello = Hello(
        v=PROTOCOL_VERSION,
        role="robot",
        robot=RobotInfo(
            id="a-realistic-go2-hostname-01",
            name="a-realistic-go2-hostname-01",
            model="unitree_go2",
        ),
        manifest=manifest_of(cockpit(layout=GO2_LAYOUT)),
    )
    size = len(encode_datagram(hello))
    assert size <= MAX_CONTROL_PAYLOAD_BYTES, f"go2 hello grew to {size} B"


@dataclass(frozen=True)
class _OpsNote:
    text: str
    priority: int


@web_encoder("path.ck.v1")
def _encode_path_xy(msg: Path) -> EncodedPayload:
    payload = b"".join(struct.pack("<ff", p.position.x, p.position.y) for p in msg.poses)
    return EncodedPayload(payload, {"n": len(msg.poses)})


def test_channel_shared_tx_accepted() -> None:
    channel = Channel(
        "human_input",
        str,
        dir="tx",
        encoding="text.json.v1",
        publish="shared",
        required_scope="chat:send",
    )
    assert channel.publish == "shared"
    assert channel.required_scope == "chat:send"
    # required_scope stays optional.
    assert Channel("goal", dict, dir="tx", publish="shared").required_scope is None


def test_channel_publish_policy_rules() -> None:
    # rx channels never declare a policy or scope.
    with pytest.raises(ValueError, match="publish='none'"):
        Channel("note", dict, publish="shared")
    with pytest.raises(ValueError, match="required_scope needs a publish policy"):
        Channel("note", dict, required_scope="chat:send")
    # Exclusive arrives with the lease ticket.
    with pytest.raises(ValueError, match=r"\(W8\)"):
        Channel("goal", dict, dir="tx", publish="exclusive")
    # publish="none" tx streams are the specialized protocol paths (teleop).
    with pytest.raises(ValueError, match="specialized protocol paths"):
        Channel("goal", dict, dir="tx")
    # Generic publish is reliable-only.
    with pytest.raises(ValueError, match="delivery='reliable'"):
        Channel("goal", dict, dir="tx", publish="shared", delivery="latest")
    # Scope uses the manifest id bound.
    with pytest.raises(ValueError, match="required_scope must be 1..64"):
        Channel("goal", dict, dir="tx", publish="shared", required_scope="")
    with pytest.raises(ValueError, match="required_scope must be 1..64"):
        Channel("goal", dict, dir="tx", publish="shared", required_scope="x" * 65)


def test_channel_message_type_must_be_a_class() -> None:
    with pytest.raises(TypeError, match="message_type must be a class"):
        Channel("note", "str")


@pytest.mark.parametrize(
    "build",
    [
        lambda: Channel("", dict),
        lambda: Channel("x" * 65, dict),
        lambda: Channel("@note", dict),
        lambda: Channel("note", dict, dir="sideways"),
        lambda: Channel("note", dict, encoding=""),
        lambda: Channel("note", dict, encoding="x" * 65),
        lambda: Channel("note", dict, delivery="mostly"),
        lambda: Channel("note", dict, max_hz=0),
        lambda: Channel("note", dict, max_hz=True),
        lambda: Channel("note", dict, publish="all"),
        lambda: Channel("note", dict, params=[("a", 1)]),
    ],
    ids=[
        "empty_stream",
        "long_stream",
        "reserved_stream",
        "bad_dir",
        "empty_encoding",
        "long_encoding",
        "bad_delivery",
        "zero_rate",
        "bool_rate",
        "bad_publish",
        "params_not_mapping",
    ],
)
def test_channel_validation_errors(build) -> None:
    with pytest.raises(ValueError):
        build()


def test_channel_params_are_copied() -> None:
    params = {"scale": 2, "nested": {"a": [1, 2]}}
    channel = Channel("note", dict, params=params)
    params["scale"] = 99
    params["nested"]["a"].append(3)
    assert channel.params == {"scale": 2, "nested": {"a": (1, 2)}}


def test_channel_params_are_immutable() -> None:
    channel = Channel("note", dict, params={"scale": 2, "nested": {"a": [1, 2]}})
    with pytest.raises(TypeError, match="immutable"):
        channel.params["scale"] = 10
    with pytest.raises(TypeError, match="immutable"):
        del channel.params["scale"]
    with pytest.raises(TypeError, match="immutable"):
        channel.params.update({"scale": 10})
    with pytest.raises(TypeError, match="immutable"):
        channel.params["nested"]["a"] = []
    # Nested sequences freeze to tuples: no append surface at all.
    assert channel.params["nested"]["a"] == (1, 2)
    restored = pickle.loads(pickle.dumps(channel))
    assert restored.params == channel.params
    with pytest.raises(TypeError, match="immutable"):
        restored.params["scale"] = 10


def test_channel_params_must_be_json_shaped() -> None:
    with pytest.raises(ValueError, match="params keys must be strings"):
        Channel("note", dict, params={1: "x"})
    with pytest.raises(ValueError, match="JSON-shaped"):
        Channel("note", dict, params={"blob": b"\x00"})
    with pytest.raises(ValueError, match="JSON-shaped"):
        Channel("note", dict, params={"n": float("nan")})


def test_channel_nested_params_emit_plain_json_in_the_manifest() -> None:
    blueprint = cockpit(
        channels=[Channel("cfg", dict, params={"tags": ["a", "b"], "nested": {"n": 1}})]
    )
    (atom,) = blueprint.blueprints
    manifest = atom.kwargs["manifest"]
    (channel,) = manifest["channels"]
    # The frozen authoring form (tuples/_FrozenDict) must not leak into the
    # manifest: pinned manifests compare with == and json round-trips must
    # be idempotent.
    assert channel["params"] == {"tags": ["a", "b"], "nested": {"n": 1}}
    assert isinstance(channel["params"]["tags"], list)
    assert type(channel["params"]["nested"]) is dict
    assert parse_manifest(manifest).model_dump() == manifest
    (spec,) = atom.kwargs["channels"]
    assert isinstance(spec.params["tags"], list)


def test_channels_only_blueprint() -> None:
    blueprint = cockpit(
        channels=[
            Channel("nav_path", Path, encoding="path.ck.v1", delivery="reliable", max_hz=20.0),
            Channel("ops_note", _OpsNote),
        ]
    )
    (atom,) = blueprint.blueprints
    manifest = atom.kwargs["manifest"]
    assert [c["ch"] for c in manifest["channels"]] == ["nav_path", "ops_note"]
    assert manifest["channels"][0]["encoding"] == "path.ck.v1"
    assert manifest["panels"] == [] and manifest["layout"] is None and manifest["pages"] == []
    assert parse_manifest(manifest).model_dump() == manifest
    # Custom streams ride a generated RelayBridgeModule subclass with real
    # typed ports.
    assert atom.module is not RelayBridgeModule
    assert issubclass(atom.module, RelayBridgeModule)
    assert any(
        s.name == "nav_path" and s.type is Path and s.direction == "in" for s in atom.streams
    )
    specs = atom.kwargs["channels"]
    assert [s.ch for s in specs] == ["nav_path", "ops_note"]
    assert specs[0].encoder is _encode_path_xy and specs[0].message_type is Path
    assert specs[0].encoder_takes_params is False
    assert specs[1].encoder is encode_json_v1 and specs[1].message_type is _OpsNote
    # Blueprint kwargs cross the forkserver Pipe: class identity and the
    # by-reference encoder must survive pickling.
    restored = pickle.loads(pickle.dumps(blueprint))
    (ratom,) = restored.blueprints
    assert ratom.module is atom.module
    assert ratom.kwargs["channels"][0].encoder is _encode_path_xy
    assert ratom.kwargs["manifest"] == manifest


def test_publish_tx_channel_blueprint() -> None:
    blueprint = cockpit(
        channels=[
            Channel(
                "human_input",
                str,
                dir="tx",
                encoding="text.json.v1",
                publish="shared",
                required_scope="chat:send",
                max_hz=2.0,
            ),
        ]
    )
    (atom,) = blueprint.blueprints
    manifest = atom.kwargs["manifest"]
    (channel,) = manifest["channels"]
    assert channel["dir"] == "tx" and channel["delivery"] == "reliable"
    assert channel["publish"] == "shared" and channel["requiredScope"] == "chat:send"
    assert parse_manifest(manifest).model_dump() == manifest
    # Publish streams ride a generated subclass with a typed Out port.
    assert issubclass(atom.module, RelayBridgeModule) and atom.module is not RelayBridgeModule
    assert any(
        s.name == "human_input" and s.type is str and s.direction == "out" for s in atom.streams
    )
    (spec,) = atom.kwargs["channels"]
    assert spec.dir == "tx" and spec.publish == "shared" and spec.required_scope == "chat:send"
    assert spec.decoder is decode_text and spec.decoder_takes_context is False
    assert spec.encoder is None
    # Blueprint kwargs cross the forkserver Pipe: the by-reference decoder
    # must survive pickling.
    restored = pickle.loads(pickle.dumps(blueprint))
    (ratom,) = restored.blueprints
    assert ratom.module is atom.module
    assert ratom.kwargs["channels"][0].decoder is decode_text


def test_publish_tx_generic_json_and_dataclass_rejection() -> None:
    (atom,) = cockpit(channels=[Channel("counter", int, dir="tx", publish="shared")]).blueprints
    (spec,) = atom.kwargs["channels"]
    assert spec.decoder is decode_json_v1 and spec.decoder_takes_context is False
    # Reconstructing a dataclass from untrusted browser JSON needs an
    # explicit decoder (narrower than the encoder side).
    with pytest.raises(ValueError, match="register an explicit decoder"):
        cockpit(channels=[Channel("ops_note", _OpsNote, dir="tx", publish="shared")])


def test_publish_tx_codec_errors() -> None:
    with pytest.raises(ValueError, match=r"@web_decoder\('goal.json.v1'\)"):
        cockpit(
            channels=[Channel("goal", dict, dir="tx", encoding="goal.json.v1", publish="shared")]
        )
    with pytest.raises(ValueError, match="decodes to str, not int"):
        cockpit(
            channels=[
                Channel("human_input", int, dir="tx", encoding="text.json.v1", publish="shared")
            ]
        )
    # Non-JSON-family encodings fail the manifest's invalid_publish rule.
    with pytest.raises(ManifestError, match="invalid_publish"):
        cockpit(channels=[Channel("blob", bytes, dir="tx", encoding="blob.v1", publish="shared")])


def test_publish_channel_conflicts_with_teleop_panel() -> None:
    with pytest.raises(ValueError, match="conflicting requirements for stream 'tele_cmd_vel'"):
        cockpit(
            layout=Teleop(),
            channels=[
                Channel("tele_cmd_vel", Twist, dir="tx", encoding="twist.json.v1", publish="shared")
            ],
        )


def test_default_path_channels_kwarg_matches_manifest() -> None:
    (atom,) = cockpit().blueprints
    assert atom.module is RelayBridgeModule
    specs = atom.kwargs["channels"]
    rx = [c["ch"] for c in GO2_MANIFEST["channels"] if c["dir"] == "rx"]
    assert [s.ch for s in specs] == rx
    assert all(s.encoder is not None for s in specs)
    costmap = next(s for s in specs if s.ch == "global_costmap")
    assert costmap.resend_on_subscribe


def test_pages_only_still_gets_the_default_preset() -> None:
    manifest = manifest_of(cockpit(pages=[Video("color_image", max_hz=12.0)]))
    # The preset grid keeps ids p0..p2; the page panel follows as p3.
    assert [p["id"] for p in manifest["panels"]] == ["p0", "p1", "p2", "p3"]
    assert manifest["pages"] == ["p3"]


def test_explicit_channel_merges_with_panel_request() -> None:
    blueprint = cockpit(
        layout=Video("front_cam", quality=60, max_hz=12.0),
        channels=[
            Channel(
                "front_cam",
                Image,
                encoding="jpeg.v1",
                delivery="latest",
                max_hz=24.0,
                params={"quality": 60},
            )
        ],
    )
    (atom,) = blueprint.blueprints
    manifest = atom.kwargs["manifest"]
    (channel,) = manifest["channels"]
    assert channel["ch"] == "front_cam" and channel["maxHz"] == 24.0
    assert manifest["panels"][0]["channels"] == ["front_cam"]
    assert any(s.name == "front_cam" and s.type is Image for s in atom.streams)


@pytest.mark.parametrize(
    "channel",
    [
        Channel("front_cam", Image, encoding="jpeg.v1", delivery="latest", params={"quality": 90}),
        Channel("front_cam", Image),
        Channel(
            "front_cam", Image, encoding="jpeg.v1", delivery="reliable", params={"quality": 60}
        ),
    ],
    ids=["params_conflict", "encoding_conflict", "delivery_conflict"],
)
def test_explicit_channel_conflicts_with_panel_raise(channel: Channel) -> None:
    with pytest.raises(ValueError, match="conflicting requirements for stream 'front_cam'"):
        cockpit(layout=Video("front_cam", quality=60), channels=[channel])


def test_builtin_stream_type_and_table_mismatches() -> None:
    with pytest.raises(ValueError, match="does not match the bridge port type PoseStamped"):
        cockpit(channels=[Channel("odom", Twist, encoding="pose.json.v1")])
    with pytest.raises(ValueError, match="'odom' encodes pose.json.v1, not json.v1"):
        cockpit(channels=[Channel("odom", PoseStamped)])
    with pytest.raises(ValueError, match="'odom' delivers reliable, not latest"):
        cockpit(channels=[Channel("odom", PoseStamped, encoding="pose.json.v1", delivery="latest")])


def test_json_v1_requires_explicit_codec_for_large_types() -> None:
    # A mistaken image declaration must fail at authoring, not serialize a
    # pixel buffer per frame.
    with pytest.raises(
        ValueError, match=r"'snapshot'.*Image.*not supported by json\.v1.*@web_encoder"
    ):
        cockpit(channels=[Channel("snapshot", Image)])


def test_unregistered_custom_encoding_names_the_decorator() -> None:
    with pytest.raises(
        ValueError, match=r"'blob'.*no encoder registered.*@web_encoder\('blob\.bin\.v9'\)"
    ):
        cockpit(channels=[Channel("blob", dict, encoding="blob.bin.v9")])


def test_panel_on_undeclared_custom_stream_still_raises_unknown() -> None:
    # The typo guard survives channels=: panels alone cannot mint streams,
    # and the error lists the declared ones next to the built-ins.
    with pytest.raises(ValueError, match="unknown stream 'lidr'.*nav_path"):
        cockpit(
            layout=Video("lidr"),
            channels=[Channel("nav_path", Path, encoding="path.ck.v1", delivery="reliable")],
        )


def test_duplicate_channel_declaration_raises() -> None:
    with pytest.raises(ValueError, match="duplicate channel declaration for stream 'note'"):
        cockpit(channels=[Channel("note", dict), Channel("note", dict)])


def test_non_channel_entry_raises() -> None:
    with pytest.raises(ValueError, match="channels entries must be Channel"):
        cockpit(channels=[Video("color_image")])


@pytest.mark.parametrize("stream", ["encoded", "ref", "class"])
def test_reserved_stream_names_rejected(stream: str) -> None:
    with pytest.raises(ValueError):
        cockpit(channels=[Channel(stream, dict)])


def test_channel_ordering_builtins_then_customs_then_publish() -> None:
    blueprint = cockpit(
        layout=GO2_LAYOUT,
        channels=[
            Channel("human_input", str, dir="tx", encoding="text.json.v1", publish="shared"),
            Channel("target_pose", PoseStamped, encoding="pose.json.v1", max_hz=5.0),
            Channel("ops_note", _OpsNote),
        ],
    )
    (atom,) = blueprint.blueprints
    assert [c["ch"] for c in atom.kwargs["manifest"]["channels"]] == [
        "color_image",
        "odom",
        "global_costmap",
        "target_pose",
        "ops_note",
        "tele_cmd_vel",
        "human_input",
    ]


def test_import_stays_light() -> None:
    # The authoring surface must be importable without the [web] extra:
    # neither the bridge module nor aioquic may load until cockpit() runs.
    code = (
        "import sys; import dimos.web.cockpit; "
        "assert 'dimos.web.relay_bridge.relay_bridge_module' not in sys.modules; "
        "assert 'aioquic' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
