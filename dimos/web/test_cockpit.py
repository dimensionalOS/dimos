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

import pickle
import subprocess
import sys

import pytest

from dimos.web.cockpit import ChannelRequest, Col, Map2D, Panel, Row, Teleop, Video, cockpit
from dimos.web.relay_bridge.manifest import parse_manifest
from dimos.web.relay_bridge.protocol import PROTOCOL_VERSION, Hello, RobotInfo, encode_datagram
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule
from dimos.web.relay_bridge.wt_client import _HELLO_DATAGRAM_MAX_BYTES

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
        {
            "ch": "tele_cmd_vel",
            "dir": "tx",
            "encoding": "twist.json.v1",
            "delivery": "latest",
            "maxHz": 15.0,
            "params": {"maxLinear": 0.8, "maxAngular": 1.0, "boost": 2.0, "watchdogMs": 300.0},
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


def test_go2_hello_fits_the_datagram_budget() -> None:
    # The whole manifest rides one hello datagram (wt_client caps it at
    # _HELLO_DATAGRAM_MAX_BYTES and raises loudly beyond). Pin the go2
    # example under 1000 B so >= 100 B of headroom remains for longer
    # hostnames before authors ever see that error. The teleop channel
    # brought this to 999 B: the next channel (chat, T8) cannot fit and
    # must rethink the budget (trim params, or move hello off datagrams).
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
    assert size <= 1000, f"go2 hello grew to {size} B (cap {_HELLO_DATAGRAM_MAX_BYTES})"


def test_import_stays_light() -> None:
    # The authoring surface must be importable without the [web] extra:
    # neither the bridge module nor aioquic may load until cockpit() runs.
    code = (
        "import sys; import dimos.web.cockpit; "
        "assert 'dimos.web.relay_bridge.relay_bridge_module' not in sys.modules; "
        "assert 'aioquic' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
