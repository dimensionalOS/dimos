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

"""Unit tests for HostedStatsModule — cmd-raw tap + recorder republish."""

from __future__ import annotations

from collections.abc import Iterator
import json
from unittest.mock import MagicMock

from dimos_cdrgen.registry import encode
import pytest

from dimos.messages.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.time import time_from_seconds
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.utils.video_stats import VideoStats
from dimos.web.relay_bridge.protocol import FrameHeader, decode_data_frame, encode_data_frame


@pytest.fixture
def module(mocker) -> Iterator[HostedStatsModule]:
    instance = HostedStatsModule()
    mocker.patch.object(instance, "go2", MagicMock())
    for port in ("cmd_vel_stamped", "video_stats", "telemetry_out"):
        mocker.patch.object(getattr(instance, port), "publish")
    yield instance
    instance.stop()


def _cmd(vx: float = 0.3, ts: float = 123.0) -> bytes:
    value = TwistStamped(
        Header(time_from_seconds(ts), "operator"), Twist(Vector3(vx, 0, 0), Vector3(0, 0, 0))
    )
    return encode_data_frame(
        FrameHeader(
            ch="twist",
            seq=0,
            ts=ts,
            delivery="latest",
            meta={"encoding": "cdr", "type": TwistStamped.__msgtype__},
        ),
        encode(value),
    )


def test_telemetry_without_go2(module, mocker) -> None:
    mocker.patch.object(module, "go2", None)
    module._on_robot_state(b'{"robot_type": "arm"}')

    payload = module._telemetry_payload()

    assert payload["soc"] is None
    assert payload["state"] == {"robot_type": "arm"}
    assert payload["type"] == "robot_telemetry"


def test_cmd_raw_republishes_stamped_for_recorder(module: HostedStatsModule) -> None:
    # Regression: the raw cmd tap must re-publish the decoded TwistStamped on
    # cmd_vel_stamped so the recorder gets a drive trace (was silently dropped).
    module._on_cmd_raw(_cmd(vx=0.5, ts=42.0))
    module.cmd_vel_stamped.publish.assert_called_once()
    published = module.cmd_vel_stamped.publish.call_args[0][0]
    assert published.header.stamp == time_from_seconds(42.0)
    assert published.twist.linear.x == 0.5


def test_cmd_raw_records_stats(module: HostedStatsModule) -> None:
    # Two frames so the stats accumulator has a rate/jitter snapshot.
    module._on_cmd_raw(_cmd(ts=1.0))
    module._on_cmd_raw(_cmd(ts=1.05))
    assert module._cmd_stats.snapshot() is not None  # accumulator saw the frames


def test_cmd_raw_ignores_undecodable_frame(module: HostedStatsModule) -> None:
    # A foreign / non-TwistStamped frame on the shared plane must not raise or
    # republish.
    module._on_cmd_raw(b"\x00\x01\x02not-a-twist")
    module.cmd_vel_stamped.publish.assert_not_called()


def test_state_json_dispatches_video_stats(module: HostedStatsModule) -> None:
    payload = json.dumps(
        {"type": "video_stats", "fps": 30.0, "bitrate_kbps": 1000, "width": 640, "height": 480}
    ).encode()
    module._on_state_json(payload)
    module.video_stats.publish.assert_called_once()


def test_state_json_ignores_foreign_kind(module: HostedStatsModule) -> None:
    # estop/sport/etc. on the shared state plane are owned by other modules.
    module._on_state_json(json.dumps({"type": "estop", "nonce": "x"}).encode())
    module.video_stats.publish.assert_not_called()


@pytest.mark.parametrize("damage", ["foreign", "encoding", "trailing", "encapsulation"])
def test_malformed_or_foreign_frames_do_not_record_stats_and_valid_frames_recover(module, damage):
    frame = _cmd()
    if damage == "trailing":
        frame += b"extra"
    elif damage == "encapsulation":
        value = decode_data_frame(frame)
        frame = encode_data_frame(value.header, b"bad!" + value.payload[4:])
    else:
        frame = (
            frame.replace(b"cdr", b"raw")
            if damage == "encoding"
            else frame.replace(b"TwistStamped", b"OtherMessage")
        )
    module._on_cmd_raw(frame)
    module.cmd_vel_stamped.publish.assert_not_called()
    assert module._cmd_stats.snapshot() is None
    module._on_cmd_raw(_cmd(ts=1.0))
    assert module._cmd_stats.snapshot() is None
    module._on_cmd_raw(_cmd(ts=1.05))
    assert module.cmd_vel_stamped.publish.call_count == 2
    assert module._cmd_stats.snapshot() is not None


def test_video_json_is_adapted_to_canonical_positional_joy(module):
    module._on_state_json(
        json.dumps(
            {
                "type": "video_stats",
                "ts": 42.25,
                "fps": 30,
                "kbps": 1000,
                "width": 640,
                "height": 480,
            }
        ).encode()
    )
    value = module.video_stats.publish.call_args.args[0]
    assert value.__msgtype__ == "sensor_msgs/msg/Joy"
    helper = VideoStats.from_joy(value)
    assert (helper.ts, helper.fps, helper.kbps, helper.width, helper.height) == (
        42.25,
        30,
        1000,
        640,
        480,
    )
