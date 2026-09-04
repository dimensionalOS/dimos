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
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.hosted.hosted_stats import HostedStatsModule


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[HostedStatsModule]:
    """A HostedStatsModule with its tap-path state initialized for real (only the
    framework Module.__init__ is skipped) and its ports / driver ref / config mocked."""
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    module = HostedStatsModule()
    module.go2 = MagicMock()
    module.config = SimpleNamespace(telemetry_hz=3.0)
    for port in ("cmd_vel_stamped", "video_stats", "telemetry_out"):
        setattr(module, port, MagicMock())
    yield module


def _cmd(vx: float = 0.3, ts: float = 123.0) -> bytes:
    return TwistStamped(ts=ts, linear=Vector3(vx, 0, 0), angular=Vector3(0, 0, 0)).lcm_encode()


def test_cmd_raw_republishes_stamped_for_recorder(module: HostedStatsModule) -> None:
    # Regression: the raw cmd tap must re-publish the decoded TwistStamped on
    # cmd_vel_stamped so the recorder gets a drive trace (was silently dropped).
    module._on_cmd_raw(_cmd(vx=0.5, ts=42.0))
    module.cmd_vel_stamped.publish.assert_called_once()
    published = module.cmd_vel_stamped.publish.call_args[0][0]
    assert published.ts == 42.0
    assert published.linear.x == 0.5


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
