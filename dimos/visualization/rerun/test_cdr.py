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

"""Generated values stay plain while the viewer owns rendering and subscriptions."""

import json
from unittest.mock import Mock

import numpy as np
import pytest
import rerun as rr

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import PointStamped
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.cdr import generated_archetypes
from dimos.web.relay_bridge.gen_cdr_fixtures import build_messages


@pytest.fixture
def bridge(mocker):
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    instance = RerunBridgeModule(pubsubs=[])
    yield instance
    instance.stop()


def test_pose_axes_fields_and_frame_are_kept_separate_from_value_class() -> None:
    message = build_messages()[0][1]
    entries = generated_archetypes(message, "world/pose")
    assert entries is not None
    assert [path for path, _ in entries] == ["world/pose", "world/pose", "world/pose/message"]
    transform, axes, text = [archetype for _, archetype in entries]
    np.testing.assert_allclose(
        transform.translation.as_arrow_array().to_pylist(), [[1.5, -2.5, 0.25]]
    )
    np.testing.assert_allclose(
        transform.quaternion.as_arrow_array().to_pylist(), [[0, 0, 0.6, 0.8]]
    )
    assert transform.parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
    assert isinstance(axes, rr.TransformAxes3D)
    fields = json.loads(text.text.as_arrow_array().to_pylist()[0])
    assert fields["header"] == {
        "stamp": {"sec": 1757400000, "nanosec": 500000123},
        "frame_id": "map",
    }
    assert "__msgtype__" not in fields
    assert not hasattr(message, "to_rerun")


def test_point_frame_and_unknown_values() -> None:
    pose = build_messages()[0][1]
    msg = PointStamped(header=pose.header, point=pose.pose.position)
    entries = generated_archetypes(msg, "world/point")
    assert entries is not None
    assert isinstance(entries[0][1], rr.Points3D)
    assert entries[1][1].parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
    assert generated_archetypes({"x": 1}, "world/no") is None


@pytest.mark.parametrize("nanoseconds", [0, 1757400000500000123])
def test_source_time_is_scoped_and_reset_after_render_failure(bridge, mocker, nanoseconds) -> None:
    msg = build_messages()[0][1]
    msg.header.stamp = Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)
    render = mocker.patch.object(bridge, "_on_message", side_effect=ValueError("bad renderer"))
    set_time = mocker.patch.object(rr, "set_time")
    clear_time = mocker.patch.object(rr, "disable_timeline")
    with pytest.raises(ValueError, match="bad renderer"):
        bridge._on_cdr_message(msg, topic="/pose")
    render.assert_called_once_with(msg, "/pose")
    set_time.assert_called_once_with("source", timestamp=np.datetime64(nanoseconds, "ns"))
    clear_time.assert_called_once_with("source")


def test_subscription_cleanup_order_and_failed_start(bridge) -> None:
    operations: list[str] = []
    transport = Mock(spec=CdrTransport)
    transport.subscribe.return_value = lambda: operations.append("unsubscribe")
    transport.stop.side_effect = lambda: operations.append("stop")
    bridge._subscribe_cdr("pose", transport)
    bridge.stop()
    assert operations == ["unsubscribe", "stop"]
    failed = Mock(spec=CdrTransport)
    failed.subscribe.side_effect = RuntimeError("native start failed")
    with pytest.raises(RuntimeError, match="native start failed"):
        bridge._subscribe_cdr("pose", failed)
    failed.stop.assert_called_once_with()


def test_bridge_default_renderer_handles_generated_pose(bridge, mocker) -> None:
    log = mocker.patch.object(rr, "log")
    bridge._on_message(build_messages()[0][1], "/pose")
    assert [call.args[0] for call in log.call_args_list] == [
        "world/pose",
        "world/pose",
        "world/pose/message",
    ]
    assert isinstance(log.call_args_list[0].args[1], rr.Transform3D)
