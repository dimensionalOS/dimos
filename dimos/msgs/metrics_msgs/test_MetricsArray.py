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

from dimos_lcm.diagnostic_msgs import KeyValue
import rerun as rr

from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.metrics_msgs.MetricsArray import MetricsArray
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule


def test_numeric_metrics_round_trip_through_lcm() -> None:
    original = MetricsArray.from_numeric_values(
        "metrics/camera/color",
        {"fps": 2.0, "invalid_timestamps": 1},
        timestamp=123.25,
        hardware_id="booster_b1",
    )

    decoded = MetricsArray.lcm_decode(original.lcm_encode())

    assert resolve_msg_type("metrics_msgs.MetricsArray") is MetricsArray
    assert isinstance(decoded, MetricsArray)
    assert decoded.header.stamp.sec == 123
    assert decoded.header.stamp.nsec == 250_000_000
    [status] = decoded.status
    assert status.name == "metrics/camera/color"
    assert status.hardware_id == "booster_b1"
    assert [(value.key, value.value) for value in status.values] == [
        ("fps", "2.0"),
        ("invalid_timestamps", "1"),
    ]


def test_numeric_metrics_convert_to_rerun_scalar_paths(mocker) -> None:
    message = MetricsArray.from_numeric_values(
        "metrics/camera/depth",
        {"fps": 15.0, "frame_age/p95_ms": 42.5},
        timestamp=123.25,
    )
    message.status[0].values.append(KeyValue(key="state", value="healthy"))
    message.status[0].values_length += 1
    scalars = mocker.patch("rerun.Scalars", side_effect=lambda value: f"scalar:{value}")

    result = message.to_rerun()

    assert result == [
        ("metrics/camera/depth/fps", "scalar:15.0"),
        ("metrics/camera/depth/frame_age/p95_ms", "scalar:42.5"),
    ]
    assert scalars.call_count == 2


def test_rerun_bridge_logs_metrics_into_metrics_tree(mocker) -> None:
    message = MetricsArray.from_numeric_values(
        "metrics/camera/color",
        {"fps": 15.0, "frame_age/p95_ms": 42.5},
        timestamp=123.25,
    )
    bridge = object.__new__(RerunBridgeModule)
    bridge.config = Config(pubsubs=[])
    bridge._min_intervals = {}
    bridge._last_log = {}
    bridge._override_cache = {}
    bridge._frame_attached = {}
    log = mocker.patch("dimos.visualization.rerun.bridge.rr.log")

    bridge._on_message(
        message,
        Topic("/camera_metrics", lcm_type=MetricsArray),
    )

    assert [entry.args[0] for entry in log.call_args_list] == [
        "metrics/camera/color/fps",
        "metrics/camera/color/frame_age/p95_ms",
    ]
    assert all(isinstance(entry.args[1], rr.Scalars) for entry in log.call_args_list)
