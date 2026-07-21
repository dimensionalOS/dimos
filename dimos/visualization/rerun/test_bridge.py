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

from collections.abc import Iterator
import threading
from typing import Any

import numpy as np
import pytest

from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.patterns import Glob
from dimos.visualization.rerun.bridge import RerunBridgeModule


@pytest.fixture()
def bridge(mocker) -> Iterator[RerunBridgeModule]:
    mocker.patch.object(global_config, "transport", "lcm")
    instance = RerunBridgeModule(
        blueprint=None,
        rerun_open="none",
        rerun_web=False,
        visual_override={
            "world/clause1/image": {
                "mode": "points",
                "voxel_size": 0.02,
                "colors": [255, 0, 0],
            },
            "world/clause1/pointcloud": {
                "mode": "boxes",
                "voxel_size": 0.12,
                "colors": [0, 0, 255],
                "fill_mode": "densewireframe",
                "bottom_cutoff": 0.0,
            },
        },
    )
    yield instance
    instance.stop()


@pytest.fixture()
def raw_lcm(lcm_url: str) -> Iterator[LCMPubSubBase]:
    pubsub = LCMPubSubBase(url=lcm_url)
    pubsub.start()
    yield pubsub
    pubsub.stop()


def _image() -> Image:
    return Image(np.full((2, 2, 3), [1, 2, 3], dtype=np.uint8), frame_id="camera", ts=1.0)


def _pointcloud() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar", timestamp=1.0
    )


def _rerun_image(image: Image) -> Any:
    return image.to_rerun()


def _start_bridge(bridge: RerunBridgeModule, mocker) -> None:
    mocker.patch("dimos.visualization.rerun.bridge.rerun_init", return_value="rerun+http://host")
    mocker.patch("dimos.visualization.rerun.bridge.rr.get_recording_id", return_value="recording")
    mocker.patch.object(bridge, "_log_connect_hints")


def test_clause_1_native_bridge_keeps_default_heavy_packets_out_of_python_callback(
    bridge: RerunBridgeModule,
    mocker,
    raw_lcm: LCMPubSubBase,
    retry_until,
) -> None:
    """Clause 1 DIM-1125: native-owned Image packets never reach the Python packet callback."""
    _start_bridge(bridge, mocker)
    native = mocker.Mock()
    start_native = mocker.patch(
        "dimos.visualization.rerun.bridge.start_native_rerun_bridge", return_value=native
    )
    on_packet = mocker.spy(bridge, "_on_packet")
    light_received = threading.Event()

    def on_message(_: Any, topic: Topic) -> None:
        if topic.lcm_type is Twist:
            light_received.set()

    mocker.patch.object(bridge, "_on_message", side_effect=on_message)
    image_decode = mocker.spy(Image, "lcm_decode")
    pointcloud_decode = mocker.spy(PointCloud2, "lcm_decode")
    image_topic = Topic("/clause1/image", Image)
    pointcloud_topic = Topic("/clause1/pointcloud", PointCloud2)
    light_topic = Topic("/clause1/light", Twist)
    image = _image()
    pointcloud = _pointcloud()
    twist = Twist((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    bridge.start()
    retry_until(
        light_received,
        lambda: (
            raw_lcm.publish(image_topic, image.lcm_encode()),
            raw_lcm.publish(pointcloud_topic, pointcloud.lcm_encode()),
            raw_lcm.publish(light_topic, twist.lcm_encode()),
        ),
    )

    start_native.assert_called_once()
    native_topics = start_native.call_args.kwargs["native_topics"]
    assert {entity: config.model_dump() for entity, config in native_topics.items()} == {
        "world/clause1/image": {
            "voxel_size": 0.02,
            "colors": (255, 0, 0),
            "mode": "points",
            "fill_mode": "solid",
            "bottom_cutoff": None,
        },
        "world/clause1/pointcloud": {
            "voxel_size": 0.12,
            "colors": (0, 0, 255),
            "mode": "boxes",
            "fill_mode": "densewireframe",
            "bottom_cutoff": 0.0,
        },
    }
    assert on_packet.call_count == 1
    assert on_packet.call_args.args[1].lcm_type is Twist
    assert image_decode.call_count == 0
    assert pointcloud_decode.call_count == 0


def test_bridge_native_startup_failure_surfaces_without_python_heavy_logging(
    bridge: RerunBridgeModule,
    mocker,
) -> None:
    """Native startup failure propagates and never activates Python heavy decode or logging."""
    _start_bridge(bridge, mocker)
    mocker.patch(
        "dimos.visualization.rerun.bridge.start_native_rerun_bridge",
        side_effect=FileNotFoundError("native/rust is unavailable"),
    )
    log = mocker.patch("dimos.visualization.rerun.bridge.rr.log")
    decode = mocker.spy(Image, "lcm_decode")

    with pytest.raises(FileNotFoundError, match="native/rust is unavailable"):
        bridge.start()

    assert decode.call_count == 0
    assert log.call_count == 0


def test_callable_glob_only_subscribes_matching_heavy_packets_to_python(
    bridge: RerunBridgeModule,
    mocker,
    raw_lcm: LCMPubSubBase,
    retry_until,
) -> None:
    bridge.config.visual_override = {Glob("world/glob/*"): _rerun_image}
    _start_bridge(bridge, mocker)
    native = mocker.Mock()
    start_native = mocker.patch(
        "dimos.visualization.rerun.bridge.start_native_rerun_bridge", return_value=native
    )
    accepted = mocker.spy(bridge, "_decode_in_python")
    received_topics: set[str] = set()
    received = threading.Event()
    matched_topic = Topic("/glob/matched", Image)
    unmatched_topic = Topic("/outside/unmatched", Image)
    light_topic = Topic("/glob/light", Twist)
    expected_topics = {matched_topic.topic, light_topic.topic}

    def on_message(_: Any, topic: Topic) -> None:
        received_topics.add(topic.topic)
        if received_topics == expected_topics:
            received.set()

    mocker.patch.object(bridge, "_on_message", side_effect=on_message)
    image = _image()
    twist = Twist((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    bridge.start()
    retry_until(
        received,
        lambda: (
            raw_lcm.publish(unmatched_topic, image.lcm_encode()),
            raw_lcm.publish(matched_topic, image.lcm_encode()),
            raw_lcm.publish(light_topic, twist.lcm_encode()),
        ),
    )

    accepted_topics = {call.args[0].topic for call in accepted.call_args_list}
    assert accepted_topics == expected_topics
    assert received_topics == expected_topics
    assert start_native.call_args.kwargs["python_topic_patterns"] == [Glob("world/glob/*").pattern]
