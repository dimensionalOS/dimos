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

from collections.abc import Callable, Iterator
import json
from pathlib import Path
import signal
import subprocess
from typing import Any, Literal

import numpy as np
import pytest
import rerun as rr
from rerun.experimental import RrdReader

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, LCMPubSubBase, Topic as LCMTopic
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, Zenoh, ZenohPubSubBase
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.visualization.rerun.bridge import RerunBridgeModule

Backend = Literal["lcm", "zenoh"]
_NATIVE_RUST_DIR = DIMOS_PROJECT_ROOT / "native" / "rust"


@pytest.fixture(scope="module")
def native_rerun_binary() -> Path:
    """Build the native Rerun bridge once for the opt-in end-to-end tests."""
    subprocess.run(
        ["cargo", "build", "-p", "dimos-rerun-bridge"],
        cwd=_NATIVE_RUST_DIR,
        check=True,
    )
    return _NATIVE_RUST_DIR / "target" / "debug" / "dimos-rerun-bridge"


@pytest.fixture()
def native_process(
    native_rerun_binary: Path,
) -> Iterator[Callable[[dict[str, Any]], subprocess.Popen[bytes]]]:
    """Start native bridge processes and terminate any left by a failed assertion."""
    processes: list[subprocess.Popen[bytes]] = []

    def start(config: dict[str, Any]) -> subprocess.Popen[bytes]:
        process = subprocess.Popen(
            [str(native_rerun_binary)],
            cwd=_NATIVE_RUST_DIR,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        assert process.stdin is not None
        process.stdin.write(json.dumps({"config": config}).encode() + b"\n")
        process.stdin.close()
        processes.append(process)
        return process

    yield start

    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=5)


@pytest.fixture()
def publisher_factory(
    lcm_url: str,
) -> Iterator[Callable[[Backend, bool], LCMPubSubBase | ZenohPubSubBase]]:
    """Create started typed or raw pubsubs and release their transport resources."""
    publishers: list[LCMPubSubBase | ZenohPubSubBase] = []
    pools: list[ZenohSessionPool] = []

    def create(backend: Backend, typed: bool = True) -> LCMPubSubBase | ZenohPubSubBase:
        if backend == "lcm":
            publisher = LCM(url=lcm_url) if typed else LCMPubSubBase(url=lcm_url)
        else:
            pool = ZenohSessionPool()
            pools.append(pool)
            publisher = Zenoh(session_pool=pool) if typed else ZenohPubSubBase(session_pool=pool)
        publisher.start()
        publishers.append(publisher)
        return publisher

    yield create

    for publisher in publishers:
        publisher.stop()
    for pool in pools:
        pool.close_all()


def _recorded_entity_paths(path: Path) -> set[str]:
    return {chunk.entity_path.lstrip("/") for chunk in RrdReader(path).stream().to_chunks()}


def _recorded_archetypes(path: Path, entity: str) -> set[str]:
    archetypes: set[str] = set()
    for chunk in RrdReader(path).stream().to_chunks():
        if chunk.entity_path.lstrip("/") != entity:
            continue
        for field in chunk.to_record_batch().schema:
            if field.metadata is not None and b"rerun:archetype" in field.metadata:
                archetypes.add(field.metadata[b"rerun:archetype"].decode())
    return archetypes


def _rerun_image(image: Image) -> Any:
    return image.to_rerun()


@pytest.mark.native_rust
@pytest.mark.parametrize("backend", ("lcm", "zenoh"))
def test_glob_callback_only_sends_matching_heavy_packets_to_python(
    backend: Backend,
    native_process: Callable[[dict[str, Any]], subprocess.Popen[bytes]],
    publisher_factory: Callable[[Backend, bool], LCMPubSubBase | ZenohPubSubBase],
    mocker,
    tmp_path: Path,
    wait_until,
) -> None:
    """Native-owned heavy packets stay out of Python beside a callable Glob."""
    recording_id = f"native-rerun-{backend}-{tmp_path.name}"
    recording_path = tmp_path / f"{backend}.rrd"
    topic_base = f"e2e/{backend}"
    if backend == "lcm":
        topic_type = LCMTopic
        topic_prefix = "/"
    else:
        topic_type = ZenohTopic
        topic_prefix = "dimos/"
    image_topic = topic_type(f"{topic_prefix}{topic_base}/image", Image)
    python_image_topic = topic_type(f"{topic_prefix}{topic_base}/python/image", Image)
    points_topic = topic_type(f"{topic_prefix}{topic_base}/points", PointCloud2)
    boxes_topic = topic_type(f"{topic_prefix}{topic_base}/boxes", PointCloud2)
    ignored_topic = topic_type(f"{topic_prefix}{topic_base}/ignored", PointCloud2)
    light_topic = topic_type(f"{topic_prefix}{topic_base}/light", PointStamped)
    light_entity = f"world/{topic_base}/light"
    python_image_entity = f"world/{topic_base}/python/image"
    expected_entities = {
        f"world/{topic_base}/image",
        f"world/{topic_base}/points",
        f"world/{topic_base}/boxes",
    }
    ignored_entity = f"world/{topic_base}/ignored"
    visual_override = {
        Glob(f"world/{topic_base}/python/*"): _rerun_image,
        f"world/{topic_base}/points": {
            "voxel_size": 0.02,
            "colors": [255, 0, 0],
            "mode": "points",
            "fill_mode": "solid",
            "bottom_cutoff": None,
        },
        f"world/{topic_base}/boxes": {
            "voxel_size": 0.12,
            "colors": [0, 0, 255],
            "mode": "boxes",
            "fill_mode": "densewireframe",
            "bottom_cutoff": 0.0,
        },
        ignored_entity: None,
    }
    mocker.patch.object(global_config, "transport", backend)
    bridge = RerunBridgeModule(
        blueprint=None,
        connect_url="rerun+http://127.0.0.1:9876/proxy",
        visual_override=visual_override,
        rerun_open="none",
        rerun_web=False,
    )
    mocker.patch(
        "dimos.visualization.rerun.bridge.rerun_init",
        return_value="rerun+http://127.0.0.1:9876/proxy",
    )
    mocker.patch("dimos.visualization.rerun.bridge.rr.get_recording_id", return_value=recording_id)
    mocker.patch.object(bridge, "_log_connect_hints")
    python_log = mocker.patch("dimos.visualization.rerun.bridge.rr.log")
    python_accept = mocker.spy(bridge, "_decode_in_python")
    python_callback = mocker.spy(bridge, "_on_packet")
    image_decode = mocker.spy(Image, "lcm_decode")
    pointcloud_decode = mocker.spy(PointCloud2, "lcm_decode")
    image_convert = mocker.spy(Image, "to_rerun")
    pointcloud_convert = mocker.spy(PointCloud2, "to_rerun")
    light_decode = mocker.spy(PointStamped, "lcm_decode")

    def start_native(**config: Any) -> Any:
        config.pop("g")
        config["native_topics"] = {
            entity: topic_config.model_dump(mode="json")
            for entity, topic_config in config["native_topics"].items()
        }
        config["save_path"] = str(recording_path)
        process = native_process(config)

        def stop() -> None:
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=5) == 0

        native = mocker.Mock()
        native.stop.side_effect = stop
        return native

    mocker.patch(
        "dimos.visualization.rerun.bridge.start_native_rerun_bridge", side_effect=start_native
    )
    publisher = publisher_factory(backend)
    image = Image(np.full((2, 2, 3), [1, 2, 3], dtype=np.uint8), frame_id="camera", ts=1.0)
    pointcloud = PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar", timestamp=1.0
    )
    light = PointStamped(1.0, 2.0, 3.0, ts=1.0, frame_id="light")

    def publish_until_recorded() -> bool:
        publisher.publish(image_topic, image)
        publisher.publish(python_image_topic, image)
        publisher.publish(points_topic, pointcloud)
        publisher.publish(boxes_topic, pointcloud)
        publisher.publish(ignored_topic, pointcloud)
        publisher.publish(light_topic, light)
        python_entities = {call.args[0] for call in python_log.call_args_list}
        return (
            python_image_entity in python_entities
            and recording_path.exists()
            and expected_entities <= _recorded_entity_paths(recording_path)
        )

    bridge.start()
    try:
        wait_until(
            publish_until_recorded,
            timeout=10,
            interval=0.05,
            message=f"Mixed {backend} stream did not reach both Rerun bridges",
        )
    finally:
        bridge.stop()

    reader = RrdReader(recording_path)
    assert [entry.recording_id for entry in reader.recordings()] == [recording_id]
    recorded_entities = _recorded_entity_paths(recording_path)
    assert expected_entities <= recorded_entities
    assert ignored_entity not in recorded_entities
    assert python_image_entity not in recorded_entities
    callback_topics = {call.args[1].topic for call in python_callback.call_args_list}
    expected_python_topics = {python_image_topic.topic, light_topic.topic}
    assert callback_topics == expected_python_topics
    accepted_topics = {call.args[0].topic for call in python_accept.call_args_list}
    assert accepted_topics == expected_python_topics
    image_callbacks = sum(
        call.args[1].topic == python_image_topic.topic for call in python_callback.call_args_list
    )
    light_callbacks = sum(
        call.args[1].topic == light_topic.topic for call in python_callback.call_args_list
    )
    assert image_decode.call_count == image_callbacks
    assert light_decode.call_count == light_callbacks
    assert pointcloud_decode.call_count == 0
    assert image_convert.call_count == image_callbacks
    assert pointcloud_convert.call_count == 0
    python_entities = {call.args[0] for call in python_log.call_args_list}
    assert python_entities == {light_entity, python_image_entity}
    assert any(isinstance(call.args[1], rr.Points3D) for call in python_log.call_args_list)
    assert _recorded_archetypes(recording_path, f"world/{topic_base}/image") == {
        "rerun.archetypes.Image",
        "rerun.archetypes.Transform3D",
    }
    assert _recorded_archetypes(recording_path, f"world/{topic_base}/points") == {
        "rerun.archetypes.Points3D",
        "rerun.archetypes.Transform3D",
    }
    assert _recorded_archetypes(recording_path, f"world/{topic_base}/boxes") == {
        "rerun.archetypes.Boxes3D",
        "rerun.archetypes.Transform3D",
    }
