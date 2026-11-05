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

"""DdsBridge config checks, and an end to end run: rust DDS writers -> bridge -> python over zenoh."""

from __future__ import annotations

from collections.abc import Callable
import json
import os
import subprocess
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.protocol.dds_bridge.module import (
    DIMOS_TYPE_OF_ROS_TYPE,
    DdsBridge,
    DdsBridgeConfig,
    DdsQos,
    DdsToDimos,
)
from dimos.utils.testing.waiting import wait_until

_CRATE = DIMOS_PROJECT_ROOT / "dimos" / "protocol" / "dds_bridge" / "rust"
SAMPLES_PER_TOPIC = 3
# A domain no default ROS 2 or DDS install joins, varied per run so parallel runs stay apart.
TEST_DOMAIN_ID = 100 + os.getpid() % 100
DELIVERY_TIMEOUT_S = 30.0


def test_an_unknown_dds_type_is_rejected_naming_topic_and_type() -> None:
    with pytest.raises(ValidationError, match=r"'rt/gait'.*'dr_msgs/msg/Gait'"):
        DdsToDimos(dds_topic="rt/gait", dds_type="dr_msgs/msg/Gait", zenoh_topic="gait")


def test_two_topics_cannot_publish_onto_one_stream() -> None:
    with pytest.raises(ValidationError, match="same"):
        DdsBridgeConfig(
            dds_to_dimos=[
                DdsToDimos(dds_topic=topic, dds_type="std_msgs/msg/Int32", zenoh_topic="same")
                for topic in ("rt/a", "rt/b")
            ]
        )


def test_every_rust_table_row_has_a_stream_type() -> None:
    table = (_CRATE / "src" / "translate.rs").read_text()
    rust_types = {line.split('"')[1] for line in table.splitlines() if 'ros_type: "' in line}
    assert rust_types == set(DIMOS_TYPE_OF_ROS_TYPE)


def test_each_bridge_gets_a_port_on_its_streams_channel(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    odom = DdsToDimos(dds_topic="rt/odom", dds_type="nav_msgs/msg/Odometry", zenoh_topic="odom")
    bridge = DdsBridge(
        executable="/bin/echo", source_dir=None, build_command=None, dds_to_dimos=[odom]
    )
    try:
        channel = make_transport("/odom", DIMOS_TYPE_OF_ROS_TYPE[odom.dds_type]).channel
        assert bridge._collect_topics() == {"odom": channel}
        assert channel.endswith("/nav_msgs.Odometry")
        launch = json.loads(bridge._stdin_blob(bridge._collect_topics()))
        assert launch["config"]["dds_to_dimos"][0]["qos"] == {
            "reliability": "best_effort",
            "durability": "volatile",
            "history_depth": 10,
        }
    finally:
        bridge.stop()


def _check_int(msg: Any) -> int:
    return int(msg.data)


def _check_bool(msg: Any) -> int:
    return int(msg.data)


def _check_twist(msg: Any) -> int:
    assert msg.angular.z == 0.5
    return int(msg.linear.x)


def _check_pose_stamped(msg: Any) -> int:
    assert msg.frame_id == "map"
    assert msg.ts == pytest.approx(msg.position.x + 0.5)
    return int(msg.position.x)


def _check_odometry(msg: Any) -> int:
    assert (msg.frame_id, msg.child_frame_id) == ("odom", "base_link")
    assert msg.pose.position.y == 2.0
    assert msg.twist.linear.x == 1.5
    assert msg.ts == pytest.approx(msg.x + 0.5)
    return int(msg.x)


def _check_point_cloud(msg: Any) -> int:
    assert msg.frame_id == "lidar"
    points = msg.points_f32()
    last = len(points) - 1
    assert points[-1].tolist() == [last, 2 * last, 3 * last]
    return last


def _check_imu(msg: Any) -> int:
    assert msg.frame_id == "imu"
    assert msg.linear_acceleration.z == pytest.approx(9.81)
    return int(msg.angular_velocity.z)


def _check_tf(msg: Any) -> int:
    odom_to_base, base_to_lidar = msg.transforms
    assert (odom_to_base.frame_id, odom_to_base.child_frame_id) == ("odom", "base_link")
    assert (base_to_lidar.frame_id, base_to_lidar.child_frame_id) == ("base_link", "lidar")
    assert base_to_lidar.translation.x == pytest.approx(0.2)
    return int(odom_to_base.translation.x)


# What sample i of dds_bridge_publish carries, read back as i from the DimOS message.
_SAMPLE_INDEX: dict[str, Callable[[Any], int]] = {
    "std_msgs/msg/Int32": _check_int,
    "std_msgs/msg/UInt32": _check_int,
    "std_msgs/msg/Bool": _check_bool,
    "geometry_msgs/msg/Twist": _check_twist,
    "geometry_msgs/msg/PoseStamped": _check_pose_stamped,
    "nav_msgs/msg/Odometry": _check_odometry,
    "sensor_msgs/msg/PointCloud2": _check_point_cloud,
    "sensor_msgs/msg/Imu": _check_imu,
    "tf2_msgs/msg/TFMessage": _check_tf,
}


@pytest.mark.native_e2e
def test_dds_samples_arrive_as_dimos_messages(monkeypatch: pytest.MonkeyPatch) -> None:
    publisher = _CRATE / "result" / "bin" / "dds_bridge_publish"
    if not (_CRATE / "result" / "bin" / "dds_bridge").exists() or not publisher.exists():
        pytest.fail(f"bridge binaries missing; run: (cd {_CRATE} && nix build -L path:.)")
    monkeypatch.setattr(global_config, "transport", "zenoh")

    bridges = [
        DdsToDimos(
            dds_topic=f"rt/dds_bridge_test/{ros_type.rsplit('/', 1)[1].lower()}",
            dds_type=ros_type,
            zenoh_topic=f"dds_bridge_test_{ros_type.rsplit('/', 1)[1].lower()}",
            # The publisher's samples are reliable; a reliable reader gets every one.
            qos=DdsQos(reliability="reliable"),
        )
        for ros_type in _SAMPLE_INDEX
    ]
    received: dict[str, list[Any]] = {b.zenoh_topic: [] for b in bridges}
    transports = [make_transport(f"/{b.zenoh_topic}", b.dimos_type) for b in bridges]
    for bridge_entry, transport in zip(bridges, transports, strict=True):
        transport.subscribe(received[bridge_entry.zenoh_topic].append)

    # The binaries the check above found, not a rebuild inside the test.
    bridge = DdsBridge(
        executable=str(_CRATE / "result" / "bin" / "dds_bridge"),
        source_dir=None,
        build_command=None,
        domain_id=TEST_DOMAIN_ID,
        dds_to_dimos=bridges,
    )
    publishers: list[subprocess.Popen[bytes]] = []
    try:
        bridge.start()
        for b in bridges:
            command = [str(publisher), str(TEST_DOMAIN_ID), b.dds_topic, b.dds_type]
            publishers.append(subprocess.Popen([*command, str(SAMPLES_PER_TOPIC)]))
        try:
            wait_until(
                lambda: all(len(msgs) >= SAMPLES_PER_TOPIC for msgs in received.values()),
                timeout=DELIVERY_TIMEOUT_S,
            )
        except TimeoutError:
            counts = {stream: len(msgs) for stream, msgs in received.items()}
            pytest.fail(f"received per stream after {DELIVERY_TIMEOUT_S}s: {counts}")
        for b in bridges:
            indices = sorted(_SAMPLE_INDEX[b.dds_type](m) for m in received[b.zenoh_topic])
            if b.dds_type == "std_msgs/msg/Bool":
                assert indices == [0, 0, 1], b.dds_type
            else:
                assert indices == list(range(SAMPLES_PER_TOPIC)), b.dds_type
    finally:
        for process in publishers:
            if process.poll() is None:
                process.kill()
            process.wait()
        bridge.stop()
        for transport in transports:
            transport.stop()
