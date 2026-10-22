# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from contextlib import ExitStack
from functools import partial
import queue
import sys
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np
import pytest

from dimos.core import transport as transport_module
from dimos.core.global_config import GlobalConfig
from dimos.core.transport_factory import make_transport
from dimos.hardware.drive_trains.transport.adapter import TransportTwistAdapter
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.sensor_msgs.msg import CompressedImage
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.geometry_msgs.Transform import Transform as MathTransform
from dimos.msgs.geometry_msgs.Vector3 import Vector3 as MathVector
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.robot.galaxea.r1pro import lio
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.utils.testing.waiting import wait_until


@pytest.fixture(params=["lcm", "zenoh"])
def wires(request, monkeypatch):
    monkeypatch.setattr(transport_module, "LCM", partial(LCM, url="memq://"))
    monkeypatch.setattr(
        transport_module,
        "Zenoh",
        partial(
            Zenoh,
            scouting=False,
            multicast=False,
            gossip=False,
            connect=[],
            listen=["tcp/127.0.0.1:0"],
        ),
    )
    build = partial(make_transport, g=GlobalConfig(transport=request.param))
    owned = {}

    def shared(topic, kind):
        # memq is endpoint-local; share each logical wire as the coordinator does.
        key = topic, kind
        if key not in owned:
            owned[key] = build(topic, kind)
        return owned[key]

    try:
        yield shared
    finally:
        for wire in owned.values():
            wire.stop()


def _speed(sec, nanosec=1):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        twist=SimpleNamespace(linear=SimpleNamespace(x=1.0, y=0.5), angular=SimpleNamespace(z=0.2)),
    )


def test_wheel_feedback_preserves_exact_source_stamp_and_tf_ownership(wires):
    with ExitStack() as cleanup:
        connection = R1ProConnection()
        cleanup.callback(connection.stop)
        received = {}
        for port, kind in [("odometry", Odometry), ("odom", PoseStamped), ("tf", TFMessage)]:
            wire = wires("/chassis/" + port, kind)
            connection.set_transport(port, wire)
            received[port] = []
            cleanup.callback(wire.subscribe(received[port].append))
        adapter = TransportTwistAdapter(hardware_id="chassis", transport_cls=wires)
        cleanup.callback(adapter.disconnect)
        assert adapter.connect()
        connection._on_chassis_speed(_speed(1_700_000_000), None)
        source = _speed(1_700_000_000, 500_000_001)
        connection._on_chassis_speed(source, None)
        wait_until(
            lambda: all(received.values()) and adapter.read_odometry() is not None, timeout=5
        )
        odom, pose, tf = (received[name][0] for name in ("odometry", "odom", "tf"))
        assert (odom.header.stamp.sec, odom.header.stamp.nanosec) == (1_700_000_000, 500_000_001)
        assert pose.pose.position.x == pytest.approx(0.5)
        assert adapter.read_odometry() == pytest.approx([0.5, 0.25, 0.1])
        assert odom.child_frame_id == tf.transforms[0].child_frame_id == connection.config.frame_id
        assert tf.transforms[0].transform.translation.x == pose.pose.position.x
        np.testing.assert_array_equal(odom.pose.covariance, np.zeros(36))
        assert odom.twist.twist.linear.y == 0.5
        source.header.stamp.nanosec = 99
        assert odom.header.stamp.nanosec == 500_000_001
        connection._on_chassis_speed(_speed(1_700_000_000, 500_000_001), None)
        connection._on_chassis_speed(_speed(1_700_000_010), None)
        assert connection._odom_x == pytest.approx(0.5)


def test_pointlio_mode_keeps_wheel_feedback_without_second_tf_parent(mocker):
    with ExitStack() as cleanup:
        connection = R1ProConnection(publish_odom_tf=False)
        cleanup.callback(connection.stop)
        odometry = mocker.patch.object(connection.odometry, "publish")
        pose = mocker.patch.object(connection.odom, "publish")
        tf = mocker.patch.object(connection.tf, "publish")
        connection._on_chassis_speed(_speed(17), None)
        connection._on_chassis_speed(_speed(17, 500_000_001), None)
        assert odometry.call_args.args[0].pose.pose.position.x == pytest.approx(0.5)
        pose.assert_not_called()
        tf.assert_not_called()


def test_chassis_command_roundtrip_uses_existing_ros_scalar_contract(wires, mocker, monkeypatch):
    ros_types = ModuleType("geometry_msgs.msg")
    ros_types.TwistStamped = lambda: SimpleNamespace(
        header=SimpleNamespace(stamp=None),
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0), angular=SimpleNamespace(z=0.0)),
    )
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", ros_types)
    mocker.patch("dimos.robot.galaxea.r1pro.connection._ros_stamp_now", return_value="ros-clock")
    with ExitStack() as cleanup:
        connection = R1ProConnection()
        cleanup.callback(connection.stop)
        ros = mocker.Mock()
        mocker.patch.multiple(connection, _ros=ros, _speed_topic="chassis-speed")
        wire = wires("/chassis/cmd_vel", Twist)
        connection.set_transport("cmd_vel", wire)
        cleanup.callback(wire.subscribe(connection._on_cmd_vel))
        adapter = TransportTwistAdapter(hardware_id="chassis", transport_cls=wires)
        cleanup.callback(adapter.disconnect)
        assert adapter.connect()
        assert not adapter.write_velocities([1.0, 2.0, 3.0])
        assert adapter.write_enable(True)
        assert adapter.write_velocities([1.0, 2.0, 3.0])
        wait_until(lambda: ros.publish.call_count == 1, timeout=5)
        topic, value = ros.publish.call_args.args
        assert (topic, value.header.stamp) == ("chassis-speed", "ros-clock")
        assert (value.twist.linear.x, value.twist.linear.y, value.twist.angular.z) == (
            1.0,
            2.0,
            3.0,
        )
        assert adapter.write_enable(False)
        wait_until(lambda: ros.publish.call_count == 2, timeout=5)
        stop = ros.publish.call_args.args[1].twist
        assert (stop.linear.x, stop.linear.y, stop.angular.z) == (0.0, 0.0, 0.0)
        assert not adapter.write_velocities([1.0, 2.0, 3.0])


@pytest.mark.parametrize("frame", ["", "wrist-optical"])
def test_jpeg_worker_preserves_encoded_bytes_source_ns_and_frame_fallback(wires, frame):
    success, jpeg = cv2.imencode(".jpg", np.full((2, 3, 3), 71, dtype=np.uint8))
    assert success
    with ExitStack() as cleanup:
        connection = R1ProConnection(color_publish_hz=0)
        cleanup.callback(connection.stop)
        wire = wires("/wrist_left_color", CompressedImage)
        connection.set_transport("wrist_left_color", wire)
        values = []
        cleanup.callback(wire.subscribe(values.append))
        source = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=1_700_000_000, nanosec=1), frame_id=frame
            ),
            data=jpeg.tobytes(),
        )
        pending = queue.Queue()
        connection._make_rx_cb("wrist_left_color", pending)(source, None)
        pending.put(None)
        connection._compressed_image_loop("wrist_left_color", pending)
        wait_until(lambda: bool(values), timeout=5)
        value = values[0]
        assert value.data.tobytes() == jpeg.tobytes()
        assert value.format == "jpeg"
        assert value.header.frame_id == (frame or "wrist_left_color")
        assert (value.header.stamp.sec, value.header.stamp.nanosec) == (1_700_000_000, 1)
        stats = connection.sensor_stats()["wrist_left_color"]
        assert (stats["received"], stats["decoded"], stats["errors"], stats["bytes_in"]) == (
            1,
            1,
            0,
            len(jpeg),
        )


def test_lio_mount_math_preserves_exact_header_and_source_ownership(wires, mocker):
    with ExitStack() as cleanup:
        module = lio.R1ProLio()
        cleanup.callback(module.stop)
        mocker.patch.object(
            module,
            "_lidar_to_base",
            MathTransform(
                translation=MathVector(-0.5, 0.25, 0.0),
                frame_id="lidar",
                child_frame_id="base_link",
            ),
            create=True,
        )
        wire = wires("/chassis/pose", PoseStamped)
        module.set_transport("pose", wire)
        poses = []
        cleanup.callback(wire.subscribe(poses.append))
        source = Odometry(
            Header(Time(1_700_000_000, 1), "world"),
            "lidar",
            PoseWithCovariance(
                Pose(Point(2.0, 3.0, 4.0), Quaternion(0.0, 0.0, 0.0, 1.0)), np.zeros(36)
            ),
            TwistWithCovariance(
                Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), np.zeros(36)
            ),
        )
        module._on_odometry(source)
        wait_until(lambda: bool(poses), timeout=5)
        value = poses[0]
        assert (value.pose.position.x, value.pose.position.y, value.pose.position.z) == (
            1.5,
            3.25,
            4.0,
        )
        assert (value.header.stamp.sec, value.header.stamp.nanosec, value.header.frame_id) == (
            1_700_000_000,
            1,
            "world",
        )
        source.header.stamp.nanosec = 2
        assert value.header.stamp.nanosec == 1


def test_lio_static_mounts_are_canonical_without_loading_external_urdf(mocker):
    model = SimpleNamespace(
        joints=[
            SimpleNamespace(
                child_link="lidar_chassis_left_link",
                parent_link="base_link",
                origin_xyz=[0.5, 0.25, 0.75],
                origin_rpy=[0.0, 0.0, 0.0],
            ),
            SimpleNamespace(
                child_link="lidar_pointlio_link",
                parent_link="lidar_chassis_left_link",
                origin_xyz=[0.0, 0.0, 0.0],
                origin_rpy=[0.0, 0.0, 0.0],
            ),
        ]
    )
    mocker.patch.object(lio, "lio_model", return_value=model)
    with ExitStack() as cleanup:
        module = lio.R1ProLio()
        cleanup.callback(module.stop)
        moving_mount, chassis_mount = module.transforms()
        assert (moving_mount.header.frame_id, moving_mount.child_frame_id) == (
            "lidar_pointlio_link",
            "base_link",
        )
        t = moving_mount.transform.translation
        assert (t.x, t.y, t.z) == (-0.5, -0.25, -0.75)
        assert (chassis_mount.header.frame_id, chassis_mount.child_frame_id) == (
            "base_link",
            "lidar_chassis_left_link",
        )
        t = chassis_mount.transform.translation
        assert (t.x, t.y, t.z) == (0.5, 0.25, 0.75)
