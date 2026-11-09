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

"""Real G1 message ports with simulated inputs and mocked external SDKs."""

import asyncio
from contextlib import ExitStack
from copy import deepcopy
from functools import partial
from threading import Condition, Event

import numpy as np
import pytest
from reactivex import Subject

from dimos.core.module import Module
from dimos.core.transport import LCMTransport
from dimos.message_utils import image, pointcloud
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.g1_tf_publisher import G1TfPublisher, mount_transforms
from dimos.robot.unitree.g1.mujoco_sim import G1SimConnection
from dimos.robot.unitree.mujoco_connection import MujocoConnection


def test_simulated_sensor_and_drive_ports_use_cdr_without_launching_simulator(mocker):
    driver = mocker.Mock()
    driver.camera_info_static = MujocoConnection.camera_info_static
    factory = mocker.patch(
        "dimos.robot.unitree.g1.mujoco_sim.MujocoConnection", return_value=driver
    )
    odom, clouds, images = Subject(), Subject(), Subject()
    driver.odom_stream.return_value = odom
    driver.lidar_stream.return_value = clouds
    driver.video_stream.return_value = images
    received = {}
    condition = Condition()
    moved = Event()
    driver.move.side_effect = lambda value, duration: moved.set()

    def receive(name, value):
        with condition:
            received[name] = value
            condition.notify_all()

    with ExitStack() as cleanup:
        module = G1SimConnection()
        cleanup.callback(module.stop)
        for name, port in {**module.inputs, **module.outputs}.items():
            wire = LCMTransport("g1_sim_" + name, port.type, url="memq://")
            cleanup.callback(wire.stop)
            module.set_transport(name, wire)
            if name in module.outputs:
                cleanup.callback(wire.subscribe(partial(receive, name)))
        module.start()
        pose = PoseStamped(
            Header(Time(0, 0), "world"), Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 1))
        )
        odom.on_next(pose)
        points = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
        colors = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float64)
        source_cloud = pointcloud.from_xyz_rgb(
            points, (colors * 255).astype(np.uint8), header=Header(Time(0, 0), "")
        )
        clouds.on_next(source_cloud)
        pixels = np.full((2, 3, 3), [255, 0, 0], dtype=np.uint8)
        images.on_next(
            image.from_array(pixels, encoding="rgb8", header=Header(Time(0, 0), "camera_optical"))
        )
        with condition:
            assert condition.wait_for(lambda: len(received) == 5, timeout=5)
        assert received["odom"].header.stamp == Time(0, 0)
        assert received["odom"].pose.position.x == 1
        assert [(v.header.frame_id, v.child_frame_id) for v in received["tf"].transforms] == [
            ("world", "base_link"),
            ("base_link", "camera_link"),
            ("camera_link", "camera_optical"),
            ("map", "world"),
        ]
        assert received["tf"].transforms[0].header.stamp == Time(0, 0)
        np.testing.assert_array_equal(pointcloud.xyz(received["lidar"]), points)
        np.testing.assert_array_equal(pointcloud.rgb(received["lidar"]), [[255, 0, 0], [0, 255, 0]])
        np.testing.assert_array_equal(image.view(received["color_image"]), pixels)
        assert received["color_image"].header.stamp == Time(0, 0)
        assert received["camera_info"].width == driver.camera_info_static.width
        command = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0.5))
        module.cmd_vel.transport.publish(command)
        assert moved.wait(5)
        value, duration = driver.move.call_args.args
        assert value.linear.x == 0.3 and value.angular.z == 0.5 and duration == 0
        assert command.linear.x == 0.3
        factory.assert_called_once()
    driver.stop.assert_called_once()


def test_hardware_module_command_decodes_cdr_before_mocked_sdk(mocker):
    driver = mocker.Mock()
    factory = mocker.patch(
        "dimos.robot.unitree.g1.connection.UnitreeWebRTCConnection", return_value=driver
    )
    ready = Event()
    driver.move.side_effect = lambda value, duration: ready.set()
    with ExitStack() as cleanup:
        module = G1Connection(connection_type="webrtc")
        cleanup.callback(Module.stop, module)
        # stop() requires a constructed driver, so register after successful start.
        wire = LCMTransport("g1_hardware_cmd", Twist, url="memq://")
        cleanup.callback(wire.stop)
        module.set_transport("cmd_vel", wire)
        module.start()
        cleanup.callback(module.stop)
        source = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.4))
        wire.publish(source)
        assert ready.wait(5)
        value, duration = driver.move.call_args.args
        assert value.linear.x == 0.2 and value.angular.z == -0.4 and duration == 0
        factory.assert_called_once()
    driver.stop.assert_called_once()


@pytest.mark.asyncio
async def test_mount_publisher_emits_canonical_transforms_with_one_exact_epoch_stamp(mocker):
    mocker.patch("dimos.message_utils.time.time.time_ns", return_value=1_789_654_321_000_000_001)
    with ExitStack() as cleanup:
        publisher = G1TfPublisher()
        cleanup.callback(publisher.stop)
        wire = LCMTransport("g1_mounts", publisher.tf.type, url="memq://")
        cleanup.callback(wire.stop)
        publisher.set_transport("tf", wire)
        received = []

        def receive(value):
            received.append(value)
            publisher._stop_event.set()

        cleanup.callback(wire.subscribe(receive))
        await asyncio.wait_for(publisher._publish_loop(), timeout=2)
        assert len(received) == 1
        expected = mount_transforms()
        for value in expected:
            value.header.stamp = Time(1_789_654_321, 1)
        assert received[0].transforms == expected
        snapshot = deepcopy(received[0])
        received[0].transforms[0].header.frame_id = "modified"
        assert received[0].transforms[1].header == snapshot.transforms[1].header
