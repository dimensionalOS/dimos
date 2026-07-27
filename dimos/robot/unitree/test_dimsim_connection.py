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

from unittest.mock import MagicMock

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.dimsim_connection import DimSimConnection


def test_dimsim_camera_info_matches_browser_sensor_frames():
    camera_info = DimSimConnection.camera_info_static

    assert camera_info.width == 640
    assert camera_info.height == 288


@pytest.fixture
def dimsim_connection(mocker):
    process = mocker.patch("dimos.robot.unitree.dimsim_connection.DimSimProcess").return_value
    transports = [MagicMock(), MagicMock(), MagicMock()]
    mocker.patch(
        "dimos.robot.unitree.dimsim_connection.make_transport",
        side_effect=transports,
    )
    tf = MagicMock()
    mocker.patch("dimos.robot.unitree.dimsim_connection.tf_backend").return_value.return_value = tf

    callbacks = []
    unsubscribes = []
    for transport in transports:
        unsubscribe = MagicMock()
        unsubscribes.append(unsubscribe)

        def subscribe(callback, *, _unsubscribe=unsubscribe):
            callbacks.append(callback)
            return _unsubscribe

        transport.subscribe.side_effect = subscribe

    connection = DimSimConnection(GlobalConfig(simulation="dimsim"))
    connection.start()
    try:
        yield connection, process, transports, tf, callbacks, unsubscribes
    finally:
        connection.stop()


def test_dimsim_connection_relays_sensor_topics(dimsim_connection):
    connection, _, _, tf, callbacks, _ = dimsim_connection
    received_odom = []
    received_lidar = []
    received_video = []
    connection.odom_stream().subscribe(received_odom.append)
    connection.lidar_stream().subscribe(received_lidar.append)
    connection.video_stream().subscribe(received_video.append)

    odom = PoseStamped(ts=1.0)
    lidar = PointCloud2(ts=1.0)
    image = Image(ts=1.0)
    callbacks[0](odom)
    callbacks[1](lidar)
    callbacks[2](image)

    assert received_odom == [odom]
    assert received_lidar == [lidar]
    assert received_video == [image]
    assert tf.publish.call_count == 1


def test_dimsim_connection_drops_republished_sensor_packets(dimsim_connection):
    connection, _, _, _, callbacks, _ = dimsim_connection
    received_odom = []
    received_lidar = []
    received_video = []
    connection.odom_stream().subscribe(received_odom.append)
    connection.lidar_stream().subscribe(received_lidar.append)
    connection.video_stream().subscribe(received_video.append)

    odom = PoseStamped(ts=1.0)
    lidar = PointCloud2(ts=1.0)
    image = Image(ts=1.0)
    for callback, message in zip(callbacks, (odom, lidar, image), strict=True):
        callback(message)
        callback(message)

    assert received_odom == [odom]
    assert received_lidar == [lidar]
    assert received_video == [image]


def test_dimsim_connection_stops_all_resources(dimsim_connection):
    connection, process, transports, tf, _, unsubscribes = dimsim_connection

    connection.stop()

    assert [unsubscribe.call_count for unsubscribe in unsubscribes] == [1, 1, 1]
    assert [transport.stop.call_count for transport in transports] == [1, 1, 1]
    tf.stop.assert_called_once_with()
    process.stop.assert_called_once_with()
