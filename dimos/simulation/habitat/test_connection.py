# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos_lcm.geometry_msgs.Twist import Twist as WireTwist
import numpy as np

from dimos.core.native_module import NativeModule
from dimos.core.transport import LCMTransport
from dimos.message_utils import image
from dimos.messages.geometry_msgs.msg import Twist, Vector3
from dimos.messages.sensor_msgs.msg import Image
from dimos.simulation.habitat import connection, server
from dimos.simulation.habitat.connection import HabitatConnection


def test_foreign_wire_bridge_preserves_pixels_and_command_values(mocker):
    wire = mocker.Mock(spec=connection.ZenohPubSubBase)
    mocker.patch.object(connection, "ZenohPubSubBase", return_value=wire)
    native_start = mocker.patch.object(NativeModule, "start")
    mocker.patch.object(
        NativeModule,
        "_collect_topics",
        return_value={"color_image": "public/image", "cmd_vel": "public/cmd"},
    )
    module = HabitatConnection()
    publish = mocker.patch.object(module.color_image, "publish")
    commands = LCMTransport("habitat_bridge_commands", Twist, url="memq://")
    module.set_transport("cmd_vel", commands)
    try:
        module.start()
        native_start.assert_called_once()
        topics = module._collect_topics()
        assert topics["color_image"] != "public/image"
        assert topics["cmd_vel"] != "public/cmd"
        topic, receive = wire.subscribe.call_args.args
        pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        receive(server.image_msg(pixels, "rgb8", "camera_optical", 1.5), topic)
        value = publish.call_args.args[0]
        assert type(value) is Image
        np.testing.assert_array_equal(image.view(value), pixels)
        assert value.header.frame_id == "camera_optical"
        assert value.header.stamp.sec == 1
        assert value.header.stamp.nanosec == 500_000_000
        module._send_command(Twist(Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0)))
        sent_topic, payload = wire.publish.call_args.args
        assert sent_topic.topic == topics["cmd_vel"]
        command = WireTwist.lcm_decode(payload)
        assert (command.linear.x, command.linear.y, command.linear.z) == (1.0, 2.0, 3.0)
        assert (command.angular.x, command.angular.y, command.angular.z) == (4.0, 5.0, 6.0)
    finally:
        module.stop()
        commands.stop()
    wire.stop.assert_called_once()
