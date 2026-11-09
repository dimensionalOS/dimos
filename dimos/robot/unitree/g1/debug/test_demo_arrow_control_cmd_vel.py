# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from threading import Condition

from dimos.core.transport import LCMTransport
from dimos.messages.geometry_msgs.msg import Twist, Vector3
from dimos.robot.unitree.g1.debug.demo_arrow_control_cmd_vel import publish_twist


def test_demo_commands_use_the_registered_generated_transport_contract():
    condition = Condition()
    received = []

    def receive(value):
        with condition:
            received.append(value)
            condition.notify_all()

    transport = LCMTransport("demo_commands", Twist, url="memq://")
    unsubscribe = transport.subscribe(receive)
    try:
        publish_twist(transport, Twist(Vector3(0.3, -0.2, 0.0), Vector3(0.0, 0.0, 0.5)))
        with condition:
            assert condition.wait_for(lambda: bool(received), timeout=2)
        value = received[0]
        assert type(value) is Twist
        assert (value.linear.x, value.linear.y, value.linear.z) == (0.3, -0.2, 0.0)
        assert (value.angular.x, value.angular.y, value.angular.z) == (0.0, 0.0, 0.5)
        assert transport.topic.message.encoding == "cdr"
    finally:
        unsubscribe()
        transport.stop()
