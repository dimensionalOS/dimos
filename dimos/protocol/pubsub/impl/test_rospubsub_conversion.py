"""Exercise the CDR adapter without requiring an installed ROS distribution."""

import sys
from types import ModuleType

from dimos_cdrgen.registry import encode

from dimos.messages.geometry_msgs.msg import Vector3
from dimos.protocol.pubsub.impl.rospubsub_conversion import (
    derive_ros_type,
    dimos_to_ros,
    ros_to_dimos,
)


def test_generated_ros_adapter_uses_ros_serializer_and_preserves_message(monkeypatch):
    class RosVector:
        def __init__(self, payload):
            self.payload = payload

    serialization = ModuleType("rclpy.serialization")
    calls = []

    def deserialize(payload, kind):
        calls.append(("deserialize", kind))
        return kind(payload)

    def serialize(message):
        calls.append(("serialize", type(message)))
        return message.payload

    serialization.deserialize_message = deserialize
    serialization.serialize_message = serialize
    package = ModuleType("geometry_msgs.msg")
    package.Vector3 = RosVector
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", package)
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    original = Vector3(1.25, -2.5, 3.75)
    assert derive_ros_type(Vector3) is RosVector
    ros = dimos_to_ros(original, RosVector)
    assert ros.payload == encode(original)
    result = ros_to_dimos(ros, Vector3)
    assert type(result) is Vector3
    assert result == original
    assert calls == [("deserialize", RosVector), ("serialize", RosVector)]
