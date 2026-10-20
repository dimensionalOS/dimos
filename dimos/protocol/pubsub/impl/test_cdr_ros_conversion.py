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

"""Generated ROS bridge; mock only ROS runtime using an independent CDR oracle.

These tests do not claim a real ROS node installation or discovery acceptance.
"""

from contextlib import ExitStack
import pickle
import sys
from types import ModuleType, SimpleNamespace

from dimos_cdrgen.registry import encode, schema
import numpy as np
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.core.transport import ROSTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from dimos.messages.sensor_msgs.msg import Image, Imu
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.Vector3 import Vector3 as LegacyVector
from dimos.protocol.pubsub.impl import rospubsub, rospubsub_conversion as conversion


@pytest.fixture
def ros_boundary(mocker, monkeypatch):
    definitions = {}
    for value in (PoseStamped, Image, Imu):
        definitions.update(get_types_from_msg(schema(value.__msgtype__), value.__msgtype__))
    # A separate rosbags store replaces its global usertypes module on registration.
    # Restore that module immediately: generated classes must retain pickle identity.
    with monkeypatch.context() as isolated:
        isolated.setitem(sys.modules, "rosbags.usertypes", sys.modules["rosbags.usertypes"])
        oracle = get_typestore(Stores.EMPTY)
        oracle.register(definitions)
    for package in ("geometry_msgs", "sensor_msgs"):
        module = ModuleType(package + ".msg")
        for name, cls in oracle.types.items():
            if name.startswith(package + "/msg/"):
                setattr(module, name.rsplit("/", 1)[1], cls)
        monkeypatch.setitem(sys.modules, package + ".msg", module)
    serialization = ModuleType("rclpy.serialization")
    serialization.serialize_message = mocker.Mock(
        side_effect=lambda value: bytes(oracle.serialize_cdr(value, value.__msgtype__))
    )
    serialization.deserialize_message = mocker.Mock(
        side_effect=lambda data, cls: oracle.deserialize_cdr(data, cls.__msgtype__)
    )
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    mocker.patch.object(
        conversion,
        "derive_lcm_type",
        side_effect=AssertionError("generated values must never use LCM"),
    )
    return SimpleNamespace(oracle=oracle, serialization=serialization)


def _sample(kind):
    header = Header(Time(1_789_654_321, 1), "sensor")
    if kind is PoseStamped:
        return PoseStamped(header, Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 1)))
    if kind is Image:
        return Image(header, 2, 2, "rgb8", 0, 8, np.arange(16, dtype=np.uint8))
    return Imu(
        header,
        Quaternion(0, 0, 0, 1),
        np.arange(9, dtype=np.float64),
        Vector3(0.1, 0.2, 0.3),
        np.arange(9, dtype=np.float64) + 10,
        Vector3(1, 2, 3),
        np.arange(9, dtype=np.float64) + 20,
    )


@pytest.mark.parametrize("kind", [PoseStamped, Image, Imu])
@pytest.mark.parametrize("little_endian", [True, False])
def test_generated_values_reuse_ros_cdr_without_legacy_roundtrip(ros_boundary, kind, little_endian):
    source = _sample(kind)
    oracle = ros_boundary.oracle
    cls = oracle.types[kind.__msgtype__]
    ros = conversion.dimos_to_ros(source, cls)
    assert type(ros) is cls
    assert ros.header.stamp.nanosec == 1
    assert bytes(oracle.serialize_cdr(ros, kind.__msgtype__)) == encode(source)
    ros_boundary.serialization.serialize_message.side_effect = lambda value: bytes(
        oracle.serialize_cdr(value, value.__msgtype__, little_endian=little_endian)
    )
    restored = conversion.ros_to_dimos(ros, kind)
    assert type(restored) is kind
    assert type(restored.header) is Header
    assert encode(restored) == encode(source)
    restored.header.frame_id = "changed"
    assert source.header.frame_id == "sensor" and ros.header.frame_id == "sensor"


def test_wrong_ros_schema_is_rejected_before_serialization(ros_boundary):
    source = _sample(PoseStamped)
    with pytest.raises(TypeError, match="Target ROS class"):
        conversion.dimos_to_ros(source, ros_boundary.oracle.types[Image.__msgtype__])
    with pytest.raises(TypeError, match="Source ROS class"):
        conversion.ros_to_dimos(source, PoseStamped)
    ros_boundary.serialization.serialize_message.assert_not_called()
    ros_boundary.serialization.deserialize_message.assert_not_called()


def test_imposter_generated_class_is_rejected_before_importing_ros():
    class Imposter:
        __msgtype__ = "geometry_msgs/msg/PoseStamped"

    with pytest.raises(TypeError, match="registry-owned"):
        conversion.derive_ros_type(Imposter)


def test_ros_cdr_validation_rejects_malformed_payload_without_fallback(ros_boundary):
    ros = conversion.dimos_to_ros(
        _sample(PoseStamped), ros_boundary.oracle.types[PoseStamped.__msgtype__]
    )
    ros_boundary.serialization.serialize_message.return_value = b"malformed"
    ros_boundary.serialization.serialize_message.side_effect = None
    with pytest.raises(ValueError, match="XCDR1"):
        conversion.ros_to_dimos(ros, PoseStamped)


def test_transport_keeps_existing_ros_wrapper_lifecycle_and_generated_values(ros_boundary, mocker):
    raw = mocker.Mock()
    factory = mocker.patch.object(rospubsub, "RawROS", return_value=raw)
    with ExitStack() as cleanup:
        transport = ROSTransport("/camera", Image, node_name="cdr_offline_test")
        cleanup.callback(transport.stop)
        values = []
        cleanup.callback(transport.subscribe(values.append))
        source = _sample(Image)
        transport.publish(source)
        topic, ros = raw.publish.call_args.args
        assert topic.topic == "/camera" and topic.ros_type is type(ros)
        raw.subscribe.call_args.args[1](ros, topic)
        assert len(values) == 1 and type(values[0]) is Image
        assert encode(values[0]) == encode(source)
        raw.start.assert_called_once_with()
        factory.assert_called_once_with("cdr_offline_test", None)
    raw.stop.assert_called_once_with()
    raw.subscribe.return_value.assert_called_once_with()


def test_wrong_generated_transport_value_fails_before_ros_start(ros_boundary, mocker):
    factory = mocker.patch.object(rospubsub, "RawROS")
    transport = ROSTransport("/camera", Image)
    with pytest.raises(TypeError, match="declared type"):
        transport.publish(_sample(PoseStamped))
    factory.assert_not_called()


def test_pickled_ros_transport_preserves_declared_type_and_node_options(ros_boundary):
    transport = ROSTransport("/camera", Image, node_name="offline")
    restored = pickle.loads(pickle.dumps(transport))
    assert restored.topic.topic == "/camera" and restored.topic.msg_type is Image
    assert restored._kwargs == {"node_name": "offline"}


def test_explicit_legacy_flat_conversion_stays_separate(mocker):
    target = SimpleNamespace(
        get_fields_and_field_types=lambda: {"x": "double", "y": "double", "z": "double"},
        x=1.0,
        y=2.0,
        z=3.0,
    )
    serializer = mocker.patch.object(
        conversion, "_serialization", side_effect=AssertionError("legacy path must not select CDR")
    )
    value = conversion.ros_to_dimos(target, LegacyVector)
    assert (value.x, value.y, value.z) == (1, 2, 3)
    serializer.assert_not_called()


def test_ros_transport_rejects_imposter_before_constructing_runtime(mocker):
    factory = mocker.patch.object(rospubsub, "RawROS")

    class Imposter:
        __msgtype__ = Image.__msgtype__

    with pytest.raises(TypeError, match="registry-owned"):
        ROSTransport("/camera", Imposter)
    factory.assert_not_called()


def test_missing_ros_serialization_reports_runtime_dependency(ros_boundary, monkeypatch):
    monkeypatch.delitem(sys.modules, "rclpy.serialization")
    with pytest.raises(ImportError, match="rclpy and installed ROS message packages"):
        conversion.dimos_to_ros(_sample(Image), ros_boundary.oracle.types[Image.__msgtype__])
