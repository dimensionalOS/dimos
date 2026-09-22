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

"""Convert generated DimOS and ROS2 messages through their shared CDR format."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any, TypeVar, cast

try:
    import rclpy.serialization as ros_serialization
except ImportError:
    ros_serialization = None  # type: ignore[assignment]

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg
    from dimos.protocol.pubsub.impl.rospubsub import ROSMessage


MessageT = TypeVar("MessageT", bound="DimosMsg")


def derive_ros_type(dimos_type: type[DimosMsg]) -> type[ROSMessage]:
    """Resolve the matching installed ROS type by canonical package/msg/Type name."""
    parts = dimos_type.msg_name.split("/")
    if len(parts) != 3 or parts[1] != "msg" or not all(part.isidentifier() for part in parts):
        raise ValueError(f"Invalid message name {dimos_type.msg_name!r}; expected package/msg/Type")
    package, _, name = parts
    return cast("type[ROSMessage]", getattr(importlib.import_module(f"{package}.msg"), name))


def dimos_to_ros(msg: DimosMsg, ros_type: type[ROSMessage]) -> ROSMessage:
    """Deserialize generated CDR with ROS's own type support, preserving every field."""
    if ros_serialization is None:
        raise ImportError("ROS message conversion requires rclpy; install and source ROS 2.")
    if derive_ros_type(type(msg)) is not ros_type:
        raise TypeError(f"ROS target does not match {msg.msg_name}")
    return cast("ROSMessage", ros_serialization.deserialize_message(msg.encode(), ros_type))


def ros_to_dimos(msg: Any, dimos_type: type[MessageT]) -> MessageT:
    """Decode ROS CDR with the generated codec; no LCM or field-copy adapter is needed."""
    if ros_serialization is None:
        raise ImportError("ROS message conversion requires rclpy; install and source ROS 2.")
    if type(msg) is not derive_ros_type(dimos_type):
        raise TypeError(f"ROS source does not match {dimos_type.msg_name}")
    return dimos_type.decode(ros_serialization.serialize_message(msg))
