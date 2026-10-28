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

"""Convert generated messages at the ROS 2 serialization boundary."""

from __future__ import annotations

import importlib
from typing import Any, cast

from dimos_cdrgen.registry import encode

from dimos.protocol.pubsub.codec import CDRCodec
from dimos.protocol.pubsub.message_types import resolve_message_type


def get_dimos_type(msg_name: str) -> type | None:
    return resolve_message_type(msg_name)


def derive_ros_type(message_type: type) -> type:
    name = getattr(message_type, "__msgtype__", "")
    if resolve_message_type(name) is not message_type:
        raise TypeError("Expected a registry-owned generated message type")
    package, _, message = name.split("/")
    return cast("type", getattr(importlib.import_module(f"{package}.msg"), message))


def _serialization() -> Any:
    try:
        return importlib.import_module("rclpy.serialization")
    except ImportError as error:
        raise ImportError(
            "ROS conversion requires rclpy and installed ROS message packages"
        ) from error


def dimos_to_ros(message: Any, ros_type: type) -> Any:
    if derive_ros_type(type(message)) is not ros_type:
        raise TypeError("Target ROS class does not match the generated schema")
    return _serialization().deserialize_message(encode(message), ros_type)


def ros_to_dimos(message: Any, message_type: type) -> Any:
    if type(message) is not derive_ros_type(message_type):
        raise TypeError("Source ROS class does not match the generated schema")
    return CDRCodec(message_type).decode(_serialization().serialize_message(message))
