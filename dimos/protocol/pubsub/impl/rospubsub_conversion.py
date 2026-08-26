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

"""Conversion between dimos messages and the `rosless.Message` rosless exchanges.

rosless hands a decoded ROS message over as a `rosless.Message`, whose fields are
attributes, and takes one back to encode. That is already a field-for-field match
for the LCM message dimos types are built from, so the conversion is a copy driven
by the ROS schema rather than a re-serialization.

Types whose dimos representation differs from their wire layout (point clouds,
images) still go through an LCM roundtrip to reach that representation.
"""

from __future__ import annotations

import array
from dataclasses import dataclass
import importlib
from typing import TYPE_CHECKING, Any

import rosless

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg


# Types whose dimos representation (numpy arrays, custom accessors) is not a
# plain field bag, so they have to be rebuilt through their LCM encoding.
COMPLEX_TYPES: set[str] = {
    "sensor_msgs.PointCloud2",
    "sensor_msgs.Image",
    "sensor_msgs.CompressedImage",
    "sensor_msgs.CameraInfo",
    "geometry_msgs.PoseStamped",
}

# dimos_lcm carries ROS 1 field spellings for a few types. Keyed by type because
# `std_msgs/msg/ColorRGBA.r` must not pick up `CameraInfo`'s `r` -> `R`.
_ROS_TO_LCM_FIELD_MAP: dict[str, dict[str, str]] = {
    "builtin_interfaces/msg/Time": {"nanosec": "nsec"},
    "builtin_interfaces/msg/Duration": {"nanosec": "nsec"},
    "sensor_msgs/msg/CameraInfo": {"d": "D", "k": "K", "r": "R", "p": "P"},
}

_lcm_type_cache: dict[str, type[Any]] = {}


@dataclass(frozen=True, slots=True)
class _FieldPlan:
    """How one ROS field maps onto its LCM counterpart."""

    ros_name: str
    lcm_name: str
    nested_type: str | None
    is_array: bool


_field_plans: dict[str, tuple[_FieldPlan, ...]] = {}


def derive_ros_type_name(dimos_type: type[DimosMsg]) -> str:
    """`sensor_msgs.PointCloud2` -> `sensor_msgs/msg/PointCloud2`."""
    package, _, message_name = dimos_type.msg_name.partition(".")
    if not message_name:
        raise ValueError(
            f"Invalid msg_name format: {dimos_type.msg_name}, expected 'package.MessageName'"
        )
    return f"{package}/msg/{message_name}"


def derive_lcm_type(dimos_type: type[DimosMsg]) -> type[Any]:
    """Find the `dimos_lcm` class a dimos message type encodes to."""
    msg_name = dimos_type.msg_name
    cached = _lcm_type_cache.get(msg_name)
    if cached is not None:
        return cached

    package, _, message_name = msg_name.partition(".")
    if not message_name:
        raise ValueError(f"Invalid msg_name format: {msg_name}, expected 'package.MessageName'")
    module = importlib.import_module(f"dimos_lcm.{package}.{message_name}")
    lcm_type: type[Any] = getattr(module, message_name)
    _lcm_type_cache[msg_name] = lcm_type
    return lcm_type


def _lcm_type_for_ros_type(ros_type_name: str) -> type[Any]:
    """`sensor_msgs/msg/PointField` -> `dimos_lcm.sensor_msgs.PointField`."""
    cached = _lcm_type_cache.get(ros_type_name)
    if cached is not None:
        return cached

    package, _, message_name = ros_type_name.split("/")
    module = importlib.import_module(f"dimos_lcm.{package}.{message_name}")
    lcm_type: type[Any] = getattr(module, message_name)
    _lcm_type_cache[ros_type_name] = lcm_type
    return lcm_type


def _plan_for(ros_type_name: str) -> tuple[_FieldPlan, ...]:
    cached = _field_plans.get(ros_type_name)
    if cached is not None:
        return cached

    schema = rosless.lookup(ros_type_name)
    if schema is None:
        raise ValueError(f"{ros_type_name} is not in the rosless type catalogue")
    renames = _ROS_TO_LCM_FIELD_MAP.get(ros_type_name, {})
    plan = tuple(
        _FieldPlan(
            ros_name=field.name,
            lcm_name=renames.get(field.name, field.name),
            nested_type=field.type_name if rosless.lookup(field.type_name) else None,
            is_array=field.multiplicity != "unit",
        )
        for field in schema.fields
    )
    _field_plans[ros_type_name] = plan
    return plan


def _read_fields(source: Any, ros_type_name: str) -> rosless.Message:
    fields: dict[str, Any] = {}
    for field in _plan_for(ros_type_name):
        if not hasattr(source, field.lcm_name):
            continue
        value = getattr(source, field.lcm_name)
        if field.nested_type is None:
            fields[field.ros_name] = value
        elif field.is_array:
            fields[field.ros_name] = [_read_fields(item, field.nested_type) for item in value]
        else:
            fields[field.ros_name] = _read_fields(value, field.nested_type)
    return rosless.Message(ros_type_name, **fields)


def _write_fields(target: Any, source: Any, ros_type_name: str) -> None:
    for field in _plan_for(ros_type_name):
        value = getattr(source, field.ros_name, None)
        if value is None or not hasattr(target, field.lcm_name):
            continue

        if field.nested_type is None:
            setattr(target, field.lcm_name, value)
        elif field.is_array:
            element_type = _lcm_type_for_ros_type(field.nested_type)
            elements = []
            for item in value:
                element = element_type()
                _write_fields(element, item, field.nested_type)
                elements.append(element)
            setattr(target, field.lcm_name, elements)
        else:
            _write_fields(getattr(target, field.lcm_name), value, field.nested_type)

        length_field = f"{field.lcm_name}_length"
        stored = getattr(target, field.lcm_name)
        if hasattr(target, length_field) and isinstance(
            stored, (list, tuple, bytes, bytearray, array.array)
        ):
            setattr(target, length_field, len(stored))


def dimos_to_ros(msg: DimosMsg, ros_type_name: str) -> rosless.Message:
    """Flatten a dimos message into the `rosless.Message` rosless encodes from."""
    if type(msg).msg_name in COMPLEX_TYPES:
        source: Any = derive_lcm_type(type(msg)).lcm_decode(msg.lcm_encode())
    else:
        source = msg
    return _read_fields(source, ros_type_name)


def ros_to_dimos(ros_msg: rosless.Message, dimos_type: type[DimosMsg]) -> DimosMsg:
    """Build a dimos message from the `rosless.Message` rosless decoded."""
    ros_type_name = derive_ros_type_name(dimos_type)

    if dimos_type.msg_name in COMPLEX_TYPES:
        lcm_msg = derive_lcm_type(dimos_type)()
        _write_fields(lcm_msg, ros_msg, ros_type_name)
        return dimos_type.lcm_decode(lcm_msg.lcm_encode())

    dimos_msg = dimos_type()
    _write_fields(dimos_msg, ros_msg, ros_type_name)
    return dimos_msg
