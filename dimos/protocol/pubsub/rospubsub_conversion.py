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

"""Conversion functions between dimos messages and ROS messages.

This module provides conversion functions between dimos message types and ROS messages.
It handles three categories of types:

1. Complex types (different internal representation) - use LCM roundtrip
2. Simple types (field structures match) - use direct field copy
3. No dimos.msgs equivalent - return dimos_lcm type
"""

from __future__ import annotations

import importlib
import re
from typing import TYPE_CHECKING, Any, cast

if TYPE_CHECKING:
    from dimos.msgs import DimosMsg
    from dimos.protocol.pubsub.rospubsub import ROSMessage


# Complex types that need LCM roundtrip (explicit list)
# These types have different internal representations in dimos vs ROS/LCM
COMPLEX_TYPES: set[str] = {
    "sensor_msgs.PointCloud2",
    "sensor_msgs.Image",
    "sensor_msgs.CameraInfo",
    "geometry_msgs.PoseStamped",
}

# Cache for dynamic imports of dimos types
_dimos_type_cache: dict[str, type[DimosMsg] | None] = {}

# Cache for LCM type derivation
_lcm_type_cache: dict[str, type[Any]] = {}

# Field name mappings between ROS and LCM (ROS name -> LCM name)
# This is some mixup in dimos_lcm having ROS1 and ROS2 message definitions?
# Would be good to clarify later, but this works for now
_ROS_TO_LCM_FIELD_MAP: dict[str, str] = {
    "nanosec": "nsec",  # ROS2 Time.nanosec -> LCM Time.nsec
}

# Reverse mapping (LCM name -> ROS name)
_LCM_TO_ROS_FIELD_MAP: dict[str, str] = {v: k for k, v in _ROS_TO_LCM_FIELD_MAP.items()}


def get_dimos_type(msg_name: str) -> type[DimosMsg] | None:
    """Try to import dimos.msgs type, return None if not found. Cached.

    Args:
        msg_name: Message name in format "package.MessageName" (e.g., "geometry_msgs.Vector3")

    Returns:
        The dimos message type, or None if not found
    """
    if msg_name in _dimos_type_cache:
        return _dimos_type_cache[msg_name]

    try:
        package, name = msg_name.split(".")
        module = importlib.import_module(f"dimos.msgs.{package}.{name}")
        dimos_type = cast("type[DimosMsg]", getattr(module, name))
        _dimos_type_cache[msg_name] = dimos_type
        return dimos_type
    except (ImportError, AttributeError, ValueError):
        _dimos_type_cache[msg_name] = None
        return None


def derive_lcm_type(dimos_type: type[DimosMsg]) -> type[Any]:
    """Derive the LCM message type from a dimos message type.

    Args:
        dimos_type: A dimos message type (e.g., dimos.msgs.sensor_msgs.PointCloud2)

    Returns:
        The corresponding LCM message type (e.g., dimos_lcm.sensor_msgs.PointCloud2)
    """
    msg_name = dimos_type.msg_name  # e.g., "sensor_msgs.PointCloud2"

    if msg_name in _lcm_type_cache:
        return _lcm_type_cache[msg_name]

    parts = msg_name.split(".")
    if len(parts) != 2:
        raise ValueError(f"Invalid msg_name format: {msg_name}, expected 'package.MessageName'")

    package, message_name = parts
    lcm_module = importlib.import_module(f"dimos_lcm.{package}.{message_name}")
    lcm_type: type[Any] = getattr(lcm_module, message_name)
    _lcm_type_cache[msg_name] = lcm_type
    return lcm_type


def derive_ros_type(dimos_type: type[DimosMsg]) -> type[ROSMessage]:
    """Derive the ROS message type from a dimos message type.

    Args:
        dimos_type: A dimos message type (e.g., dimos.msgs.geometry_msgs.Vector3)

    Returns:
        The corresponding ROS message type (e.g., geometry_msgs.msg.Vector3)

    Example:
        msg_name = "geometry_msgs.Vector3" -> geometry_msgs.msg.Vector3
    """
    msg_name = dimos_type.msg_name  # e.g., "geometry_msgs.Vector3"
    parts = msg_name.split(".")
    if len(parts) != 2:
        raise ValueError(f"Invalid msg_name format: {msg_name}, expected 'package.MessageName'")

    package, message_name = parts
    ros_module = importlib.import_module(f"{package}.msg")
    return cast("type[ROSMessage]", getattr(ros_module, message_name))


def _copy_ros_to_lcm_recursive(ros_msg: Any, lcm_msg: Any) -> None:
    """Recursively copy fields from ROS message to LCM message.

    Handles nested messages, arrays, and primitive types.

    Args:
        ros_msg: Source ROS message
        lcm_msg: Target LCM message (modified in place)
    """
    if not hasattr(ros_msg, "get_fields_and_field_types"):
        raise TypeError(f"Expected ROS message, got {type(ros_msg).__name__}")

    field_types = ros_msg.get_fields_and_field_types()
    for ros_field_name in field_types:
        # Map ROS field name to LCM field name
        lcm_field_name = _ROS_TO_LCM_FIELD_MAP.get(ros_field_name, ros_field_name)

        if not hasattr(lcm_msg, lcm_field_name):
            continue

        ros_value = getattr(ros_msg, ros_field_name)
        lcm_value = getattr(lcm_msg, lcm_field_name)

        # Handle nested messages
        if hasattr(ros_value, "get_fields_and_field_types"):
            _copy_ros_to_lcm_recursive(ros_value, lcm_value)
        # Handle arrays of messages
        elif isinstance(ros_value, (list, tuple)) and len(ros_value) > 0:
            if hasattr(ros_value[0], "get_fields_and_field_types"):
                # Array of nested messages - create LCM instances
                lcm_array = []
                for ros_item in ros_value:
                    # Get the LCM element type from the first lcm_value element if available
                    # Otherwise try to derive from ros item
                    if isinstance(lcm_value, list) and len(lcm_value) > 0:
                        lcm_item = type(lcm_value[0])()
                    else:
                        # Try to create matching LCM type
                        lcm_item = _create_lcm_instance_for_ros_msg(ros_item)
                    _copy_ros_to_lcm_recursive(ros_item, lcm_item)
                    lcm_array.append(lcm_item)
                setattr(lcm_msg, lcm_field_name, lcm_array)
            else:
                # Array of primitives - direct copy
                setattr(lcm_msg, lcm_field_name, list(ros_value))
        # Handle bytes/data fields
        elif isinstance(ros_value, (bytes, bytearray)):
            setattr(lcm_msg, lcm_field_name, bytes(ros_value))
        # Handle array.array (ROS uses this for data fields)
        elif hasattr(ros_value, "tobytes"):
            setattr(lcm_msg, lcm_field_name, ros_value.tobytes())
        else:
            # Primitive type - direct copy
            setattr(lcm_msg, lcm_field_name, ros_value)

        # Update length fields if present (LCM convention: field_name_length)
        length_field = f"{lcm_field_name}_length"
        if hasattr(lcm_msg, length_field):
            value = getattr(lcm_msg, lcm_field_name)
            if isinstance(value, (list, tuple, bytes, bytearray)):
                setattr(lcm_msg, length_field, len(value))


def _copy_lcm_to_ros_recursive(lcm_msg: Any, ros_msg: Any) -> None:
    """Recursively copy fields from LCM message to ROS message.

    Handles nested messages, arrays, and primitive types.

    Args:
        lcm_msg: Source LCM message
        ros_msg: Target ROS message (modified in place)
    """
    if not hasattr(ros_msg, "get_fields_and_field_types"):
        raise TypeError(f"Expected ROS message, got {type(ros_msg).__name__}")

    field_types = ros_msg.get_fields_and_field_types()
    for ros_field_name in field_types:
        # Map ROS field name to LCM field name
        lcm_field_name = _ROS_TO_LCM_FIELD_MAP.get(ros_field_name, ros_field_name)

        if not hasattr(lcm_msg, lcm_field_name):
            continue

        lcm_value = getattr(lcm_msg, lcm_field_name)
        ros_value = getattr(ros_msg, ros_field_name)

        # Handle nested messages
        if hasattr(ros_value, "get_fields_and_field_types"):
            _copy_lcm_to_ros_recursive(lcm_value, ros_value)
        # Handle arrays of messages
        elif isinstance(lcm_value, (list, tuple)) and len(lcm_value) > 0:
            if hasattr(lcm_value[0], "lcm_encode"):
                # Array of nested LCM messages
                ros_array = []
                for lcm_item in lcm_value:
                    ros_item = _create_ros_instance_for_lcm_msg(
                        lcm_item, field_types[ros_field_name]
                    )
                    _copy_lcm_to_ros_recursive(lcm_item, ros_item)
                    ros_array.append(ros_item)
                setattr(ros_msg, ros_field_name, ros_array)
            else:
                # Array of primitives - direct copy
                setattr(ros_msg, ros_field_name, list(lcm_value))
        # Handle bytes/data fields
        elif isinstance(lcm_value, (bytes, bytearray)):
            # ROS data fields might expect array.array
            if hasattr(ros_value, "frombytes"):
                import array

                arr = array.array("B")
                arr.frombytes(lcm_value)
                setattr(ros_msg, ros_field_name, arr)
            else:
                setattr(ros_msg, ros_field_name, bytes(lcm_value))
        else:
            # Primitive type - direct copy
            setattr(ros_msg, ros_field_name, lcm_value)


def _create_lcm_instance_for_ros_msg(ros_msg: Any) -> Any:
    """Create an LCM message instance that matches the ROS message type.

    Args:
        ros_msg: ROS message to match

    Returns:
        New LCM message instance
    """
    # Get the ROS type name (e.g., "std_msgs.msg.Header" -> "std_msgs.Header")
    ros_type = type(ros_msg)
    module_name = ros_type.__module__  # e.g., "std_msgs.msg"
    class_name = ros_type.__name__  # e.g., "Header"

    # Convert to LCM module path (std_msgs.msg.Header -> dimos_lcm.std_msgs.Header)
    package = module_name.split(".")[0]  # e.g., "std_msgs"
    lcm_module = importlib.import_module(f"dimos_lcm.{package}.{class_name}")
    lcm_type = getattr(lcm_module, class_name)
    return lcm_type()


def _create_ros_instance_for_lcm_msg(lcm_msg: Any, ros_type_hint: str) -> Any:
    """Create a ROS message instance that matches the LCM message type.

    Args:
        lcm_msg: LCM message to match
        ros_type_hint: ROS type hint string (e.g., "sequence<sensor_msgs/PointField>")

    Returns:
        New ROS message instance
    """
    # Parse the type hint to get the message type
    # e.g., "sequence<sensor_msgs/PointField>" -> "sensor_msgs", "PointField"
    # e.g., "sensor_msgs/PointField" -> "sensor_msgs", "PointField"

    match = re.search(r"(\w+)/(\w+)", ros_type_hint)
    if match:
        package, class_name = match.groups()
        ros_module = importlib.import_module(f"{package}.msg")
        ros_type = getattr(ros_module, class_name)
        return ros_type()

    # Fallback: try to derive from LCM type
    lcm_type = type(lcm_msg)
    module_name = lcm_type.__module__  # e.g., "dimos_lcm.std_msgs.Header"
    class_name = lcm_type.__name__
    parts = module_name.split(".")
    if len(parts) >= 2:
        package = parts[1]  # e.g., "std_msgs"
        ros_module = importlib.import_module(f"{package}.msg")
        ros_type = getattr(ros_module, class_name)
        return ros_type()

    raise ValueError(f"Cannot determine ROS type for LCM message: {lcm_type}")


def dimos_to_ros(msg: DimosMsg, ros_type: type[ROSMessage]) -> ROSMessage:
    """Convert a dimos message to a ROS message.

    For complex types (PointCloud2, Image, CameraInfo), uses LCM roundtrip
    to properly convert internal representations. For simple types, uses
    direct field copy.

    Args:
        msg: Dimos message instance
        ros_type: Target ROS message type

    Returns:
        ROS message instance
    """
    msg_name = type(msg).msg_name

    if msg_name in COMPLEX_TYPES:
        # Complex: dimos → encode → decode LCM → copy to ROS
        lcm_type = derive_lcm_type(type(msg))
        lcm_bytes = msg.lcm_encode()
        lcm_msg = lcm_type.lcm_decode(lcm_bytes)
        ros_msg = ros_type()
        _copy_lcm_to_ros_recursive(lcm_msg, ros_msg)
        return ros_msg

    # Simple: recursive field copy (handles nested messages)
    ros_msg = ros_type()
    _copy_lcm_to_ros_recursive(msg, ros_msg)
    return ros_msg


def ros_to_dimos(msg: Any, dimos_type: type[DimosMsg]) -> DimosMsg:
    """Convert a ROS message to a dimos message.

    For complex types (PointCloud2, Image, CameraInfo), uses LCM roundtrip
    to properly build the dimos internal representation. For simple types,
    uses direct field copy.

    Args:
        msg: ROS message instance
        dimos_type: Target dimos message type

    Returns:
        Dimos message instance
    """
    msg_name = dimos_type.msg_name

    if msg_name in COMPLEX_TYPES:
        # Complex: ROS → LCM → encode → decode → dimos
        lcm_type = derive_lcm_type(dimos_type)
        lcm_msg = lcm_type()
        _copy_ros_to_lcm_recursive(msg, lcm_msg)
        return dimos_type.lcm_decode(lcm_msg.lcm_encode())

    # Simple type: recursive field copy (handles nested messages)
    dimos_msg = dimos_type()
    _copy_ros_to_lcm_recursive(msg, dimos_msg)
    return dimos_msg
