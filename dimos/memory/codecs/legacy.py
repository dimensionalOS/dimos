# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from functools import lru_cache
import importlib
import io
from typing import Any

from dimos_cdrgen.registry import initialize, message_types
import numpy as np
from rosbags.interfaces import Nodetype
from rosbags.interfaces.typing import FieldDesc


@lru_cache(maxsize=256)
def _target_fields(ros_type: str) -> dict[str, FieldDesc]:
    # The runtime registry is frozen; inspect schema metadata once per type,
    # rather than rereading the installed package on every nested recording value.
    return dict(initialize().fielddefs[ros_type][1])


def _convert_fields(old: Any, sequences: dict[str, int], path: str = "") -> Any:
    if isinstance(old, (list, tuple)):
        return [_convert_fields(item, sequences, f"{path}[{i}]") for i, item in enumerate(old)]
    if not hasattr(old, "__slots__"):
        return old
    package = type(old).__module__.split(".")[-2]
    name = type(old).__name__
    if package == "std_msgs" and name in {"Time", "Duration"}:
        package = "builtin_interfaces"
    ros_type = f"{package}/msg/{name}"
    target = message_types().get(ros_type)
    if target is None:
        raise ValueError(f"Missing generated {ros_type}")
    values = {}
    fields = _target_fields(ros_type)
    for key in old.__slots__:
        value = getattr(old, key)
        if ros_type == "std_msgs/msg/Header" and key == "seq":
            sequences[f"{path}.seq".lstrip(".")] = value
            continue
        if key.endswith("_length") and key[:-7] in old.__slots__:
            if value != len(getattr(old, key[:-7])):
                raise ValueError(f"Invalid LCM sequence length at {path}.{key}")
            continue
        new_key = "nanosec" if package == "builtin_interfaces" and key == "nsec" else key
        if ros_type == "sensor_msgs/msg/CameraInfo" and key in {"D", "K", "R", "P"}:
            new_key = key.lower()
        if new_key not in target.__annotations__:
            raise ValueError(f"No lossless field mapping for {ros_type}.{key}")
        converted = _convert_fields(value, sequences, f"{path}.{key}".lstrip("."))
        definition = fields[new_key]
        if definition[0] == Nodetype.ARRAY or definition[0] == Nodetype.SEQUENCE:
            element = definition[1][0]
            if element[0] == Nodetype.BASE and element[1][0] != "string":
                scalar = element[1][0]
                dtype = np.dtype({"byte": "uint8", "char": "uint8"}.get(scalar, scalar))
                converted = (
                    np.frombuffer(converted, dtype=dtype).copy()
                    if isinstance(converted, bytes)
                    else np.asarray(converted, dtype=dtype)
                )
        values[new_key] = converted
    return target(**values)


def decode_lcm(data: bytes, message_type: type) -> Any:
    name = getattr(message_type, "__msgtype__", "")
    package, _, message = name.split("/")
    if message_types().get(name) is not message_type:
        raise TypeError("Expected a registered generated message type")
    if name == "dimos_msgs/msg/LineSegments3D":
        from dimos_lcm.nav_msgs import Path

        from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D

        stream = io.BytesIO(data)
        old = Path.lcm_decode(stream)
        if stream.tell() != len(data) or len(old.poses) % 2:
            raise ValueError("Historical segments require complete pose pairs")
        converted = _convert_fields(old, {})
        return LineSegments3D(
            header=converted.header,
            segments=[
                LineSegment3D(
                    start=a.pose.position, end=b.pose.position, weight=a.pose.orientation.w
                )
                for a, b in zip(converted.poses[::2], converted.poses[1::2], strict=True)
            ],
        )
    if name == "geometry_msgs/msg/TransformStamped":
        from dimos_lcm.tf2_msgs import TFMessage

        if data[:8] == TFMessage._get_packed_fingerprint():
            stream = io.BytesIO(data)
            old = TFMessage.lcm_decode(stream)
            if stream.tell() != len(data) or not old.transforms:
                raise ValueError("Historical transform requires a complete nonempty TF envelope")
            return _convert_fields(old.transforms[0], {})
    module = importlib.import_module(f"dimos_lcm.{package}.{message}")
    stream = io.BytesIO(data)
    old = getattr(module, message).lcm_decode(stream)
    if stream.tell() != len(data):
        raise ValueError("Trailing bytes after LCM message")
    return _convert_fields(old, {})
