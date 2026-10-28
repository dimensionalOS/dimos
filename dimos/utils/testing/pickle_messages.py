# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Read historical message pickle state without importing retired message classes."""

from __future__ import annotations

from collections.abc import Mapping
import pickle
from typing import Any, BinaryIO

from dimos_cdrgen.registry import message_types
import numpy as np

from dimos.message_utils import image, pointcloud
from dimos.message_utils.time import time_from_seconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.std_msgs.msg import Header


class _MessageState:
    type_name: str
    args: tuple[Any, ...]

    def __new__(cls, *args: Any) -> _MessageState:
        value = super().__new__(cls)
        value.args = args
        return value

    def __setstate__(self, state: Any) -> None:
        if isinstance(state, tuple):
            for part in state:
                if isinstance(part, Mapping):
                    self.__dict__.update(part)
        elif isinstance(state, Mapping):
            self.__dict__.update(state)
        else:
            raise ValueError(f"Unsupported historical state for {self.type_name}")


class _MessageUnpickler(pickle.Unpickler):
    def find_class(self, module: str, name: str) -> Any:
        if module.startswith("dimos.msgs."):
            return type(name, (_MessageState,), {"type_name": module + "." + name})
        return super().find_class(module, name)


def _convert(value: Any) -> Any:
    if isinstance(value, tuple):
        return tuple(_convert(item) for item in value)
    if isinstance(value, list):
        return [_convert(item) for item in value]
    if isinstance(value, dict):
        return {key: _convert(item) for key, item in value.items()}
    if not isinstance(value, _MessageState):
        return value
    name = value.type_name.rsplit(".", 1)[-1]
    raw = value.__dict__
    fields = {key: _convert(item) for key, item in raw.items() if key != "args"}
    if name == "ImageFormat":
        return value.args[0]
    if name in {"Vector3", "Point", "Quaternion"}:
        vector_cls = {"Vector3": Vector3, "Point": Point, "Quaternion": Quaternion}[name]
        keys = ("x", "y", "z", "w") if name == "Quaternion" else ("x", "y", "z")
        return vector_cls(**{key: fields[key] for key in keys})
    header = Header(time_from_seconds(float(fields.get("ts", 0.0))), fields.get("frame_id", ""))
    if name in {"Pose", "PoseStamped"}:
        position = fields["position"]
        pose = Pose(Point(position.x, position.y, position.z), fields["orientation"])
        return PoseStamped(header, pose) if name == "PoseStamped" else pose
    if name == "Transform":
        return TransformStamped(
            header,
            fields.get("child_frame_id", ""),
            Transform(fields["translation"], fields["rotation"]),
        )
    if name == "Image":
        encoding = {
            "rgb": "rgb8",
            "bgr": "bgr8",
            "rgba": "rgba8",
            "bgra": "bgra8",
            "gray": "mono8",
            "gray16": "mono16",
            "depth": "32FC1",
            "depth16": "16UC1",
        }[str(fields["format"]).lower()]
        return image.from_array(fields["data"], encoding=encoding, header=header)
    if name == "PointCloud2":
        return pointcloud.from_xyz(np.asarray(fields["_pcd_numpy"]), header=header)
    canonical = value.type_name.split(".")[2] + "/msg/" + name
    cls = message_types().get(canonical)
    if cls is None:
        raise ValueError(f"Unsupported historical message {canonical}")
    fields.pop("ts", None)
    fields.pop("frame_id", None)
    annotations = cls.__annotations__
    if "header" in annotations:
        fields["header"] = header
    for key in ("position", "velocity", "effort", "axes", "buttons"):
        if key in fields and isinstance(fields[key], list):
            dtype = np.int32 if key == "buttons" else np.float32 if key == "axes" else np.float64
            fields[key] = np.asarray(fields[key], dtype=dtype)
    return cls(**{key: item for key, item in fields.items() if key in annotations})


def load_message_pickle(source: BinaryIO) -> Any:
    return _convert(_MessageUnpickler(source).load())
