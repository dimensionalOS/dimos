# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Pack the Go2 lidar's sensor fields without an Open3D message wrapper."""

from copy import deepcopy
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.message_utils import pointcloud
from dimos.messages.sensor_msgs.msg import PointCloud2, PointField
from dimos.messages.std_msgs.msg import Header


def sensor_cloud(
    points: NDArray[Any],
    *,
    header: Header,
    intensities: NDArray[Any] | None = None,
    offset_times: NDArray[Any] | None = None,
    tags: NDArray[Any] | None = None,
    lines: NDArray[Any] | None = None,
) -> PointCloud2:
    columns = [
        ("x", np.asarray(points)[:, 0], "<f4", PointField.FLOAT32),
        ("y", np.asarray(points)[:, 1], "<f4", PointField.FLOAT32),
        ("z", np.asarray(points)[:, 2], "<f4", PointField.FLOAT32),
    ]
    for name, values, dtype, datatype in [
        ("intensity", intensities, "<f4", PointField.FLOAT32),
        ("offset_time", offset_times, "<u4", PointField.UINT32),
        ("tag", tags, "u1", PointField.UINT8),
        ("line", lines, "u1", PointField.UINT8),
    ]:
        if values is not None:
            if len(values) != len(points):
                raise ValueError(f"{name} length differs from XYZ")
            columns.append((name, np.asarray(values).reshape(-1), dtype, datatype))
    packed = np.empty(len(points), dtype=[(name, dtype) for name, _, dtype, _ in columns])
    fields = []
    assert packed.dtype.fields is not None
    for name, values, _, datatype in columns:
        packed[name] = values
        fields.append(PointField(name, packed.dtype.fields[name][1], datatype, 1))
    stride = packed.dtype.itemsize
    return PointCloud2(
        deepcopy(header),
        1,
        len(points),
        fields,
        False,
        stride,
        stride * len(points),
        packed.view(np.uint8).reshape(-1),
        bool(np.isfinite(points).all()),
    )


def sensor_field(cloud: PointCloud2, name: str) -> NDArray[Any] | None:
    if not any(field.name == name for field in cloud.fields):
        return None
    return pointcloud.view(cloud)[name].reshape(-1).copy()
