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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""NumPy conversions for generated PointCloud2 messages."""

from typing import Any

from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray

_FIELD_KINDS = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def pointcloud_view(message: PointCloud2) -> NDArray[Any]:
    """Borrow read-only structured (height, width) points, including row/field padding.

    Fields retain their declared counts and endianness. The returned array keeps
    the message's storage alive. Use ``view.copy()`` for independent mutable data.
    """
    if message.width and not message.point_step:
        raise ValueError("nonempty point cloud requires a positive point_step")
    if message.row_step < message.width * message.point_step:
        raise ValueError("point cloud row_step is smaller than its row of points")
    if len(message.data) != message.height * message.row_step:
        raise ValueError("point cloud data length does not match height * row_step")
    names: list[str] = []
    formats: list[Any] = []
    offsets: list[int] = []
    for field in message.fields:
        if not field.name or field.name in names:
            raise ValueError("point cloud field names must be nonempty and unique")
        if field.datatype not in _FIELD_KINDS or not field.count:
            raise ValueError(f"invalid point cloud datatype/count for {field.name!r}")
        dtype = np.dtype((">" if message.is_bigendian else "<") + _FIELD_KINDS[field.datatype])
        if field.offset + dtype.itemsize * field.count > message.point_step:
            raise ValueError(f"point cloud field {field.name!r} exceeds point_step")
        names.append(field.name)
        formats.append(dtype if field.count == 1 else (dtype, (field.count,)))
        offsets.append(field.offset)
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": message.point_step}
    )
    return np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=message.data.view(),
        strides=(message.row_step, message.point_step),
    )


def pointcloud_xyz(message: PointCloud2) -> NDArray[np.float64]:
    """Copy scalar x/y/z fields into an N-by-3 float64 array, preserving point order."""
    points = pointcloud_view(message)
    for name in ("x", "y", "z"):
        if name not in (points.dtype.names or ()) or points[name].ndim != 2:
            raise ValueError(f"point cloud requires a scalar {name!r} field")
    return np.stack([points[name].ravel() for name in ("x", "y", "z")], axis=-1).astype(np.float64)


def pointcloud_from_xyz(points: NDArray[Any], *, header: Header) -> PointCloud2:
    """Copy Nx3 or HxWx3 coordinates into a tightly packed little-endian XYZ cloud.

    XYZ fields are float32. NaNs/infinities remain missing-point markers and set
    is_dense=False; finite coordinates outside float32 range are rejected.
    """
    values = np.asarray(points)
    if values.ndim not in (2, 3) or values.shape[-1] != 3:
        raise ValueError("XYZ coordinates must have shape (N, 3) or (H, W, 3)")
    if values.dtype.kind not in "fiu":
        raise ValueError("XYZ coordinates must be numeric")
    finite = np.isfinite(values)
    if np.any(np.abs(values[finite].astype(np.float64)) > np.finfo(np.float32).max):
        raise ValueError("XYZ coordinates exceed float32 range")
    packed = np.ascontiguousarray(values, dtype="<f4")
    height, width = (1, packed.shape[0]) if packed.ndim == 2 else packed.shape[:2]
    return PointCloud2(
        header=header,
        height=height,
        width=width,
        fields=[
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z"))
        ],
        is_bigendian=False,
        point_step=12,
        row_step=width * 12,
        data=packed.view(np.uint8).reshape(-1),
        is_dense=bool(finite.all()),
    )


def select_points(message: PointCloud2, keep: NDArray[np.bool_]) -> PointCloud2:
    """Copy selected point records in row order, retaining every field and point byte.

    The result is unorganized with no row padding. Point padding, endianness,
    field metadata, source header, and the conservative density flag survive.
    """
    pointcloud_view(message)  # Validate the full declared layout before selecting bytes.
    if keep.dtype != np.bool_ or keep.shape != (message.height * message.width,):
        raise ValueError("point selection must be a flat boolean mask matching the point count")
    records = np.ndarray(
        (message.height, message.width, message.point_step),
        dtype=np.uint8,
        buffer=message.data.view(),
        strides=(message.row_step, message.point_step, 1),
    )
    selected = records.reshape(message.height * message.width, message.point_step)[keep]
    width = len(selected)
    return PointCloud2(
        header=message.header,
        height=1,
        width=width,
        fields=message.fields,
        is_bigendian=message.is_bigendian,
        point_step=message.point_step,
        row_step=width * message.point_step,
        data=selected.tobytes(),
        is_dense=message.is_dense,
    )
