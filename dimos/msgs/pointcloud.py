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

from dimos_generated.sensor_msgs.msg import PointCloud2
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
