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

import math
from typing import Any

from dimos_generated.geometry_msgs.msg import TransformStamped
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.camera_info import intrinsic_matrix
from dimos.msgs.geometry import transform_matrix
from dimos.msgs.image import image_to_rgb, image_view
from dimos.msgs.time import to_nanoseconds

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
    records: NDArray[np.uint8] = np.ndarray(
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


def pointcloud_rgb(message: PointCloud2) -> NDArray[np.uint8] | None:
    """Copy RGB bytes from the standard packed rgb/rgba field, if present.

    FLOAT32 fields carry color bits, not numeric floating-point color values.
    Both representations preserve the declared byte order and row padding.
    """
    points = pointcloud_view(message)
    names = points.dtype.names or ()
    name = next((name for name in ("rgb", "rgba") if name in names), None)
    if name is None:
        return None
    field = points[name]
    if field.ndim != 2 or field.dtype.kind not in "fu" or field.dtype.itemsize != 4:
        raise ValueError("packed point cloud color must be a scalar FLOAT32 or UINT32 field")
    packed = field.view(np.dtype(">u4" if message.is_bigendian else "<u4")).ravel()
    return np.stack([(packed >> shift) & 255 for shift in (16, 8, 0)], axis=-1).astype(np.uint8)


def concatenate_clouds(first: PointCloud2, second: PointCloud2) -> PointCloud2:
    """Join compatible point records, preserving fields, endian, and point padding.

    Row padding is removed. The first frame and newest exact source stamp survive.
    Empty inputs are identities; differing nonempty frames/layouts are rejected.
    """
    pointcloud_view(first)
    pointcloud_view(second)
    if not first.width * first.height:
        return PointCloud2.decode(second.encode())
    if not second.width * second.height:
        return PointCloud2.decode(first.encode())
    if first.header.frame_id != second.header.frame_id:
        raise ValueError("cannot concatenate point clouds in different frames")
    if (
        first.fields != second.fields
        or first.point_step != second.point_step
        or first.is_bigendian != second.is_bigendian
    ):
        raise ValueError("cannot concatenate point clouds with different record layouts")
    a = select_points(first, np.ones(first.width * first.height, dtype=np.bool_))
    b = select_points(second, np.ones(second.width * second.height, dtype=np.bool_))
    stamp = max((first.header.stamp, second.header.stamp), key=to_nanoseconds)
    return PointCloud2(
        header=Header(frame_id=first.header.frame_id, stamp=stamp),
        height=1,
        width=a.width + b.width,
        fields=first.fields,
        is_bigendian=first.is_bigendian,
        point_step=first.point_step,
        row_step=a.row_step + b.row_step,
        data=bytes(a.data) + bytes(b.data),
        is_dense=first.is_dense and second.is_dense,
    )


def transform_cloud(message: PointCloud2, transform: TransformStamped) -> PointCloud2:
    """Apply parent←child to XYZ while retaining the complete organized record layout.

    Copy storage before changing coordinates. Keep all other fields, point/row
    padding, endian and the cloud's exact source stamp. Frames must match;
    integer XYZ is rejected because transformed coordinates would be truncated.
    """
    if message.header.frame_id != transform.child_frame_id:
        raise ValueError("point cloud frame does not match transform child frame")
    view = pointcloud_view(message)
    for name in ("x", "y", "z"):
        if name not in (view.dtype.names or ()) or view.dtype[name].kind != "f":
            raise ValueError("transform requires scalar floating-point XYZ fields")
    matrix = transform_matrix(transform.transform)
    xyz = pointcloud_xyz(message)
    transformed = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    for index, name in enumerate(("x", "y", "z")):
        values = transformed[:, index]
        if np.any(np.abs(values[np.isfinite(values)]) > np.finfo(view.dtype[name]).max):
            raise ValueError(f"transformed {name} coordinates exceed field range")
    payload = bytearray(bytes(message.data))
    writable = np.ndarray(
        (message.height, message.width),
        dtype=view.dtype,
        buffer=payload,
        strides=(message.row_step, message.point_step),
    )
    for index, name in enumerate(("x", "y", "z")):
        writable[name] = transformed[:, index].reshape(message.height, message.width)
    output = PointCloud2.decode(message.encode())
    output.data = bytes(payload)
    output.header = Header(stamp=message.header.stamp, frame_id=transform.header.frame_id)
    return output


def pointcloud_from_xyz_rgb(
    points: NDArray[Any], colors: NDArray[np.uint8], *, header: Header
) -> PointCloud2:
    """Copy XYZ and RGB8 into a declared XYZ FLOAT32 / packed RGB UINT32 layout."""
    cloud = pointcloud_from_xyz(points, header=header)
    rgb = np.asarray(colors)
    if rgb.dtype != np.uint8 or rgb.shape != np.asarray(points).shape:
        raise ValueError("RGB must be uint8 with the same (..., 3) shape as XYZ")
    records = np.empty((cloud.height, cloud.width), dtype=[("xyz", "<f4", (3,)), ("rgb", "<u4")])
    records["xyz"] = np.asarray(points).reshape(cloud.height, cloud.width, 3)
    rgb = rgb.reshape(cloud.height, cloud.width, 3).astype(np.uint32)
    records["rgb"] = (rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]
    cloud.fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
    cloud.point_step = 16
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = records.tobytes()
    return cloud


def pointcloud_from_rgbd(
    color: Image,
    depth: Image,
    calibration: CameraInfo,
    *,
    depth_scale: float = 1.0,
    depth_trunc: float = 5.0,
) -> PointCloud2:
    """Project a rectified RGB/depth pair into the depth optical frame.

    Integer depth is multiplied by depth_scale to obtain meters; 32FC1 already
    represents meters. Invalid/nonpositive/too-distant samples are removed.
    Scale K when calibration resolution differs, retaining the depth header.
    """
    if (
        not math.isfinite(depth_scale)
        or depth_scale <= 0
        or not math.isfinite(depth_trunc)
        or depth_trunc <= 0
    ):
        raise ValueError("depth scale and truncation must be finite and positive")
    if (color.width, color.height) != (depth.width, depth.height):
        raise ValueError("color and depth dimensions do not match")
    if depth.encoding not in ("16UC1", "mono16", "32FC1"):
        raise ValueError("RGBD requires uint16 or float32 depth")
    if calibration.width <= 0 or calibration.height <= 0:
        raise ValueError("calibration dimensions must be positive")
    matrix = intrinsic_matrix(calibration).copy()
    matrix[0, :] *= depth.width / calibration.width
    matrix[1, :] *= depth.height / calibration.height
    distances = image_view(depth).astype(np.float64)
    if depth.encoding != "32FC1":
        distances *= depth_scale
    valid = np.isfinite(distances) & (distances > 0) & (distances < depth_trunc)
    rows, columns = np.indices(distances.shape)
    z = distances[valid]
    xyz = np.column_stack(
        (
            (columns[valid] - matrix[0, 2]) * z / matrix[0, 0],
            (rows[valid] - matrix[1, 2]) * z / matrix[1, 1],
            z,
        )
    )
    return pointcloud_from_xyz_rgb(xyz, image_to_rgb(color)[valid], header=depth.header)


def voxel_downsample_cloud(message: PointCloud2, voxel_size: float = 0.025) -> PointCloud2:
    """Downsample XYZ/RGB clouds with the existing Open3D tensor backend.

    Non-positive sizes and clouds below twenty points retain the existing fast
    path. Unsupported extra fields are rejected rather than silently dropped.
    Source headers are preserved; generated messages own no numerical methods.
    """
    if not math.isfinite(voxel_size):
        raise ValueError("voxel_size must be finite")
    if voxel_size <= 0 or message.width * message.height < 20:
        return message
    names = {field.name for field in message.fields}
    if names not in ({"x", "y", "z"}, {"x", "y", "z", "rgb"}):
        raise ValueError("voxel downsampling supports only XYZ and optional RGB fields")
    points = pointcloud_xyz(message).astype(np.float32)
    if not np.isfinite(points).all():
        raise ValueError("voxel downsampling requires finite XYZ positions")
    colors = pointcloud_rgb(message)
    import open3d as o3d  # type: ignore[import-untyped]

    cloud = o3d.t.geometry.PointCloud(o3d.core.Tensor(points))
    if colors is not None:
        cloud.point["colors"] = o3d.core.Tensor(colors.astype(np.float32) / 255.0)
    reduced = cloud.voxel_down_sample(voxel_size)
    xyz = reduced.point["positions"].numpy()
    if colors is None:
        return pointcloud_from_xyz(xyz, header=message.header)
    rgb = np.clip(reduced.point["colors"].numpy() * 255.0, 0, 255).astype(np.uint8)
    return pointcloud_from_xyz_rgb(xyz, rgb, header=message.header)


def pointcloud_to_open3d(message: PointCloud2) -> Any:
    """Copy generated XYZ/RGB values into a native geometry algorithm input."""
    import open3d as o3d  # type: ignore[import-untyped]

    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(pointcloud_xyz(message))
    colors = pointcloud_rgb(message)
    if colors is not None:
        result.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
    return result


def cloud_bounds_intersect(first: PointCloud2, second: PointCloud2) -> bool:
    """Test closed axis-aligned bounds; empty or entirely nonfinite clouds do not intersect."""
    a, b = pointcloud_xyz(first), pointcloud_xyz(second)
    a, b = a[np.isfinite(a).all(axis=1)], b[np.isfinite(b).all(axis=1)]
    if len(a) == 0 or len(b) == 0:
        return False
    return bool(np.all(a.min(axis=0) <= b.max(axis=0)) and np.all(a.max(axis=0) >= b.min(axis=0)))
