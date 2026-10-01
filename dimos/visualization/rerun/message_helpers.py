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

"""Rerun presentation helpers kept separate from generated wire types."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Literal

from dimos_generated.dimos_msgs.msg import EntityMarkers
from dimos_generated.foxglove_msgs.msg import CompressedVideo
from dimos_generated.geometry_msgs.msg import PointStamped, PoseStamped
from dimos_generated.nav_msgs.msg import OccupancyGrid, Odometry, Path
from dimos_generated.sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import Detection3DArray
import matplotlib
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.image import image_from_compressed, image_to_jpeg, image_view
from dimos.msgs.occupancy import grid_to_world, occupancy_view
from dimos.msgs.pointcloud import pointcloud_rgb, pointcloud_xyz

if TYPE_CHECKING:
    import rerun as rr


def tf_archetypes(message: TFMessage) -> list[tuple[str, rr.Transform3D]]:
    """Render named TF edges without embedding viewer behavior in messages."""
    import rerun as rr

    result = []
    for edge in message.transforms:
        t, q = edge.transform.translation, edge.transform.rotation
        result.append(
            (
                f"tf_links/{rr.escape_entity_path_part(edge.child_frame_id)}",
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                    parent_frame="tf#/" + edge.header.frame_id,
                    child_frame="tf#/" + edge.child_frame_id,
                ),
            )
        )
    return result


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register 256 class colors for clouds carrying colormap indices."""
    import rerun as rr

    colors = (matplotlib.colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=color.tolist())
                )
                for i, color in enumerate(colors)
            ]
        ),
        static=True,
    )


def camera_pinhole(info: CameraInfo, *, optical_frame: str | None = None) -> rr.Pinhole:
    """Calibration attached to the camera's optical frame."""
    import rerun as rr

    frame = info.header.frame_id if optical_frame is None else optical_frame
    return rr.Pinhole(
        focal_length=[info.k[0], info.k[4]],
        principal_point=[info.k[2], info.k[5]],
        width=info.width,
        height=info.height,
        image_plane_distance=1.0,
        parent_frame=f"tf#/{frame}" if frame else None,
    )


def image_archetype(message: Image | CompressedImage) -> rr.Image | rr.DepthImage | rr.EncodedImage:
    """Render ROS image encodings without adding methods to generated values."""
    import rerun as rr

    if isinstance(message, CompressedImage):
        format_name = message.format.lower()
        if "jxl" in format_name:
            return image_archetype(image_from_compressed(message))
        if "jpeg" in format_name or "jpg" in format_name:
            media_type = "image/jpeg"
        elif "png" in format_name:
            media_type = "image/png"
        else:
            raise ValueError(f"unsupported compressed image format {message.format!r}")
        return rr.EncodedImage(contents=bytes(message.data), media_type=media_type)
    if message.encoding in ("16UC1", "32FC1"):
        return rr.DepthImage(
            image_view(message), meter=1000.0 if message.encoding == "16UC1" else 1.0
        )
    if message.encoding == "mono16":
        return rr.Image(image_view(message), color_model="L")
    return rr.EncodedImage(contents=image_to_jpeg(message), media_type="image/jpeg")


def detection_boxes(message: Detection3DArray) -> rr.Boxes3D:
    """Render detection geometry and labels from generated values."""
    import rerun as rr

    centers = []
    half_sizes = []
    rotations = []
    labels = []
    for detection in message.detections:
        box = detection.bbox
        p, q, size = box.center.position, box.center.orientation, box.size
        centers.append((p.x, p.y, p.z))
        half_sizes.append((size.x / 2, size.y / 2, size.z / 2))
        rotations.append((q.x, q.y, q.z, q.w))
        identifier = detection.id.strip()
        label = next(
            (
                result.hypothesis.class_id.strip()
                for result in detection.results
                if result.hypothesis.class_id.strip()
            ),
            "",
        )
        labels.append(
            f"{label} id={identifier}"
            if label and identifier
            else label or (f"id={identifier}" if identifier else "")
        )
    return rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=rotations, labels=labels)


def cloud_archetype(
    message: PointCloud2,
    *,
    voxel_size: float = 0.05,
    mode: str = "spheres",
    colors: Sequence[int] | NDArray[np.uint8] | None = None,
    rgb: bool = True,
    bottom_cutoff: float | None = None,
    ui_radius: float = 2.0,
    fill_mode: Literal["solid", "majorwireframe", "densewireframe"] = "solid",
) -> rr.Points3D | rr.Boxes3D:
    """Render finite XYZ points with packed RGB or an explicit height colormap."""
    import rerun as rr

    points = pointcloud_xyz(message)
    if colors is None:
        colors = pointcloud_rgb(message) if rgb else None
    else:
        color_array = np.asarray(colors)
        if color_array.shape in {(3,), (4,)}:
            color_array = np.broadcast_to(color_array, (len(points), color_array.shape[0]))
        if color_array.shape not in {(len(points), 3), (len(points), 4)}:
            raise ValueError("colors must be RGB/RGBA or one RGB/RGBA row per point")
        if color_array.dtype.kind not in "iu" or np.any((color_array < 0) | (color_array > 255)):
            raise ValueError("color channels must be integers in [0, 255]")
        colors = np.asarray(color_array, dtype=np.uint8)
    keep = np.isfinite(points).all(axis=1)
    if mode not in {"points", "boxes", "spheres"}:
        raise ValueError("cloud mode must be points, boxes or spheres")
    if bottom_cutoff is not None:
        keep &= points[:, 2] >= bottom_cutoff
    points = points[keep]
    if len(points) == 0:
        return rr.Boxes3D(centers=[]) if mode == "boxes" else rr.Points3D([])
    if colors is not None:
        colors = colors[keep]
    else:
        height = points[:, 2]
        normalized = (height - height.min()) / (height.max() - height.min() + 1e-8)
        colors = (matplotlib.colormaps["turbo"](normalized)[:, :3] * 255).astype(np.uint8)
    if mode == "boxes":
        return rr.Boxes3D(
            centers=points, half_sizes=[voxel_size / 2] * 3, colors=colors, fill_mode=fill_mode
        )
    return rr.Points3D(
        positions=points, colors=colors, radii=-ui_radius if mode == "points" else voxel_size / 2
    )


def navigation_archetype(
    message: PointStamped | PoseStamped | Odometry | Path,
    *,
    color: tuple[int, int, int] = (0, 255, 128),
    z_offset: float = 0.5,
) -> rr.Points3D | rr.Transform3D | rr.LineStrips3D:
    """Render generated navigation values in their declared parent frame."""
    import rerun as rr

    if isinstance(message, PointStamped):
        p = message.point
        return rr.Points3D([[p.x, p.y, p.z]])
    if isinstance(message, Path):
        points = [
            [p.pose.position.x, p.pose.position.y, p.pose.position.z + z_offset]
            for p in message.poses
        ]
        return rr.LineStrips3D([points] if points else [], colors=color, radii=0.05)
    pose = message.pose.pose if isinstance(message, Odometry) else message.pose
    p, q = pose.position, pose.orientation
    return rr.Transform3D(
        translation=[p.x, p.y, p.z],
        quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
        parent_frame=f"tf#/{message.header.frame_id}" if message.header.frame_id else None,
    )


def occupancy_mesh(message: OccupancyGrid) -> rr.Mesh3D:
    """Render an occupancy texture on the grid's fully transformed plane."""
    import rerun as rr

    cells = occupancy_view(message)
    if cells.size == 0:
        return rr.Mesh3D(vertex_positions=[])
    intensity = 1 - np.clip(cells.astype(np.float32), 0, 100) / 100
    colors = (intensity[..., None] * np.array([72, 73, 129])).astype(np.uint8)
    colors[cells < 0] = 0
    rgba = np.concatenate([colors, np.full((*cells.shape, 1), 255, dtype=np.uint8)], axis=2)
    width, height = message.info.width, message.info.height
    corners = [
        grid_to_world(message, xy) for xy in ((0, 0), (width, 0), (width, height), (0, height))
    ]
    return rr.Mesh3D(
        vertex_positions=[[p.x, p.y, p.z] for p in corners],
        triangle_indices=[[0, 1, 2], [0, 2, 3]],
        vertex_texcoords=[[0, 1], [1, 1], [1, 0], [0, 0]],
        albedo_texture=np.ascontiguousarray(rgba[::-1]),
    )


def video_archetype(message: CompressedVideo) -> rr.VideoStream:
    """Log an encoded packet; inter-frame codecs require ordered packets from a keyframe."""
    import rerun as rr

    codecs = {
        "h264": rr.VideoCodec.H264,
        "h265": rr.VideoCodec.H265,
        "av1": rr.VideoCodec.AV1,
        "vp8": rr.VideoCodec.VP8,
        "vp9": rr.VideoCodec.VP9,
    }
    codec = codecs.get(message.format.lower())
    if codec is None:
        raise ValueError(f"no rerun VideoCodec for format {message.format!r}")
    return rr.VideoStream(codec, sample=bytes(message.data))


def entity_points(message: EntityMarkers) -> rr.Points3D:
    """Render generated entity values with labels/colors outside the wire classes."""
    import rerun as rr

    colors = {
        "person": (255, 100, 100, 255),
        "object": (100, 255, 100, 255),
        "location": (100, 100, 255, 255),
    }
    return rr.Points3D(
        positions=[
            [marker.position.x, marker.position.y, marker.position.z] for marker in message.markers
        ],
        labels=[f"{marker.entity_id}: {marker.label[:40]}" for marker in message.markers],
        colors=[colors.get(marker.entity_type, (200, 200, 200, 255)) for marker in message.markers],
        radii=[0.15] * len(message.markers),
    )
