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

"""Viewer adapters live outside the generated plain value classes."""

from __future__ import annotations

from dataclasses import asdict
from functools import lru_cache
import json
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.messages.dimos_msgs.msg import EntityMarkers
from dimos.messages.geometry_msgs.msg import PointStamped, PoseStamped
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from dimos.msgs.image import image_from_compressed, image_to_rgb, image_view
from dimos.msgs.pointcloud import pointcloud_rgb, pointcloud_xyz

if TYPE_CHECKING:
    import rerun as rr

    from dimos.visualization.rerun.bridge import RerunMulti


def message_frame_id(message: Any) -> str | None:
    header = getattr(message, "header", None)
    frame = (
        getattr(header, "frame_id", None)
        if header is not None
        else getattr(message, "frame_id", None)
    )
    return frame if isinstance(frame, str) else None


def camera_pinhole(info: CameraInfo) -> rr.Pinhole:
    import rerun as rr

    return rr.Pinhole(
        focal_length=[info.k[0], info.k[4]],
        principal_point=[info.k[2], info.k[5]],
        width=info.width,
        height=info.height,
        image_plane_distance=1.0,
        parent_frame=f"tf#/{info.header.frame_id}" if info.header.frame_id else None,
    )


def image_archetype(message: Image | CompressedImage) -> rr.Image | rr.DepthImage | rr.EncodedImage:
    import rerun as rr

    if isinstance(message, CompressedImage):
        name = message.format.lower()
        if "jxl" in name or ("png" in name and name.startswith("16uc1;")):
            return image_archetype(image_from_compressed(message))
        if "jpeg" in name or "jpg" in name:
            return rr.EncodedImage(contents=bytes(message.data), media_type="image/jpeg")
        if "png" in name:
            return rr.EncodedImage(contents=bytes(message.data), media_type="image/png")
        raise ValueError(f"unsupported compressed image format {message.format!r}")
    if message.encoding in ("16UC1", "32FC1", "mono16"):
        pixels = image_view(message)
        pixels = np.ascontiguousarray(pixels, dtype=pixels.dtype.newbyteorder("="))
        if message.encoding != "mono16":
            return rr.DepthImage(pixels, meter=1000.0 if message.encoding == "16UC1" else 1.0)
        return rr.Image(
            bytes=pixels.tobytes(),
            width=message.width,
            height=message.height,
            datatype=np.uint16,
            color_model="L",
        )
    return rr.Image(
        bytes=image_to_rgb(message).tobytes(),
        width=message.width,
        height=message.height,
        datatype=np.uint8,
        color_model="RGB",
    )


def cloud_archetype(
    message: PointCloud2,
    *,
    voxel_size: float = 0.05,
    mode: str = "points",
    bottom_cutoff: float | None = None,
    height_colors: bool = False,
    color: tuple[int, int, int] | None = None,
) -> rr.Points3D | rr.Boxes3D:
    """Render plain cloud values, keeping color/geometry filters aligned.

    Existing bridge defaults use screen-space dots. Offline map tools explicitly
    select world-space spheres/boxes and the existing height-based turbo colors.
    """
    from matplotlib import colormaps
    import rerun as rr

    points, colors = pointcloud_xyz(message), pointcloud_rgb(message)
    keep = np.isfinite(points).all(axis=1)
    if bottom_cutoff is not None:
        keep &= points[:, 2] >= bottom_cutoff
    points = points[keep]
    if colors is not None:
        colors = colors[keep]
    elif height_colors and len(points):
        z = points[:, 2]
        levels = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)
        colors = (colormaps["turbo"](levels)[:, :3] * 255).astype(np.uint8)
    if color is not None:
        colors = np.asarray(color, dtype=np.uint8)
    if mode == "boxes":
        return rr.Boxes3D(centers=points, half_sizes=[voxel_size / 2] * 3, colors=colors)
    return rr.Points3D(
        points,
        colors=colors,
        radii=rr.components.Radius.ui_points(2.0) if mode == "points" else voxel_size / 2,
    )


@lru_cache(maxsize=16)
def _colormap_lut(name: str) -> np.ndarray[Any, np.dtype[np.uint8]]:
    from matplotlib import colormaps

    return (colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register the height palette outside the generated point-cloud value type."""
    import rerun as rr

    lut = _colormap_lut(name)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=lut[i].tolist())
                )
                for i in range(256)
            ]
        ),
        static=True,
    )


def entity_points(message: EntityMarkers) -> rr.Points3D:
    """Render generated world entities with the existing labels, colors and radii."""
    import rerun as rr

    colors = {
        "person": (255, 100, 100, 255),
        "object": (100, 255, 100, 255),
        "location": (100, 100, 255, 255),
    }
    return rr.Points3D(
        positions=[[m.position.x, m.position.y, m.position.z] for m in message.markers],
        labels=[f"{m.entity_id}: {m.label[:40]}" for m in message.markers],
        colors=[colors.get(m.entity_type, (200, 200, 200, 255)) for m in message.markers],
        radii=[0.15] * len(message.markers),
    )


def path_archetype(
    message: Path,
    *,
    color: tuple[int, int, int] = (0, 255, 128),
    z_offset: float = 0.5,
    radii: float = 0.05,
) -> rr.LineStrips3D:
    import rerun as rr

    if not message.poses:
        return rr.LineStrips3D([])
    points = [
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + z_offset]
        for pose in message.poses
    ]
    return rr.LineStrips3D([points], colors=[color], radii=radii)


def generated_archetypes(message: Any, entity: str) -> RerunMulti | None:
    """Render stamped geometry and its exact fields; other domains remain explicit."""
    import rerun as rr

    if type(message) in (Image, CompressedImage, PointCloud2):
        archetype = (
            cloud_archetype(message) if type(message) is PointCloud2 else image_archetype(message)
        )
        entries: RerunMulti = [(entity, archetype)]
        frame = message_frame_id(message)
        if frame:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{frame}")))
        return entries
    if type(message) is Path:
        entries = [(entity, path_archetype(message))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    elif type(message) is EntityMarkers:
        entries = [(entity, entity_points(message))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    elif type(message) is PoseStamped:
        point, rotation = message.pose.position, message.pose.orientation
        entries = [
            (
                entity,
                rr.Transform3D(
                    translation=[point.x, point.y, point.z],
                    quaternion=rr.Quaternion(xyzw=[rotation.x, rotation.y, rotation.z, rotation.w]),
                    parent_frame=f"tf#/{message.header.frame_id}"
                    if message.header.frame_id
                    else None,
                ),
            ),
            (entity, rr.TransformAxes3D(0.35)),
        ]
    elif type(message) is PointStamped:
        point = message.point
        entries = [(entity, rr.Points3D([[point.x, point.y, point.z]]))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    else:
        return None
    fields = asdict(
        message,
        dict_factory=lambda items: {key: value for key, value in items if key != "__msgtype__"},
    )
    entries.append((entity + "/message", rr.TextDocument(json.dumps(fields, indent=2))))
    return entries
