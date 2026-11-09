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
import json
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.message_utils import image, occupancy, pointcloud
from dimos.messages.dimos_msgs.msg import EntityMarkers, LineSegments3D
from dimos.messages.geometry_msgs.msg import PointStamped, PoseStamped
from dimos.messages.nav_msgs.msg import OccupancyGrid, Odometry, Path
from dimos.messages.sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.messages.vision_msgs.msg import Detection3DArray
from dimos.visualization.rerun.colors import occupancy_lut

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


def camera_pinhole(info: CameraInfo, optical_frame: str | None = None) -> rr.Pinhole:
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
    import rerun as rr

    if isinstance(message, CompressedImage):
        name = message.format.lower()
        if "jxl" in name or ("png" in name and name.startswith("16uc1;")):
            return image_archetype(image.from_compressed(message))
        if "jpeg" in name or "jpg" in name:
            return rr.EncodedImage(contents=bytes(message.data), media_type="image/jpeg")
        if "png" in name:
            return rr.EncodedImage(contents=bytes(message.data), media_type="image/png")
        raise ValueError(f"unsupported compressed image format {message.format!r}")
    if message.encoding in ("16UC1", "32FC1", "mono16"):
        pixels = image.view(message)
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
        bytes=image.to_rgb(message).tobytes(),
        width=message.width,
        height=message.height,
        datatype=np.uint8,
        color_model="RGB",
    )


def cloud_archetype(
    message: PointCloud2,
    *,
    voxel_size: float = 0.05,
    ui_radius: float = 2.0,
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

    points, colors = pointcloud.xyz(message), pointcloud.rgb(message)
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
        radii=rr.components.Radius.ui_points(ui_radius) if mode == "points" else voxel_size / 2,
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


def detection_boxes(message: Detection3DArray) -> rr.Boxes3D:
    import rerun as rr

    centers, sizes, rotations, labels = [], [], [], []
    for detection in message.detections:
        bbox = detection.bbox
        center, rotation, size = bbox.center.position, bbox.center.orientation, bbox.size
        centers.append((center.x, center.y, center.z))
        sizes.append((size.x / 2.0, size.y / 2.0, size.z / 2.0))
        rotations.append((rotation.x, rotation.y, rotation.z, rotation.w))
        marker_id = detection.id.strip()
        class_id = next(
            (
                result.hypothesis.class_id.strip()
                for result in detection.results
                if result.hypothesis.class_id.strip()
            ),
            "",
        )
        labels.append(
            f"{class_id} id={marker_id}"
            if marker_id and class_id
            else class_id or (f"id={marker_id}" if marker_id else "")
        )
    return rr.Boxes3D(centers=centers, half_sizes=sizes, quaternions=rotations, labels=labels)


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
    if type(message) is CameraInfo:
        entries = [(entity, camera_pinhole(message))]
    elif type(message) is TFMessage:
        entries = []
        for value in message.transforms:
            t, q = value.transform.translation, value.transform.rotation
            entries.append(
                (
                    "tf#/" + rr.escape_entity_path_part(value.child_frame_id),
                    rr.Transform3D(
                        translation=[t.x, t.y, t.z],
                        quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                        parent_frame="tf#/" + value.header.frame_id,
                        child_frame="tf#/" + value.child_frame_id,
                    ),
                )
            )
    elif type(message) is Path:
        entries = [(entity, path_archetype(message))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    elif type(message) is LineSegments3D:
        entries = [(entity, lines_archetype(message))]
    elif type(message) is OccupancyGrid:
        entries = [(entity, occupancy_archetype(message))]
    elif type(message) is Detection3DArray:
        entries = [(entity, detection_boxes(message))]
    elif type(message) is EntityMarkers:
        entries = [(entity, entity_points(message))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    elif type(message) in (PoseStamped, Odometry):
        pose = message.pose.pose if type(message) is Odometry else message.pose
        point, rotation = pose.position, pose.orientation
        entries = [
            (
                entity,
                rr.Transform3D(
                    translation=[point.x, point.y, point.z],
                    quaternion=rr.Quaternion(xyzw=[rotation.x, rotation.y, rotation.z, rotation.w]),
                    parent_frame=f"tf#/{message.header.frame_id}"
                    if message.header.frame_id
                    else None,
                    child_frame=f"tf#/{message.child_frame_id}"
                    if type(message) is Odometry and message.child_frame_id
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
    entries.append(
        (entity + "/message", rr.TextDocument(json.dumps(fields, indent=2, default=_json_value)))
    )
    return entries


def _json_value(value: Any) -> Any:
    """Represent generated numerical arrays without changing their source values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Cannot render {type(value).__name__} as JSON")


def occupancy_archetype(
    message: OccupancyGrid,
    colormap: str | None = None,
    z_offset: float = 0,
    opacity: float = 1.0,
    cost_range: tuple[int, int] | None = None,
    background: str | None = None,
    color_lookup_table: np.ndarray | None = None,
) -> rr.Mesh3D:
    """Convert to 3D textured mesh overlay on floor plane.

    Uses a single quad with the occupancy grid as a texture.
    Much more efficient than per-cell quads (4 vertices vs n_cells*4).
    """
    import rerun as rr

    if occupancy.view(message).size == 0:
        return rr.Mesh3D(vertex_positions=[])

    # Downsample grid before LUT to avoid allocating huge RGBA array
    max_tex = 256
    grid = occupancy.view(message)[::-1]  # flip for world coords (view, no copy)
    if grid.shape[0] > max_tex or grid.shape[1] > max_tex:
        step_h = max(1, grid.shape[0] // max_tex)
        step_w = max(1, grid.shape[1] // max_tex)
        grid = grid[::step_h, ::step_w]

    if color_lookup_table is None:
        color_lookup_table = occupancy_lut(colormap, opacity, background)
    rgba = np.ascontiguousarray(color_lookup_table[np.clip(grid + 1, 0, 101)])

    # Apply cost_range filter on downsampled grid
    if cost_range is not None:
        out_of_range = ((grid < cost_range[0]) | (grid > cost_range[1])) & (grid != -1)
        if np.any(out_of_range):
            bg_rgb = [0, 0, 0]
            if background is not None:
                bg = background.lstrip("#")
                bg_rgb = [int(bg[i : i + 2], 16) for i in (0, 2, 4)]
            rgba[out_of_range, :3] = bg_rgb
            rgba[out_of_range, 3] = 255

    # Single quad covering entire grid
    ox = message.info.origin.position.x
    oy = message.info.origin.position.y
    w = message.info.width * message.info.resolution
    h = message.info.height * message.info.resolution

    vertices = np.array(
        [
            [ox, oy, z_offset],  # 0: bottom-left (world)
            [ox + w, oy, z_offset],  # 1: bottom-right
            [ox + w, oy + h, z_offset],  # 2: top-right
            [ox, oy + h, z_offset],  # 3: top-left
        ],
        dtype=np.float32,
    )

    indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)

    # UV coords: Rerun uses top-left origin for textures
    # Grid row 0 is at world y=oy (bottom), row H-1 at y=oy+h (top)
    # Texture row 0 = grid row 0, so:
    #   world bottom (v0,v1) -> texture v=1 (bottom of texture)
    #   world top (v2,v3) -> texture v=0 (top of texture)
    texcoords = np.array(
        [
            [0.0, 1.0],  # v0: bottom-left world -> bottom-left tex
            [1.0, 1.0],  # v1: bottom-right world -> bottom-right tex
            [1.0, 0.0],  # v2: top-right world -> top-right tex
            [0.0, 0.0],  # v3: top-left world -> top-left tex
        ],
        dtype=np.float32,
    )

    return rr.Mesh3D(
        vertex_positions=vertices,
        triangle_indices=indices,
        vertex_texcoords=texcoords,
        albedo_texture=rgba,
    )


def lines_archetype(
    message: LineSegments3D, z_offset: float = 0.0, radii: float = 0.04
) -> rr.LineStrips3D:
    """Render generated line endpoints with the established log-weight palette."""
    import rerun as rr

    if not message.segments:
        return rr.LineStrips3D([])
    strips = np.asarray(
        [
            [
                [segment.start.x, segment.start.y, segment.start.z],
                [segment.end.x, segment.end.y, segment.end.z],
            ]
            for segment in message.segments
        ],
        dtype=np.float32,
    )
    strips[:, :, 2] += z_offset
    weights = np.asarray([segment.weight for segment in message.segments], dtype=np.float64)
    log_w = np.log10(np.maximum(weights, 1e-6))
    lo, hi = float(log_w.min()), float(log_w.max())
    norm = (log_w - lo) / (hi - lo) if hi > lo else np.zeros_like(log_w)
    red, green = (255 * norm).astype(np.uint8), (255 * (1.0 - norm)).astype(np.uint8)
    colors = np.column_stack([red, green, np.full_like(red, 60), np.full_like(red, 220)])
    return rr.LineStrips3D(strips, colors=colors, radii=radii)
