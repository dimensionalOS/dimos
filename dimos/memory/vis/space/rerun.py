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

"""Rerun renderer for Space. Logs scene elements as 3D archetypes."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.nav_msgs.msg import OccupancyGrid
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.occupancy.visualizations import generate_rgba_texture
from dimos.memory.type.observation import Observation
from dimos.memory.vis.color import Color
from dimos.memory.vis.space.elements import Arrow, Box3D, Camera, Point, Polyline, Pose, Text
from dimos.memory.vis.space.geometry import message_position, message_yaw
from dimos.msgs.geometry import (
    compose_transforms,
    pose_matrix,
    transform_from_pose,
    transform_matrix,
)
from dimos.msgs.occupancy import occupancy_extent, occupancy_view
from dimos.msgs.pointcloud import pointcloud_rgb, pointcloud_xyz

if TYPE_CHECKING:
    from dimos.memory.vis.space.space import Space


def _rgba(el: Any) -> tuple[int, int, int, int]:
    """Combine element color + opacity into an RGBA u8 tuple for rerun."""
    c = Color.coerce(getattr(el, "color", "#000000"))
    opacity = float(getattr(el, "opacity", 1.0))
    return c.with_alpha(c.a * opacity).rgba_u8()


# base_link → camera_optical extrinsics (applied at render time for image observations)
_BASE_TO_OPTICAL = TransformStamped(
    header=Header(frame_id="base_link"),
    child_frame_id="camera_optical",
    transform=Transform(
        translation=Vector3(x=0.3),
        rotation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5),
    ),
)


def render(space: Space, app_id: str = "space", spawn: bool = True) -> None:
    """Render a Space to a Rerun viewer."""
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.visualization.rerun.init import rerun_init

    rerun_init(app_id, spawn=spawn)

    # Collect elements by type
    points: list[Point] = []
    poses: list[Pose] = []
    arrows: list[Arrow] = []
    boxes: list[Box3D] = []
    cameras: list[Camera] = []
    polylines: list[Polyline] = []
    texts: list[Text] = []
    grids: list[OccupancyGrid] = []
    pointclouds: list[PointCloud2] = []
    observations: list[Observation[Any]] = []
    panels: list[Observation[Any]] = []

    for el in space.elements:
        if isinstance(el, Observation):
            if _is_image(el.data) and el.pose is None:
                panels.append(el)
            else:
                observations.append(el)
        elif isinstance(el, Point):
            points.append(el)
        elif isinstance(el, Pose):
            poses.append(el)
        elif isinstance(el, Arrow):
            arrows.append(el)
        elif isinstance(el, Box3D):
            boxes.append(el)
        elif isinstance(el, Camera):
            cameras.append(el)
        elif isinstance(el, Polyline):
            polylines.append(el)
        elif isinstance(el, Text):
            texts.append(el)
        elif isinstance(el, OccupancyGrid):
            grids.append(el)
        elif isinstance(el, PointCloud2):
            pointclouds.append(el)

    # Build and send blueprint
    has_images = (
        any(c.image is not None for c in cameras)
        or any(_has_image(obs) for obs in observations)
        or bool(panels)
    )
    views: list[Any] = [
        rrb.Spatial3DView(
            origin="scene",
            name="Scene",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        )
    ]
    if has_images:
        views.append(rrb.Spatial2DView(origin="scene", name="Images"))

    blueprint = rrb.Blueprint(
        rrb.Horizontal(*views, column_shares=[2, 1]) if len(views) > 1 else views[0]
    )
    rr.send_blueprint(blueprint)

    # Log elements
    if grids:
        for i, el in enumerate(grids):
            cells = occupancy_view(el)
            if cells.size == 0:
                continue
            width, height = occupancy_extent(el)
            matrix = pose_matrix(el.info.origin)
            corners = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]])
            vertices = corners @ matrix[:3, :3].T + matrix[:3, 3]
            rr.log(
                f"scene/map/{i}",
                rr.Mesh3D(
                    vertex_positions=vertices,
                    triangle_indices=[[0, 1, 2], [0, 2, 3]],
                    vertex_texcoords=[[0, 1], [1, 1], [1, 0], [0, 0]],
                    albedo_texture=np.ascontiguousarray(np.flipud(generate_rgba_texture(cells))),
                ),
                static=True,
            )

    if pointclouds:
        for i, el in enumerate(pointclouds):
            rr.log(
                f"scene/pointcloud/{i}",
                rr.Points3D(pointcloud_xyz(el), colors=pointcloud_rgb(el)),
                static=True,
            )

    if points:
        rr.log(
            "scene/points",
            rr.Points3D(
                positions=[
                    [
                        message_position(p.msg).x,
                        message_position(p.msg).y,
                        message_position(p.msg).z,
                    ]
                    for p in points
                ],
                colors=[_rgba(p) for p in points],
                radii=[max(p.radius, 0.05) for p in points],
                labels=[p.label or "" for p in points] if any(p.label for p in points) else None,
            ),
            static=True,
        )

    if poses:
        rr.log(
            "scene/poses",
            rr.Points3D(
                positions=[
                    [message_position(p.msg).x, message_position(p.msg).y, 0] for p in poses
                ],
                colors=[_rgba(p) for p in poses],
                radii=[p.size * 0.3 for p in poses],
                labels=[p.label or "" for p in poses] if any(p.label for p in poses) else None,
            ),
            static=True,
        )
        rr.log(
            "scene/poses/headings",
            rr.Arrows3D(
                origins=[[message_position(p.msg).x, message_position(p.msg).y, 0] for p in poses],
                vectors=[
                    [
                        math.cos(message_yaw(p.msg)) * p.size,
                        math.sin(message_yaw(p.msg)) * p.size,
                        0,
                    ]
                    for p in poses
                ],
                colors=[_rgba(p) for p in poses],
            ),
            static=True,
        )

    if arrows:
        rr.log(
            "scene/arrows",
            rr.Arrows3D(
                origins=[[message_position(a.msg).x, message_position(a.msg).y, 0] for a in arrows],
                vectors=[
                    [
                        math.cos(message_yaw(a.msg)) * a.length,
                        math.sin(message_yaw(a.msg)) * a.length,
                        0,
                    ]
                    for a in arrows
                ],
                colors=[_rgba(a) for a in arrows],
            ),
            static=True,
        )

    if boxes:
        rr.log(
            "scene/boxes",
            rr.Boxes3D(
                centers=[[b.center.position.x, b.center.position.y, 0] for b in boxes],
                half_sizes=[[b.size.x / 2, b.size.y / 2, b.size.z / 2] for b in boxes],
                colors=[_rgba(b) for b in boxes],
                labels=[b.label or "" for b in boxes] if any(b.label for b in boxes) else None,
            ),
            static=True,
        )

    for i, el in enumerate(polylines):
        rr.log(
            f"scene/polylines/{i}",
            rr.LineStrips3D(
                strips=[[[p.pose.position.x, p.pose.position.y, 0] for p in el.msg.poses]],
                colors=[_rgba(el)],
                radii=[el.width / 2],
            ),
            static=True,
        )

    if texts:
        rr.log(
            "scene/texts",
            rr.Points3D(
                positions=[[t.position[0], t.position[1], 0] for t in texts],
                labels=[t.text for t in texts],
                colors=[_rgba(t) for t in texts],
                radii=[0.01] * len(texts),
            ),
            static=True,
        )

    for i, el in enumerate(cameras):
        path = f"scene/cameras/{i}"
        matrix = pose_matrix(el.pose.pose)
        rr.log(path, rr.Transform3D(translation=matrix[:3, 3], mat3x3=matrix[:3, :3]), static=True)
        if el.camera_info:
            pinhole = el.camera_info.to_rerun()
            assert not isinstance(pinhole, list)
            rr.log(path, pinhole, static=True)
        elif el.image:
            h, w = el.image.shape[:2]
            focal = max(w, h)
            rr.log(
                path,
                rr.Pinhole(focal_length=focal, principal_point=[w / 2, h / 2], resolution=[w, h]),
                static=True,
            )
        if el.image:
            rr.log(f"{path}/image", el.image.to_rerun(), static=True)

    for i, obs in enumerate(observations):
        path = f"scene/observations/{i}"
        data = obs.data
        ps = obs.pose_stamped
        if ps is None:
            continue
        img = _as_image(data)
        if img is not None:
            # Apply base→optical extrinsics for camera frustum rendering
            world_T_optical = compose_transforms(
                transform_from_pose(ps, child_frame_id="base_link"), _BASE_TO_OPTICAL
            )
            matrix = transform_matrix(world_T_optical.transform)
            rr.log(
                path, rr.Transform3D(translation=matrix[:3, 3], mat3x3=matrix[:3, :3]), static=True
            )
            h, w = img.shape[:2]
            focal = max(w, h)
            rr.log(
                path,
                rr.Pinhole(
                    focal_length=focal,
                    principal_point=[w / 2, h / 2],
                    resolution=[w, h],
                    image_plane_distance=1.0,
                ),
                static=True,
            )
            rr.log(f"{path}/image", img.to_rerun(), static=True)
        elif isinstance(data, PointCloud2):
            matrix = pose_matrix(ps.pose)
            rr.log(
                path, rr.Transform3D(translation=matrix[:3, 3], mat3x3=matrix[:3, :3]), static=True
            )
            rr.log(
                f"{path}/pointcloud",
                rr.Points3D(pointcloud_xyz(data), colors=pointcloud_rgb(data)),
                static=True,
            )
        elif isinstance(data, (int, float)):
            rr.log(
                path,
                rr.Points3D(
                    positions=[[ps.pose.position.x, ps.pose.position.y, 0]],
                    labels=[str(data)],
                    radii=[0.025],
                ),
                static=True,
            )
        elif isinstance(data, str):
            # Word-wrap for label
            words = data.split()
            lines: list[str] = []
            line: str = ""
            for word in words:
                if line and len(line) + len(word) + 1 > 40:
                    lines.append(line)
                    line = word
                else:
                    line = f"{line} {word}" if line else word
            if line:
                lines.append(line)
            label = "\n".join(lines)
            x, y = ps.pose.position.x, ps.pose.position.y
            # Pin: line from ground up, label at the tip
            rr.log(
                f"{path}/pin",
                rr.LineStrips3D(
                    strips=[[[x, y, 1.5], [x, y, 3.0]]],
                    colors=[(100, 100, 100)],
                    radii=[0.01],
                ),
                static=True,
            )
            rr.log(
                f"{path}/label",
                rr.Points3D(
                    positions=[[x, y, 3.5]],
                    labels=[label],
                    radii=[0.001],
                ),
                static=True,
            )
        else:
            rr.log(
                path,
                rr.Points3D(positions=[[ps.pose.position.x, ps.pose.position.y, 0]], radii=[0.05]),
                static=True,
            )

    for i, obs in enumerate(panels):
        img = _as_image(obs.data)
        if img is not None:
            rr.log(f"scene/panels/{i}", img.to_rerun(), static=True)


def _as_image(data: Any) -> Any | None:
    """Return an Image if data is an Image or ImageDetections, else None."""
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.imageDetections import ImageDetections

    if isinstance(data, Image):
        return data
    if isinstance(data, ImageDetections):
        return data.annotated_image()
    return None


def _is_image(data: Any) -> bool:
    return _as_image(data) is not None


def _has_image(obs: Observation[Any]) -> bool:
    return _is_image(obs.data)
