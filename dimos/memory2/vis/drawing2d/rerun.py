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

"""Rerun renderer for Drawing2D. Logs scene elements as 3D archetypes."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

from dimos.memory2.type.observation import EmbeddedObservation
from dimos.memory2.vis.color import hex_to_rgb
from dimos.memory2.vis.type import Arrow, Box3D, Camera, Point, Polyline, Pose, Text
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory2.vis.drawing2d.drawing2d import Drawing2D


def render(drawing: Drawing2D, app_id: str = "drawing2d", spawn: bool = True) -> None:
    """Render a Drawing2D to a Rerun viewer."""
    import rerun as rr
    import rerun.blueprint as rrb

    rr.init(app_id, spawn=spawn)

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
    embedded_obs: list[EmbeddedObservation[Any]] = []

    for el in drawing.elements:
        if isinstance(el, EmbeddedObservation):
            embedded_obs.append(el)
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
    has_images = any(c.image is not None for c in cameras) or any(
        _has_image(obs) for obs in embedded_obs
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
        views.append(rrb.Spatial2DView(origin="scene/cameras", name="Images"))

    blueprint = rrb.Blueprint(
        rrb.Horizontal(*views, column_shares=[2, 1]) if len(views) > 1 else views[0]
    )
    rr.send_blueprint(blueprint)

    # Log elements
    if grids:
        for i, el in enumerate(grids):
            rr.log(f"scene/map/{i}", el.to_rerun(), static=True)

    if pointclouds:
        for i, el in enumerate(pointclouds):
            rr.log(f"scene/pointcloud/{i}", el.to_rerun(), static=True)

    if points:
        rr.log(
            "scene/points",
            rr.Points3D(
                positions=[[p.msg.x, p.msg.y, p.msg.z] for p in points],
                colors=[hex_to_rgb(p.color) for p in points],
                radii=[max(p.radius, 0.05) for p in points],
                labels=[p.label or "" for p in points] if any(p.label for p in points) else None,
            ),
            static=True,
        )

    if poses:
        rr.log(
            "scene/poses",
            rr.Points3D(
                positions=[[p.msg.x, p.msg.y, 0] for p in poses],
                colors=[hex_to_rgb(p.color) for p in poses],
                radii=[p.size * 0.3 for p in poses],
                labels=[p.label or "" for p in poses] if any(p.label for p in poses) else None,
            ),
            static=True,
        )
        rr.log(
            "scene/poses/headings",
            rr.Arrows3D(
                origins=[[p.msg.x, p.msg.y, 0] for p in poses],
                vectors=[
                    [math.cos(p.msg.yaw) * p.size, math.sin(p.msg.yaw) * p.size, 0] for p in poses
                ],
                colors=[hex_to_rgb(p.color) for p in poses],
            ),
            static=True,
        )

    if arrows:
        rr.log(
            "scene/arrows",
            rr.Arrows3D(
                origins=[[a.msg.x, a.msg.y, 0] for a in arrows],
                vectors=[
                    [math.cos(a.msg.yaw) * a.length, math.sin(a.msg.yaw) * a.length, 0]
                    for a in arrows
                ],
                colors=[hex_to_rgb(a.color) for a in arrows],
            ),
            static=True,
        )

    if boxes:
        rr.log(
            "scene/boxes",
            rr.Boxes3D(
                centers=[[b.center.x, b.center.y, 0] for b in boxes],
                half_sizes=[[b.size.x / 2, b.size.y / 2, b.size.z / 2] for b in boxes],
                colors=[hex_to_rgb(b.color) for b in boxes],
                labels=[b.label or "" for b in boxes] if any(b.label for b in boxes) else None,
            ),
            static=True,
        )

    for i, el in enumerate(polylines):
        rr.log(
            f"scene/polylines/{i}",
            rr.LineStrips3D(
                strips=[[[p.x, p.y, 0] for p in el.msg.poses]],
                colors=[hex_to_rgb(el.color)],
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
                colors=[hex_to_rgb(t.color) for t in texts],
                radii=[0.01] * len(texts),
            ),
            static=True,
        )

    for i, el in enumerate(cameras):
        path = f"scene/cameras/{i}"
        rr.log(path, el.pose.to_rerun(), static=True)
        if el.camera_info:
            rr.log(path, el.camera_info.to_rerun(), static=True)
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

    for i, obs in enumerate(embedded_obs):
        path = f"scene/observations/{i}"
        rr.log(path, obs.pose_stamped.to_rerun(), static=True)
        if _has_image(obs):
            img = obs.data
            h, w = img.shape[:2]
            focal = max(w, h)
            rr.log(
                path,
                rr.Pinhole(focal_length=focal, principal_point=[w / 2, h / 2], resolution=[w, h]),
                static=True,
            )
            rr.log(f"{path}/image", img.to_rerun(), static=True)


def _has_image(obs: EmbeddedObservation[Any]) -> bool:
    from dimos.msgs.sensor_msgs.Image import Image

    return isinstance(obs.data, Image)
